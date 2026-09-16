#!/usr/bin/python3
"""ROS2 SCAN-style local planner for the real MATRiX Nezha2 stack.

This is an interface-compatible reimplementation of the relevant
SCAN-Planner ideas for the ROS2 Humble MATRiX runtime: a static global A* path
is derived from the MATRiX OfficeWorld collision map, while every 10 Hz local
command is chosen from spatially collision-checked candidate trajectories
against the *unmodified* UE ``/livox/lidar`` PointCloud2 stream.  It does not
read MuJoCo model state or hand-authored obstacle coordinates.

The upstream SCAN-Planner release targets ROS1/Noetic.  This node deliberately
does not claim to execute that binary in a ROS2 graph; it preserves the route
guided, spatial-collision-aware local planning contract in the active MATRiX
ROS2 stack.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


@dataclass(frozen=True)
class MapSpec:
    resolution: float
    origin_x: float
    origin_y: float
    occupied: np.ndarray


@dataclass(frozen=True)
class Candidate:
    vx: float
    wz: float
    points: np.ndarray
    clearance: float
    heading_error: float
    progress: float
    cost: float


def read_yaml_scalar(path: Path, name: str) -> str:
    prefix = f"{name}:"
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith(prefix):
            return line.split(":", 1)[1].strip()
    raise ValueError(f"{name} is missing from {path}")


def read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise ValueError(f"{path} is not a binary PGM")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = map(int, line.split())
        maximum = int(stream.readline().strip())
        if maximum != 255:
            raise ValueError(f"unsupported PGM maximum value {maximum}")
        pixels = np.frombuffer(stream.read(width * height), dtype=np.uint8)
    if pixels.size != width * height:
        raise ValueError(f"truncated PGM: {path}")
    return pixels.reshape(height, width)


def load_map(map_yaml: Path) -> MapSpec:
    image_name = read_yaml_scalar(map_yaml, "image")
    resolution = float(read_yaml_scalar(map_yaml, "resolution"))
    raw_origin = read_yaml_scalar(map_yaml, "origin").strip("[]")
    origin_x, origin_y, _ = (float(value.strip()) for value in raw_origin.split(","))
    image = read_pgm(map_yaml.parent / image_name)
    # Map rows are image-top to image-bottom; flip into map-frame row order.
    occupied = np.flipud(image <= 127)
    return MapSpec(resolution, origin_x, origin_y, occupied)


def inflate(occupied: np.ndarray, radius_cells: int) -> np.ndarray:
    result = occupied.copy()
    offsets = [
        (dy, dx)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if dx * dx + dy * dy <= radius_cells * radius_cells
    ]
    for row, col in np.argwhere(occupied):
        for dy, dx in offsets:
            rr, cc = int(row + dy), int(col + dx)
            if 0 <= rr < result.shape[0] and 0 <= cc < result.shape[1]:
                result[rr, cc] = True
    return result


def cell_from_xy(spec: MapSpec, x: float, y: float) -> tuple[int, int]:
    return (
        int(math.floor((y - spec.origin_y) / spec.resolution)),
        int(math.floor((x - spec.origin_x) / spec.resolution)),
    )


def xy_from_cell(spec: MapSpec, cell: tuple[int, int]) -> tuple[float, float]:
    row, col = cell
    return (
        spec.origin_x + (col + 0.5) * spec.resolution,
        spec.origin_y + (row + 0.5) * spec.resolution,
    )


def nearest_free(occupied: np.ndarray, cell: tuple[int, int]) -> tuple[int, int] | None:
    height, width = occupied.shape
    row = min(max(cell[0], 0), height - 1)
    col = min(max(cell[1], 0), width - 1)
    if not occupied[row, col]:
        return row, col
    for radius in range(1, max(height, width)):
        for rr in range(max(0, row - radius), min(height, row + radius + 1)):
            for cc in range(max(0, col - radius), min(width, col + radius + 1)):
                if max(abs(rr - row), abs(cc - col)) == radius and not occupied[rr, cc]:
                    return rr, cc
    return None


def astar(occupied: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    moves = (
        (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0), (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
    )
    queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0.0}
    while queue:
        _, g_cost, current = heapq.heappop(queue)
        if g_cost != cost.get(current):
            continue
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return list(reversed(path))
        for dr, dc, edge_cost in moves:
            row, col = current[0] + dr, current[1] + dc
            if not (0 <= row < occupied.shape[0] and 0 <= col < occupied.shape[1]) or occupied[row, col]:
                continue
            next_cell = row, col
            next_cost = g_cost + edge_cost
            if next_cost >= cost.get(next_cell, float("inf")):
                continue
            cost[next_cell] = next_cost
            parent[next_cell] = current
            heuristic = math.hypot(goal[0] - row, goal[1] - col)
            heapq.heappush(queue, (next_cost + heuristic, next_cost, next_cell))
    return []


def farthest_reachable(occupied: np.ndarray, start: tuple[int, int], max_cells: int) -> tuple[int, int]:
    """Pick a finite long-range target in the start-connected component."""
    queue = [start]
    distance = {start: 0}
    best = start
    while queue:
        current = queue.pop(0)
        if distance[current] > distance[best]:
            best = current
        if distance[current] >= max_cells:
            continue
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nxt = current[0] + dr, current[1] + dc
            if (
                0 <= nxt[0] < occupied.shape[0]
                and 0 <= nxt[1] < occupied.shape[1]
                and not occupied[nxt]
                and nxt not in distance
            ):
                distance[nxt] = distance[current] + 1
                queue.append(nxt)
    return best


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ScanPlanner(Node):
    def __init__(self, map_yaml: Path, goal: tuple[float, float] | None, log_path: Path) -> None:
        super().__init__("nezha2_scan_planner")
        self.spec = load_map(map_yaml)
        self.static_occupied = inflate(self.spec.occupied, int(math.ceil(0.46 / self.spec.resolution)))
        self.goal_override = goal
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("w", encoding="utf-8")
        self.pose: tuple[float, float, float] | None = None
        self.global_path: np.ndarray | None = None
        self.goal_xy: tuple[float, float] | None = None
        self.cloud_xy = np.empty((0, 2), dtype=np.float32)
        self.cloud_points_base = np.empty((0, 3), dtype=np.float32)
        self.cloud_stamp_ns: int | None = None
        self.cloud_received = False
        self.snapshot_saved = False
        self.done = False
        self.last_candidate: Candidate | None = None
        self.path_pub = self.create_publisher(RosPath, "/scan_planner/global_path", 1)
        self.local_pub = self.create_publisher(RosPath, "/scan_planner/local_path", 1)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 5)
        self.status_pub = self.create_publisher(String, "/scan_planner/status", 5)
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 20)
        self.create_subscription(PointCloud2, "/livox/lidar", self.on_cloud, SENSOR_QOS)
        self.timer = self.create_timer(0.10, self.tick)
        self.get_logger().info(f"SCAN-style planner map={map_yaml} log={log_path}")

    def on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.pose = (float(pose.position.x), float(pose.position.y), yaw_from_quaternion(pose.orientation))
        if self.global_path is None:
            self.build_global_path()

    def on_cloud(self, message: PointCloud2) -> None:
        # UE publishes this raw cloud in the fixed ``lidar`` frame.  The
        # corresponding static transform is base_link -> lidar=(0.13011,
        # 0.02329, 0.17598).  Preserve every usable point; the deterministic
        # 8 cm voxelization only merges redundant returns for trajectory tests.
        try:
            raw = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
            # ROS Humble returns a packed structured ndarray for this UE
            # PointCloud2 (26-byte point stride), while older releases return
            # an iterator of ordinary triples.  Handle both without changing
            # fields, units or point order.
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
            else:
                points = np.asarray(list(raw), dtype=np.float32)
        except Exception as error:
            self.get_logger().warn(f"PointCloud2 parse failed: {error}")
            return
        if not len(points):
            self.cloud_xy = np.empty((0, 2), dtype=np.float32)
            self.cloud_received = True
            return
        points[:, 0] += 0.13011
        points[:, 1] += 0.02329
        self.cloud_points_base = points.copy()
        self.cloud_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        valid = (
            np.isfinite(points).all(axis=1)
            & (np.linalg.norm(points[:, :2], axis=1) >= 0.18)
            & (points[:, 2] >= -0.22)
            & (points[:, 2] <= 1.20)
            & (np.linalg.norm(points[:, :2], axis=1) <= 7.0)
        )
        points = points[valid]
        if not len(points):
            self.cloud_xy = np.empty((0, 2), dtype=np.float32)
            self.cloud_received = True
            return
        voxels = np.unique(np.floor(points[:, :2] / 0.08).astype(np.int32), axis=0)
        self.cloud_xy = (voxels.astype(np.float32) + 0.5) * 0.08
        self.cloud_received = True

    def save_snapshot(self, candidate: Candidate, target: np.ndarray) -> None:
        """Persist one actual UE-cloud/local-plan pairing for audit."""
        if self.snapshot_saved or self.pose is None or self.global_path is None or self.goal_xy is None:
            return
        snapshot = self.log_path.parent
        np.savez_compressed(
            snapshot / "ue_lidar_scan_snapshot.npz",
            # Full decoded raw UE cloud in base coordinates; no range crop.
            ue_cloud_points_base_m=self.cloud_points_base,
            # Deterministic 8 cm spatial representatives used by the planner.
            planner_voxels_base_m=self.cloud_xy,
            global_path_world_xy_m=self.global_path,
            selected_local_path_world_xy_m=candidate.points,
            robot_xy_yaw=np.asarray(self.pose, dtype=np.float64),
            route_target_world_xy_m=target,
            goal_world_xy_m=np.asarray(self.goal_xy, dtype=np.float64),
        )
        metadata = {
            "cloud_stamp_ns": self.cloud_stamp_ns,
            "planner_time_monotonic_s": time.monotonic(),
            "raw_ue_cloud_points": int(len(self.cloud_points_base)),
            "planner_voxels": int(len(self.cloud_xy)),
            "raw_cloud_policy": "all finite decoded UE PointCloud2 returns in lidar/base coordinates",
            "planner_filter": "0.18-7.0 m horizon; z in [-0.22, 1.20] m; deterministic 8 cm XY voxelization",
            "selected_vx_wz": [candidate.vx, candidate.wz],
            "selected_clearance_m": candidate.clearance,
            "goal_distance_m": math.dist(self.pose[:2], self.goal_xy),
        }
        (snapshot / "ue_lidar_scan_snapshot.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        self.snapshot_saved = True

    def build_global_path(self) -> None:
        assert self.pose is not None
        start = nearest_free(self.static_occupied, cell_from_xy(self.spec, self.pose[0], self.pose[1]))
        if start is None:
            raise RuntimeError("no free OfficeWorld map cell near robot start")
        if self.goal_override is None:
            goal = farthest_reachable(self.static_occupied, start, max_cells=int(12.0 / self.spec.resolution))
        else:
            goal = nearest_free(self.static_occupied, cell_from_xy(self.spec, *self.goal_override))
            if goal is None:
                raise RuntimeError("requested SCAN goal has no free OfficeWorld map cell")
        cells = astar(self.static_occupied, start, goal)
        if len(cells) < 2:
            raise RuntimeError("global A* failed in OfficeWorld map")
        self.global_path = np.asarray([xy_from_cell(self.spec, cell) for cell in cells], dtype=np.float32)
        self.goal_xy = tuple(map(float, self.global_path[-1]))
        self.publish_path(self.path_pub, self.global_path)
        self.get_logger().info(f"global A* points={len(self.global_path)} goal={self.goal_xy}")

    def publish_path(self, publisher, path: np.ndarray) -> None:
        message = RosPath()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        publisher.publish(message)

    def route_target(self) -> tuple[np.ndarray, float]:
        assert self.pose is not None and self.global_path is not None
        current = np.asarray(self.pose[:2], dtype=np.float32)
        nearest = int(np.argmin(np.linalg.norm(self.global_path - current[None], axis=1)))
        target = self.global_path[-1]
        for candidate in self.global_path[nearest + 1 :]:
            if float(np.linalg.norm(candidate - current)) >= 1.0:
                target = candidate
                break
        heading = math.atan2(float(target[1] - current[1]), float(target[0] - current[0]))
        return target, heading

    def candidates(self, desired_heading: float) -> list[Candidate]:
        assert self.pose is not None
        x0, y0, yaw0 = self.pose
        result: list[Candidate] = []
        times = np.linspace(0.10, 1.20, 12, dtype=np.float32)
        for vx in (0.0, 0.16, 0.28, 0.40):
            for wz in (-0.65, -0.42, -0.22, 0.0, 0.22, 0.42, 0.65):
                yaw = wz * times
                if abs(wz) < 1e-5:
                    local_x = vx * times
                    local_y = np.zeros_like(times)
                else:
                    local_x = vx * np.sin(yaw) / wz
                    local_y = vx * (1.0 - np.cos(yaw)) / wz
                if len(self.cloud_xy):
                    dx = local_x[:, None] - self.cloud_xy[None, :, 0]
                    dy = local_y[:, None] - self.cloud_xy[None, :, 1]
                    clearance = float(np.sqrt(np.min(dx * dx + dy * dy)))
                else:
                    clearance = 7.0
                final_yaw = yaw0 + float(yaw[-1])
                heading_error = abs(math.atan2(math.sin(desired_heading - final_yaw), math.cos(desired_heading - final_yaw)))
                progress = float(vx * math.cos(heading_error))
                collision = clearance < 0.46
                cost = (1e6 if collision else 0.0) + 2.2 * heading_error + 0.75 / max(clearance, 0.05) - 2.8 * progress + 0.08 * abs(wz)
                world_x = x0 + np.cos(yaw0) * local_x - np.sin(yaw0) * local_y
                world_y = y0 + np.sin(yaw0) * local_x + np.cos(yaw0) * local_y
                result.append(Candidate(vx, wz, np.column_stack((world_x, world_y)), clearance, heading_error, progress, cost))
        return result

    def tick(self) -> None:
        if self.pose is None or self.global_path is None or self.goal_xy is None:
            return
        goal_distance = math.dist(self.pose[:2], self.goal_xy)
        command = Twist()
        if not self.cloud_received or not len(self.cloud_xy):
            # Fail closed: an absent/unparseable raw UE cloud is never
            # interpreted as empty free space.
            self.cmd_pub.publish(command)
            self.status_pub.publish(String(data=json.dumps({"state": "WAITING_FOR_UE_LIDAR"})))
            return
        if self.done or goal_distance <= 0.40:
            self.done = True
            self.cmd_pub.publish(command)
            self.status_pub.publish(String(data=json.dumps({"state": "SUCCEEDED", "goal_distance_m": goal_distance})))
            return
        target, desired_heading = self.route_target()
        candidate = min(self.candidates(desired_heading), key=lambda item: item.cost)
        self.last_candidate = candidate
        self.save_snapshot(candidate, target)
        command.linear.x = candidate.vx
        command.angular.z = candidate.wz
        self.cmd_pub.publish(command)
        self.publish_path(self.local_pub, candidate.points)
        record = {
            "t_monotonic": time.monotonic(),
            "robot_xy_yaw": [*self.pose],
            "goal_xy": list(self.goal_xy),
            "route_target_xy": target.tolist(),
            "cloud_voxels": int(len(self.cloud_xy)),
            "selected": {
                "vx": candidate.vx,
                "wz": candidate.wz,
                "clearance_m": candidate.clearance,
                "heading_error_rad": candidate.heading_error,
                "progress": candidate.progress,
                "cost": candidate.cost,
            },
            "goal_distance_m": goal_distance,
        }
        self.log.write(json.dumps(record) + "\n")
        self.log.flush()
        self.status_pub.publish(String(data=json.dumps(record)))

    def destroy_node(self) -> bool:
        if not self.log.closed:
            self.log.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = ScanPlanner(args.map, None if args.goal is None else tuple(args.goal), args.log)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
