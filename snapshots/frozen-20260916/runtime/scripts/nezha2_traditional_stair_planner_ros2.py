#!/usr/bin/python3
"""Traditional LiDAR-grid A* stair avoidance for the real MATRiX YardWorld.

The route geometry (stair width, riser spacing and goal landing) is known
terrain semantics.  Every obstacle cell comes only from UE's live
``/livox/lidar`` PointCloud2.  The planner inflates the observed occupancy by
the Nezha2 footprint, runs 2-D A*, and publishes a rate-limited velocity
reference to ``/cmd_vel_nav``.  It never reads obstacle coordinates from XML.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


RESOLUTION = 0.05
X_MIN, X_MAX = -1.50, 1.50
Y_MIN, Y_MAX = -0.35, float(os.getenv("NEZHA2_STAIR_MAP_Y_MAX", "5.30"))
STAIR_FIRST_Y = float(os.getenv("NEZHA2_STAIR_FIRST_Y", "0.75"))
STAIR_TREAD = float(os.getenv("NEZHA2_STAIR_TREAD", "0.28"))
STAIR_RISE = float(os.getenv("NEZHA2_STAIR_RISE", "0.05"))
STAIR_COUNT = int(os.getenv("NEZHA2_STAIR_COUNT", "10"))
GOAL = np.array([
    float(os.getenv("NEZHA2_STAIR_GOAL_X", "0.65")),
    float(os.getenv("NEZHA2_STAIR_GOAL_Y", "4.80")),
], dtype=np.float64)
# Validated Nezha2 entry offset: avoids both wheel pairs striking the same
# riser at once.  It is a terrain-route prior, not an obstacle coordinate.
FOOTPRINT_RADIUS = 0.52
LIDAR_OFFSET = np.array([0.13011, 0.02329, 0.17598], dtype=np.float64)
UE_LIDAR_SCALE = float(os.getenv("NEZHA2_UE_LIDAR_SCALE", "1.0"))
UE_LIDAR_POINTS_ARE_WORLD = os.getenv("NEZHA2_UE_LIDAR_POINTS_ARE_WORLD", "0") in {"1", "true", "TRUE"}
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


def stair_height(y: np.ndarray) -> np.ndarray:
    index = np.floor((y - STAIR_FIRST_Y) / STAIR_TREAD).astype(np.int32) + 1
    index = np.clip(index, 0, STAIR_COUNT)
    index[y < STAIR_FIRST_Y] = 0
    return index.astype(np.float64) * STAIR_RISE


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cell_from_xy(xy: np.ndarray) -> tuple[int, int]:
    return int(math.floor((xy[1] - Y_MIN) / RESOLUTION)), int(math.floor((xy[0] - X_MIN) / RESOLUTION))


def xy_from_cell(cell: tuple[int, int]) -> np.ndarray:
    row, col = cell
    return np.array([X_MIN + (col + 0.5) * RESOLUTION, Y_MIN + (row + 0.5) * RESOLUTION], dtype=np.float64)


def nearest_free(grid: np.ndarray, cell: tuple[int, int]) -> tuple[int, int] | None:
    height, width = grid.shape
    row, col = min(max(cell[0], 0), height - 1), min(max(cell[1], 0), width - 1)
    if not grid[row, col]:
        return row, col
    for radius in range(1, max(height, width)):
        for rr in range(max(0, row - radius), min(height, row + radius + 1)):
            for cc in range(max(0, col - radius), min(width, col + radius + 1)):
                if max(abs(rr - row), abs(cc - col)) == radius and not grid[rr, cc]:
                    return rr, cc
    return None


def astar(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    neighbours = ((-1, -1, math.sqrt(2)), (-1, 0, 1), (-1, 1, math.sqrt(2)), (0, -1, 1), (0, 1, 1), (1, -1, math.sqrt(2)), (1, 0, 1), (1, 1, math.sqrt(2)))
    queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    score = {start: 0.0}
    while queue:
        _, cost, current = heapq.heappop(queue)
        if cost != score.get(current):
            continue
        if current == goal:
            route = [current]
            while current in parent:
                current = parent[current]
                route.append(current)
            return list(reversed(route))
        for dr, dc, edge in neighbours:
            nxt = current[0] + dr, current[1] + dc
            if not (0 <= nxt[0] < grid.shape[0] and 0 <= nxt[1] < grid.shape[1]) or grid[nxt]:
                continue
            candidate = cost + edge
            if candidate >= score.get(nxt, float("inf")):
                continue
            score[nxt] = candidate
            parent[nxt] = current
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(queue, (candidate + heuristic, candidate, nxt))
    return []


class StairGridPlanner(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("nezha2_traditional_stair_planner")
        self.output = output
        output.mkdir(parents=True, exist_ok=True)
        self.log = (output / "traditional_stair_planner.jsonl").open("w", encoding="utf-8")
        self.pose: tuple[float, float, float, float] | None = None
        self.raw_points = np.empty((0, 3), dtype=np.float32)
        self.cloud_stamp_ns: int | None = None
        self.cloud_received = False
        self.snapshot_saved = False
        self.done = False
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 5)
        self.global_pub = self.create_publisher(RosPath, "/traditional_stair/global_path", 1)
        self.local_pub = self.create_publisher(RosPath, "/traditional_stair/local_path", 1)
        self.status_pub = self.create_publisher(String, "/traditional_stair/status", 5)
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 20)
        self.create_subscription(PointCloud2, "/livox/lidar", self.on_cloud, SENSOR_QOS)
        self.create_timer(0.20, self.plan_and_command)

    def on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.pose = (float(pose.position.x), float(pose.position.y), float(pose.position.z), yaw_from_quaternion(pose.orientation))

    def on_cloud(self, message: PointCloud2) -> None:
        try:
            raw = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
            else:
                points = np.asarray(list(raw), dtype=np.float32)
        except Exception as error:
            self.get_logger().warn(f"UE PointCloud2 parse failed: {error}")
            return
        # MATRiX maps may publish UE centimetres (CustomWorld) or metres
        # (pre-scaled map packages).  The launcher records the explicit scale
        # used for this run; all terrain semantics below are in metres.
        self.raw_points = points * UE_LIDAR_SCALE
        self.cloud_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        self.cloud_received = True

    def publish_path(self, publisher, points: np.ndarray) -> None:
        msg = RosPath()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = float(x), float(y), 1.0
            msg.poses.append(pose)
        publisher.publish(msg)

    def observed_obstacles(self) -> tuple[np.ndarray, np.ndarray]:
        assert self.pose is not None
        x, y, z, yaw = self.pose
        points = self.raw_points.astype(np.float64)
        # CustomWorld's packaged sensor emits UE world coordinates in
        # centimetres even though its PointCloud2 header remains ``lidar``.
        # Other MATRiX maps use a conventional lidar frame.  Handle this
        # documented runtime distinction explicitly instead of applying a
        # second transform and misclassifying ground returns as obstacles.
        if UE_LIDAR_POINTS_ARE_WORLD:
            world = points
        else:
            points += LIDAR_OFFSET
            c, s = math.cos(yaw), math.sin(yaw)
            world = np.empty_like(points)
            world[:, 0] = x + c * points[:, 0] - s * points[:, 1]
            world[:, 1] = y + s * points[:, 0] + c * points[:, 1]
            world[:, 2] = z + points[:, 2]
        xy_range = np.linalg.norm(points[:, :2], axis=1)
        in_route = (np.abs(world[:, 0]) <= 1.55) & (world[:, 1] >= Y_MIN - 0.2) & (world[:, 1] <= Y_MAX + 0.2)
        terrain_z = stair_height(world[:, 1])
        treads = np.abs(world[:, 2] - terrain_z) <= 0.08
        riser = np.zeros(len(world), dtype=bool)
        for index in range(STAIR_COUNT):
            edge = STAIR_FIRST_Y + index * STAIR_TREAD
            riser |= (np.abs(world[:, 1] - edge) <= 0.06) & (world[:, 2] >= index * STAIR_RISE - 0.04) & (world[:, 2] <= (index + 1) * STAIR_RISE + 0.08)
        obstacle = in_route & ~treads & ~riser & (world[:, 2] - terrain_z >= 0.10) & (world[:, 2] - terrain_z <= 1.20) & (xy_range >= 0.18) & (xy_range <= 7.0)
        return world, world[obstacle]

    def occupancy(self, observed: np.ndarray) -> np.ndarray:
        height = int(round((Y_MAX - Y_MIN) / RESOLUTION))
        width = int(round((X_MAX - X_MIN) / RESOLUTION))
        grid = np.zeros((height, width), dtype=bool)
        x_centres = X_MIN + (np.arange(width) + 0.5) * RESOLUTION
        grid[:, np.abs(x_centres) > 1.50 - FOOTPRINT_RADIUS] = True
        for point in observed:
            row, col = cell_from_xy(point[:2])
            radius = int(math.ceil(FOOTPRINT_RADIUS / RESOLUTION))
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if dr * dr + dc * dc > radius * radius:
                        continue
                    rr, cc = row + dr, col + dc
                    if 0 <= rr < height and 0 <= cc < width:
                        grid[rr, cc] = True
        return grid

    def save_snapshot(self, world: np.ndarray, obstacles: np.ndarray, route: np.ndarray, local_route: np.ndarray, command: Twist) -> None:
        if self.snapshot_saved or self.pose is None:
            return
        np.savez_compressed(
            self.output / "ue_stair_lidar_snapshot.npz",
            ue_cloud_points_world_m=world,
            planner_obstacle_points_world_m=obstacles,
            global_route_world_xy_m=route,
            local_route_world_xy_m=local_route,
            robot_xyz_yaw=np.asarray(self.pose, dtype=np.float64),
        )
        metadata = {
            "cloud_stamp_ns": self.cloud_stamp_ns,
            "raw_ue_points": int(len(world)),
            "ue_lidar_scale_to_m": UE_LIDAR_SCALE,
            "ue_lidar_points_are_world": UE_LIDAR_POINTS_ARE_WORLD,
            "obstacle_points": int(len(obstacles)),
            "terrain_rule": "known stair tread/riser height semantics; all obstacles are extracted from UE LiDAR",
            "traditional_planner": "5 cm occupancy grid + 0.52 m footprint inflation + A*",
            "command_vx_wz": [command.linear.x, command.angular.z],
        }
        (self.output / "ue_stair_lidar_snapshot.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        self.snapshot_saved = True

    def plan_and_command(self) -> None:
        zero = Twist()
        if self.pose is None or not self.cloud_received or not len(self.raw_points):
            self.cmd_pub.publish(zero)
            return
        x, y, _, yaw = self.pose
        if self.done or math.hypot(GOAL[0] - x, GOAL[1] - y) < 0.38:
            self.done = True
            self.cmd_pub.publish(zero)
            self.status_pub.publish(String(data=json.dumps({"state": "SUCCEEDED", "goal_distance_m": math.hypot(GOAL[0] - x, GOAL[1] - y)})))
            return
        world, obstacles = self.observed_obstacles()
        grid = self.occupancy(obstacles)
        start = nearest_free(grid, cell_from_xy(np.array([x, y])))
        goal = nearest_free(grid, cell_from_xy(GOAL))
        cells = astar(grid, start, goal) if start and goal else []
        command = Twist()
        route = np.array([[0.0, 0.0], [GOAL[0], STAIR_FIRST_Y], [GOAL[0], STAIR_FIRST_Y + STAIR_COUNT * STAIR_TREAD], GOAL], dtype=np.float64)
        local_route = np.array([xy_from_cell(cell) for cell in cells], dtype=np.float64) if cells else np.empty((0, 2))
        if len(local_route) > 1:
            target = local_route[-1]
            for point in local_route[1:]:
                if math.hypot(point[0] - x, point[1] - y) >= 0.60:
                    target = point
                    break
            dx, dy = target[0] - x, target[1] - y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            heading = math.atan2(local_y, max(local_x, 1e-6))
            distance = math.hypot(local_x, local_y)
            forward = 0.62 * distance * math.cos(heading)
            # The validated Nezha2 stair policy needs a positive forward
            # reference to enter its wheel-leg climb gait.  A near-zero A*
            # look-ahead speed is safe on level ground but stalls at a riser.
            minimum_forward = 0.45 if y >= 0.60 else 0.12
            command.linear.x = float(np.clip(forward, minimum_forward, 0.62))
            command.angular.z = float(np.clip(0.60 * heading, -0.15, 0.15))
        self.cmd_pub.publish(command)
        self.publish_path(self.global_pub, route)
        self.publish_path(self.local_pub, local_route)
        self.save_snapshot(world, obstacles, route, local_route, command)
        record = {"time_monotonic_s": time.monotonic(), "robot_xy": [x, y], "obstacle_points": int(len(obstacles)), "path_found": bool(len(local_route) > 1), "cmd": [command.linear.x, command.angular.z], "goal_distance_m": math.hypot(GOAL[0] - x, GOAL[1] - y)}
        self.log.write(json.dumps(record) + "\n")
        self.log.flush()
        self.status_pub.publish(String(data=json.dumps(record)))

    def destroy_node(self) -> bool:
        if not self.log.closed:
            self.log.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = StairGridPlanner(args.output_dir)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
