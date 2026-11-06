#!/usr/bin/python3
"""Prove that YardWorld cylinders reach the Go2-W local obstacle grid."""

from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


CYLINDERS = ((0.41, 18.09), (-0.49, 18.09), (-1.39, 18.09))
LIDAR_OFFSET = np.array((0.13011, 0.02329, 0.17598), dtype=float)


def rotate(q: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    xyz = q[:3]
    w = q[3]
    cross = np.cross(np.broadcast_to(xyz, vectors.shape), vectors)
    return vectors + 2.0 * (w * cross + np.cross(np.broadcast_to(xyz, vectors.shape), cross))


class Inspector(Node):
    def __init__(self) -> None:
        super().__init__("go2w_obstacle_cell_inspector")
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom = None
        self.cloud = None
        self.costmap = None
        self.create_subscription(Odometry, "/odom/mujoco_odom", self._odom, 5)
        self.create_subscription(PointCloud2, "/livox/lidar", self._cloud, sensor_qos)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._costmap, 5)

    def _odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _cloud(self, msg: PointCloud2) -> None:
        self.cloud = msg

    def _costmap(self, msg: OccupancyGrid) -> None:
        self.costmap = msg


def main() -> None:
    rclpy.init()
    node = Inspector()
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline and (
        node.odom is None or node.cloud is None or node.costmap is None
    ):
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None or node.cloud is None or node.costmap is None:
        print("[OBSTACLE_CELL_FAIL] missing odom, PointCloud2, or local costmap", file=sys.stderr)
        raise SystemExit(1)

    # The first OccupancyGrid can precede the first integrated cloud.  Wait for
    # multiple 5 Hz local-costmap updates and inspect the latest synchronized state.
    settle_deadline = time.monotonic() + 2.0
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    position = node.odom.pose.pose.position
    orientation = node.odom.pose.pose.orientation
    # robot_forward intentionally publishes base_link at z=0.05 while MuJoCo
    # odometry reports the floating-base center.  Costmaps follow the TF value.
    origin = np.array((position.x, position.y, 0.05), dtype=float)
    quat = np.array((orientation.x, orientation.y, orientation.z, orientation.w), dtype=float)
    raw_points = point_cloud2.read_points(
        node.cloud, field_names=("x", "y", "z"), skip_nans=True
    )
    if isinstance(raw_points, np.ndarray) and raw_points.dtype.names:
        points = np.column_stack(
            (raw_points["x"], raw_points["y"], raw_points["z"])
        ).astype(float, copy=False)
    else:
        points = np.asarray(list(raw_points), dtype=float)
    world = origin + rotate(quat, points + LIDAR_OFFSET)

    grid = node.costmap
    costs = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    resolution = grid.info.resolution
    grid_x = grid.info.origin.position.x
    grid_y = grid.info.origin.position.y
    print(
        f"[OBSTACLE_CELL] robot=({position.x:.3f},{position.y:.3f},{position.z:.3f}) "
        f"cloud_points={len(world)} costmap={grid.info.width}x{grid.info.height}@{resolution:.3f} "
        f"grid_origin=({grid_x:.3f},{grid_y:.3f})"
    )
    print(
        "[OBSTACLE_CELL] raw_bbox="
        f"x[{points[:, 0].min():.2f},{points[:, 0].max():.2f}] "
        f"y[{points[:, 1].min():.2f},{points[:, 1].max():.2f}] "
        f"z[{points[:, 2].min():.2f},{points[:, 2].max():.2f}] "
        f"world_z[{world[:, 2].min():.2f},{world[:, 2].max():.2f}]"
    )
    print(
        f"[OBSTACLE_CELL] costmap_max={int(costs.max(initial=-1))} "
        f"positive_cells={int((costs > 0).sum())} lethal_cells={int((costs >= 99).sum())}"
    )

    lethal_rows, lethal_cols = np.nonzero(costs >= 99)
    lethal_x = grid_x + (lethal_cols.astype(float) + 0.5) * resolution
    lethal_y = grid_y + (lethal_rows.astype(float) + 0.5) * resolution
    if len(lethal_x):
        print(
            "[OBSTACLE_CELL] lethal_bbox="
            f"x[{lethal_x.min():.2f},{lethal_x.max():.2f}] "
            f"y[{lethal_y.min():.2f},{lethal_y.max():.2f}]"
        )

    all_marked = True
    for cx, cy in CYLINDERS:
        distances = np.hypot(world[:, 0] - cx, world[:, 1] - cy)
        cluster = (distances <= 0.30) & (world[:, 2] >= 0.10) & (world[:, 2] <= 1.20)
        nearest_index = int(np.argmin(distances))
        nearest_world = world[nearest_index]
        nearest_raw = points[nearest_index]
        col = int((cx - grid_x) / resolution)
        row = int((cy - grid_y) / resolution)
        radius = max(1, int(math.ceil(0.30 / resolution)))
        if 0 <= row < grid.info.height and 0 <= col < grid.info.width:
            patch = costs[
                max(0, row - radius) : min(grid.info.height, row + radius + 1),
                max(0, col - radius) : min(grid.info.width, col + radius + 1),
            ]
            max_cost = int(patch.max(initial=-1))
        else:
            max_cost = -2
        marked = max_cost >= 99
        all_marked = all_marked and marked
        if len(lethal_x):
            nearest_lethal = float(np.hypot(lethal_x - cx, lethal_y - cy).min())
        else:
            nearest_lethal = math.inf
        print(
            f"[OBSTACLE_CELL] cylinder=({cx:.2f},{cy:.2f}) "
            f"nearest_point={float(distances.min(initial=999.0)):.3f}m "
            f"height_filtered_points={int(cluster.sum())} max_cost={max_cost} "
            f"nearest_lethal={nearest_lethal:.3f}m "
            f"nearest_raw=({nearest_raw[0]:.2f},{nearest_raw[1]:.2f},{nearest_raw[2]:.2f}) "
            f"nearest_world=({nearest_world[0]:.2f},{nearest_world[1]:.2f},{nearest_world[2]:.2f})"
        )

    node.destroy_node()
    rclpy.shutdown()
    if not all_marked:
        print("[OBSTACLE_CELL_FAIL] one or more visible cylinders are not lethal in local_costmap")
        raise SystemExit(1)
    print("[OBSTACLE_CELL_OK] visible cylinders are lethal in local_costmap")


if __name__ == "__main__":
    main()
