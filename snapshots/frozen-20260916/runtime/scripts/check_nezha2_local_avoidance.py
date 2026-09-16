#!/usr/bin/python3
"""Verify that Nezha2 LiDAR data is producing local obstacle costs."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class LocalAvoidanceProbe(Node):
    def __init__(self) -> None:
        super().__init__("nezha2_local_avoidance_probe")
        self.costmap: OccupancyGrid | None = None
        self.cloud: PointCloud2 | None = None
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_costmap, map_qos
        )
        self.create_subscription(PointCloud2, "/livox/lidar", self._on_cloud, cloud_qos)

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self.costmap = message

    def _on_cloud(self, message: PointCloud2) -> None:
        self.cloud = message


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--require-lethal", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = LocalAvoidanceProbe()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and (
            node.costmap is None or node.cloud is None
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.costmap is None or node.cloud is None:
            missing = []
            if node.costmap is None:
                missing.append("/local_costmap/costmap")
            if node.cloud is None:
                missing.append("/livox/lidar")
            print(f"LOCAL_AVOIDANCE_FAIL missing={','.join(missing)}", file=sys.stderr)
            return 1

        values = list(node.costmap.data)
        occupied = sum(value > 0 for value in values)
        lethal = sum(value >= 99 for value in values)
        points = int(node.cloud.width) * int(node.cloud.height)
        print(
            "LOCAL_AVOIDANCE_OK "
            f"cloud_frame={node.cloud.header.frame_id} points={points} "
            f"costmap_frame={node.costmap.header.frame_id} "
            f"size={node.costmap.info.width}x{node.costmap.info.height} "
            f"resolution={node.costmap.info.resolution:.3f} "
            f"occupied={occupied} lethal={lethal}"
        )
        if points == 0 or (args.require_lethal and lethal == 0):
            return 2
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
