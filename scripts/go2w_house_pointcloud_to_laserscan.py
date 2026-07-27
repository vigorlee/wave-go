#!/usr/bin/env python3
"""Project the HouseWorld LiDAR PointCloud2 stream into a 2-D LaserScan."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ProjectionConfig:
    min_height: float
    max_height: float
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    scan_time: float

    def __post_init__(self) -> None:
        if not self.min_height < self.max_height:
            raise ValueError("min_height must be lower than max_height")
        if not self.angle_min < self.angle_max:
            raise ValueError("angle_min must be lower than angle_max")
        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError("range_min must be non-negative and lower than range_max")
        if self.scan_time <= 0.0:
            raise ValueError("scan_time must be positive")

    @property
    def bin_count(self) -> int:
        return max(
            1,
            int(
                math.floor(
                    (self.angle_max - self.angle_min) / self.angle_increment
                    + 1.0e-9
                )
            )
            + 1,
        )

    @property
    def published_angle_max(self) -> float:
        return self.angle_min + (self.bin_count - 1) * self.angle_increment


def project_points_to_ranges(
    points_xyz: np.ndarray,
    config: ProjectionConfig,
) -> np.ndarray:
    """Return the nearest planar range in each angular bin."""

    points = np.asarray(points_xyz, dtype=np.float32)
    if points.size == 0:
        return np.full(config.bin_count, np.inf, dtype=np.float32)
    points = points.reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    height = (points[:, 2] >= config.min_height) & (
        points[:, 2] <= config.max_height
    )
    planar_ranges = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    valid = (
        finite
        & height
        & (planar_ranges >= config.range_min)
        & (planar_ranges <= config.range_max)
        & (angles >= config.angle_min)
        & (angles <= config.published_angle_max)
    )

    output = np.full(config.bin_count, np.inf, dtype=np.float32)
    if not np.any(valid):
        return output
    indices = np.floor(
        (angles[valid] - config.angle_min) / config.angle_increment
    ).astype(np.int64)
    indices = np.clip(indices, 0, config.bin_count - 1)
    np.minimum.at(output, indices, planar_ranges[valid])
    return output


def _build_node_class() -> tuple[type[Any], Any]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2

    class HousePointCloudToLaserScan(Node):
        def __init__(self) -> None:
            super().__init__("go2w_house_pointcloud_to_laserscan")
            self.declare_parameter("input_topic", "/livox/lidar")
            self.declare_parameter("output_topic", "/scan")
            self.declare_parameter("frame_id", "lidar")
            self.declare_parameter("min_height", -0.10)
            self.declare_parameter("max_height", 0.45)
            self.declare_parameter("angle_min", -math.pi)
            self.declare_parameter("angle_max", math.pi)
            self.declare_parameter("angle_increment", math.pi / 360.0)
            self.declare_parameter("range_min", 0.20)
            self.declare_parameter("range_max", 12.0)
            self.declare_parameter("scan_time", 0.10)

            self.input_topic = str(self.get_parameter("input_topic").value)
            self.output_topic = str(self.get_parameter("output_topic").value)
            self.frame_id = str(self.get_parameter("frame_id").value)
            self.config = ProjectionConfig(
                min_height=float(self.get_parameter("min_height").value),
                max_height=float(self.get_parameter("max_height").value),
                angle_min=float(self.get_parameter("angle_min").value),
                angle_max=float(self.get_parameter("angle_max").value),
                angle_increment=float(
                    self.get_parameter("angle_increment").value
                ),
                range_min=float(self.get_parameter("range_min").value),
                range_max=float(self.get_parameter("range_max").value),
                scan_time=float(self.get_parameter("scan_time").value),
            )
            if not self.input_topic or not self.output_topic or not self.frame_id:
                raise ValueError("input_topic, output_topic, and frame_id are required")

            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.publisher = self.create_publisher(
                LaserScan, self.output_topic, sensor_qos
            )
            self.subscription = self.create_subscription(
                PointCloud2, self.input_topic, self.on_cloud, sensor_qos
            )
            self.published = 0
            self.frame_warning_reported = False
            self.get_logger().info(
                "House PointCloud2 to LaserScan ready: "
                f"{self.input_topic} -> {self.output_topic} frame={self.frame_id} "
                f"height={self.config.min_height:.2f}..{self.config.max_height:.2f}m"
            )

        @staticmethod
        def _xyz(message: PointCloud2) -> np.ndarray:
            raw = point_cloud2.read_points(
                message, field_names=["x", "y", "z"], skip_nans=True
            )
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                return np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
                    np.float32, copy=False
                )
            return np.asarray(list(raw), dtype=np.float32).reshape((-1, 3))

        def on_cloud(self, message: PointCloud2) -> None:
            if (
                message.header.frame_id
                and message.header.frame_id != self.frame_id
                and not self.frame_warning_reported
            ):
                self.frame_warning_reported = True
                self.get_logger().warning(
                    f"input frame is {message.header.frame_id!r}; publishing scan in "
                    f"configured frame {self.frame_id!r}"
                )
            points = self._xyz(message)
            ranges = project_points_to_ranges(points, self.config)

            scan = LaserScan()
            scan.header = message.header
            scan.header.frame_id = self.frame_id
            scan.angle_min = self.config.angle_min
            scan.angle_max = self.config.published_angle_max
            scan.angle_increment = self.config.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = self.config.scan_time
            scan.range_min = self.config.range_min
            scan.range_max = self.config.range_max
            scan.ranges = ranges.tolist()
            self.publisher.publish(scan)
            self.published += 1
            if self.published == 1:
                finite_count = int(np.isfinite(ranges).sum())
                self.get_logger().info(
                    f"published first /scan: points={len(points)} "
                    f"occupied_bins={finite_count}/{len(ranges)}"
                )

    return HousePointCloudToLaserScan, rclpy


def main() -> None:
    node_class, rclpy = _build_node_class()
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
