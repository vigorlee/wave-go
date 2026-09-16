#!/usr/bin/env python3
"""Expose MATRiX's unmodified UE LiDAR stream on RoamerX's input topic.

The UE runtime publishes its physical ray-cast PointCloud2 as
``/livox/lidar_raw``.  RoamerX/Nav2 consumes ``/livox/lidar``.  This relay
does not crop, synthesize, transform, filter, or fuse points: it republishes
the original message object with its original ``lidar`` frame and fields.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class RawLidarRelay(Node):
    def __init__(self) -> None:
        super().__init__("nezha2_raw_lidar_relay")
        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar", SENSOR_QOS)
        self.subscription = self.create_subscription(
            PointCloud2, "/livox/lidar_raw", self.on_cloud, SENSOR_QOS
        )
        self.message_count = 0
        self.get_logger().info(
            "relay armed: /livox/lidar_raw -> /livox/lidar (unmodified PointCloud2)"
        )

    def on_cloud(self, message: PointCloud2) -> None:
        self.publisher.publish(message)
        self.message_count += 1
        if self.message_count == 1:
            self.get_logger().info(
                f"first raw cloud relayed: frame={message.header.frame_id} points={message.width * message.height}"
            )


def main() -> None:
    rclpy.init()
    node = RawLidarRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
