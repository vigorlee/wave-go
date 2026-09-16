#!/usr/bin/env python3
"""Verify that OLI remains upright after navigation motion is disarmed."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HoldCheck(Node):
    def __init__(self) -> None:
        super().__init__("oli_navigation_hold_check")
        self.first_position = None
        self.last_position = None
        self.minimum_height = float("inf")
        self.maximum_tilt = 0.0
        self.create_subscription(
            Odometry, "/odom/mujoco_odom", self.odom_callback, 20
        )

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        current = (position.x, position.y, position.z)
        self.first_position = self.first_position or current
        self.last_position = current
        self.minimum_height = min(self.minimum_height, position.z)

        qw = orientation.w
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(
            max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        )
        self.maximum_tilt = max(
            self.maximum_tilt, abs(roll), abs(pitch)
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--maximum-drift", type=float, default=0.15)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = HoldCheck()
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if node.first_position is None or node.last_position is None:
            raise RuntimeError("No OLI odometry received during hold check")

        drift = math.hypot(
            node.last_position[0] - node.first_position[0],
            node.last_position[1] - node.first_position[1],
        )
        hold_qualified = (
            node.minimum_height > 0.75
            and node.last_position[2] > 0.75
            and node.maximum_tilt < math.radians(35.0)
            and drift <= args.maximum_drift
        )

        result = json.loads(args.result.read_text(encoding="utf-8"))
        result.update(
            {
                "post_navigation_hold_seconds": args.duration,
                "post_hold_qualified": hold_qualified,
                "post_hold_final_position": list(node.last_position),
                "post_hold_minimum_base_height": node.minimum_height,
                "post_hold_maximum_tilt_degrees": math.degrees(
                    node.maximum_tilt
                ),
                "post_hold_drift_meters": drift,
            }
        )
        result["qualified"] = bool(result.get("qualified")) and hold_qualified
        args.result.write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, indent=2))
        print("HOLD_SUCCESS" if hold_qualified else "HOLD_FAILED")
        return 0 if hold_qualified else 1
    except (OSError, RuntimeError, json.JSONDecodeError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
