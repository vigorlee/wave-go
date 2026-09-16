#!/usr/bin/env python3
"""Capture one fresh compressed camera frame with its synchronized G1 pose."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Capture(Node):
    def __init__(self) -> None:
        super().__init__("capture_g1_target_pose")
        self.lock = threading.Lock()
        self.image: bytes | None = None
        self.image_time = 0.0
        self.pose: tuple[float, float, float] | None = None
        self.pose_time = 0.0
        self.create_subscription(CompressedImage, "/image_raw/compressed", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 10)

    def on_image(self, message: CompressedImage) -> None:
        with self.lock:
            self.image = bytes(message.data)
            self.image_time = time.monotonic()

    def on_odom(self, message: Odometry) -> None:
        value = message.pose.pose
        with self.lock:
            self.pose = (float(value.position.x), float(value.position.y), yaw(value.orientation))
            self.pose_time = time.monotonic()

    def snapshot(self):
        with self.lock:
            return self.image, self.image_time, self.pose, self.pose_time


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Capture()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            image, image_time, pose, pose_time = node.snapshot()
            now = time.monotonic()
            if image is not None and pose is not None and now - image_time < 1.0 and now - pose_time < 1.0:
                image_path = output / "goal.jpg"
                image_path.write_bytes(image)
                payload = {
                    "schema_version": 1,
                    "label": args.label,
                    "robot_id": "g1-12dof-dreamwaq",
                    "scene_id": "matrix-officeworld",
                    "pose_xy_yaw": list(pose),
                    "image": str(image_path),
                    "image_sha256": hashlib.sha256(image).hexdigest(),
                    "captured_at": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
                }
                (output / "target.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
                print(json.dumps(payload, indent=2))
                return 0
        raise TimeoutError("fresh camera/odometry not available")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
