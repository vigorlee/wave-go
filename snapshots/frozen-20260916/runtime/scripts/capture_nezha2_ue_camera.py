#!/usr/bin/env python3
"""Save one frame from MATRiX's actual UE compressed RGB topic.

This deliberately reads the simulator camera topic instead of the desktop
window, so a capture reflects the configured sensor pose and contains no
desktop, RViz, or generated overlay pixels.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class Capture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("nezha2_ue_camera_capture")
        self.frame: np.ndarray | None = None
        self.subscription = self.create_subscription(
            CompressedImage, topic, self.callback, 5
        )

    def callback(self, message: CompressedImage) -> None:
        decoded = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if decoded is not None:
            self.frame = decoded


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/image_raw/compressed")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout must be positive")

    rclpy.init()
    node = Capture(args.topic)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while rclpy.ok() and node.frame is None:
            rclpy.spin_once(node, timeout_sec=0.25)
            if node.get_clock().now().nanoseconds >= deadline:
                raise TimeoutError(f"no UE image received on {args.topic}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), node.frame):
            raise RuntimeError(f"failed to write {args.output}")
        print(f"saved UE camera frame: {args.output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
