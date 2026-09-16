#!/usr/bin/python3
"""Drive one continuous Nav2 action over the physical ramp and verify 3-D motion."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


ROUTE = (
    (0.25, 13.35, math.pi / 2.0),
    (0.50, 15.40, math.pi / 2.0),
    (0.50, 16.30, math.pi / 2.0),
    (0.55, 18.05, math.pi / 2.0),
)


def rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class RampValidator(Node):
    def __init__(self) -> None:
        super().__init__("go2w_physical_ramp_validator")
        self.client = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.samples: list[dict[str, float]] = []
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 50)

    def on_odom(self, message: Odometry) -> None:
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        roll, pitch, yaw = rpy(q.x, q.y, q.z, q.w)
        linear = message.twist.twist.linear
        self.samples.append(
            {
                "time": time.time(),
                "x": float(p.x),
                "y": float(p.y),
                "z": float(p.z),
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "vx": float(linear.x),
                "vy": float(linear.y),
            }
        )

    def pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def wait(self, future: Any, deadline: float) -> bool:
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def execute(self, timeout: float) -> tuple[int | None, str]:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not self.samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.samples:
            return None, "odometry_unavailable"
        if not self.client.wait_for_server(timeout_sec=min(10.0, timeout)):
            return None, "navigate_through_poses_unavailable"
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.pose(*item) for item in ROUTE]
        request = self.client.send_goal_async(goal)
        if not self.wait(request, deadline):
            return None, "goal_request_timeout"
        handle = request.result()
        if handle is None or not handle.accepted:
            return None, "goal_rejected"
        result = handle.get_result_async()
        if not self.wait(result, deadline):
            handle.cancel_goal_async()
            return None, "goal_result_timeout"
        return int(result.result().status), "goal_completed"


def range_samples(samples: list[dict[str, float]], low: float, high: float) -> list[dict[str, float]]:
    return [item for item in samples if low <= item["y"] <= high]


def analyze(samples: list[dict[str, float]], action_status: int | None, reason: str) -> dict[str, Any]:
    approach = range_samples(samples, 12.10, 12.85)
    uphill = range_samples(samples, 12.75, 15.55)
    downhill = range_samples(samples, 15.45, 18.30)
    exit_samples = range_samples(samples, 17.80, 18.85)
    approach_z = min((item["z"] for item in approach), default=None)
    crest_z = max((item["z"] for item in uphill + downhill), default=None)
    exit_z = exit_samples[-1]["z"] if exit_samples else None
    gain = crest_z - approach_z if crest_z is not None and approach_z is not None else None
    drop = crest_z - exit_z if crest_z is not None and exit_z is not None else None
    up_pitch = max((abs(item["pitch"]) for item in uphill), default=0.0)
    down_pitch = max((abs(item["pitch"]) for item in downhill), default=0.0)
    obstacle_distances = {
        "ramp_up_center_cylinder_m": min(
            (math.hypot(item["x"] + 0.05, item["y"] - 14.15) for item in samples),
            default=None,
        ),
        "ramp_down_center_cylinder_m": min(
            (math.hypot(item["x"] - 0.05, item["y"] - 17.05) for item in samples),
            default=None,
        ),
    }
    passed = bool(
        action_status == GoalStatus.STATUS_SUCCEEDED
        and gain is not None
        and drop is not None
        and gain >= 0.30
        and drop >= 0.225
        and crest_z is not None
        and crest_z >= 0.72
        and exit_z is not None
        and exit_z <= 0.58
        and up_pitch >= 0.035
        and down_pitch >= 0.07
        and all(value is not None and value >= 0.34 for value in obstacle_distances.values())
    )
    return {
        "passed": passed,
        "action_status": action_status,
        "action_reason": reason,
        "execution_mode": "single_navigate_through_poses_goal",
        "pose_count": len(ROUTE),
        "sample_count": len(samples),
        "approach_z_m": approach_z,
        "crest_max_z_m": crest_z,
        "exit_z_m": exit_z,
        "elevation_gain_m": gain,
        "elevation_drop_m": drop,
        "uphill_max_abs_pitch_rad": up_pitch,
        "downhill_max_abs_pitch_rad": down_pitch,
        "obstacle_clearance": obstacle_distances,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = RampValidator()
    try:
        status, reason = node.execute(max(30.0, args.timeout_sec))
        report = analyze(node.samples, status, reason)
        trace = args.output.with_suffix(".csv")
        if node.samples:
            with trace.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(node.samples[0]))
                writer.writeheader()
                writer.writerows(node.samples)
        report["trace"] = str(trace)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
