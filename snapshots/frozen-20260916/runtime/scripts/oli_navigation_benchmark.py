#!/usr/bin/env python3
"""Send one OLI navigation goal and verify arrival, posture, and settling."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class Benchmark(Node):
    def __init__(self) -> None:
        super().__init__("oli_navigation_benchmark")
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.create_subscription(
            Odometry, "/odom/mujoco_odom", self.odom_callback, 20
        )
        self.first_pose = None
        self.last_pose = None
        self.min_z = float("inf")
        self.max_tilt = 0.0

    def odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = (position.x, position.y, position.z)
        self.first_pose = self.first_pose or pose
        self.last_pose = pose
        self.min_z = min(self.min_z, position.z)

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
        self.max_tilt = max(self.max_tilt, abs(roll), abs(pitch))


def spin_until(node: Node, future, deadline: float) -> bool:
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.done()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float, nargs="?", default=0.0)
    parser.add_argument("y", type=float, nargs="?", default=0.8)
    parser.add_argument("yaw", type=float, nargs="?", default=math.pi / 2.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "/home/unitree/matrix_go2w_lcm_demo/"
            "artifacts/oli_navigation/latest_result.json"
        ),
    )
    args = parser.parse_args()

    rclpy.init()
    node = Benchmark()
    status = None
    timed_out = False
    fell = False

    try:
        odom_deadline = time.monotonic() + 10.0
        while node.last_pose is None and time.monotonic() < odom_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_pose is None:
            raise RuntimeError("No OLI odometry received")
        if not node.client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/navigate_to_pose is not available")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

        send_future = node.client.send_goal_async(goal)
        if not spin_until(node, send_future, time.monotonic() + 10.0):
            raise RuntimeError("Timed out while sending navigation goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Navigation goal was rejected")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.last_pose is not None and node.last_pose[2] < 0.62:
                fell = True
                goal_handle.cancel_goal_async()
                break
            if time.monotonic() >= deadline:
                timed_out = True
                goal_handle.cancel_goal_async()
                break
        if result_future.done():
            status = result_future.result().status

        settle_deadline = time.monotonic() + args.settle
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except RuntimeError as error:
        node.get_logger().error(str(error))
    finally:
        first = node.first_pose or (float("nan"),) * 3
        last = node.last_pose or (float("nan"),) * 3
        final_distance = math.hypot(last[0] - args.x, last[1] - args.y)
        moved = math.hypot(last[0] - first[0], last[1] - first[1])
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not timed_out
            and not fell
            and node.min_z > 0.72
            and last[2] > 0.75
            and node.max_tilt < math.radians(35.0)
            and final_distance <= 0.25
            and moved >= 0.20
        )
        result = {
            "qualified": succeeded,
            "action_status": status,
            "timed_out": timed_out,
            "fell": fell,
            "target": [args.x, args.y, args.yaw],
            "initial_position": list(first),
            "final_position": list(last),
            "distance_moved": moved,
            "final_goal_distance": final_distance,
            "minimum_base_height": node.min_z,
            "maximum_tilt_degrees": math.degrees(node.max_tilt),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        print("NAVIGATION_SUCCESS" if succeeded else "NAVIGATION_FAILED")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if succeeded else 1


if __name__ == "__main__":
    raise SystemExit(main())
