#!/usr/bin/env python3
"""Validate the flat/global Nav2 segments of the extended mixed-terrain route.

The stair and ramp surfaces are executed by the Go2-W locomotion/terrain guard;
this check deliberately asks Nav2 only whether each consecutive map-frame goal
is reachable in the known YardWorld corridor.  It never publishes a velocity
or NavigateToPose command.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


ROUTE = (
    ("descend_far_staircase", 0.2, 10.2, math.pi / 2.0),
    ("slalom_pass_first_pair", 0.08, 11.15, math.pi / 2.0),
    ("slalom_pass_second_pair", -0.08, 11.85, math.pi / 2.0),
    ("slalom_exit_before_ramp", 0.08, 12.48, math.pi / 2.0),
    ("ramp_up_entry", 0.25, 13.35, math.pi / 2.0),
    ("ramp_up_crest", 0.50, 15.40, math.pi / 2.0),
    ("ramp_down_entry", 0.50, 16.30, math.pi / 2.0),
    ("ramp_down_exit", 0.55, 18.05, math.pi / 2.0),
    ("approach_remote_obstacles", 0.85, 18.95, math.pi / 2.0),
    ("cross_dynamic_pedestrian_zone", 0.85, 20.80, math.pi / 2.0),
    ("clear_remote_obstacles", 0.40, 22.00, math.pi / 2.0),
    ("traverse_north_corridor", 0.40, 28.80, math.pi / 2.0),
    ("round_north_corner", 2.50, 32.80, 0.0),
    ("cross_north_gallery", 17.50, 32.50, 0.0),
    ("enter_east_corridor", 18.00, 31.00, -math.pi / 2.0),
    ("traverse_east_corridor", 18.00, 26.50, -math.pi / 2.0),
    ("navigate_to_farthest_endpoint", 18.00, 22.80, -math.pi / 2.0),
)


def make_pose(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_length(path: Any) -> float:
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(path.poses, path.poses[1:])
    )


class Validator(Node):
    def __init__(self, timeout: float, margin: float) -> None:
        super().__init__("go2w_extended_route_validator")
        self.timeout = timeout
        self.margin = margin
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def wait(self, future: Any, label: str) -> Any:
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"timed out waiting for {label}")
        return future.result()

    def segment(self, start: tuple[str, float, float, float], goal: tuple[str, float, float, float]) -> dict[str, Any]:
        request = ComputePathToPose.Goal()
        request.start = make_pose(self, start[1], start[2], start[3])
        request.goal = make_pose(self, goal[1], goal[2], goal[3])
        request.planner_id = "GridBased"
        request.use_start = True
        handle = self.wait(self.client.send_goal_async(request), f"{goal[0]} response")
        if handle is None or not handle.accepted:
            raise RuntimeError(f"planner rejected {start[0]} -> {goal[0]}")
        wrapped = self.wait(handle.get_result_async(), f"{goal[0]} result")
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"planner failed {start[0]} -> {goal[0]} status={wrapped.status}")
        poses = wrapped.result.path.poses
        if len(poses) < 2:
            raise RuntimeError(f"empty path for {start[0]} -> {goal[0]}")
        points = [(float(p.pose.position.x), float(p.pose.position.y)) for p in poses]
        min_x = min(start[1], goal[1]) - self.margin
        max_x = max(start[1], goal[1]) + self.margin
        min_y = min(start[2], goal[2]) - self.margin
        max_y = max(start[2], goal[2]) + self.margin
        overrun = max(
            math.hypot(
                max(min_x - x, 0.0, x - max_x),
                max(min_y - y, 0.0, y - max_y),
            )
            for x, y in points
        )
        endpoint_error = math.hypot(points[-1][0] - goal[1], points[-1][1] - goal[2])
        return {
            "stage_id": goal[0],
            "start": {"x": start[1], "y": start[2]},
            "goal": {"x": goal[1], "y": goal[2], "yaw_rad": goal[3]},
            "pose_count": len(poses),
            "path_length_m": path_length(wrapped.result.path),
            "endpoint_error_m": endpoint_error,
            "corridor_overrun_m": overrun,
            "validated": endpoint_error <= 0.55 and overrun <= 1e-6,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--corridor-margin", type=float, default=1.5)
    args = parser.parse_args()
    rclpy.init()
    node = Validator(max(2.0, args.timeout_sec), max(0.25, args.corridor_margin))
    report: dict[str, Any] = {
        "route": "farthest_end_via_stairs_ramps_cylinders_pedestrians",
        "drives_robot": False,
        "segments": [],
    }
    try:
        if not node.client.wait_for_server(timeout_sec=node.timeout):
            raise RuntimeError("/compute_path_to_pose is unavailable")
        start = ("validated_stair_exit", 0.2, 10.2, math.pi / 2.0)
        for goal in ROUTE[1:]:
            result = node.segment(start, goal)
            report["segments"].append(result)
            print(
                f"[PATH] {result['stage_id']} length={result['path_length_m']:.2f}m "
                f"poses={result['pose_count']} overrun={result['corridor_overrun_m']:.3f}m "
                f"validated={result['validated']}"
            )
            start = goal
        report["total_flat_extension_m"] = sum(item["path_length_m"] for item in report["segments"])
        report["validated"] = all(item["validated"] for item in report["segments"])
    except Exception as exc:
        report["validated"] = False
        report["error"] = str(exc)
        print(f"[PATH_FAIL] {exc}", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if report.get("validated") else 1


if __name__ == "__main__":
    raise SystemExit(main())
