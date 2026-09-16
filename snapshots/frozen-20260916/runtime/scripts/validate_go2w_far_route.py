#!/usr/bin/python3
"""Validate the long YardWorld route with Nav2 without commanding motion."""

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


ROOT_DIR = Path(__file__).resolve().parents[1]

# This candidate follows the recorded YardWorld corridor instead of asking the
# global planner to invent a shortcut through a building outline. The first
# pose is the already validated endpoint of far_end_via_stairs.
ROUTE_POSES = (
    ("validated_far_end", 0.4, 14.4, math.pi / 2.0),
    ("approach_remote_obstacles", 1.2, 17.2, math.pi / 2.0),
    ("clear_remote_obstacles", 1.2, 19.5, math.pi / 2.0),
    ("traverse_north_corridor", 0.4, 28.8, math.pi / 2.0),
    ("round_north_corner", 2.5, 32.8, 0.0),
    ("cross_north_gallery", 17.5, 32.5, 0.0),
    ("enter_east_corridor", 18.0, 31.0, -math.pi / 2.0),
    ("traverse_east_corridor", 18.0, 26.5, -math.pi / 2.0),
    ("farthest_safe_endpoint", 18.0, 22.8, -math.pi / 2.0),
)


def pose_stamped(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
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


def corridor_overrun(
    start: tuple[float, float],
    goal: tuple[float, float],
    point: tuple[float, float],
    margin: float,
) -> float:
    min_x = min(start[0], goal[0]) - margin
    max_x = max(start[0], goal[0]) + margin
    min_y = min(start[1], goal[1]) - margin
    max_y = max(start[1], goal[1]) + margin
    outside_x = max(min_x - point[0], 0.0, point[0] - max_x)
    outside_y = max(min_y - point[1], 0.0, point[1] - max_y)
    return math.hypot(outside_x, outside_y)


class RoutePlanValidator(Node):
    def __init__(self, timeout_sec: float, corridor_margin: float) -> None:
        super().__init__("go2w_far_route_plan_validator")
        self.timeout_sec = timeout_sec
        self.corridor_margin = corridor_margin
        self.client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )

    def wait(self, future: Any, label: str) -> Any:
        deadline = time.monotonic() + self.timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"timed out waiting for {label}")
        return future.result()

    def validate_segment(
        self,
        start: tuple[str, float, float, float],
        goal: tuple[str, float, float, float],
    ) -> dict[str, Any]:
        request = ComputePathToPose.Goal()
        request.start = pose_stamped(self, start[1], start[2], start[3])
        request.goal = pose_stamped(self, goal[1], goal[2], goal[3])
        request.planner_id = "GridBased"
        request.use_start = True

        goal_handle = self.wait(
            self.client.send_goal_async(request), f"{goal[0]} goal response"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"planner rejected segment {start[0]} -> {goal[0]}")
        wrapped = self.wait(
            goal_handle.get_result_async(), f"{goal[0]} planning result"
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"planner failed segment {start[0]} -> {goal[0]} "
                f"with status {wrapped.status}"
            )

        path = wrapped.result.path
        if len(path.poses) < 2:
            raise RuntimeError(
                f"planner returned an empty path for {start[0]} -> {goal[0]}"
            )
        points = [
            (item.pose.position.x, item.pose.position.y) for item in path.poses
        ]
        max_overrun = max(
            corridor_overrun(
                (start[1], start[2]),
                (goal[1], goal[2]),
                point,
                self.corridor_margin,
            )
            for point in points
        )
        endpoint_error = math.hypot(points[-1][0] - goal[1], points[-1][1] - goal[2])
        return {
            "stage_id": goal[0],
            "start": {"x": start[1], "y": start[2]},
            "goal": {"x": goal[1], "y": goal[2], "yaw_rad": goal[3]},
            "pose_count": len(points),
            "path_length_m": path_length(path),
            "planning_time_sec": (
                wrapped.result.planning_time.sec
                + wrapped.result.planning_time.nanosec / 1_000_000_000.0
            ),
            "endpoint_error_m": endpoint_error,
            "bounds": {
                "min_x": min(point[0] for point in points),
                "max_x": max(point[0] for point in points),
                "min_y": min(point[1] for point in points),
                "max_y": max(point[1] for point in points),
            },
            "corridor_margin_m": self.corridor_margin,
            "corridor_overrun_m": max_overrun,
            "validated": endpoint_error <= 0.55 and max_overrun <= 1e-6,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--corridor-margin", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RoutePlanValidator(
        timeout_sec=max(2.0, args.timeout_sec),
        corridor_margin=max(0.25, args.corridor_margin),
    )
    report: dict[str, Any] = {
        "route": "beyond_obstacles_via_stairs_candidate",
        "drives_robot": False,
        "segments": [],
    }
    try:
        if not node.client.wait_for_server(timeout_sec=node.timeout_sec):
            raise RuntimeError("/compute_path_to_pose is unavailable")
        for start, goal in zip(ROUTE_POSES, ROUTE_POSES[1:]):
            result = node.validate_segment(start, goal)
            report["segments"].append(result)
            print(
                f"[PATH] {result['stage_id']} length={result['path_length_m']:.2f}m "
                f"poses={result['pose_count']} overrun={result['corridor_overrun_m']:.3f}m "
                f"validated={result['validated']}"
            )
        report["total_extension_m"] = sum(
            segment["path_length_m"] for segment in report["segments"]
        )
        report["validated"] = all(
            segment["validated"] for segment in report["segments"]
        )
    except Exception as exc:
        report["validated"] = False
        report["error"] = str(exc)
        print(f"[PATH_FAIL] {exc}", file=sys.stderr)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n")
    print(rendered)
    return 0 if report.get("validated") else 1


if __name__ == "__main__":
    raise SystemExit(main())
