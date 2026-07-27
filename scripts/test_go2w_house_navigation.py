#!/usr/bin/python3
"""Plan or execute one approved HouseWorld route and write a JSON report."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
except ModuleNotFoundError as runtime_import_error:
    rclpy = None
    GoalStatus = None
    PoseStamped = Any
    ComputePathToPose = None
    NavigateToPose = None
    Odometry = Any
    ActionClient = None
    Node = object
    RUNTIME_IMPORT_ERROR: ModuleNotFoundError | None = runtime_import_error
else:
    RUNTIME_IMPORT_ERROR = None

from cosmos_vln_protocol import RouteStage, load_route_catalog
from go2w_house_mission import (
    bounded_env_float,
    point_in_polygon,
    stage_speed_profile,
    transition_speed_profile,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ROUTES = ROOT_DIR / "config/cosmos_vln_house_routes.json"
DEFAULT_OUTPUT = ROOT_DIR / ".run/go2w_house/navigation_test.json"
DEFAULT_RUNTIME_STATE = ROOT_DIR / ".run/go2w_house/runtime.json"
DEFAULT_NAV_MODE = ROOT_DIR / ".run/go2w_house/nav_mode"


def domain_id(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed <= 232:
        raise argparse.ArgumentTypeError("ROS domain ID must be in [0, 232]")
    return parsed


def validate_drive_runtime(
    requested_domain: int,
    runtime_state: Path = DEFAULT_RUNTIME_STATE,
    nav_mode_file: Path = DEFAULT_NAV_MODE,
) -> list[str]:
    errors: list[str] = []
    try:
        state = json.loads(runtime_state.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"HouseWorld runtime state is unavailable: {exc}"]
    if state.get("scene") != "HouseWorld" or state.get("scene_id") != 6:
        errors.append("active runtime is not HouseWorld scene 6")
    if state.get("ready") is not True:
        errors.append("HouseWorld runtime is not ready")
    if state.get("map_source") != "known_map":
        errors.append(
            "HouseWorld route driving requires --known-map-nav "
            f"(map_source={state.get('map_source')!r})"
        )
    if state.get("ros_domain_id") != requested_domain:
        errors.append(
            f"runtime ROS domain is {state.get('ros_domain_id')}, "
            f"requested {requested_domain}"
        )
    try:
        nav_mode = nav_mode_file.read_text(encoding="utf-8").strip()
    except OSError as exc:
        errors.append(f"HouseWorld navigation mode is unavailable: {exc}")
    else:
        if nav_mode != "avoid":
            errors.append(f"HouseWorld navigation mode must be avoid, got {nav_mode!r}")
    return errors


def pose_stamped(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_length(poses: list[PoseStamped]) -> float:
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(poses, poses[1:])
    )


def quaternion_tilt(message: Odometry) -> tuple[float, float]:
    orientation = message.pose.pose.orientation
    roll = math.atan2(
        2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
        1.0 - 2.0 * (orientation.x**2 + orientation.y**2),
    )
    pitch = math.asin(
        max(
            -1.0,
            min(
                1.0,
                2.0
                * (orientation.w * orientation.y - orientation.z * orientation.x),
            ),
        )
    )
    return roll, pitch


class HouseRouteTest(Node):
    def __init__(
        self, boundary: tuple[tuple[float, float], ...], timeout_sec: float
    ) -> None:
        super().__init__("go2w_house_navigation_test")
        self.boundary = boundary
        self.timeout_sec = timeout_sec
        self.compute_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 20)
        self.last_odom: Odometry | None = None
        self.last_odom_received_at = 0.0
        self.stage_distance = 0.0
        self.stage_last_xy: tuple[float, float] | None = None
        self.stage_min_z = math.inf
        self.stage_max_tilt = 0.0
        self.stage_boundary_violation = False
        self.normal_speed = bounded_env_float(
            "GO2W_HOUSE_NAV_SPEED", 0.32, 0.05, 0.35
        )
        self.door_speed = min(
            self.normal_speed,
            bounded_env_float(
                "GO2W_HOUSE_DOOR_SPEED", 0.12, 0.05, self.normal_speed
            ),
        )
        self.speed_profile = ""

    def on_odom(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_received_at = time.monotonic()
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self.stage_last_xy is not None:
            self.stage_distance += math.hypot(
                current[0] - self.stage_last_xy[0], current[1] - self.stage_last_xy[1]
            )
        self.stage_last_xy = current
        self.stage_min_z = min(self.stage_min_z, position.z)
        roll, pitch = quaternion_tilt(message)
        self.stage_max_tilt = max(self.stage_max_tilt, abs(roll), abs(pitch))
        if not point_in_polygon(position.x, position.y, self.boundary):
            self.stage_boundary_violation = True

    def wait_future(self, future: Any, label: str, timeout_sec: float | None = None) -> Any:
        deadline = time.monotonic() + (timeout_sec or self.timeout_sec)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"timed out waiting for {label}")
        return future.result()

    def wait_for_odom(self) -> None:
        deadline = time.monotonic() + 10.0
        while (
            self.last_odom is None
            or time.monotonic() - self.last_odom_received_at > 1.0
        ) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if (
            self.last_odom is None
            or time.monotonic() - self.last_odom_received_at > 1.0
        ):
            raise RuntimeError("fresh /odom/mujoco_odom is unavailable")

    def validate_drive_start(self, spawn: dict[str, Any]) -> None:
        self.wait_for_odom()
        position = self.last_odom.pose.pose.position
        error = math.hypot(position.x - float(spawn["x"]), position.y - float(spawn["y"]))
        if error > 0.75:
            raise RuntimeError(
                f"robot is {error:.2f} m from the approved House spawn; reset first"
            )
        if not point_in_polygon(position.x, position.y, self.boundary):
            raise RuntimeError("robot start pose is outside the House boundary")

    def set_speed_profile(self, profile: str) -> None:
        transition = transition_speed_profile(
            self.speed_profile, profile, self.normal_speed, self.door_speed
        )
        self.speed_profile = transition.applied_profile
        if not transition.success:
            raise RuntimeError(
                f"cannot apply {profile} speed profile: {transition.detail}; "
                f"rollback={transition.rollback_succeeded}"
            )

    def plan_segment(
        self,
        start: tuple[str, float, float, float],
        stage: RouteStage,
    ) -> dict[str, Any]:
        request = ComputePathToPose.Goal()
        request.start = pose_stamped(self, start[1], start[2], start[3])
        request.goal = pose_stamped(self, stage.x, stage.y, stage.yaw_rad)
        request.planner_id = "GridBased"
        request.use_start = True
        handle = self.wait_future(
            self.compute_client.send_goal_async(request),
            f"plan acceptance for {stage.stage_id}",
        )
        if handle is None or not handle.accepted:
            raise RuntimeError(f"planner rejected stage {stage.stage_id}")
        wrapped = self.wait_future(
            handle.get_result_async(), f"plan result for {stage.stage_id}"
        )
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"planner failed stage {stage.stage_id} with status {wrapped.status}"
            )
        poses = list(wrapped.result.path.poses)
        if len(poses) < 2:
            raise RuntimeError(f"planner returned an empty path for {stage.stage_id}")
        outside = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in poses
            if not point_in_polygon(
                pose.pose.position.x, pose.pose.position.y, self.boundary
            )
        ]
        endpoint = poses[-1].pose.position
        endpoint_error = math.hypot(endpoint.x - stage.x, endpoint.y - stage.y)
        result = {
            "stage_id": stage.stage_id,
            "start": {"x": start[1], "y": start[2], "yaw_rad": start[3]},
            "goal": {"x": stage.x, "y": stage.y, "yaw_rad": stage.yaw_rad},
            "path_length_m": path_length(poses),
            "pose_count": len(poses),
            "endpoint_error_m": endpoint_error,
            "outside_house_pose_count": len(outside),
            "planning_time_sec": (
                wrapped.result.planning_time.sec
                + wrapped.result.planning_time.nanosec / 1_000_000_000.0
            ),
        }
        result["validated"] = endpoint_error <= 0.55 and not outside
        return result

    def reset_stage_metrics(self) -> None:
        self.stage_distance = 0.0
        self.stage_last_xy = None
        self.stage_min_z = math.inf
        self.stage_max_tilt = 0.0
        self.stage_boundary_violation = False

    def drive_stage(self, stage: RouteStage) -> dict[str, Any]:
        self.reset_stage_metrics()
        self.set_speed_profile(stage_speed_profile(stage.stage_id))
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped(self, stage.x, stage.y, stage.yaw_rad)
        handle = self.wait_future(
            self.navigate_client.send_goal_async(goal),
            f"navigation acceptance for {stage.stage_id}",
            10.0,
        )
        if handle is None or not handle.accepted:
            raise RuntimeError(f"navigation rejected stage {stage.stage_id}")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.timeout_sec
        safety_reason = ""
        while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - self.last_odom_received_at > 1.0:
                safety_reason = "odom_lost"
                break
            if self.stage_boundary_violation:
                safety_reason = "house_boundary_violation"
                break
            if self.stage_min_z < 0.25 or self.stage_max_tilt > math.radians(40.0):
                safety_reason = "robot_attitude_unsafe"
                break
        timed_out = not result_future.done() and not safety_reason
        cancel_confirmed = True
        if (timed_out or safety_reason) and not result_future.done():
            cancel_future = handle.cancel_goal_async()
            cancel_response = self.wait_future(
                cancel_future, f"navigation cancellation for {stage.stage_id}", 5.0
            )
            cancel_confirmed = bool(
                cancel_response is not None and cancel_response.goals_canceling
            )
            try:
                self.wait_future(
                    result_future,
                    f"cancelled navigation result for {stage.stage_id}",
                    5.0,
                )
            except TimeoutError:
                cancel_confirmed = False
        wrapped = None
        if result_future.done():
            wrapped = result_future.result()
        status = wrapped.status if wrapped is not None else None
        position = self.last_odom.pose.pose.position if self.last_odom else None
        final_error = (
            math.hypot(position.x - stage.x, position.y - stage.y)
            if position is not None
            else math.inf
        )
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not timed_out
            and not safety_reason
            and cancel_confirmed
            and final_error <= 0.35
        )
        return {
            "stage_id": stage.stage_id,
            "action_status": status,
            "succeeded": succeeded,
            "timed_out": timed_out,
            "safety_reason": safety_reason,
            "cancel_confirmed": cancel_confirmed,
            "distance_traveled_m": self.stage_distance,
            "final_error_m": final_error,
            "minimum_base_height_m": self.stage_min_z,
            "maximum_tilt_deg": math.degrees(self.stage_max_tilt),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", default="house_north_lounge")
    parser.add_argument("--routes-file", type=Path, default=DEFAULT_ROUTES)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--drive", dest="drive", action="store_true")
    mode.add_argument("--dry-run", dest="drive", action="store_false")
    parser.set_defaults(drive=False)
    parser.add_argument("--domain-id", type=domain_id, default=90)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        payload = json.loads(args.routes_file.read_text(encoding="utf-8"))
        boundary = tuple(
            (float(point[0]), float(point[1]))
            for point in payload["scene"]["bounds_polygon"]
        )
        spawn = payload["spawn"]
        routes = load_route_catalog(args.routes_file)
        route = routes[args.route]
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        print(f"[HOUSE_TEST_FAIL] cannot load route: {exc}", file=sys.stderr)
        return 2

    report: dict[str, Any] = {
        "scene": "HouseWorld",
        "route_id": route.route_id,
        "drives_robot": bool(args.drive),
        "mode": "drive" if args.drive else "dry-run",
        "ros_domain_id": args.domain_id,
        "plans": [],
        "executions": [],
    }
    if args.drive:
        runtime_errors = validate_drive_runtime(args.domain_id)
        if runtime_errors:
            report["validated"] = False
            report["error"] = "; ".join(runtime_errors)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            print(f"[HOUSE_TEST_FAIL] {report['error']}", file=sys.stderr)
            return 1
    if rclpy is None:
        print(f"[HOUSE_TEST_FAIL] ROS runtime is unavailable: {RUNTIME_IMPORT_ERROR}", file=sys.stderr)
        return 2
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    rclpy.init()
    node = HouseRouteTest(boundary, max(10.0, args.timeout_sec))
    success = False
    try:
        if not node.compute_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/compute_path_to_pose is unavailable")
        start = (
            "spawn",
            float(spawn["x"]),
            float(spawn["y"]),
            float(spawn["yaw_rad"]),
        )
        for stage in route.stages:
            result = node.plan_segment(start, stage)
            report["plans"].append(result)
            print(
                f"[HOUSE_PATH] {stage.stage_id} "
                f"length={result['path_length_m']:.2f}m "
                f"poses={result['pose_count']} validated={result['validated']}"
            )
            if not result["validated"]:
                raise RuntimeError(f"path safety failed for {stage.stage_id}")
            start = (stage.stage_id, stage.x, stage.y, stage.yaw_rad)

        if args.drive:
            if not node.navigate_client.wait_for_server(timeout_sec=20.0):
                raise RuntimeError("/navigate_to_pose is unavailable")
            node.validate_drive_start(spawn)
            for stage in route.stages:
                result = node.drive_stage(stage)
                report["executions"].append(result)
                print(
                    f"[HOUSE_DRIVE] {stage.stage_id} "
                    f"distance={result['distance_traveled_m']:.2f}m "
                    f"error={result['final_error_m']:.2f}m "
                    f"succeeded={result['succeeded']}"
                )
                if not result["succeeded"]:
                    raise RuntimeError(f"navigation failed at {stage.stage_id}")
        report["total_planned_length_m"] = sum(
            item["path_length_m"] for item in report["plans"]
        )
        report["validated"] = True
        success = True
    except Exception as exc:
        report["validated"] = False
        report["error"] = str(exc)
        print(f"[HOUSE_TEST_FAIL] {exc}", file=sys.stderr)
    finally:
        if args.drive and node.speed_profile not in {"", "normal"}:
            try:
                node.set_speed_profile("normal")
                report["normal_speed_restored"] = True
            except Exception as exc:
                report["normal_speed_restored"] = False
                report.setdefault("cleanup_errors", []).append(str(exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
