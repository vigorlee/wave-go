#!/usr/bin/python3
"""Coordinate FAR-NE Nav2 handoff, visual docking, and safe realignment."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
from typing import Any

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import Empty, String
except ModuleNotFoundError as runtime_import_error:
    rclpy = None
    GoalStatus = Any
    PoseStamped = Any
    NavigateToPose = Any
    Odometry = Any
    ActionClient = Any
    Node = object
    Empty = String = Any
    RUNTIME_IMPORT_ERROR: ModuleNotFoundError | None = runtime_import_error
else:
    RUNTIME_IMPORT_ERROR = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_RUNTIME_STATE = ROOT_DIR / ".run/go2w_house/runtime.json"
DEFAULT_VISUALIZATION = (
    ROOT_DIR / ".run/go2w_house/cosmos/latest_visualization.jpg"
)
DEFAULT_OUTPUT_DIR = ROOT_DIR / ".run/go2w_house/far_ne_hybrid"
DEFAULT_TASK = (
    "Search maplessly for the QR-marked robot charging dock in the unknown "
    "environment. Build the map online only for visualization, keep clear of "
    "walls, use NWM-Cosmos3Edge to verify ID 560, approach until RGB-D confirms "
    "the dock is within 0.40 meters, stop completely at the dock, and only then "
    "crouch to charge."
)

TERMINAL_STATES = {"succeeded", "blocked", "failed", "canceled"}
SUCCESS_CHAIN = (
    "target_confirmed",
    "close_marker_confirmation",
    "arrived_stopped",
    "charging",
    "succeeded",
)


@dataclass(frozen=True)
class NavGoal:
    name: str
    x: float
    y: float
    yaw_deg: float

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


INITIAL_HANDOFF = NavGoal("initial_handoff", -0.10, 7.50, 140.0)
REALIGNMENT = (
    NavGoal("realign_departure", 1.00, 6.60, 135.0),
    NavGoal("realign_handoff", -0.10, 7.50, 140.0),
)


def domain_id(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed <= 232:
        raise argparse.ArgumentTypeError("ROS domain ID must be in [0, 232]")
    return parsed


def positive_seconds(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be a positive finite number")
    return parsed


def validate_runtime(path: Path, requested_domain: int) -> list[str]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"runtime state unavailable: {exc}"]
    errors: list[str] = []
    if payload.get("scene") != "HouseWorld" or payload.get("scene_id") != 6:
        errors.append("active runtime is not HouseWorld scene 6")
    if payload.get("ready") is not True:
        errors.append("HouseWorld runtime is not ready")
    if payload.get("map_source") != "known_map":
        errors.append("FAR-NE hybrid execution requires --known-map-nav")
    if payload.get("ros_domain_id") != requested_domain:
        errors.append(
            f"runtime domain {payload.get('ros_domain_id')!r} does not match "
            f"requested domain {requested_domain}"
        )
    processes = payload.get("processes")
    if not isinstance(processes, dict):
        errors.append("runtime process inventory is missing")
    else:
        for name in ("sim", "navigation", "charger_search"):
            entry = processes.get(name)
            if not isinstance(entry, dict) or entry.get("running") is not True:
                errors.append(f"required process {name!r} is not running")
    return errors


def quaternion_yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
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


def pose_payload(message: Odometry | None) -> dict[str, float] | None:
    if message is None:
        return None
    position = message.pose.pose.position
    twist = message.twist.twist
    roll, pitch = quaternion_tilt(message)
    return {
        "x": float(position.x),
        "y": float(position.y),
        "z": float(position.z),
        "yaw_deg": math.degrees(quaternion_yaw(message)),
        "roll_rad": roll,
        "pitch_rad": pitch,
        "linear_speed_mps": math.sqrt(
            twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2
        ),
        "yaw_rate_rps": float(twist.angular.z),
    }


def make_pose(node: Node, goal: NavGoal) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    pose.pose.orientation.z = math.sin(goal.yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(goal.yaw_rad / 2.0)
    return pose


class FarNeHybridCoordinator(Node):
    def __init__(
        self,
        *,
        navigation_timeout_sec: float,
        task_timeout_sec: float,
        output_dir: Path,
        visualization: Path,
    ) -> None:
        super().__init__("go2w_house_far_ne_hybrid")
        self.navigation_timeout_sec = navigation_timeout_sec
        self.task_timeout_sec = task_timeout_sec
        self.output_dir = output_dir
        self.visualization = visualization
        self.navigate_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.task_publisher = self.create_publisher(
            String, "/cosmos_vln/charger_search", 10
        )
        self.cancel_publisher = self.create_publisher(
            Empty, "/cosmos_vln/cancel", 10
        )
        self.create_subscription(
            Odometry, "/odom/mujoco_odom", self.on_odom, 20
        )
        self.create_subscription(
            String,
            "/cosmos_vln/charger_search_status",
            self.on_search_status,
            20,
        )
        self.last_odom: Odometry | None = None
        self.last_odom_at = 0.0
        self.status_sequence = 0
        self.status_events: list[dict[str, Any]] = []

    def on_odom(self, message: Odometry) -> None:
        self.last_odom = message
        self.last_odom_at = time.monotonic()

    def on_search_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"state": "invalid_status_json", "raw": message.data}
        if not isinstance(payload, dict):
            payload = {"state": "invalid_status_type", "raw": message.data}
        self.status_sequence += 1
        self.status_events.append(
            {
                "sequence": self.status_sequence,
                "monotonic_sec": time.monotonic(),
                "wall_time": datetime.now(timezone.utc).isoformat(),
                "payload": payload,
            }
        )

    def spin_until(
        self, predicate: Any, timeout_sec: float, label: str
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not predicate():
            raise TimeoutError(f"timed out waiting for {label}")

    def wait_for_odom(self, timeout_sec: float = 10.0) -> None:
        self.spin_until(
            lambda: (
                self.last_odom is not None
                and time.monotonic() - self.last_odom_at <= 1.0
            ),
            timeout_sec,
            "fresh odometry",
        )

    def validate_pose_safety(self) -> None:
        self.wait_for_odom()
        assert self.last_odom is not None
        pose = pose_payload(self.last_odom)
        assert pose is not None
        if pose["z"] < 0.35:
            raise RuntimeError(f"unsafe base height: {pose['z']:.3f} m")
        if max(abs(pose["roll_rad"]), abs(pose["pitch_rad"])) > 0.30:
            raise RuntimeError("robot tilt exceeds 0.30 rad")

    def wait_stopped(
        self, *, hold_sec: float = 1.0, timeout_sec: float = 10.0
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        stable_since: float | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.last_odom is None
                or time.monotonic() - self.last_odom_at > 1.0
            ):
                stable_since = None
                continue
            twist = self.last_odom.twist.twist
            linear = math.sqrt(
                twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2
            )
            if linear <= 0.03 and abs(twist.angular.z) <= 0.06:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= hold_sec:
                    self.validate_pose_safety()
                    return
            else:
                stable_since = None
        raise TimeoutError("robot did not reach a stable stop")

    def cancel_visual_task(self) -> None:
        self.cancel_publisher.publish(Empty())
        self.wait_stopped()

    def drive(self, goal: NavGoal) -> dict[str, Any]:
        self.cancel_visual_task()
        self.wait_for_odom()
        assert self.last_odom is not None
        start = pose_payload(self.last_odom)
        started = time.monotonic()
        request = NavigateToPose.Goal()
        request.pose = make_pose(self, goal)
        acceptance = self.navigate_client.send_goal_async(request)
        self.spin_until(acceptance.done, 10.0, f"{goal.name} acceptance")
        handle = acceptance.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"Nav2 rejected {goal.name}")
        result_future = handle.get_result_async()
        timed_out = False
        odom_lost = False
        deadline = started + self.navigation_timeout_sec
        while (
            rclpy.ok()
            and not result_future.done()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - self.last_odom_at > 2.0:
                odom_lost = True
                break
        if not result_future.done() and not odom_lost:
            timed_out = True
        cancel_confirmed = True
        if not result_future.done():
            cancellation = handle.cancel_goal_async()
            self.spin_until(
                cancellation.done, 5.0, f"{goal.name} cancellation"
            )
            response = cancellation.result()
            cancel_confirmed = bool(
                response is not None and response.goals_canceling
            )
            try:
                self.spin_until(
                    result_future.done, 5.0, f"{goal.name} canceled result"
                )
            except TimeoutError:
                cancel_confirmed = False
        wrapped = result_future.result() if result_future.done() else None
        status = wrapped.status if wrapped is not None else None
        self.wait_stopped()
        assert self.last_odom is not None
        end = pose_payload(self.last_odom)
        assert end is not None
        final_error = math.hypot(end["x"] - goal.x, end["y"] - goal.y)
        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not timed_out
            and not odom_lost
            and cancel_confirmed
            and final_error <= 0.40
        )
        result = {
            "name": goal.name,
            "goal": {
                "x": goal.x,
                "y": goal.y,
                "yaw_deg": goal.yaw_deg,
            },
            "start_pose": start,
            "end_pose": end,
            "action_status": status,
            "duration_sec": time.monotonic() - started,
            "final_position_error_m": final_error,
            "timed_out": timed_out,
            "odom_lost": odom_lost,
            "cancel_confirmed": cancel_confirmed,
            "succeeded": succeeded,
        }
        if not succeeded:
            raise RuntimeError(f"navigation failed: {json.dumps(result)}")
        self.capture(goal.name)
        return result

    def wait_for_task_subscriber(self) -> None:
        self.spin_until(
            lambda: self.task_publisher.get_subscription_count() > 0,
            10.0,
            "charger-search subscriber",
        )

    def run_visual_task(self, task: str, attempt: int) -> dict[str, Any]:
        self.wait_stopped()
        self.wait_for_task_subscriber()
        first_sequence = self.status_sequence
        started = time.monotonic()
        message = String()
        message.data = task
        self.task_publisher.publish(message)
        deadline = started + self.task_timeout_sec
        terminal: dict[str, Any] | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - self.last_odom_at > 2.0:
                raise RuntimeError("odometry lost while visual task was active")
            for event in self.status_events:
                if event["sequence"] <= first_sequence:
                    continue
                payload = event["payload"]
                if payload.get("state") in TERMINAL_STATES:
                    terminal = payload
                    break
            if terminal is not None:
                break
        if terminal is None:
            self.cancel_visual_task()
            terminal = {"state": "failed", "reason": "task_timeout"}
        events = [
            event
            for event in self.status_events
            if event["sequence"] > first_sequence
        ]
        states = [
            str(event["payload"].get("state", ""))
            for event in events
        ]
        close_confirmations = sum(
            state == "close_marker_confirmation" for state in states
        )
        success_chain_complete = (
            terminal.get("state") == "succeeded"
            and close_confirmations >= 3
            and all(state in states for state in SUCCESS_CHAIN)
        )
        result = {
            "attempt": attempt,
            "duration_sec": time.monotonic() - started,
            "terminal": terminal,
            "states": states,
            "close_confirmation_count": close_confirmations,
            "success_chain_complete": success_chain_complete,
            "final_pose": pose_payload(self.last_odom),
            "events": events,
        }
        self.capture(
            f"visual_{attempt}_{terminal.get('state', 'unknown')}"
        )
        return result

    def capture(self, label: str) -> None:
        if not self.visualization.is_file():
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(
            self.visualization,
            self.output_dir / f"{label}_visualization.jpg",
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain-id", type=domain_id, default=90)
    parser.add_argument(
        "--runtime-state", type=Path, default=DEFAULT_RUNTIME_STATE
    )
    parser.add_argument(
        "--visualization", type=Path, default=DEFAULT_VISUALIZATION
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument(
        "--navigation-timeout-sec", type=positive_seconds, default=420.0
    )
    parser.add_argument(
        "--task-timeout-sec", type=positive_seconds, default=300.0
    )
    parser.add_argument("--max-realignments", type=int, default=2)
    return parser.parse_args()


def write_report(output_dir: Path, report: dict[str, Any]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "run_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    if args.max_realignments < 0 or args.max_realignments > 5:
        print("[HYBRID_FAIL] --max-realignments must be in [0, 5]", file=sys.stderr)
        return 2
    runtime_errors = validate_runtime(args.runtime_state, args.domain_id)
    if runtime_errors:
        print(f"[HYBRID_FAIL] {'; '.join(runtime_errors)}", file=sys.stderr)
        return 1
    if rclpy is None:
        print(
            f"[HYBRID_FAIL] ROS runtime unavailable: {RUNTIME_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2

    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    report: dict[str, Any] = {
        "method": "map-based hybrid navigation",
        "scene": "HouseWorld",
        "condition": "FAR-NE",
        "requested_spawn": {"x": 11.20, "y": 6.80, "yaw_deg": -90.0},
        "started_at": datetime.now(timezone.utc).isoformat(),
        "navigation": [],
        "visual_tasks": [],
        "max_realignments": args.max_realignments,
        "success": False,
    }
    rclpy.init()
    node = FarNeHybridCoordinator(
        navigation_timeout_sec=args.navigation_timeout_sec,
        task_timeout_sec=args.task_timeout_sec,
        output_dir=args.output_dir,
        visualization=args.visualization,
    )
    exit_code = 1
    try:
        if not node.navigate_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("/navigate_to_pose is unavailable")
        node.wait_for_odom()
        node.validate_pose_safety()
        report["actual_start_pose"] = pose_payload(node.last_odom)
        report["navigation"].append(node.drive(INITIAL_HANDOFF))

        for visual_attempt in range(1, args.max_realignments + 2):
            visual = node.run_visual_task(args.task, visual_attempt)
            report["visual_tasks"].append(visual)
            if visual["success_chain_complete"]:
                report["success"] = True
                exit_code = 0
                break
            terminal = visual["terminal"]
            recoverable = (
                terminal.get("state") == "blocked"
                and terminal.get("reason") == "charging_marker_lost"
            )
            if not recoverable or visual_attempt > args.max_realignments:
                break
            for goal in REALIGNMENT:
                report["navigation"].append(node.drive(goal))
        if not report["success"]:
            terminal = (
                report["visual_tasks"][-1]["terminal"]
                if report["visual_tasks"]
                else {"state": "failed", "reason": "visual_task_not_started"}
            )
            report["failure"] = terminal
    except Exception as exc:
        report["failure"] = {
            "state": "failed",
            "reason": str(exc),
        }
        print(f"[HYBRID_FAIL] {exc}", file=sys.stderr)
    finally:
        report["finished_at"] = datetime.now(timezone.utc).isoformat()
        report["final_pose"] = pose_payload(node.last_odom)
        write_report(args.output_dir, report)
        try:
            if not report["success"]:
                node.cancel_visual_task()
        except Exception as exc:
            report["cleanup_error"] = str(exc)
            write_report(args.output_dir, report)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if report["success"]:
        print(
            "[HYBRID_OK] complete FAR-NE navigation, visual docking, stop, "
            "and crouch evidence chain"
        )
    else:
        print(
            f"[HYBRID_FAIL] {json.dumps(report.get('failure', {}))}",
            file=sys.stderr,
        )
    print(f"[HYBRID_REPORT] {args.output_dir / 'run_report.json'}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
