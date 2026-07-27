#!/usr/bin/python3
"""Execute high-level NWM-Cosmos3Edge missions through a fail-closed Nav2 supervisor."""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import subprocess
import time
from typing import Any
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String

from cosmos_vln_protocol import (
    ACTION_COMPLETE,
    ACTION_HOLD,
    ACTION_NAVIGATE,
    RouteDefinition,
    load_route_catalog,
    validate_required_route,
    validate_route_command,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
STAIR_UP_PROFILE = "stair_up"
STAIR_DOWN_PROFILE = "stair_down"
GENERIC_PROFILE = "generic"


def env_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except ValueError:
        return default
    return value if math.isfinite(value) else default


def env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except ValueError:
        return default


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    received_at: float


@dataclass(frozen=True)
class MissionStage:
    """Normalized, catalog-owned route stage used by the runtime supervisor."""

    stage_id: str
    profile: str
    navigation_mode: str
    frame_id: str
    x: float
    y: float
    yaw_rad: float


class CosmosVlnMission(Node):
    def __init__(self) -> None:
        super().__init__("cosmos_vln_mission")
        self.instruction_publisher = self.create_publisher(
            String, "/cosmos_vln/instruction", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/cosmos_vln/mission_status", 10
        )
        self.target_publisher = self.create_publisher(
            PoseStamped, "/cosmos_vln/target_pose", 10
        )
        self.create_subscription(String, "/cosmos_vln/mission", self.on_mission, 10)
        self.create_subscription(Empty, "/cosmos_vln/mission_cancel", self.on_cancel, 10)
        self.create_subscription(
            String, "/cosmos_vln/route_command", self.on_route_command, 10
        )
        self.create_subscription(String, "/cosmos_vln/status", self.on_planner_status, 10)
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 20)
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.routes_file = Path(
            os.environ.get(
                "COSMOS_VLN_ROUTES_FILE",
                str(ROOT_DIR / "config/cosmos_vln_routes.json"),
            )
        )
        self.routes = load_route_catalog(self.routes_file)
        self.required_route_id = os.environ.get(
            "COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID", ""
        ).strip()
        if self.required_route_id and self.required_route_id not in self.routes:
            raise RuntimeError(
                "COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID is not in the route catalog: "
                f"{self.required_route_id}"
            )

        self.min_confidence = min(
            1.0, max(0.0, env_float("COSMOS_VLN_MISSION_MIN_CONFIDENCE", 0.35))
        )
        self.max_plan_failures = max(
            0, env_int("COSMOS_VLN_MISSION_MAX_PLAN_FAILURES", 2)
        )
        self.timeout_sec = max(
            30.0, env_float("COSMOS_VLN_MISSION_TIMEOUT_SEC", 480.0)
        )
        self.settle_sec = max(
            0.0, env_float("COSMOS_VLN_MISSION_SETTLE_SEC", 2.0)
        )
        self.odom_max_age_sec = max(
            0.2, env_float("COSMOS_VLN_ODOM_MAX_AGE_SEC", 1.0)
        )
        self.stair_start_max_lateral = max(
            0.05, env_float("COSMOS_VLN_STAIR_START_MAX_LATERAL", 0.35)
        )
        self.stair_start_min_y = env_float("COSMOS_VLN_STAIR_START_MIN_Y", -0.30)
        self.stair_start_max_y = env_float("COSMOS_VLN_STAIR_START_MAX_Y", 0.45)
        self.stair_start_max_heading = max(
            0.05, env_float("COSMOS_VLN_STAIR_START_MAX_HEADING", 0.25)
        )
        self.stair_max_lateral = max(
            self.stair_start_max_lateral,
            env_float("COSMOS_VLN_STAIR_MAX_LATERAL", 0.65),
        )
        self.stair_max_heading = max(
            self.stair_start_max_heading,
            env_float("COSMOS_VLN_STAIR_MAX_HEADING", 0.55),
        )
        self.stair_max_roll = max(
            0.2, env_float("COSMOS_VLN_STAIR_MAX_ROLL", 0.60)
        )
        self.stair_max_pitch = max(
            0.2, env_float("COSMOS_VLN_STAIR_MAX_PITCH", 0.75)
        )
        self.stair_no_progress_sec = max(
            2.0, env_float("COSMOS_VLN_STAIR_NO_PROGRESS_SEC", 10.0)
        )
        self.stair_platform_min_y = env_float(
            "COSMOS_VLN_STAIR_PLATFORM_MIN_Y", 4.20
        )
        self.stair_platform_min_z = env_float(
            "COSMOS_VLN_STAIR_PLATFORM_MIN_Z", 1.40
        )
        self.stair_platform_settle_sec = max(
            0.2, env_float("COSMOS_VLN_STAIR_PLATFORM_SETTLE_SEC", 1.0)
        )
        self.stair_platform_timeout_sec = max(
            self.stair_platform_settle_sec,
            env_float("COSMOS_VLN_STAIR_PLATFORM_TIMEOUT_SEC", 15.0),
        )
        self.stair_down_platform_min_y = env_float(
            "COSMOS_VLN_STAIR_DOWN_PLATFORM_MIN_Y", 9.60
        )
        self.stair_down_platform_max_z = env_float(
            "COSMOS_VLN_STAIR_DOWN_PLATFORM_MAX_Z", 0.80
        )
        self.stair_down_start_min_y = env_float(
            "COSMOS_VLN_STAIR_DOWN_START_MIN_Y", 4.20
        )
        self.stair_down_start_min_z = env_float(
            "COSMOS_VLN_STAIR_DOWN_START_MIN_Z", 1.40
        )
        self.stair_down_start_max_lateral = max(
            self.stair_start_max_lateral,
            env_float("COSMOS_VLN_STAIR_DOWN_START_MAX_LATERAL", 0.55),
        )
        self.stair_down_start_max_heading = max(
            self.stair_start_max_heading,
            env_float("COSMOS_VLN_STAIR_DOWN_START_MAX_HEADING", 0.35),
        )
        self.route_max_lateral = max(
            0.25, env_float("COSMOS_VLN_ROUTE_MAX_LATERAL", 1.50)
        )
        self.route_max_heading = max(
            0.35, env_float("COSMOS_VLN_ROUTE_MAX_HEADING", 1.40)
        )
        self.route_max_roll = max(
            0.20, env_float("COSMOS_VLN_ROUTE_MAX_ROLL", 0.60)
        )
        self.route_max_pitch = max(
            0.20, env_float("COSMOS_VLN_ROUTE_MAX_PITCH", 0.75)
        )
        self.route_no_progress_sec = max(
            2.0, env_float("COSMOS_VLN_ROUTE_NO_PROGRESS_SEC", 20.0)
        )
        self.route_progress_min_displacement = max(
            0.02,
            env_float("COSMOS_VLN_ROUTE_PROGRESS_MIN_DISPLACEMENT", 0.10),
        )
        self.nav_mode_script = Path(
            os.environ.get(
                "COSMOS_VLN_NAV_MODE_SCRIPT",
                str(ROOT_DIR / "scripts/set_go2w_nav_mode.sh"),
            )
        )
        self.nav_mode_file = Path(
            os.environ.get(
                "GO2W_NAV_MODE_FILE", str(ROOT_DIR / ".run/go2w/nav_mode")
            )
        )

        self.mission = ""
        self.mission_id = ""
        self.profile = "pending"
        self.active_route: RouteDefinition | None = None
        self.active_stages: tuple[MissionStage, ...] = ()
        self.stage_index = -1
        self.stage_start_pose: RobotPose | None = None
        self.accepted_plan: dict[str, Any] = {}
        self.accepted_confidence = 0.0
        self.pending_instruction = ""
        self.started_at = 0.0
        self.step = 0
        self.plan_failures = 0
        self.planning = False
        self.navigation_active = False
        self.awaiting_platform_validation = False
        self.next_plan_at: float | None = None
        self.goal_handle: Any = None
        self.recovery_hint = ""
        self.robot_pose: RobotPose | None = None
        self.stair_mode_active = False
        self.route_progress_y = 0.0
        self.stage_best_distance = math.inf
        self.route_progress_pose: RobotPose | None = None
        self.route_progress_at = 0.0
        self.platform_stable_since = 0.0
        self.platform_validation_started = 0.0
        self.goal_request_future: Any = None
        self.mode_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="nav-mode")
        self.mode_switch_future: Future[bool] | None = None
        self.mode_switch_mission_id = ""
        self.mode_switch_stage_index = -1
        self.mode_switch_mode = "avoid"
        self.mode_switch_plan: dict[str, Any] = {}
        self.mode_switch_confidence = 0.0
        self.next_avoid_restore_at = 0.0
        self.current_nav_mode = "avoid"
        try:
            mode = self.nav_mode_file.read_text().strip()
            if mode in {"avoid", "up", "down", "flat"}:
                self.current_nav_mode = mode
            self.stair_mode_active = self.current_nav_mode != "avoid"
        except OSError:
            pass
        self.timer = self.create_timer(0.2, self.on_timer)
        self.get_logger().info(
            "NWM-Cosmos3Edge mission ready: input=/cosmos_vln/mission "
            f"routes={','.join(sorted(self.routes))} "
            f"min_confidence={self.min_confidence:.2f} "
            f"required_route={self.required_route_id or 'none'}"
        )

    @property
    def active(self) -> bool:
        return bool(self.mission)

    @property
    def current_stage(self) -> MissionStage | None:
        if 0 <= self.stage_index < len(self.active_stages):
            return self.active_stages[self.stage_index]
        return None

    def route_stages(self, route: RouteDefinition) -> tuple[MissionStage, ...]:
        raw_stages = tuple(getattr(route, "stages", ()) or ())
        if not raw_stages:
            raw_stages = (route,)

        stages: list[MissionStage] = []
        for index, stage in enumerate(raw_stages):
            stage_id = str(
                getattr(stage, "stage_id", "")
                or (route.route_id if len(raw_stages) == 1 else f"stage_{index + 1}")
            ).strip()
            profile = str(getattr(stage, "profile", "")).strip()
            navigation_mode = str(getattr(stage, "navigation_mode", "")).strip()
            frame_id = str(getattr(stage, "frame_id", "")).strip()
            if not stage_id or not frame_id:
                raise ValueError(f"route stage {index} is missing an identifier or frame")
            valid_mode = (
                (profile == STAIR_UP_PROFILE and navigation_mode == "up")
                or (profile == STAIR_DOWN_PROFILE and navigation_mode == "down")
                or (profile == GENERIC_PROFILE and navigation_mode in {"avoid", "flat"})
            )
            if not valid_mode:
                raise ValueError(
                    f"route stage {stage_id} has unsupported profile/mode "
                    f"{profile}/{navigation_mode}"
                )
            values = (
                float(getattr(stage, "x")),
                float(getattr(stage, "y")),
                float(getattr(stage, "yaw_rad")),
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"route stage {stage_id} has a non-finite target")
            stages.append(
                MissionStage(
                    stage_id=stage_id,
                    profile=profile,
                    navigation_mode=navigation_mode,
                    frame_id=frame_id,
                    x=values[0],
                    y=values[1],
                    yaw_rad=values[2],
                )
            )
        return tuple(stages)

    def publish_status(self, state: str, **details: Any) -> None:
        stage = self.current_stage
        payload = {
            "state": state,
            "time": time.time(),
            "mission_id": self.mission_id,
            "mission": self.mission,
            "profile": self.profile,
            "step": self.step,
            "stage_id": stage.stage_id if stage is not None else "",
            "stage_index": self.stage_index,
            "stage_count": len(self.active_stages),
            **details,
        }
        self.status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.robot_pose = RobotPose(
            x=position.x,
            y=position.y,
            z=position.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            received_at=time.monotonic(),
        )

    def pose_details(self) -> dict[str, float]:
        pose = self.robot_pose
        if pose is None:
            return {}
        return {
            "x": pose.x,
            "y": pose.y,
            "z": pose.z,
            "roll": pose.roll,
            "pitch": pose.pitch,
            "yaw": pose.yaw,
        }

    def stage_start_is_safe(self, stage: MissionStage) -> tuple[bool, str]:
        pose = self.robot_pose
        if pose is None:
            return False, "stage_start_odom_unavailable"
        if time.monotonic() - pose.received_at > self.odom_max_age_sec:
            return False, "stage_start_odom_stale"
        if abs(pose.roll) > 0.30 or abs(pose.pitch) > 0.30 or pose.z < 0.18:
            return False, "stage_start_attitude_unsafe"

        if stage.profile == STAIR_UP_PROFILE:
            if abs(wrap_angle(pose.yaw - stage.yaw_rad)) > self.stair_start_max_heading:
                return False, "stage_start_heading_misaligned"
        elif stage.profile == STAIR_DOWN_PROFILE:
            if (
                abs(wrap_angle(pose.yaw - stage.yaw_rad))
                > self.stair_down_start_max_heading
            ):
                return False, "stage_start_heading_misaligned"

        if stage.profile == STAIR_UP_PROFILE:
            if abs(pose.x - stage.x) > self.stair_start_max_lateral:
                return False, "stair_start_outside_centerline"
            if not self.stair_start_min_y <= pose.y <= self.stair_start_max_y:
                return False, "stair_start_outside_entry"
        elif stage.profile == STAIR_DOWN_PROFILE:
            if abs(pose.x - stage.x) > self.stair_down_start_max_lateral:
                return False, "stair_down_start_outside_centerline"
            if pose.y < self.stair_down_start_min_y or pose.z < self.stair_down_start_min_z:
                return False, "stair_down_start_outside_upper_platform"
        return True, ""

    def stage_lateral_error(self, stage: MissionStage, pose: RobotPose) -> float:
        start = self.stage_start_pose
        if start is None:
            return math.inf
        delta_x = stage.x - start.x
        delta_y = stage.y - start.y
        length = math.hypot(delta_x, delta_y)
        if length < 0.05:
            return math.hypot(pose.x - stage.x, pose.y - stage.y)
        return ((pose.x - start.x) * delta_y - (pose.y - start.y) * delta_x) / length

    def generic_stage_corridor_overrun(
        self, stage: MissionStage, pose: RobotPose
    ) -> float:
        """Return distance outside the bounded generic-stage route envelope."""
        start = self.stage_start_pose
        if start is None:
            return math.inf
        margin = self.route_max_lateral
        min_x = min(start.x, stage.x) - margin
        max_x = max(start.x, stage.x) + margin
        min_y = min(start.y, stage.y) - margin
        max_y = max(start.y, stage.y) + margin
        outside_x = max(min_x - pose.x, 0.0, pose.x - max_x)
        outside_y = max(min_y - pose.y, 0.0, pose.y - max_y)
        return math.hypot(outside_x, outside_y)

    def on_mission(self, message: String) -> None:
        mission = message.data.strip()
        if not mission:
            self.publish_status("rejected", reason="empty_mission")
            return
        if self.active:
            self.publish_status("rejected", reason="mission_already_active")
            return
        if self.mode_switch_future is not None:
            self.publish_status("rejected", reason="navigation_mode_switch_in_progress")
            return
        if self.stair_mode_active:
            self.publish_status("rejected", reason="stair_mode_not_restored")
            return

        self.mission = mission
        self.mission_id = uuid.uuid4().hex
        self.profile = "pending"
        self.active_route = None
        self.active_stages = ()
        self.stage_index = -1
        self.stage_start_pose = None
        self.accepted_plan = {}
        self.accepted_confidence = 0.0
        self.pending_instruction = ""
        self.started_at = time.monotonic()
        self.step = 0
        self.plan_failures = 0
        self.planning = False
        self.navigation_active = False
        self.awaiting_platform_validation = False
        self.next_plan_at = time.monotonic() + 0.5
        self.recovery_hint = ""
        self.route_progress_y = self.robot_pose.y if self.robot_pose else 0.0
        self.stage_best_distance = math.inf
        self.route_progress_pose = self.robot_pose
        self.route_progress_at = time.monotonic()
        self.platform_stable_since = 0.0
        self.platform_validation_started = 0.0
        self.publish_status("started", robot_pose=self.pose_details())

    def on_cancel(self, _message: Empty) -> None:
        if not self.active:
            return
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.finish("canceled")

    def on_timer(self) -> None:
        now = time.monotonic()
        self.poll_mode_switch(now)
        if not self.active and self.stair_mode_active and now >= self.next_avoid_restore_at:
            if self.set_nav_mode("avoid"):
                self.stair_mode_active = False
                self.current_nav_mode = "avoid"
            else:
                self.next_avoid_restore_at = now + 2.0
        if not self.active:
            return
        if now - self.started_at > self.timeout_sec:
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.finish("failed", reason="mission_timeout", robot_pose=self.pose_details())
            return
        if self.current_stage is not None and (
            self.navigation_active or self.awaiting_platform_validation
        ):
            if not self.check_stage_route(now):
                return
        if self.next_plan_at is None or now < self.next_plan_at:
            return
        self.next_plan_at = None
        self.request_plan()

    def check_stage_route(self, now: float) -> bool:
        pose = self.robot_pose
        if pose is None or now - pose.received_at > self.odom_max_age_sec:
            self.abort_stage("stage_odom_lost")
            return False
        stage = self.current_stage
        if stage is None:
            self.abort_stage("stage_definition_lost")
            return False

        stair_stage = stage.profile in {STAIR_UP_PROFILE, STAIR_DOWN_PROFILE}
        lateral_error = self.stage_lateral_error(stage, pose)
        heading_error = wrap_angle(pose.yaw - stage.yaw_rad)
        corridor_overrun = (
            0.0 if stair_stage else self.generic_stage_corridor_overrun(stage, pose)
        )
        max_roll = self.stair_max_roll if stair_stage else self.route_max_roll
        max_pitch = self.stair_max_pitch if stair_stage else self.route_max_pitch
        if (
            (stair_stage and abs(lateral_error) > self.stair_max_lateral)
            or (stair_stage and abs(heading_error) > self.stair_max_heading)
            or (not stair_stage and corridor_overrun > 0.0)
            or abs(pose.roll) > max_roll
            or abs(pose.pitch) > max_pitch
            or pose.z < 0.18
        ):
            self.abort_stage(
                "stage_safety_boundary",
                lateral_error=lateral_error,
                heading_error=heading_error,
                corridor_overrun=corridor_overrun,
                robot_pose=self.pose_details(),
            )
            return False

        platform_safe = False
        platform_reason = ""
        if stage.profile == STAIR_UP_PROFILE:
            platform_safe = (
                pose.y >= self.stair_platform_min_y
                and pose.z >= self.stair_platform_min_z
                and abs(lateral_error) <= 0.55
                and abs(heading_error) <= 0.35
                and abs(pose.roll) <= 0.25
                and abs(pose.pitch) <= 0.25
            )
            platform_reason = "upper_platform_verified"
        elif stage.profile == STAIR_DOWN_PROFILE:
            platform_safe = (
                pose.y >= self.stair_down_platform_min_y
                and pose.z <= self.stair_down_platform_max_z
                and abs(lateral_error) <= 0.55
                and abs(heading_error) <= 0.35
                and abs(pose.roll) <= 0.25
                and abs(pose.pitch) <= 0.25
            )
            platform_reason = "lower_platform_verified"

        if platform_safe:
            if self.platform_stable_since == 0.0:
                self.platform_stable_since = now
            elif now - self.platform_stable_since >= self.stair_platform_settle_sec:
                self.complete_stage(
                    reason=platform_reason,
                    cancel_active_goal=True,
                    robot_pose=self.pose_details(),
                )
                return False
        else:
            self.platform_stable_since = 0.0

        if self.navigation_active and not platform_safe:
            distance = math.hypot(stage.x - pose.x, stage.y - pose.y)
            target_progress = distance <= self.stage_best_distance - 0.05
            displacement_since_progress = 0.0
            movement_progress = False
            if not stair_stage:
                progress_pose = self.route_progress_pose
                if progress_pose is None:
                    self.route_progress_pose = pose
                else:
                    displacement_since_progress = math.hypot(
                        pose.x - progress_pose.x, pose.y - progress_pose.y
                    )
                    movement_progress = (
                        displacement_since_progress
                        >= self.route_progress_min_displacement
                    )
            if target_progress:
                self.stage_best_distance = distance
            if target_progress or movement_progress:
                self.route_progress_at = now
                if not stair_stage:
                    self.route_progress_pose = pose
            no_progress_sec = (
                self.stair_no_progress_sec if stair_stage else self.route_no_progress_sec
            )
            if now - self.route_progress_at >= no_progress_sec:
                self.abort_stage(
                    "stage_no_progress",
                    distance_to_target=distance,
                    displacement_since_progress=displacement_since_progress,
                    robot_pose=self.pose_details(),
                )
                return False

        if self.awaiting_platform_validation:
            if now - self.platform_validation_started >= self.stair_platform_timeout_sec:
                self.finish(
                    "failed",
                    reason=(
                        "upper_platform_validation_timeout"
                        if stage.profile == STAIR_UP_PROFILE
                        else "lower_platform_validation_timeout"
                    ),
                    robot_pose=self.pose_details(),
                )
                return False
            if platform_safe:
                return True
        return True

    def poll_mode_switch(self, _now: float) -> None:
        future = self.mode_switch_future
        if future is None or not future.done():
            return
        request_mission_id = self.mode_switch_mission_id
        request_stage_index = self.mode_switch_stage_index
        requested_mode = self.mode_switch_mode
        plan = self.mode_switch_plan
        confidence = self.mode_switch_confidence
        self.mode_switch_future = None
        self.mode_switch_mission_id = ""
        self.mode_switch_stage_index = -1
        self.mode_switch_plan = {}
        try:
            armed = future.result()
        except Exception as exc:
            self.get_logger().error(f"Navigation mode worker failed: {exc}")
            armed = False
        if armed:
            self.current_nav_mode = requested_mode
            self.stair_mode_active = requested_mode != "avoid"

        if (
            not self.active
            or request_mission_id != self.mission_id
            or request_stage_index != self.stage_index
        ):
            if armed:
                if self.set_nav_mode("avoid"):
                    self.stair_mode_active = False
                    self.current_nav_mode = "avoid"
                else:
                    self.next_avoid_restore_at = time.monotonic() + 2.0
            if self.active and request_mission_id == self.mission_id:
                self.finish("failed", reason="stale_navigation_mode_switch")
            return
        if not armed:
            restored = self.set_nav_mode("avoid")
            if restored:
                self.current_nav_mode = "avoid"
                self.stair_mode_active = False
            self.finish(
                "failed",
                reason="failed_to_set_navigation_mode",
                requested_mode=requested_mode,
            )
            return

        stage = self.current_stage
        if stage is None or stage.navigation_mode != requested_mode:
            self.finish("failed", reason="stage_definition_lost_after_mode_switch")
            return
        safe, reason = self.stage_start_is_safe(stage)
        if not safe:
            self.finish(
                "failed", reason=reason, confidence=confidence, robot_pose=self.pose_details()
            )
            return
        self.dispatch_stage(stage, plan, confidence, request_stage_index)

    def begin_stage(self, stage_index: int) -> None:
        if not self.active or not 0 <= stage_index < len(self.active_stages):
            self.finish("failed", reason="invalid_stage_index", requested_index=stage_index)
            return
        self.stage_index = stage_index
        stage = self.current_stage
        if stage is None:
            self.finish("failed", reason="stage_definition_lost")
            return
        self.profile = stage.profile
        self.navigation_active = False
        self.awaiting_platform_validation = False
        self.goal_handle = None
        self.platform_stable_since = 0.0
        self.platform_validation_started = 0.0
        self.stage_start_pose = self.robot_pose
        self.route_progress_at = time.monotonic()
        self.route_progress_pose = self.robot_pose
        self.stage_best_distance = (
            math.hypot(stage.x - self.robot_pose.x, stage.y - self.robot_pose.y)
            if self.robot_pose is not None
            else math.inf
        )

        safe, reason = self.stage_start_is_safe(stage)
        if not safe:
            self.finish("failed", reason=reason, robot_pose=self.pose_details())
            return
        self.publish_status(
            "stage_starting",
            route_id=self.active_route.route_id if self.active_route else "",
            navigation_mode=stage.navigation_mode,
            target_source="validated_route_catalog",
        )
        if self.current_nav_mode == stage.navigation_mode:
            self.dispatch_stage(
                stage, self.accepted_plan, self.accepted_confidence, stage_index
            )
            return

        self.mode_switch_mission_id = self.mission_id
        self.mode_switch_stage_index = stage_index
        self.mode_switch_mode = stage.navigation_mode
        self.mode_switch_plan = self.accepted_plan
        self.mode_switch_confidence = self.accepted_confidence
        self.mode_switch_future = self.mode_executor.submit(
            self.set_nav_mode, stage.navigation_mode
        )
        self.publish_status(
            "switching_navigation_mode",
            navigation_mode=stage.navigation_mode,
            route_id=self.active_route.route_id if self.active_route else "",
        )

    def dispatch_stage(
        self,
        stage: MissionStage,
        plan: dict[str, Any],
        confidence: float,
        stage_index: int,
    ) -> None:
        if not self.active or stage_index != self.stage_index or stage != self.current_stage:
            return
        self.send_goal(
            x=stage.x,
            y=stage.y,
            yaw=stage.yaw_rad,
            frame_id=stage.frame_id,
            plan=plan,
            confidence=confidence,
            route_id=self.active_route.route_id if self.active_route else "",
            stage_index=stage_index,
        )

    def complete_stage(
        self, *, reason: str, cancel_active_goal: bool = False, **details: Any
    ) -> None:
        stage = self.current_stage
        if not self.active or stage is None:
            return
        completed_index = self.stage_index
        old_goal_handle = self.goal_handle if cancel_active_goal else None
        self.navigation_active = False
        self.awaiting_platform_validation = False
        self.goal_handle = None
        self.step += 1
        self.plan_failures = 0
        self.recovery_hint = ""
        self.publish_status(
            "stage_completed",
            reason=reason,
            route_id=self.active_route.route_id if self.active_route else "",
            future_prediction=self.accepted_plan.get("future_prediction", {}),
            **details,
        )

        next_index = completed_index + 1
        if next_index < len(self.active_stages):
            self.stage_index = next_index
            self.profile = self.active_stages[next_index].profile
        if old_goal_handle is not None:
            old_goal_handle.cancel_goal_async()
        if next_index < len(self.active_stages):
            self.begin_stage(next_index)
            return
        self.finish(
            "succeeded",
            reason="route_completed",
            route_id=self.active_route.route_id if self.active_route else "",
            future_prediction=self.accepted_plan.get("future_prediction", {}),
            robot_pose=self.pose_details(),
        )

    def abort_stage(self, reason: str, **details: Any) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.finish("failed", reason=reason, **details)

    def request_plan(self) -> None:
        if not self.active or self.planning or self.navigation_active:
            return
        instruction = (
            f"High-level task: {self.mission}\n"
            "Issue one approved route command. Do not generate motion controls, local "
            f"subgoals, waypoints, or coordinates. {self.recovery_hint}"
        ).strip()
        self.pending_instruction = instruction
        self.planning = True
        self.instruction_publisher.publish(String(data=instruction))
        self.publish_status("planning")

    def on_planner_status(self, message: String) -> None:
        if not self.active or not self.planning:
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state = payload.get("state")
        if state not in {"failed", "rejected", "busy"}:
            return
        self.planning = False
        self.pending_instruction = ""
        self.plan_failures += 1
        if self.plan_failures > self.max_plan_failures:
            self.finish("failed", reason=f"planner_{state}", planner_status=payload)
            return
        planner_reason = str(payload.get("reason", "invalid route command"))[:400]
        self.recovery_hint = (
            f"The previous route command was rejected: {planner_reason}. Recheck the image "
            "and output exactly the required JSON schema."
        )
        self.next_plan_at = time.monotonic() + self.settle_sec
        self.publish_status(
            "planning_retry", reason=f"planner_{state}", attempt=self.plan_failures
        )

    def on_route_command(self, message: String) -> None:
        if not self.active or not self.planning:
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            self.finish("failed", reason="invalid_route_command_json")
            return
        if not isinstance(payload, dict):
            self.finish("failed", reason="invalid_route_command_payload")
            return
        response_instruction = str(payload.get("instruction", "")).strip()
        if response_instruction != self.pending_instruction.strip():
            self.publish_status(
                "route_command_ignored",
                reason="instruction_mismatch",
                command_job_id=payload.get("job_id"),
            )
            return

        self.planning = False
        self.pending_instruction = ""
        try:
            command = validate_route_command(
                {
                    "action": payload.get("action"),
                    "route_id": payload.get("route_id"),
                    "future_prediction": payload.get("future_prediction"),
                    "confidence": payload.get("confidence"),
                    "reason": payload.get("reason"),
                },
                self.routes.keys(),
            )
        except ValueError as exc:
            self.retry_unsafe_plan(
                reason="invalid_route_command",
                hint=(
                    f"The previous command was rejected: {exc}. Output only one approved "
                    "route command."
                ),
                command_job_id=payload.get("job_id"),
            )
            return
        confidence = command["confidence"]
        if not 0.0 <= confidence <= 1.0 or confidence < self.min_confidence:
            self.retry_unsafe_plan(
                reason="low_confidence",
                confidence=confidence,
                hint=(
                    "Confidence was insufficient. Recheck visible evidence and choose "
                    "hold_position if an approved route cannot be grounded safely."
                ),
            )
            return
        action = command["action"]
        future_prediction = command["future_prediction"]
        try:
            validate_required_route(command, self.required_route_id)
        except ValueError as exc:
            self.finish(
                "blocked",
                reason="required_route_mismatch",
                action=action,
                route_id=command["route_id"],
                required_route_id=self.required_route_id,
                error=str(exc),
            )
            return
        if action == ACTION_HOLD:
            self.finish(
                "blocked",
                reason="cosmos_hold_position",
                command_reason=command["reason"],
                confidence=confidence,
                future_prediction=future_prediction,
            )
            return
        if action == ACTION_COMPLETE:
            self.finish(
                "succeeded",
                reason="cosmos_mission_complete",
                command_reason=command["reason"],
                confidence=confidence,
                future_prediction=future_prediction,
            )
            return
        if action != ACTION_NAVIGATE:
            self.finish("failed", reason="unsupported_route_command", action=action)
            return
        route = self.routes.get(command["route_id"])
        if route is None:
            self.finish(
                "blocked", reason="route_not_approved", route_id=command["route_id"]
            )
            return

        try:
            stages = self.route_stages(route)
        except (AttributeError, TypeError, ValueError) as exc:
            self.finish(
                "failed",
                reason="invalid_validated_route_catalog",
                route_id=route.route_id,
                error=str(exc),
            )
            return

        self.active_route = route
        self.active_stages = stages
        self.stage_index = 0
        self.profile = stages[0].profile
        self.accepted_plan = payload
        self.accepted_confidence = confidence
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.finish("failed", reason="navigate_to_pose_unavailable")
            return
        self.publish_status(
            "route_command_accepted",
            action=action,
            route_id=route.route_id,
            confidence=confidence,
            command_reason=command["reason"],
            future_prediction=future_prediction,
            target_source="validated_route_catalog",
        )
        self.begin_stage(0)

    def send_goal(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        plan: dict[str, Any],
        confidence: float,
        route_id: str,
        stage_index: int,
    ) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.target_publisher.publish(pose)
        self.navigation_active = True
        self.route_progress_y = self.robot_pose.y if self.robot_pose else 0.0
        self.route_progress_at = time.monotonic()
        self.route_progress_pose = self.robot_pose
        self.stage_best_distance = (
            math.hypot(x - self.robot_pose.x, y - self.robot_pose.y)
            if self.robot_pose is not None
            else math.inf
        )
        request_mission_id = self.mission_id
        future = self.navigate_client.send_goal_async(goal)
        self.goal_request_future = future
        future.add_done_callback(
            lambda completed, accepted_plan=plan, mission_id=request_mission_id,
            request_stage_index=stage_index: (
                self.on_goal_response(
                    completed, accepted_plan, mission_id, request_stage_index
                )
            )
        )
        self.publish_status(
            "goal_requested",
            confidence=confidence,
            target={"frame_id": frame_id, "x": x, "y": y, "yaw_rad": yaw},
            route_id=route_id,
            target_source="validated_route_catalog",
            future_prediction=plan.get("future_prediction", {}),
        )

    def on_goal_response(
        self,
        future: Any,
        plan: dict[str, Any],
        request_mission_id: str,
        request_stage_index: int,
    ) -> None:
        if self.goal_request_future is future:
            self.goal_request_future = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            if (
                self.active
                and request_mission_id == self.mission_id
                and request_stage_index == self.stage_index
            ):
                self.finish("failed", reason=f"goal_request_error:{exc}")
            return
        if (
            not self.active
            or request_mission_id != self.mission_id
            or request_stage_index != self.stage_index
        ):
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.finish("failed", reason="goal_rejected")
            return
        self.goal_handle = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed, accepted_plan=plan, mission_id=request_mission_id,
            result_stage_index=request_stage_index: (
                self.on_goal_result(
                    completed, accepted_plan, mission_id, result_stage_index
                )
            )
        )
        self.publish_status("navigating")

    def on_goal_result(
        self,
        future: Any,
        plan: dict[str, Any],
        request_mission_id: str,
        result_stage_index: int,
    ) -> None:
        if (
            not self.active
            or request_mission_id != self.mission_id
            or result_stage_index != self.stage_index
        ):
            return
        self.navigation_active = False
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.finish("failed", reason=f"goal_result_error:{exc}")
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.finish("failed", reason="navigation_failed", action_status=status)
            return
        stage = self.current_stage
        if stage is None:
            self.finish("failed", reason="stage_definition_lost")
            return
        if stage.profile in {STAIR_UP_PROFILE, STAIR_DOWN_PROFILE}:
            self.awaiting_platform_validation = True
            self.platform_validation_started = time.monotonic()
            self.publish_status(
                (
                    "validating_upper_platform"
                    if stage.profile == STAIR_UP_PROFILE
                    else "validating_lower_platform"
                ),
                future_prediction=plan.get("future_prediction", {}),
                robot_pose=self.pose_details(),
            )
            return
        self.complete_stage(
            reason="navigation_goal_succeeded",
            robot_pose=self.pose_details(),
        )

    def retry_unsafe_plan(self, *, reason: str, hint: str, **details: Any) -> None:
        self.plan_failures += 1
        if self.plan_failures > self.max_plan_failures:
            self.finish("blocked", reason=reason, **details)
            return
        self.recovery_hint = hint
        self.next_plan_at = time.monotonic() + self.settle_sec
        self.publish_status(
            "planning_retry", reason=reason, attempt=self.plan_failures, **details
        )

    def set_nav_mode(self, mode: str) -> bool:
        environment = os.environ.copy()
        environment["GO2W_NAV_MODE_FILE"] = str(self.nav_mode_file)
        try:
            result = subprocess.run(
                [str(self.nav_mode_script), mode],
                cwd=ROOT_DIR,
                env=environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f"Failed to set Go2-W navigation mode {mode}: {exc}")
            return False
        if result.returncode != 0:
            self.get_logger().error(
                f"Failed to set Go2-W navigation mode {mode}: {result.stdout[-1200:]}"
            )
            return False
        self.get_logger().info(f"Go2-W navigation mode set to {mode}")
        return True

    def finish(self, state: str, **details: Any) -> None:
        if not self.active:
            return
        if self.stair_mode_active:
            restored = self.set_nav_mode("avoid")
            details["avoid_mode_restored"] = restored
            self.stair_mode_active = not restored
            if restored:
                self.current_nav_mode = "avoid"
            if not restored:
                self.next_avoid_restore_at = time.monotonic() + 2.0
        else:
            details["avoid_mode_restored"] = self.current_nav_mode == "avoid"
        self.publish_status(state, **details)
        self.mission = ""
        self.started_at = 0.0
        self.profile = "pending"
        self.active_route = None
        self.active_stages = ()
        self.stage_index = -1
        self.stage_start_pose = None
        self.accepted_plan = {}
        self.accepted_confidence = 0.0
        self.pending_instruction = ""
        self.planning = False
        self.navigation_active = False
        self.awaiting_platform_validation = False
        self.next_plan_at = None
        self.goal_handle = None
        self.recovery_hint = ""
        self.platform_stable_since = 0.0
        self.platform_validation_started = 0.0
        self.stage_best_distance = math.inf
        self.route_progress_pose = None

    def shutdown(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        request_future = self.goal_request_future
        if request_future is not None and rclpy.ok():
            rclpy.spin_until_future_complete(
                self, request_future, timeout_sec=2.0
            )
            if request_future.done():
                try:
                    handle = request_future.result()
                except Exception:
                    handle = None
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
        if self.mode_switch_future is not None:
            try:
                if self.mode_switch_future.result(timeout=20.0):
                    self.stair_mode_active = True
                    self.current_nav_mode = self.mode_switch_mode
            except Exception:
                pass
        if self.stair_mode_active:
            if self.set_nav_mode("avoid"):
                self.stair_mode_active = False
                self.current_nav_mode = "avoid"
        self.mode_executor.shutdown(wait=False, cancel_futures=True)


def main() -> None:
    rclpy.init()
    node = CosmosVlnMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
