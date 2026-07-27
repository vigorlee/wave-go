#!/usr/bin/python3
"""HouseWorld-specific safety layer for the existing NWM-Cosmos3Edge mission."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
from dataclasses import dataclass
from typing import Any, Callable, Sequence

try:
    import rclpy
    from cosmos_vln_mission import CosmosVlnMission, MissionStage, RobotPose
except ModuleNotFoundError as runtime_import_error:
    # Keep geometry and speed-transition helpers available to offline tests.
    rclpy = None
    CosmosVlnMission = object
    MissionStage = Any
    RobotPose = Any
    RUNTIME_IMPORT_ERROR: ModuleNotFoundError | None = runtime_import_error
else:
    RUNTIME_IMPORT_ERROR = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ROUTES = ROOT_DIR / "config/cosmos_vln_house_routes.json"
DOOR_STAGES = {"east_door_inside", "kitchen"}
SPEED_PROFILES = {"normal", "door"}


@dataclass(frozen=True)
class ParameterSetResult:
    success: bool
    detail: str = ""


@dataclass(frozen=True)
class SpeedTransitionResult:
    success: bool
    applied_profile: str
    detail: str = ""
    rollback_succeeded: bool = True


def bounded_env_float(name: str, default: float, minimum: float, maximum: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except ValueError:
        value = default
    if not math.isfinite(value):
        value = default
    return max(minimum, min(maximum, value))


def finite_point(value: object) -> tuple[float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError("HouseWorld boundary points must be [x, y] pairs")
    if len(value) != 2:
        raise ValueError("HouseWorld boundary points must contain two values")
    x, y = float(value[0]), float(value[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("HouseWorld boundary points must be finite")
    return x, y


def point_in_polygon(x: float, y: float, polygon: Sequence[tuple[float, float]]) -> bool:
    """Return true for points inside or on the edge of a simple polygon."""
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]

        dx, dy = x2 - x1, y2 - y1
        cross = (x - x1) * dy - (y - y1) * dx
        if abs(cross) <= 1.0e-8:
            dot = (x - x1) * dx + (y - y1) * dy
            if -1.0e-8 <= dot <= dx * dx + dy * dy + 1.0e-8:
                return True

        if (y1 > y) == (y2 > y):
            continue
        intersection_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        if x <= intersection_x:
            inside = not inside
    return inside


def stage_speed_profile(stage_id: str) -> str:
    return "door" if stage_id in DOOR_STAGES else "normal"


def speed_profile_commands(
    profile: str, normal_speed: float, door_speed: float
) -> tuple[tuple[str, str, str], ...]:
    if profile not in SPEED_PROFILES:
        raise ValueError(f"unsupported HouseWorld speed profile: {profile}")
    speed = door_speed if profile == "door" else normal_speed
    angular = 0.35 if profile == "door" else 0.55
    reverse = min(0.10, speed)
    return (
        ("/controller_server", "FollowPath.vx_max", f"{speed:.3f}"),
        ("/controller_server", "FollowPath.vx_min", f"-{reverse:.3f}"),
        ("/controller_server", "FollowPath.wz_max", f"{angular:.3f}"),
        (
            "/velocity_optimizer",
            "max_velocity",
            f"[{speed:.3f}, 0.0, {angular:.3f}]",
        ),
        (
            "/velocity_optimizer",
            "min_velocity",
            f"[-{reverse:.3f}, 0.0, -{angular:.3f}]",
        ),
    )


def run_ros_parameter_set(node: str, parameter: str, value: str) -> ParameterSetResult:
    try:
        result = subprocess.run(
            ["ros2", "param", "set", "--no-daemon", node, parameter, value],
            check=False,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return ParameterSetResult(False, str(exc))
    detail = (result.stderr or result.stdout).strip()
    return ParameterSetResult(result.returncode == 0, detail)


def _apply_speed_profile(
    profile: str,
    normal_speed: float,
    door_speed: float,
    runner: Callable[[str, str, str], ParameterSetResult],
) -> ParameterSetResult:
    for node, parameter, value in speed_profile_commands(
        profile, normal_speed, door_speed
    ):
        result = runner(node, parameter, value)
        if not result.success:
            return ParameterSetResult(
                False, f"{node} {parameter}: {result.detail or 'parameter rejected'}"
            )
    return ParameterSetResult(True)


def transition_speed_profile(
    current_profile: str,
    requested_profile: str,
    normal_speed: float,
    door_speed: float,
    runner: Callable[[str, str, str], ParameterSetResult] = run_ros_parameter_set,
) -> SpeedTransitionResult:
    if requested_profile not in SPEED_PROFILES:
        return SpeedTransitionResult(
            False, current_profile or "unknown", f"invalid profile {requested_profile}"
        )
    if requested_profile == current_profile:
        return SpeedTransitionResult(True, current_profile)
    applied = _apply_speed_profile(
        requested_profile, normal_speed, door_speed, runner
    )
    if applied.success:
        return SpeedTransitionResult(True, requested_profile)

    rollback_profile = (
        current_profile if current_profile in SPEED_PROFILES else "normal"
    )
    rollback = _apply_speed_profile(
        rollback_profile, normal_speed, door_speed, runner
    )
    return SpeedTransitionResult(
        False,
        rollback_profile if rollback.success else "unknown",
        applied.detail,
        rollback_succeeded=rollback.success,
    )


class HouseCosmosVlnMission(CosmosVlnMission):
    """Add HouseWorld bounds and doorway speed profiles to the base supervisor."""

    def __init__(self) -> None:
        routes_path = Path(os.environ.get("COSMOS_VLN_ROUTES_FILE", DEFAULT_ROUTES))
        try:
            payload = json.loads(routes_path.read_text(encoding="utf-8"))
            raw_polygon = payload["scene"]["bounds_polygon"]
            polygon = tuple(finite_point(point) for point in raw_polygon)
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"cannot load HouseWorld safety boundary: {exc}") from exc
        if len(polygon) < 3:
            raise RuntimeError("HouseWorld safety boundary needs at least three points")

        self.house_polygon = polygon
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
        super().__init__()
        self.get_logger().info(
            "HouseWorld safety active: "
            f"boundary_points={len(self.house_polygon)} "
            f"speed={self.normal_speed:.2f} door_speed={self.door_speed:.2f}"
        )

    def pose_inside_house(self, pose: RobotPose) -> bool:
        return point_in_polygon(pose.x, pose.y, self.house_polygon)

    def stage_start_is_safe(self, stage: MissionStage) -> tuple[bool, str]:
        safe, reason = super().stage_start_is_safe(stage)
        if not safe:
            return safe, reason
        pose = self.robot_pose
        if pose is None or not self.pose_inside_house(pose):
            return False, "house_stage_start_outside_boundary"
        if not point_in_polygon(stage.x, stage.y, self.house_polygon):
            return False, "house_stage_target_outside_boundary"
        return True, ""

    def set_speed_profile(self, profile: str) -> bool:
        transition = transition_speed_profile(
            self.speed_profile, profile, self.normal_speed, self.door_speed
        )
        self.speed_profile = transition.applied_profile
        if not transition.success:
            self.get_logger().error(
                f"Cannot apply HouseWorld {profile} speed profile: "
                f"{transition.detail}; rollback={transition.rollback_succeeded}"
            )
            return False
        speed = self.door_speed if profile == "door" else self.normal_speed
        angular = 0.35 if profile == "door" else 0.55
        self.get_logger().info(
            f"HouseWorld speed profile={profile} vx_max={speed:.2f} wz_max={angular:.2f}"
        )
        return True

    def begin_stage(self, stage_index: int) -> None:
        if not 0 <= stage_index < len(self.active_stages):
            super().begin_stage(stage_index)
            return
        stage = self.active_stages[stage_index]
        profile = stage_speed_profile(stage.stage_id)
        if not self.set_speed_profile(profile):
            self.finish(
                "failed",
                reason="house_speed_profile_failed",
                requested_profile=profile,
            )
            return
        super().begin_stage(stage_index)

    def check_stage_route(self, now: float) -> bool:
        pose = self.robot_pose
        if pose is not None and not self.pose_inside_house(pose):
            self.abort_stage(
                "house_boundary_violation",
                robot_pose=self.pose_details(),
            )
            return False
        return super().check_stage_route(now)

    def finish(self, state: str, **details: object) -> None:
        if self.active and self.speed_profile not in {"", "normal"}:
            details["normal_speed_restored"] = self.set_speed_profile("normal")
        super().finish(state, **details)

    def shutdown(self) -> None:
        if self.speed_profile not in {"", "normal"}:
            self.set_speed_profile("normal")
        super().shutdown()


def main() -> None:
    if rclpy is None:
        raise RuntimeError(f"ROS runtime is unavailable: {RUNTIME_IMPORT_ERROR}")
    rclpy.init()
    node = HouseCosmosVlnMission()
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
