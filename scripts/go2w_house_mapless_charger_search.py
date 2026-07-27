#!/usr/bin/python3
"""Mapless LiDAR and vision search for the HouseWorld charging marker."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, replace
import base64
import http.client
from io import BytesIO
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlsplit

import cv2
import numpy as np
from PIL import Image

try:
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image as DepthImage, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Empty, String
except ModuleNotFoundError as runtime_import_error:
    Twist = Odometry = CompressedImage = DepthImage = PointCloud2 = Empty = String = Any
    HistoryPolicy = QoSProfile = ReliabilityPolicy = Any
    Node = object
    point_cloud2 = None
    rclpy = None
    RUNTIME_IMPORT_ERROR: ModuleNotFoundError | None = runtime_import_error
else:
    RUNTIME_IMPORT_ERROR = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT_DIR / "config/go2w_house_mapless_search.json"
DEFAULT_POSTURE_FILE = ROOT_DIR / ".run/go2w_house/posture"
DEFAULT_JOBS_DIR = ROOT_DIR / ".run/go2w_house/cosmos/mapless_charger_search"
DEFAULT_COSMOS_ROOT = Path("/home/unitree/matrix_g1_lcm_demo")
TARGET_KIND = "robot_charging_dock"
MIN_COSMOS_CONFIDENCE = 0.70
REASONER_MAX_NEW_TOKENS = 1024
BIN_ANGLES_DEG = (-75.0, -50.0, -25.0, 0.0, 25.0, 50.0, 75.0)
COSMOS_SETTLED_LINEAR_SPEED_MPS = 0.03
COSMOS_SETTLED_YAW_RATE_RPS = 0.06
GENERATOR_ACTION_SOURCE = "cosmos3_generator"
SAFETY_STOP_SOURCE = "safety_stop"


@dataclass(frozen=True)
class GeneratorConfig:
    required: bool
    server_url: str
    domain_name: str
    adapter: str
    raw_action_dim: int
    image_size: int
    fps: int
    action_chunk_size: int
    execute_prefix_steps: int
    candidate_seeds: tuple[int, ...]
    request_timeout_sec: float
    reasoner_timeout_sec: float
    command_ttl_sec: float
    maximum_stationary_drift_m: float
    maximum_stationary_drift_rad: float
    maximum_failures: int
    translation_scale: float
    rotation_scale: float
    lateral_yaw_gain: float
    max_linear_speed_mps: float
    max_approach_speed_mps: float
    max_yaw_rate_rps: float
    max_linear_step_mps: float
    max_yaw_step_rps: float


@dataclass(frozen=True)
class CommandState:
    linear_x: float
    angular_z: float
    source: str
    issued_at: float
    valid_until: float
    request_id: int | None = None
    chunk_step: int | None = None


@dataclass(frozen=True)
class GeneratorActionPrefix:
    generation: int
    request_id: int
    selected_seed: int
    nominal_commands: tuple[tuple[float, float], ...]
    marker_hint: MarkerObservation | None = None
    marker_image: bytes | None = None
    last_exact_seen_at: float = 0.0


@dataclass(frozen=True)
class ApproachPrefixResult:
    marker: MarkerObservation
    image: bytes
    last_exact_depth_m: float | None
    last_exact_seen_at: float


class GeneratorError(RuntimeError):
    """The required Cosmos3 Generator contract or response is invalid."""


class GeneratorBlocked(GeneratorError):
    """The Generator repeatedly failed and the task must fail closed."""


@dataclass(frozen=True)
class SearchConfig:
    generator: GeneratorConfig
    marker_id: int
    confirm_frames: int
    final_confirm_frames: int
    final_height_ratio: float
    final_center_tolerance: float
    search_speed: float
    cautious_speed: float
    reverse_speed: float
    approach_speed: float
    creep_speed: float
    approach_min_forward_speed: float
    approach_forward_error_limit: float
    approach_yaw_gain: float
    approach_max_yaw_rate: float
    approach_yaw_step_limit: float
    reacquire_yaw_rate: float
    reacquire_sweep_half_angle: float
    reacquire_direction_flip: float
    candidate_stop_deceleration: float
    candidate_stop_yaw_deceleration: float
    max_yaw_rate: float
    emergency_clearance: float
    turn_clearance: float
    slow_clearance: float
    wall_clearance: float
    charging_min_lidar_clearance: float
    depth_topic: str
    depth_width: int
    depth_height: int
    depth_horizontal_fov_deg: float
    depth_maximum_frame_delta: float
    depth_minimum_valid_samples: int
    depth_minimum_valid_fraction: float
    depth_minimum_range: float
    depth_maximum_range: float
    depth_minimum_charging_range: float
    depth_maximum_charging_range: float
    tracker_forward_min_depth: float
    minimum_approach_travel: float
    arrival_stop_hold: float
    minimum_standing_z: float
    maximum_standing_tilt: float
    posture_fault_delay: float
    cosmos_settle_hold: float
    cosmos_settle_timeout: float
    candidate_lost_hold: float
    approach_observation_hold: float
    tracker_only_timeout: float
    marker_lost_timeout: float
    search_timeout: float
    approach_timeout: float
    camera_topic: str
    lidar_topic: str
    velocity_topic: str


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float
    roll: float
    pitch: float
    received_at: float
    linear_speed: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class LidarScan:
    angles_rad: tuple[float, ...]
    clearances_m: tuple[float, ...]
    front_m: float
    left_m: float
    right_m: float
    received_at: float


@dataclass(frozen=True)
class DepthFrame:
    values: np.ndarray
    received_at: float


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    center_x: float
    center_y: float
    width: int
    height: int
    marker_height_ratio: float
    corners: tuple[tuple[float, float], ...]
    exact_id: bool = True
    verification: str = "aruco"

    @property
    def horizontal_error(self) -> float:
        return self.center_x / self.width - 0.5


@dataclass(frozen=True)
class CosmosDetection:
    target_visible: bool
    target_kind: str
    marker_visible: bool
    confidence: float
    safe_to_approach: bool
    reason: str

    @property
    def confirmed(self) -> bool:
        return (
            self.target_visible
            and self.marker_visible
            and self.target_kind == TARGET_KIND
            and self.confidence >= MIN_COSMOS_CONFIDENCE
        )


@dataclass
class VisualMarkerTracker:
    tracker: Any
    width: int
    height: int
    marker_id: int
    center_x_fraction: float
    center_y_fraction: float
    marker_width_fraction: float
    marker_height_fraction: float


def bounded_number(
    value: object, field: str, minimum: float, maximum: float
) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise ValueError(f"{field} must be in [{minimum}, {maximum}]")
    return parsed


def bounded_integer(
    value: object, field: str, minimum: int, maximum: int
) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be an integer")
    parsed = int(value)
    if float(value) != parsed or not minimum <= parsed <= maximum:
        raise ValueError(f"{field} must be in [{minimum}, {maximum}]")
    return parsed


def load_generator_config(payload: object) -> GeneratorConfig:
    if not isinstance(payload, dict):
        raise ValueError("generator must be an object")
    expected = {
        "required",
        "server_url",
        "domain_name",
        "adapter",
        "raw_action_dim",
        "image_size",
        "fps",
        "action_chunk_size",
        "execute_prefix_steps",
        "candidate_seeds",
        "request_timeout_sec",
        "reasoner_timeout_sec",
        "command_ttl_sec",
        "maximum_stationary_drift_m",
        "maximum_stationary_drift_rad",
        "maximum_failures",
        "translation_scale",
        "rotation_scale",
        "lateral_yaw_gain",
        "max_linear_speed_mps",
        "max_approach_speed_mps",
        "max_yaw_rate_rps",
        "max_linear_step_mps",
        "max_yaw_step_rps",
    }
    if set(payload) != expected:
        missing = sorted(expected - set(payload))
        extra = sorted(set(payload) - expected)
        raise ValueError(
            f"generator fields do not match the protocol; missing={missing} extra={extra}"
        )
    if payload["required"] is not True:
        raise ValueError("generator.required must be true; planner fallback is forbidden")
    server_url = str(payload["server_url"]).rstrip("/")
    parsed_url = urlsplit(server_url)
    if (
        parsed_url.scheme != "http"
        or parsed_url.hostname not in {"127.0.0.1", "localhost"}
        or parsed_url.port is None
        or parsed_url.path not in {"", "/"}
        or parsed_url.query
        or parsed_url.fragment
    ):
        raise ValueError("generator.server_url must be a local http://host:port URL")
    domain_name = str(payload["domain_name"])
    adapter = str(payload["adapter"])
    if domain_name != "av":
        raise ValueError("generator.domain_name must be av for this experimental adapter")
    if adapter != "experimental_av_relative_pose":
        raise ValueError("unsupported Generator action adapter")
    raw_action_dim = bounded_integer(
        payload["raw_action_dim"], "generator.raw_action_dim", 9, 9
    )
    fps = bounded_integer(payload["fps"], "generator.fps", 1, 30)
    action_chunk_size = bounded_integer(
        payload["action_chunk_size"], "generator.action_chunk_size", 1, 120
    )
    execute_prefix_steps = bounded_integer(
        payload["execute_prefix_steps"], "generator.execute_prefix_steps", 1, 16
    )
    if execute_prefix_steps > action_chunk_size:
        raise ValueError("generator.execute_prefix_steps exceeds action_chunk_size")
    candidate_seed_payload = payload["candidate_seeds"]
    if (
        not isinstance(candidate_seed_payload, list)
        or not 2 <= len(candidate_seed_payload) <= 8
    ):
        raise ValueError("generator.candidate_seeds must contain 2 to 8 seeds")
    candidate_seeds = tuple(
        bounded_integer(
            seed,
            f"generator.candidate_seeds[{index}]",
            0,
            2_147_483_647,
        )
        for index, seed in enumerate(candidate_seed_payload)
    )
    if len(set(candidate_seeds)) != len(candidate_seeds):
        raise ValueError("generator.candidate_seeds must be unique")
    max_linear_speed = bounded_number(
        payload["max_linear_speed_mps"],
        "generator.max_linear_speed_mps",
        0.03,
        0.90,
    )
    max_approach_speed = bounded_number(
        payload["max_approach_speed_mps"],
        "generator.max_approach_speed_mps",
        0.02,
        max_linear_speed,
    )
    return GeneratorConfig(
        required=True,
        server_url=server_url,
        domain_name=domain_name,
        adapter=adapter,
        raw_action_dim=raw_action_dim,
        image_size=bounded_integer(
            payload["image_size"], "generator.image_size", 128, 768
        ),
        fps=fps,
        action_chunk_size=action_chunk_size,
        execute_prefix_steps=execute_prefix_steps,
        candidate_seeds=candidate_seeds,
        request_timeout_sec=bounded_number(
            payload["request_timeout_sec"],
            "generator.request_timeout_sec",
            1.0,
            60.0,
        ),
        reasoner_timeout_sec=bounded_number(
            payload["reasoner_timeout_sec"],
            "generator.reasoner_timeout_sec",
            5.0,
            300.0,
        ),
        command_ttl_sec=bounded_number(
            payload["command_ttl_sec"],
            "generator.command_ttl_sec",
            0.10,
            0.50,
        ),
        maximum_stationary_drift_m=bounded_number(
            payload["maximum_stationary_drift_m"],
            "generator.maximum_stationary_drift_m",
            0.01,
            0.15,
        ),
        maximum_stationary_drift_rad=bounded_number(
            payload["maximum_stationary_drift_rad"],
            "generator.maximum_stationary_drift_rad",
            0.01,
            0.20,
        ),
        maximum_failures=bounded_integer(
            payload["maximum_failures"], "generator.maximum_failures", 1, 10
        ),
        translation_scale=bounded_number(
            payload["translation_scale"],
            "generator.translation_scale",
            0.0001,
            0.35,
        ),
        rotation_scale=bounded_number(
            payload["rotation_scale"],
            "generator.rotation_scale",
            0.01,
            2.0,
        ),
        lateral_yaw_gain=bounded_number(
            payload["lateral_yaw_gain"],
            "generator.lateral_yaw_gain",
            0.0,
            2.0,
        ),
        max_linear_speed_mps=max_linear_speed,
        max_approach_speed_mps=max_approach_speed,
        max_yaw_rate_rps=bounded_number(
            payload["max_yaw_rate_rps"],
            "generator.max_yaw_rate_rps",
            0.05,
            0.50,
        ),
        max_linear_step_mps=bounded_number(
            payload["max_linear_step_mps"],
            "generator.max_linear_step_mps",
            0.005,
            0.25,
        ),
        max_yaw_step_rps=bounded_number(
            payload["max_yaw_step_rps"],
            "generator.max_yaw_step_rps",
            0.01,
            0.20,
        ),
    )


def load_config(path: Path = DEFAULT_CONFIG) -> SearchConfig:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("mode") != "mapless_visual_search":
        raise ValueError("mapless search mode is required")
    marker = payload["marker"]
    motion = payload["motion"]
    arrival = payload["arrival"]
    posture = payload["posture_guard"]
    depth = payload["depth"]
    if marker.get("dictionary") != "DICT_4X4_1000":
        raise ValueError("only the validated DICT_4X4_1000 marker is supported")
    return SearchConfig(
        generator=load_generator_config(payload.get("generator")),
        marker_id=int(marker["id"]),
        confirm_frames=int(marker["confirm_frames"]),
        final_confirm_frames=int(marker["final_confirm_frames"]),
        final_height_ratio=bounded_number(
            marker["final_height_ratio"], "final_height_ratio", 0.05, 0.40
        ),
        final_center_tolerance=bounded_number(
            marker["final_center_tolerance"],
            "final_center_tolerance",
            0.02,
            0.12,
        ),
        search_speed=bounded_number(
            motion["search_speed_mps"], "search_speed_mps", 0.09, 0.90
        ),
        cautious_speed=bounded_number(
            motion["cautious_speed_mps"], "cautious_speed_mps", 0.09, 0.25
        ),
        reverse_speed=bounded_number(
            motion["reverse_speed_mps"], "reverse_speed_mps", -0.12, -0.085
        ),
        approach_speed=bounded_number(
            motion["approach_speed_mps"], "approach_speed_mps", 0.06, 0.12
        ),
        creep_speed=bounded_number(
            motion["creep_speed_mps"], "creep_speed_mps", 0.03, 0.08
        ),
        approach_min_forward_speed=bounded_number(
            motion["approach_min_forward_speed_mps"],
            "approach_min_forward_speed_mps",
            0.02,
            0.06,
        ),
        approach_forward_error_limit=bounded_number(
            motion["approach_forward_error_limit"],
            "approach_forward_error_limit",
            0.16,
            0.45,
        ),
        approach_yaw_gain=bounded_number(
            motion["approach_yaw_gain"], "approach_yaw_gain", 0.25, 1.00
        ),
        approach_max_yaw_rate=bounded_number(
            motion["approach_max_yaw_rate_rps"],
            "approach_max_yaw_rate_rps",
            0.10,
            0.35,
        ),
        approach_yaw_step_limit=bounded_number(
            motion["approach_yaw_step_limit_rps"],
            "approach_yaw_step_limit_rps",
            0.03,
            0.15,
        ),
        reacquire_yaw_rate=bounded_number(
            motion["reacquire_yaw_rate_rps"],
            "reacquire_yaw_rate_rps",
            0.08,
            0.25,
        ),
        reacquire_sweep_half_angle=bounded_number(
            motion["reacquire_sweep_half_angle_rad"],
            "reacquire_sweep_half_angle_rad",
            0.10,
            0.40,
        ),
        reacquire_direction_flip=bounded_number(
            motion["reacquire_direction_flip_sec"],
            "reacquire_direction_flip_sec",
            0.50,
            5.0,
        ),
        candidate_stop_deceleration=bounded_number(
            motion["candidate_stop_deceleration_mps2"],
            "candidate_stop_deceleration_mps2",
            0.10,
            1.00,
        ),
        candidate_stop_yaw_deceleration=bounded_number(
            motion["candidate_stop_yaw_deceleration_rps2"],
            "candidate_stop_yaw_deceleration_rps2",
            0.20,
            2.00,
        ),
        max_yaw_rate=bounded_number(
            motion["max_yaw_rate_rps"], "max_yaw_rate_rps", 0.20, 0.50
        ),
        emergency_clearance=bounded_number(
            motion["emergency_clearance_m"], "emergency_clearance_m", 0.30, 0.55
        ),
        turn_clearance=bounded_number(
            motion["turn_clearance_m"], "turn_clearance_m", 0.55, 0.85
        ),
        slow_clearance=bounded_number(
            motion["slow_clearance_m"], "slow_clearance_m", 0.85, 1.30
        ),
        wall_clearance=bounded_number(
            motion["wall_clearance_m"], "wall_clearance_m", 0.60, 1.30
        ),
        charging_min_lidar_clearance=bounded_number(
            motion["charging_clearance_m"],
            "charging_clearance_m",
            0.30,
            0.90,
        ),
        depth_topic=str(depth["topic"]),
        depth_width=bounded_integer(depth["width"], "depth.width", 64, 4096),
        depth_height=bounded_integer(depth["height"], "depth.height", 48, 4096),
        depth_horizontal_fov_deg=bounded_number(
            depth["horizontal_fov_deg"],
            "depth.horizontal_fov_deg",
            30.0,
            150.0,
        ),
        depth_maximum_frame_delta=bounded_number(
            depth["maximum_frame_delta_sec"],
            "depth.maximum_frame_delta_sec",
            0.10,
            1.50,
        ),
        depth_minimum_valid_samples=bounded_integer(
            depth["minimum_valid_samples"],
            "depth.minimum_valid_samples",
            4,
            1000,
        ),
        depth_minimum_valid_fraction=bounded_number(
            depth["minimum_valid_fraction"],
            "depth.minimum_valid_fraction",
            0.20,
            1.0,
        ),
        depth_minimum_range=bounded_number(
            depth["minimum_range_m"], "depth.minimum_range_m", 0.05, 1.0
        ),
        depth_maximum_range=bounded_number(
            depth["maximum_range_m"], "depth.maximum_range_m", 2.0, 30.0
        ),
        depth_minimum_charging_range=bounded_number(
            depth["minimum_charging_range_m"],
            "depth.minimum_charging_range_m",
            0.10,
            0.80,
        ),
        depth_maximum_charging_range=bounded_number(
            depth["maximum_charging_range_m"],
            "depth.maximum_charging_range_m",
            0.30,
            1.20,
        ),
        tracker_forward_min_depth=bounded_number(
            depth["tracker_forward_min_depth_m"],
            "depth.tracker_forward_min_depth_m",
            0.50,
            2.00,
        ),
        minimum_approach_travel=bounded_number(
            arrival["minimum_approach_travel_m"],
            "minimum_approach_travel_m",
            0.50,
            3.00,
        ),
        arrival_stop_hold=bounded_number(
            arrival["stop_hold_sec"], "stop_hold_sec", 1.0, 5.0
        ),
        minimum_standing_z=bounded_number(
            posture["minimum_standing_z_m"],
            "minimum_standing_z_m",
            0.30,
            0.45,
        ),
        maximum_standing_tilt=bounded_number(
            posture["maximum_standing_tilt_rad"],
            "maximum_standing_tilt_rad",
            0.15,
            0.50,
        ),
        posture_fault_delay=bounded_number(
            posture["fault_delay_sec"], "fault_delay_sec", 0.5, 3.0
        ),
        cosmos_settle_hold=bounded_number(
            posture["cosmos_settle_hold_sec"],
            "cosmos_settle_hold_sec",
            0.5,
            3.0,
        ),
        cosmos_settle_timeout=bounded_number(
            posture["cosmos_settle_timeout_sec"],
            "cosmos_settle_timeout_sec",
            2.0,
            12.0,
        ),
        candidate_lost_hold=bounded_number(
            marker["candidate_lost_hold_sec"],
            "candidate_lost_hold_sec",
            3.0,
            15.0,
        ),
        approach_observation_hold=bounded_number(
            marker["approach_observation_hold_sec"],
            "approach_observation_hold_sec",
            0.20,
            3.0,
        ),
        tracker_only_timeout=bounded_number(
            marker["tracker_only_timeout_sec"],
            "tracker_only_timeout_sec",
            1.0,
            10.0,
        ),
        marker_lost_timeout=bounded_number(
            arrival["marker_lost_timeout_sec"],
            "marker_lost_timeout_sec",
            8.0,
            45.0,
        ),
        search_timeout=bounded_number(
            payload["search_timeout_sec"], "search_timeout_sec", 60.0, 1800.0
        ),
        approach_timeout=bounded_number(
            payload["approach_timeout_sec"], "approach_timeout_sec", 20.0, 360.0
        ),
        camera_topic=str(payload["camera_topic"]),
        lidar_topic=str(payload["lidar_topic"]),
        velocity_topic=str(payload["velocity_topic"]),
    )


def posture_is_upright(pose: RobotPose, config: SearchConfig) -> bool:
    return (
        pose.z >= config.minimum_standing_z
        and abs(pose.roll) <= config.maximum_standing_tilt
        and abs(pose.pitch) <= config.maximum_standing_tilt
    )


def candidate_hold_active(
    last_seen_at: float, now: float, config: SearchConfig
) -> bool:
    return last_seen_at > 0.0 and now - last_seen_at <= config.candidate_lost_hold


def tracker_hold_active(
    last_exact_seen_at: float, now: float, config: SearchConfig
) -> bool:
    return (
        last_exact_seen_at > 0.0
        and now - last_exact_seen_at <= config.tracker_only_timeout
    )


def sensor_is_fresh(received_at: float, now: float, maximum_age: float = 1.0) -> bool:
    return received_at > 0.0 and 0.0 <= now - received_at <= maximum_age


def move_toward_zero(value: float, maximum_delta: float) -> float:
    if value > 0.0:
        return max(0.0, value - maximum_delta)
    if value < 0.0:
        return min(0.0, value + maximum_delta)
    return 0.0


def move_toward(value: float, target: float, maximum_delta: float) -> float:
    if value < target:
        return min(target, value + maximum_delta)
    if value > target:
        return max(target, value - maximum_delta)
    return target


def damped_marker_yaw_rate(
    horizontal_error: float,
    previous_yaw_rate: float,
    config: SearchConfig,
) -> float:
    # HouseWorld's Go2-W bridge moves a stationary target toward larger image x
    # under positive yaw. Image error is negative on the left, so invert it.
    desired = max(
        -config.approach_max_yaw_rate,
        min(
            config.approach_max_yaw_rate,
            -horizontal_error * config.approach_yaw_gain,
        ),
    )
    return move_toward(
        previous_yaw_rate, desired, config.approach_yaw_step_limit
    )


def damped_approach_velocity(
    marker: MarkerObservation,
    scan: LidarScan,
    previous_yaw_rate: float,
    config: SearchConfig,
) -> tuple[float, float]:
    error = marker.horizontal_error
    yaw_rate = damped_marker_yaw_rate(error, previous_yaw_rate, config)
    magnitude = abs(error)
    if magnitude >= config.approach_forward_error_limit:
        speed = 0.0
    else:
        alignment = 1.0 - magnitude / config.approach_forward_error_limit
        speed = config.approach_min_forward_speed + alignment * (
            config.approach_speed - config.approach_min_forward_speed
        )
    near_target = (
        marker.marker_height_ratio >= config.final_height_ratio * 0.65
        or scan.front_m <= 0.90
    )
    if near_target:
        speed = min(speed, config.creep_speed)
    if not marker.exact_id and scan.front_m <= 0.90:
        speed = 0.0
    if scan.front_m < config.emergency_clearance:
        speed = config.reverse_speed
    return speed, yaw_rate


def damped_reacquire_velocity(
    last_marker: MarkerObservation,
    scan: LidarScan,
    previous_yaw_rate: float,
    config: SearchConfig,
    scan_direction: float | None = None,
) -> tuple[float, float]:
    if scan.front_m < config.emergency_clearance:
        return reactive_velocity(scan, 0.0, config)
    if scan.front_m < config.turn_clearance:
        return reactive_velocity(scan, 0.0, config)
    if scan_direction is None:
        scan_direction = (
            1.0 if last_marker.horizontal_error <= 0.0 else -1.0
        )
    desired = math.copysign(config.reacquire_yaw_rate, scan_direction)
    yaw_rate = move_toward(
        previous_yaw_rate, desired, config.approach_yaw_step_limit
    )
    return 0.0, yaw_rate


def held_marker_velocity(
    last_marker: MarkerObservation,
    scan: LidarScan,
    previous_yaw_rate: float,
    config: SearchConfig,
) -> tuple[float, float]:
    speed, yaw_rate = damped_approach_velocity(
        last_marker, scan, previous_yaw_rate, config
    )
    if speed > 0.0:
        speed = 0.0
    return speed, yaw_rate


def depth_guarded_approach_speed(
    marker: MarkerObservation,
    marker_depth_m: float | None,
    last_exact_depth_m: float | None,
    requested_speed: float,
    config: SearchConfig,
) -> float:
    if requested_speed <= 0.0:
        return requested_speed
    if not marker.exact_id:
        if (
            last_exact_depth_m is None
            or last_exact_depth_m <= config.tracker_forward_min_depth
        ):
            return 0.0
        # Tracker geometry may bridge a few blurry frames, but only a freshly
        # decoded ID/depth observation authorizes the aggressive far approach.
        return min(requested_speed, config.approach_speed)
    if marker_depth_m is not None:
        if marker_depth_m <= config.depth_maximum_charging_range:
            return 0.0
        near_boundary = config.depth_maximum_charging_range * 2.0
        if marker_depth_m <= near_boundary:
            return min(requested_speed, config.approach_speed)
        medium_boundary = max(1.50, config.tracker_forward_min_depth + 0.50)
        if marker_depth_m <= medium_boundary:
            return min(requested_speed, config.cautious_speed)
        return requested_speed
    if marker.marker_height_ratio >= config.final_height_ratio:
        return 0.0
    return requested_speed


def wrapped_angle_delta(value: float, reference: float) -> float:
    delta = value - reference
    return math.atan2(math.sin(delta), math.cos(delta))


def bounded_reacquire_direction(
    yaw: float,
    center_yaw: float,
    current_direction: float,
    half_angle: float,
) -> float:
    offset = wrapped_angle_delta(yaw, center_yaw)
    if offset >= half_angle:
        return -1.0
    if offset <= -half_angle:
        return 1.0
    return 1.0 if current_direction >= 0.0 else -1.0


def candidate_stop_velocity(
    linear_x: float,
    angular_z: float,
    elapsed_sec: float,
    scan: LidarScan,
    config: SearchConfig,
) -> tuple[float, float]:
    if scan.front_m < config.turn_clearance:
        return 0.0, 0.0
    elapsed_sec = max(0.0, elapsed_sec)
    return (
        move_toward_zero(
            linear_x, config.candidate_stop_deceleration * elapsed_sec
        ),
        move_toward_zero(
            angular_z, config.candidate_stop_yaw_deceleration * elapsed_sec
        ),
    )


def cosmos_pose_is_stable(pose: RobotPose, config: SearchConfig) -> bool:
    return (
        posture_is_upright(pose, config)
        and pose.linear_speed <= COSMOS_SETTLED_LINEAR_SPEED_MPS
        and pose.yaw_rate <= COSMOS_SETTLED_YAW_RATE_RPS
    )


def charge_ready(
    marker: MarkerObservation,
    marker_depth_m: float | None,
    scan: LidarScan,
    approach_travel_m: float,
    config: SearchConfig,
) -> bool:
    return (
        marker.exact_id
        and marker.marker_id == config.marker_id
        and marker.marker_height_ratio >= config.final_height_ratio
        and abs(marker.horizontal_error) <= config.final_center_tolerance
        and marker_depth_m is not None
        and config.depth_minimum_charging_range <= marker_depth_m
        and marker_depth_m <= config.depth_maximum_charging_range
        # The low visual dock is absent from the collision point cloud. RGB-D
        # supplies dock range while LiDAR remains an independent safety veto.
        and scan.front_m >= config.charging_min_lidar_clearance
        and approach_travel_m >= config.minimum_approach_travel
    )


def quaternion_yaw_roll_pitch(message: Odometry) -> tuple[float, float, float]:
    orientation = message.pose.pose.orientation
    w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, roll, pitch


def marker_observation_from_points(
    points: np.ndarray,
    marker_id: int,
    image_width: int,
    image_height: int,
    *,
    verification: str = "aruco",
) -> MarkerObservation:
    normalized = np.asarray(points, dtype=float).reshape(4, 2)
    center_x, center_y = normalized.mean(axis=0)
    height = float(
        max(
            np.linalg.norm(normalized[0] - normalized[3]),
            np.linalg.norm(normalized[1] - normalized[2]),
        )
    )
    return MarkerObservation(
        marker_id=marker_id,
        center_x=float(center_x),
        center_y=float(center_y),
        width=image_width,
        height=image_height,
        marker_height_ratio=height / float(image_height),
        corners=tuple((float(x), float(y)) for x, y in normalized),
        verification=verification,
    )


def order_quadrilateral(points: np.ndarray) -> np.ndarray | None:
    candidate = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = candidate.sum(axis=1)
    differences = np.diff(candidate, axis=1).reshape(-1)
    indices = (
        int(np.argmin(sums)),
        int(np.argmin(differences)),
        int(np.argmax(sums)),
        int(np.argmax(differences)),
    )
    if len(set(indices)) != 4:
        return None
    return candidate[list(indices)]


def hinted_marker_quadrilaterals(
    image: np.ndarray,
    hint: MarkerObservation,
) -> list[np.ndarray]:
    hint_points = np.asarray(hint.corners, dtype=np.float32).reshape(4, 2)
    minimum_x, minimum_y = hint_points.min(axis=0)
    maximum_x, maximum_y = hint_points.max(axis=0)
    hint_width = max(8.0, float(maximum_x - minimum_x))
    hint_height = max(8.0, float(maximum_y - minimum_y))
    hint_area = hint_width * hint_height
    padding = max(hint_width, hint_height) * 0.45
    left = max(0, int(math.floor(minimum_x - padding)))
    top = max(0, int(math.floor(minimum_y - padding)))
    right = min(image.shape[1], int(math.ceil(maximum_x + padding)))
    bottom = min(image.shape[0], int(math.ceil(maximum_y + padding)))
    if right - left < 16 or bottom - top < 16:
        return []

    region = image[top:bottom, left:right]
    gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
    _threshold, otsu = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
    )
    adaptive = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31,
        5,
    )
    edges = cv2.Canny(gray, 45, 135)
    edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)

    hint_center = hint_points.mean(axis=0)
    maximum_center_distance = max(hint_width, hint_height) * 0.55
    minimum_area = max(80.0, hint_area * 0.10)
    maximum_area = min(float(region.shape[0] * region.shape[1]), hint_area * 4.0)
    scored: list[tuple[float, np.ndarray]] = []
    seen: set[tuple[int, int, int]] = set()
    for mask in (otsu, adaptive, edges):
        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            area = abs(float(cv2.contourArea(contour)))
            if not minimum_area <= area <= maximum_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter < 32.0:
                continue
            for epsilon_rate in (0.015, 0.025, 0.04, 0.06):
                approximation = cv2.approxPolyDP(
                    contour, perimeter * epsilon_rate, True
                )
                if len(approximation) != 4 or not cv2.isContourConvex(approximation):
                    continue
                ordered = order_quadrilateral(
                    approximation.reshape(4, 2).astype(np.float32)
                )
                if ordered is None:
                    continue
                ordered[:, 0] += left
                ordered[:, 1] += top
                side_lengths = [
                    float(np.linalg.norm(ordered[index] - ordered[(index + 1) % 4]))
                    for index in range(4)
                ]
                if min(side_lengths) < 8.0:
                    continue
                center = ordered.mean(axis=0)
                center_distance = float(np.linalg.norm(center - hint_center))
                if center_distance > maximum_center_distance:
                    continue
                candidate_area = abs(float(cv2.contourArea(ordered)))
                if candidate_area <= 0.0:
                    continue
                key = (
                    round(float(center[0]) / 6.0),
                    round(float(center[1]) / 6.0),
                    round(math.log(candidate_area) * 4.0),
                )
                if key in seen:
                    continue
                seen.add(key)
                area_error = abs(math.log(candidate_area / hint_area))
                score = center_distance / maximum_center_distance + area_error
                scored.append((score, ordered.copy()))
                break
    scored.sort(key=lambda item: item[0])
    return [points for _score, points in scored[:32]]


def detect_target_cells_from_hint(
    image: np.ndarray,
    marker_id: int,
    hint: MarkerObservation,
    dictionary: Any,
) -> MarkerObservation | None:
    """Strictly decode one known marker inside an already-confirmed track.

    OpenCV can reject a valid marker when its black border merges with the
    HouseWorld charging pedestal.  This fallback rectifies only quadrilaterals
    near the tracker hint, samples the 6x6 ArUco cells robustly, and accepts a
    candidate only when one threshold separates every black/white cell of the
    requested dictionary code with zero bit errors.
    """

    if hint.marker_id != marker_id or not hint.corners:
        return None
    if marker_id < 0 or marker_id >= int(dictionary.bytesList.shape[0]):
        return None
    expected = cv2.aruco.Dictionary_getBitsFromByteList(
        dictionary.bytesList[marker_id : marker_id + 1],
        4,
    ).astype(np.uint8)
    if expected.shape != (4, 4):
        return None

    hint_points = np.asarray(hint.corners, dtype=np.float32).reshape(4, 2)
    ordered_hint = order_quadrilateral(hint_points)
    quadrilaterals = hinted_marker_quadrilaterals(image, hint)
    if ordered_hint is not None:
        quadrilaterals.append(ordered_hint)

    patch_size = 96
    cell_size = patch_size // 6
    cell_margin = cell_size // 4
    destination = np.asarray(
        (
            (0.0, 0.0),
            (patch_size - 1.0, 0.0),
            (patch_size - 1.0, patch_size - 1.0),
            (0.0, patch_size - 1.0),
        ),
        dtype=np.float32,
    )
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hint_height = max(8.0, hint.marker_height_ratio * hint.height)
    best: tuple[float, MarkerObservation] | None = None
    for quadrilateral in quadrilaterals:
        ordered = order_quadrilateral(quadrilateral)
        if ordered is None:
            continue
        side_lengths = [
            float(np.linalg.norm(ordered[index] - ordered[(index + 1) % 4]))
            for index in range(4)
        ]
        if min(side_lengths) < 8.0 or max(side_lengths) / min(side_lengths) > 3.0:
            continue
        transform = cv2.getPerspectiveTransform(ordered, destination)
        rectified = cv2.warpPerspective(
            grayscale,
            transform,
            (patch_size, patch_size),
            flags=cv2.INTER_CUBIC,
            borderMode=cv2.BORDER_REPLICATE,
        )
        cell_values = np.zeros((6, 6), dtype=np.float32)
        for row in range(6):
            for column in range(6):
                top = row * cell_size + cell_margin
                bottom = (row + 1) * cell_size - cell_margin
                left = column * cell_size + cell_margin
                right = (column + 1) * cell_size - cell_margin
                cell_values[row, column] = float(
                    np.median(rectified[top:bottom, left:right])
                )
        border_values = np.concatenate(
            (
                cell_values[0, :],
                cell_values[-1, :],
                cell_values[1:-1, 0],
                cell_values[1:-1, -1],
            )
        )
        interior = cell_values[1:5, 1:5]
        for rotation in range(4):
            rotated_expected = np.rot90(expected, rotation)
            black_values = np.concatenate(
                (border_values, interior[rotated_expected == 0])
            )
            white_values = interior[rotated_expected == 1]
            if white_values.size == 0:
                continue
            # A positive margin means a single threshold reproduces all 36
            # marker cells exactly; 18 gray levels rejects weak/noisy texture.
            separation = float(np.min(white_values) - np.max(black_values))
            if separation < 18.0:
                continue
            observation = marker_observation_from_points(
                ordered,
                marker_id,
                int(image.shape[1]),
                int(image.shape[0]),
                verification="target_cell_exact",
            )
            center_distance = math.hypot(
                observation.center_x - hint.center_x,
                observation.center_y - hint.center_y,
            )
            observed_height = observation.marker_height_ratio * observation.height
            if center_distance > hint_height * 0.65:
                continue
            if not 0.40 <= observed_height / hint_height <= 2.25:
                continue
            if best is None or separation > best[0]:
                best = (separation, observation)
    return best[1] if best is not None else None


def detect_marker_from_hint(
    image: np.ndarray,
    marker_id: int,
    hint: MarkerObservation,
    dictionary: Any,
) -> MarkerObservation | None:
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshWinSizeMax = 101
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.005
    parameters.minCornerDistanceRate = 0.01
    parameters.minDistanceToBorder = 1
    parameters.polygonalApproxAccuracyRate = 0.08
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 3
    parameters.cornerRefinementMaxIterations = 30
    parameters.errorCorrectionRate = 1.0

    patch_size = 224
    padding = 36
    destination = np.asarray(
        (
            (0.0, 0.0),
            (patch_size - 1.0, 0.0),
            (patch_size - 1.0, patch_size - 1.0),
            (0.0, patch_size - 1.0),
        ),
        dtype=np.float32,
    )
    hint_height = max(8.0, hint.marker_height_ratio * hint.height)
    for quadrilateral in hinted_marker_quadrilaterals(image, hint):
        transform = cv2.getPerspectiveTransform(quadrilateral, destination)
        rectified = cv2.warpPerspective(
            image,
            transform,
            (patch_size, patch_size),
            flags=cv2.INTER_CUBIC,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(255, 255, 255),
        )
        rectified = cv2.copyMakeBorder(
            rectified,
            padding,
            padding,
            padding,
            padding,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        _threshold, otsu = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
        )
        for candidate in (rectified, gray, clahe, otsu):
            try:
                corners, ids, _rejected = cv2.aruco.detectMarkers(
                    candidate, dictionary, parameters=parameters
                )
            except cv2.error:
                continue
            if ids is None:
                continue
            for marker_corners, detected_id in zip(corners, ids.reshape(-1)):
                if int(detected_id) != marker_id:
                    continue
                rectified_points = marker_corners.reshape(4, 2).astype(np.float32)
                rectified_points -= float(padding)
                inverse = np.linalg.inv(transform)
                original_points = cv2.perspectiveTransform(
                    rectified_points.reshape(1, 4, 2), inverse
                ).reshape(4, 2)
                observation = marker_observation_from_points(
                    original_points,
                    marker_id,
                    int(image.shape[1]),
                    int(image.shape[0]),
                )
                center_distance = math.hypot(
                    observation.center_x - hint.center_x,
                    observation.center_y - hint.center_y,
                )
                observed_height = observation.marker_height_ratio * observation.height
                if center_distance > hint_height * 0.75:
                    continue
                if not 0.35 <= observed_height / hint_height <= 2.50:
                    continue
                return observation
    return detect_target_cells_from_hint(
        image,
        marker_id,
        hint,
        dictionary,
    )


def detect_marker(
    image_bytes: bytes,
    marker_id: int,
    hint: MarkerObservation | None = None,
) -> MarkerObservation | None:
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        return None
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.01
    parameters.minCornerDistanceRate = 0.02
    parameters.minDistanceToBorder = 1
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.cornerRefinementWinSize = 3
    parameters.cornerRefinementMaxIterations = 20

    def find(candidate: np.ndarray, scale: float, offset_y: float) -> MarkerObservation | None:
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            candidate, dictionary, parameters=parameters
        )
        if ids is None:
            return None
        for marker_corners, detected_id in zip(corners, ids.reshape(-1)):
            if int(detected_id) != marker_id:
                continue
            points = marker_corners.reshape(4, 2).astype(float)
            points[:, 0] /= scale
            points[:, 1] = points[:, 1] / scale + offset_y
            return marker_observation_from_points(
                points,
                marker_id,
                int(image.shape[1]),
                int(image.shape[0]),
            )
        return None

    observation = find(image, 1.0, 0.0)
    if observation is not None:
        return observation

    # HouseWorld's close marker can sit on a nearly black charging pedestal.
    # OpenCV's local adaptive threshold then merges the black ArUco border with
    # the pedestal even though the white code cells remain distinct. Exact
    # dictionary decoding on a few conservative dark-scene thresholds recovers
    # ID 560 without accepting tracker-only geometry as an ID observation.
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    for threshold_value in (40, 50, 60):
        _threshold, binary = cv2.threshold(
            grayscale,
            threshold_value,
            255,
            cv2.THRESH_BINARY,
        )
        observation = find(binary, 1.0, 0.0)
        if observation is not None:
            return observation

    # JPEG artifacts affect a 12-20 px floor marker disproportionately. Two
    # distinct scales recover different compression phases without weakening
    # the exact dictionary/ID check.
    crop_y = int(image.shape[0] * 0.38)
    lower = image[crop_y:, :]
    for scale in (2.0, 4.0):
        enlarged = cv2.resize(
            lower,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC,
        )
        observation = find(enlarged, scale, float(crop_y))
        if observation is not None:
            return observation
    if hint is not None and hint.corners:
        return detect_marker_from_hint(image, marker_id, hint, dictionary)
    return None


def create_marker_tracker(
    image_bytes: bytes,
    marker: MarkerObservation,
) -> VisualMarkerTracker | None:
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None or not marker.corners:
        return None
    if image.shape[1] != marker.width or image.shape[0] != marker.height:
        return None
    points = np.asarray(marker.corners, dtype=float)
    minimum_x, minimum_y = points.min(axis=0)
    maximum_x, maximum_y = points.max(axis=0)
    marker_width = max(4.0, float(maximum_x - minimum_x))
    marker_height = max(4.0, float(maximum_y - minimum_y))
    padding = max(marker_width, marker_height) * 0.55
    left = max(0, int(math.floor(minimum_x - padding)))
    top = max(0, int(math.floor(minimum_y - padding)))
    right = min(image.shape[1], int(math.ceil(maximum_x + padding)))
    bottom = min(image.shape[0], int(math.ceil(maximum_y + padding)))
    track_width = right - left
    track_height = bottom - top
    if track_width < 8 or track_height < 8:
        return None
    try:
        tracker = cv2.TrackerCSRT_create()
        tracker.init(image, (left, top, track_width, track_height))
    except (AttributeError, cv2.error):
        return None
    return VisualMarkerTracker(
        tracker=tracker,
        width=image.shape[1],
        height=image.shape[0],
        marker_id=marker.marker_id,
        center_x_fraction=(marker.center_x - left) / track_width,
        center_y_fraction=(marker.center_y - top) / track_height,
        marker_width_fraction=marker_width / track_width,
        marker_height_fraction=marker_height / track_height,
    )


def update_marker_tracker(
    state: VisualMarkerTracker,
    image_bytes: bytes,
) -> MarkerObservation | None:
    encoded = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        return None
    if image.shape[1] != state.width or image.shape[0] != state.height:
        return None
    try:
        tracked, box = state.tracker.update(image)
    except cv2.error:
        return None
    if not tracked:
        return None
    left, top, width, height = (float(value) for value in box)
    if not all(math.isfinite(value) for value in (left, top, width, height)):
        return None
    if width < 8.0 or height < 8.0:
        return None
    center_x = left + width * state.center_x_fraction
    center_y = top + height * state.center_y_fraction
    marker_width = width * state.marker_width_fraction
    marker_height = height * state.marker_height_fraction
    if not (0.0 <= center_x < state.width and 0.0 <= center_y < state.height):
        return None
    ratio = marker_height / state.height
    if not 0.005 <= ratio <= 0.50:
        return None
    half_width = marker_width * 0.5
    half_height = marker_height * 0.5
    corners = (
        (center_x - half_width, center_y - half_height),
        (center_x + half_width, center_y - half_height),
        (center_x + half_width, center_y + half_height),
        (center_x - half_width, center_y + half_height),
    )
    if any(
        x < 0.0 or x >= state.width or y < 0.0 or y >= state.height
        for x, y in corners
    ):
        return None
    return MarkerObservation(
        marker_id=state.marker_id,
        center_x=center_x,
        center_y=center_y,
        width=state.width,
        height=state.height,
        marker_height_ratio=ratio,
        corners=corners,
        exact_id=False,
    )


def decode_depth_frame(
    message: DepthImage,
    received_at: float,
    config: SearchConfig,
) -> DepthFrame:
    if message.encoding != "32FC1":
        raise ValueError(f"unsupported depth encoding: {message.encoding!r}")
    expected_bytes = config.depth_width * config.depth_height * 4
    if len(message.data) != expected_bytes:
        raise ValueError(
            f"depth payload is {len(message.data)} bytes, expected {expected_bytes}"
        )
    if message.width not in (0, config.depth_width):
        raise ValueError(f"unexpected depth width: {message.width}")
    if message.height not in (0, config.depth_height):
        raise ValueError(f"unexpected depth height: {message.height}")
    expected_step = config.depth_width * 4
    if message.step not in (0, expected_step):
        raise ValueError(f"unexpected depth step: {message.step}")
    dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
    values = np.frombuffer(bytes(message.data), dtype=dtype).astype(
        np.float32, copy=False
    )
    return DepthFrame(
        values=values.reshape((config.depth_height, config.depth_width)),
        received_at=received_at,
    )


def estimate_marker_depth(
    marker: MarkerObservation,
    depth: DepthFrame | None,
    image_received_at: float,
    config: SearchConfig,
) -> float | None:
    if depth is None or not marker.corners:
        return None
    if abs(depth.received_at - image_received_at) > config.depth_maximum_frame_delta:
        return None
    half_fov = math.radians(config.depth_horizontal_fov_deg) * 0.5
    rgb_fx = marker.width * 0.5 / math.tan(half_fov)
    depth_fx = config.depth_width * 0.5 / math.tan(half_fov)
    rgb_fy = rgb_fx
    depth_fy = depth_fx
    rgb_cx = marker.width * 0.5
    rgb_cy = marker.height * 0.5
    depth_cx = config.depth_width * 0.5
    depth_cy = config.depth_height * 0.5
    polygon = np.asarray(
        [
            (
                depth_cx + (x - rgb_cx) * depth_fx / rgb_fx,
                depth_cy + (y - rgb_cy) * depth_fy / rgb_fy,
            )
            for x, y in marker.corners
        ],
        dtype=np.float32,
    )
    centroid = polygon.mean(axis=0)
    polygon = centroid + (polygon - centroid) * 0.80
    polygon[:, 0] = np.clip(polygon[:, 0], 0, config.depth_width - 1)
    polygon[:, 1] = np.clip(polygon[:, 1], 0, config.depth_height - 1)
    polygon_pixels = np.rint(polygon).astype(np.int32)
    mask = np.zeros((config.depth_height, config.depth_width), dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon_pixels, 1)
    samples = depth.values[mask.astype(bool)]
    if samples.size == 0:
        return None
    valid = samples[
        np.isfinite(samples)
        & (samples >= config.depth_minimum_range)
        & (samples <= config.depth_maximum_range)
    ]
    if valid.size < config.depth_minimum_valid_samples:
        return None
    if valid.size / samples.size < config.depth_minimum_valid_fraction:
        return None

    lower_quartile = float(np.quantile(valid, 0.25))
    cluster = valid[
        (valid >= lower_quartile - 0.05)
        & (valid <= lower_quartile + 0.15)
    ]
    minimum_cluster = max(
        config.depth_minimum_valid_samples,
        int(math.ceil(valid.size * 0.30)),
    )
    if cluster.size < minimum_cluster:
        return None
    return float(np.median(cluster))


def compute_lidar_scan(points: np.ndarray, received_at: float) -> LidarScan:
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("LiDAR points must be an Nx3 array")
    finite = np.isfinite(points[:, :3]).all(axis=1)
    planar = np.hypot(points[:, 0], points[:, 1])
    keep = (
        finite
        & (points[:, 2] >= -0.05)
        & (points[:, 2] <= 1.20)
        & (planar >= 0.20)
        & (planar <= 4.0)
    )
    if not np.any(keep):
        clearances = (0.0,) * len(BIN_ANGLES_DEG)
        return LidarScan(
            tuple(math.radians(value) for value in BIN_ANGLES_DEG),
            clearances,
            0.0,
            0.0,
            0.0,
            received_at,
        )
    angles = np.arctan2(points[keep, 1], points[keep, 0])
    ranges = planar[keep]

    def sector(center_deg: float, half_width_deg: float) -> float:
        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)
        selected = ranges[np.abs(np.arctan2(np.sin(angles - center), np.cos(angles - center))) <= half_width]
        if selected.size == 0:
            return 4.0
        return float(np.quantile(selected, 0.08))

    clearances = tuple(sector(angle, 12.5) for angle in BIN_ANGLES_DEG)
    return LidarScan(
        angles_rad=tuple(math.radians(value) for value in BIN_ANGLES_DEG),
        clearances_m=clearances,
        front_m=sector(0.0, 20.0),
        left_m=sector(55.0, 30.0),
        right_m=sector(-55.0, 30.0),
        received_at=received_at,
    )


def select_exploration_heading(
    scan: LidarScan,
    pose: RobotPose,
    visits: dict[tuple[int, int], int],
    projection_m: float = 1.25,
) -> float:
    scored: list[tuple[float, float]] = []
    for angle, clearance in zip(scan.angles_rad, scan.clearances_m):
        travel = min(projection_m, max(0.0, clearance - 0.35))
        target_yaw = pose.yaw + angle
        cell = (
            round((pose.x + travel * math.cos(target_yaw)) / 0.75),
            round((pose.y + travel * math.sin(target_yaw)) / 0.75),
        )
        novelty_penalty = 0.80 * visits.get(cell, 0)
        forward_penalty = 0.10 * abs(angle)
        score = min(clearance, 3.0) - novelty_penalty - forward_penalty
        if clearance < 0.50:
            score -= 5.0
        scored.append((score, angle))
    return max(scored)[1]


def reactive_velocity(
    scan: LidarScan,
    desired_angle: float,
    config: SearchConfig,
    *,
    escaping: bool = False,
) -> tuple[float, float]:
    turn_sign = 1.0 if scan.left_m >= scan.right_m else -1.0
    if escaping or scan.front_m < config.emergency_clearance:
        return config.reverse_speed, turn_sign * config.max_yaw_rate
    if scan.front_m < config.turn_clearance:
        return config.reverse_speed * 0.65, turn_sign * config.max_yaw_rate
    if scan.left_m < 0.70:
        return config.cautious_speed * 0.60, -config.max_yaw_rate
    if scan.right_m < 0.70:
        return config.cautious_speed * 0.60, config.max_yaw_rate
    yaw_rate = max(
        -config.max_yaw_rate,
        min(config.max_yaw_rate, desired_angle * 0.65),
    )
    if scan.left_m < config.wall_clearance:
        yaw_rate -= min(
            0.40, (config.wall_clearance - scan.left_m) * 0.90
        )
    if scan.right_m < config.wall_clearance:
        yaw_rate += min(
            0.40, (config.wall_clearance - scan.right_m) * 0.90
        )
    yaw_rate = max(-config.max_yaw_rate, min(config.max_yaw_rate, yaw_rate))
    speed = (
        config.cautious_speed
        if scan.front_m < config.slow_clearance or abs(desired_angle) > 0.65
        else config.search_speed
    )
    return speed, yaw_rate


def validate_generator_server_info(
    payload: object, config: GeneratorConfig
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise GeneratorError("Generator /info response must be an object")
    expected_values = {
        "raw_action_dim": config.raw_action_dim,
        "fps": config.fps,
        "action_chunk_size": config.action_chunk_size,
    }
    for field, expected in expected_values.items():
        if payload.get(field) != expected:
            raise GeneratorError(
                f"Generator /info {field}={payload.get(field)!r}, expected {expected!r}"
            )
    if payload.get("reasoner") is not True:
        raise GeneratorError("Generator server does not expose the shared /reason head")
    if payload.get("request_seed_supported") is not True:
        raise GeneratorError("Generator server does not support per-request seeds")
    checkpoint = payload.get("checkpoint")
    if not isinstance(checkpoint, str) or not checkpoint:
        raise GeneratorError("Generator /info checkpoint is missing")
    return payload


def validate_generator_action_payload(
    payload: object, config: GeneratorConfig
) -> np.ndarray:
    if not isinstance(payload, dict):
        raise GeneratorError("Generator response must be an object")
    if payload.get("error"):
        raise GeneratorError(f"Generator returned an error: {payload['error']}")
    actions = payload.get("actions")
    if not isinstance(actions, list) or len(actions) != 1:
        raise GeneratorError("Generator response must contain exactly one action chunk")
    chunk = actions[0]
    if not isinstance(chunk, list) or len(chunk) != config.action_chunk_size:
        raise GeneratorError(
            f"Generator action chunk must have {config.action_chunk_size} steps"
        )
    for row in chunk:
        if not isinstance(row, list) or len(row) != config.raw_action_dim:
            raise GeneratorError(
                f"Generator actions must have shape "
                f"({config.action_chunk_size}, {config.raw_action_dim})"
            )
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in row
        ):
            raise GeneratorError("Generator actions must contain only JSON numbers")
    matrix = np.asarray(chunk, dtype=np.float64)
    if matrix.shape != (config.action_chunk_size, config.raw_action_dim):
        raise GeneratorError("Generator action shape changed during decoding")
    if not np.isfinite(matrix).all():
        raise GeneratorError("Generator actions contain NaN or infinity")
    return matrix


def rot6d_to_rotation_matrix(rotation: object) -> np.ndarray:
    values = np.asarray(rotation, dtype=np.float64)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise GeneratorError("rot6d must contain six finite values")
    column_0 = values[:3]
    column_1 = values[3:]
    if (
        np.linalg.norm(column_0) < 1e-8
        or np.linalg.norm(column_1) < 1e-8
        or np.linalg.norm(np.cross(column_0, column_1)) < 1e-8
    ):
        raise GeneratorError("rot6d columns are degenerate")
    approximate = np.column_stack(
        (column_0, column_1, np.cross(column_0, column_1))
    )
    left, _singular_values, right_transpose = np.linalg.svd(approximate)
    rotation_matrix = left @ right_transpose
    if np.linalg.det(rotation_matrix) < 0.0:
        left[:, -1] *= -1.0
        rotation_matrix = left @ right_transpose
    return rotation_matrix


def av_pose_action_to_twist(
    action: object, config: GeneratorConfig
) -> tuple[float, float]:
    """Adapt one AV optical-camera relative pose to an experimental base Twist.

    AV uses optical axes +X right, +Y down, +Z forward. The HouseWorld camera
    faces along base +X, so this proxy rotates those axes into +X forward,
    +Y left, +Z up. No Go2-W policy weights or calibrated camera translation
    are available; the configured scales intentionally cap vehicle-sized deltas.
    """

    values = np.asarray(action, dtype=np.float64)
    if values.shape != (config.raw_action_dim,) or not np.isfinite(values).all():
        raise GeneratorError(
            f"one Generator action must have shape ({config.raw_action_dim},)"
        )
    camera_rotation = rot6d_to_rotation_matrix(values[3:9])
    base_from_camera = np.asarray(
        (
            (0.0, 0.0, 1.0),
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
        ),
        dtype=np.float64,
    )
    base_translation = base_from_camera @ values[:3]
    base_rotation = (
        base_from_camera @ camera_rotation @ base_from_camera.transpose()
    )
    relative_yaw = math.atan2(base_rotation[1, 0], base_rotation[0, 0])
    path_yaw = math.atan2(
        base_translation[1], max(0.05, abs(float(base_translation[0])))
    )
    linear_x = (
        float(base_translation[0]) * config.translation_scale * config.fps
    )
    angular_z = (
        relative_yaw * config.rotation_scale
        + path_yaw * config.lateral_yaw_gain
    ) * config.fps
    if not math.isfinite(linear_x) or not math.isfinite(angular_z):
        raise GeneratorError("adapted Generator command is not finite")
    return linear_x, angular_z


def adapt_generator_action_chunk(
    action_chunk: np.ndarray,
    config: GeneratorConfig,
) -> tuple[float, float]:
    """Robustly adapt the Generator's short relative-pose prefix to one Twist."""

    expected_shape = (config.action_chunk_size, config.raw_action_dim)
    if action_chunk.shape != expected_shape:
        raise GeneratorError(
            f"Generator action chunk must have shape {expected_shape}"
        )
    # Rank candidates on the immediate 0.3 s intent. A longer selected prefix
    # is executed stepwise with fresh safety checks below.
    selection_steps = min(3, config.execute_prefix_steps)
    prefix_commands = [
        av_pose_action_to_twist(action, config)
        for action in action_chunk[:selection_steps]
    ]
    return (
        float(np.median([command[0] for command in prefix_commands])),
        float(np.median([command[1] for command in prefix_commands])),
    )


def limit_generator_twist(
    linear_x: float,
    angular_z: float,
    previous: tuple[float, float],
    stage: str,
    config: GeneratorConfig,
) -> tuple[float, float]:
    maximum_linear = (
        config.max_approach_speed_mps
        if stage in {"approach", "approach_hold", "reacquire"}
        else config.max_linear_speed_mps
    )
    target_linear = max(0.0, min(maximum_linear, float(linear_x)))
    target_yaw = max(
        -config.max_yaw_rate_rps,
        min(config.max_yaw_rate_rps, float(angular_z)),
    )
    previous_linear = max(0.0, min(maximum_linear, float(previous[0])))
    previous_yaw = max(
        -config.max_yaw_rate_rps,
        min(config.max_yaw_rate_rps, float(previous[1])),
    )
    if target_yaw * previous_yaw < 0.0:
        previous_yaw = 0.0
    limited_linear = (
        move_toward(previous_linear, target_linear, config.max_linear_step_mps)
        if target_linear > 0.0
        else 0.0
    )
    limited_yaw = (
        move_toward(previous_yaw, target_yaw, config.max_yaw_step_rps)
        if abs(target_yaw) > 1e-9
        else 0.0
    )
    return limited_linear, limited_yaw


def shield_generator_action(
    linear_x: float,
    angular_z: float,
    *,
    stage: str,
    scan: LidarScan,
    pose: RobotPose,
    now: float,
    config: SearchConfig,
    marker: MarkerObservation | None = None,
    marker_depth_m: float | None = None,
    last_exact_depth_m: float | None = None,
) -> tuple[float, float, tuple[str, ...]]:
    """Only reduce or veto a Generator command; never create motion."""

    reasons: list[str] = []
    if (
        not math.isfinite(linear_x)
        or not math.isfinite(angular_z)
        or not sensor_is_fresh(scan.received_at, now)
        or not sensor_is_fresh(pose.received_at, now)
        or not posture_is_upright(pose, config)
    ):
        return 0.0, 0.0, ("stale_or_unsafe_observation",)
    linear_x = max(0.0, float(linear_x))
    angular_z = float(angular_z)
    if stage in {"approach_hold", "reacquire"} and linear_x > 0.0:
        linear_x = 0.0
        reasons.append(f"{stage}_translation_veto")
    minimum_front = (
        config.emergency_clearance
        if stage in {"approach", "approach_hold", "reacquire"}
        else config.turn_clearance
    )
    if scan.front_m < minimum_front and linear_x > 0.0:
        linear_x = 0.0
        reasons.append("front_clearance_veto")
    elif stage == "search" and scan.front_m < config.slow_clearance:
        reduced = min(linear_x, config.cautious_speed)
        if reduced < linear_x:
            linear_x = reduced
            reasons.append("front_clearance_reduction")
    if scan.left_m < config.turn_clearance and angular_z > 0.0:
        angular_z = 0.0
        reasons.append("left_turn_clearance_veto")
    if scan.right_m < config.turn_clearance and angular_z < 0.0:
        angular_z = 0.0
        reasons.append("right_turn_clearance_veto")
    if min(scan.left_m, scan.right_m) < 0.70 and linear_x > config.cautious_speed:
        linear_x = config.cautious_speed
        reasons.append("side_wall_speed_reduction")
    if stage == "approach" and marker is not None:
        error = marker.horizontal_error
        desired_yaw_sign = -error
        if (
            abs(error) > config.final_center_tolerance
            and angular_z * desired_yaw_sign < 0.0
        ):
            angular_z = 0.0
            reasons.append("marker_divergence_yaw_veto")
        if (
            abs(error) >= config.approach_forward_error_limit
            and linear_x > 0.0
        ):
            linear_x = 0.0
            reasons.append("marker_alignment_forward_veto")
        elif (
            abs(error) > config.final_center_tolerance * 2.0
            and linear_x > config.cautious_speed
        ):
            linear_x = config.cautious_speed
            reasons.append("marker_alignment_speed_reduction")
        guarded = depth_guarded_approach_speed(
            marker,
            marker_depth_m,
            last_exact_depth_m,
            linear_x,
            config,
        )
        if guarded < linear_x:
            linear_x = guarded
            reasons.append("rgbd_forward_veto")
    return linear_x, angular_z, tuple(reasons)


def score_generator_candidate(
    linear_x: float,
    angular_z: float,
    *,
    stage: str,
    scan: LidarScan,
    config: SearchConfig,
    marker: MarkerObservation | None = None,
    preferred_yaw_sign: float | None = None,
) -> float:
    """Rank already-generated, already-shielded commands without creating motion."""

    if abs(linear_x) <= 1e-9 and abs(angular_z) <= 1e-9:
        return -math.inf
    yaw_fraction = min(
        1.0, abs(angular_z) / config.generator.max_yaw_rate_rps
    )
    linear_limit = (
        config.generator.max_approach_speed_mps
        if stage in {"approach", "approach_hold", "reacquire"}
        else config.generator.max_linear_speed_mps
    )
    linear_fraction = min(1.0, max(0.0, linear_x) / linear_limit)
    if angular_z > 1e-6:
        turn_clearance = scan.left_m
    elif angular_z < -1e-6:
        turn_clearance = scan.right_m
    else:
        turn_clearance = min(scan.left_m, scan.right_m)
    clearance_fraction = min(1.0, max(0.0, turn_clearance) / 4.0)

    if preferred_yaw_sign is None and marker is not None:
        preferred_yaw_sign = -marker.horizontal_error
    if (
        preferred_yaw_sign is not None
        and abs(preferred_yaw_sign) > 1e-6
        and abs(angular_z) > 1e-6
    ):
        alignment = 1.0 if angular_z * preferred_yaw_sign > 0.0 else -1.0
    else:
        alignment = 0.0

    if stage in {"approach", "approach_hold", "reacquire"}:
        return 4.0 * alignment + 1.5 * yaw_fraction + linear_fraction

    near_hazard = (
        scan.front_m < config.wall_clearance
        or min(scan.left_m, scan.right_m) < config.wall_clearance
    )
    if near_hazard:
        return (
            4.0 * clearance_fraction
            + 3.0 * yaw_fraction
            - 1.5 * linear_fraction
        )
    return 3.0 * linear_fraction + clearance_fraction - 0.25 * yaw_fraction


def command_is_live(command: CommandState, now: float) -> bool:
    return command.valid_until >= now


def dynamic_search_prefix_steps(
    scan: LidarScan,
    config: SearchConfig,
) -> int:
    """Choose a receding-horizon length without creating or changing actions."""

    maximum = config.generator.execute_prefix_steps
    side_clearance = min(scan.left_m, scan.right_m)
    if scan.front_m >= 2.0 and side_clearance >= config.wall_clearance:
        return maximum
    if (
        scan.front_m >= config.wall_clearance
        and side_clearance >= config.turn_clearance + 0.05
    ):
        return min(maximum, 12)
    return min(maximum, 8)


def generator_candidate_seeds(
    config: GeneratorConfig, stage: str, request_id: int
) -> tuple[int, ...]:
    """Bound per-cycle latency while retaining seed diversity where it matters."""

    if stage == "search":
        return (
            config.candidate_seeds[(request_id - 1) % len(config.candidate_seeds)],
        )
    return config.candidate_seeds[:2]


def observation_pose_matches(
    captured: RobotPose,
    current: RobotPose,
    config: GeneratorConfig,
) -> bool:
    return (
        math.hypot(current.x - captured.x, current.y - captured.y)
        <= config.maximum_stationary_drift_m
        and abs(wrapped_angle_delta(current.yaw, captured.yaw))
        <= config.maximum_stationary_drift_rad
    )


class CosmosGeneratorClient:
    """Small serialized client for the persistent local Cosmos3 model server."""

    def __init__(self, config: GeneratorConfig) -> None:
        parsed = urlsplit(config.server_url)
        self.config = config
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 80
        self._lock = threading.Lock()
        self._connection: http.client.HTTPConnection | None = None

    def close(self) -> None:
        with self._lock:
            if self._connection is not None:
                self._connection.close()
                self._connection = None

    def _request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None,
        timeout: float,
    ) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        headers = {"Accept": "application/json"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        with self._lock:
            try:
                if self._connection is None:
                    self._connection = http.client.HTTPConnection(
                        self.host, self.port, timeout=timeout
                    )
                else:
                    self._connection.timeout = timeout
                    if self._connection.sock is not None:
                        self._connection.sock.settimeout(timeout)
                self._connection.request(method, path, body=body, headers=headers)
                response = self._connection.getresponse()
                response_body = response.read(2_000_001)
                if len(response_body) > 2_000_000:
                    raise GeneratorError("Generator response exceeds 2 MB")
                if response.status != 200:
                    detail = response_body.decode("utf-8", errors="replace")[:400]
                    raise GeneratorError(
                        f"Generator HTTP {response.status}: {detail}"
                    )
                decoded = json.loads(response_body.decode("utf-8"))
                if not isinstance(decoded, dict):
                    raise GeneratorError("Generator JSON response must be an object")
                return decoded
            except GeneratorError:
                if self._connection is not None:
                    self._connection.close()
                    self._connection = None
                raise
            except (
                OSError,
                TimeoutError,
                http.client.HTTPException,
                json.JSONDecodeError,
                UnicodeError,
            ) as exc:
                if self._connection is not None:
                    self._connection.close()
                    self._connection = None
                raise GeneratorError(f"Generator request failed: {exc}") from exc
            finally:
                if self._connection is not None and self._connection.sock is None:
                    self._connection.close()
                    self._connection = None

    def info(self) -> dict[str, Any]:
        return self._request("GET", "/info", None, 5.0)

    def predict(
        self, image_bytes: bytes, prompt: str, *, seed: int
    ) -> dict[str, Any]:
        return self._request(
            "POST",
            "/predict_batch",
            {
                "items": [
                    {
                        "image": base64.b64encode(image_bytes).decode("ascii"),
                        "prompt": prompt,
                        "domain_name": self.config.domain_name,
                        "image_size": self.config.image_size,
                        "seed": seed,
                    }
                ]
            },
            self.config.request_timeout_sec,
        )

    def reason(self, image_bytes: bytes, prompt: str) -> str:
        response = self._request(
            "POST",
            "/reason",
            {
                "image": base64.b64encode(image_bytes).decode("ascii"),
                "prompt": prompt,
                "max_new_tokens": REASONER_MAX_NEW_TOKENS,
            },
            self.config.reasoner_timeout_sec,
        )
        text = response.get("reasoner_text")
        if not isinstance(text, str) or not text.strip():
            raise GeneratorError("shared Reasoner returned no text")
        return text


def extract_json_object(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    candidates: list[dict[str, Any]] = []
    for index, character in enumerate(text):
        if character != "{":
            continue
        try:
            payload, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict) and "target_visible" in payload:
            candidates.append(payload)
    if not candidates:
        raise ValueError("NWM-Cosmos3Edge output contains no target JSON")
    return candidates[-1]


def validate_cosmos_detection(payload: dict[str, Any]) -> CosmosDetection:
    expected = {
        "target_visible",
        "target_kind",
        "marker_visible",
        "safe_to_approach",
        "confidence",
        "reason",
    }
    if set(payload) != expected:
        raise ValueError("NWM-Cosmos3Edge target JSON fields do not match the protocol")
    for field in ("target_visible", "marker_visible", "safe_to_approach"):
        if not isinstance(payload[field], bool):
            raise ValueError(f"{field} must be boolean")
    if payload["target_kind"] not in {TARGET_KIND, "other", "none"}:
        raise ValueError("target_kind is unsupported")
    confidence = bounded_number(payload["confidence"], "confidence", 0.0, 1.0)
    reason = payload["reason"]
    if not isinstance(reason, str):
        raise ValueError("reason must be a string")
    safe = bool(payload["safe_to_approach"])
    if (
        not payload["target_visible"]
        or payload["target_kind"] != TARGET_KIND
        or not payload["marker_visible"]
    ):
        safe = False
    return CosmosDetection(
        target_visible=payload["target_visible"],
        target_kind=payload["target_kind"],
        marker_visible=payload["marker_visible"],
        confidence=confidence,
        safe_to_approach=safe,
        reason=reason.strip(),
    )


class MaplessChargerSearch(Node):
    def __init__(self) -> None:
        if RUNTIME_IMPORT_ERROR is not None:
            raise RuntimeError(
                f"ROS 2 runtime dependencies are unavailable: {RUNTIME_IMPORT_ERROR}"
            )
        super().__init__("go2w_house_mapless_charger_search")
        config_path = Path(
            os.environ.get("GO2W_MAPLESS_SEARCH_CONFIG", DEFAULT_CONFIG)
        )
        self.config = load_config(config_path)
        generator_url = os.environ.get("COSMOS3_GENERATOR_URL", "").strip()
        self.generator_config = (
            replace(self.config.generator, server_url=generator_url.rstrip("/"))
            if generator_url
            else self.config.generator
        )
        # Reuse the strict URL validation for an environment override.
        if generator_url:
            payload = {
                field: getattr(self.generator_config, field)
                for field in self.generator_config.__dataclass_fields__
            }
            payload["candidate_seeds"] = list(self.generator_config.candidate_seeds)
            self.generator_config = load_generator_config(payload)
        self.generator_client = CosmosGeneratorClient(self.generator_config)
        generator_info = validate_generator_server_info(
            self.generator_client.info(), self.generator_config
        )
        self.posture_file = Path(
            os.environ.get("GO2W_RL_POSTURE_FILE", DEFAULT_POSTURE_FILE)
        )
        self.jobs_dir = Path(
            os.environ.get("GO2W_CHARGER_SEARCH_JOBS_DIR", DEFAULT_JOBS_DIR)
        )
        cosmos_root = Path(os.environ.get("COSMOS_ROOT", DEFAULT_COSMOS_ROOT))
        self.framework_dir = Path(
            os.environ.get(
                "COSMOS_VLN_FRAMEWORK", cosmos_root / "packages/cosmos-framework"
            )
        )
        self.checkpoint_dir = Path(
            os.environ.get("COSMOS_VLN_CHECKPOINT", cosmos_root / "Cosmos3-Edge")
        )
        self.model_config = Path(
            os.environ.get(
                "COSMOS_VLN_MODEL_CONFIG",
                self.framework_dir
                / "cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml",
            )
        )
        self.jobs_dir.mkdir(parents=True, exist_ok=True)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage,
            self.config.camera_topic,
            self.on_image,
            sensor_qos,
        )
        self.create_subscription(
            DepthImage, self.config.depth_topic, self.on_depth, sensor_qos
        )
        self.create_subscription(
            PointCloud2, self.config.lidar_topic, self.on_cloud, sensor_qos
        )
        self.create_subscription(Odometry, "/odom/mujoco_odom", self.on_odom, 20)
        self.create_subscription(
            String, "/cosmos_vln/charger_search", self.on_task, 10
        )
        self.create_subscription(Empty, "/cosmos_vln/cancel", self.on_cancel, 10)
        self.velocity_publisher = self.create_publisher(
            Twist, self.config.velocity_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, "/cosmos_vln/charger_search_status", 10
        )
        self.prediction_publisher = self.create_publisher(
            String, "/cosmos_vln/future_prediction", 10
        )
        self.data_lock = threading.Lock()
        self.latest_image: tuple[bytes, float] | None = None
        self.latest_depth: DepthFrame | None = None
        self.latest_scan: LidarScan | None = None
        self.robot_pose: RobotPose | None = None
        now = time.monotonic()
        self.command = CommandState(
            0.0, 0.0, SAFETY_STOP_SOURCE, now, math.inf
        )
        self.command_expiry_reported = False
        self.worker_lock = threading.Lock()
        self.task_state_lock = threading.Lock()
        self.worker_active = False
        self.cancel_requested = False
        self.task = ""
        self.task_generation = 0
        self.generator_request_id = 0
        self.generator_failures = 0
        self.pending_search_prefix: GeneratorActionPrefix | None = None
        self.pending_approach_prefix: GeneratorActionPrefix | None = None
        self.pending_search_marker: (
            tuple[MarkerObservation, bytes, float] | None
        ) = None
        self.create_timer(0.10, self.publish_command)
        self.publish_status("ready")
        self.get_logger().info(
            "Mapless NWM-Cosmos3Edge Generator charger search ready: "
            f"marker=DICT_4X4_1000/{self.config.marker_id} "
            f"depth={self.config.depth_topic} "
            f"velocity={self.config.velocity_topic} "
            f"generator={self.generator_config.server_url} "
            f"checkpoint={generator_info['checkpoint']}; no map goals"
        )

    def on_image(self, message: CompressedImage) -> None:
        with self.data_lock:
            self.latest_image = (bytes(message.data), time.monotonic())

    def on_depth(self, message: DepthImage) -> None:
        received_at = time.monotonic()
        try:
            depth = decode_depth_frame(message, received_at, self.config)
        except ValueError as exc:
            self.get_logger().warning(
                f"Cannot parse depth image: {exc}", throttle_duration_sec=2.0
            )
            return
        with self.data_lock:
            self.latest_depth = depth

    def on_cloud(self, message: PointCloud2) -> None:
        try:
            raw = point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
                    np.float32, copy=False
                )
            else:
                points = np.asarray(list(raw), dtype=np.float32).reshape((-1, 3))
            scan = compute_lidar_scan(points, time.monotonic())
        except (TypeError, ValueError, KeyError) as exc:
            self.get_logger().warning(
                f"Cannot parse LiDAR cloud: {exc}", throttle_duration_sec=2.0
            )
            return
        with self.data_lock:
            self.latest_scan = scan

    def on_odom(self, message: Odometry) -> None:
        yaw, roll, pitch = quaternion_yaw_roll_pitch(message)
        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        pose = RobotPose(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=yaw,
            roll=roll,
            pitch=pitch,
            received_at=time.monotonic(),
            linear_speed=math.hypot(float(linear.x), float(linear.y)),
            yaw_rate=abs(float(angular.z)),
        )
        with self.data_lock:
            self.robot_pose = pose

    def on_task(self, message: String) -> None:
        task = message.data.strip()
        if not task:
            self.publish_status("rejected", reason="empty_task")
            return
        with self.worker_lock:
            if self.worker_active:
                self.publish_status("busy")
                return
            self.worker_active = True
        with self.task_state_lock:
            self.task = task
            self.cancel_requested = False
            self.task_generation += 1
        self.generator_failures = 0
        self.last_generator_twist = (0.0, 0.0)
        self.pending_search_prefix = None
        self.pending_approach_prefix = None
        self.pending_search_marker = None
        threading.Thread(target=self.run_search, daemon=True).start()

    def on_cancel(self, _message: Empty) -> None:
        with self.task_state_lock:
            self.cancel_requested = True
            self.task_generation += 1
            self.last_generator_twist = (0.0, 0.0)
            self.pending_search_prefix = None
            self.pending_approach_prefix = None
            self.pending_search_marker = None
            self.set_command(0.0, 0.0)

    def task_state_snapshot(self) -> tuple[int, bool]:
        with self.task_state_lock:
            return self.task_generation, self.cancel_requested

    def commit_generator_command_if_current(
        self,
        generation: int,
        linear_x: float,
        angular_z: float,
        *,
        request_id: int,
        chunk_step: int,
    ) -> bool:
        """Atomically reject a late nonzero action after task cancellation."""

        with self.task_state_lock:
            if self.cancel_requested or generation != self.task_generation:
                self.set_command(0.0, 0.0)
                return False
            self.set_command(
                linear_x,
                angular_z,
                source=GENERATOR_ACTION_SOURCE,
                request_id=request_id,
                chunk_step=chunk_step,
            )
            return True

    def set_command(
        self,
        linear_x: float,
        angular_z: float,
        *,
        source: str = SAFETY_STOP_SOURCE,
        request_id: int | None = None,
        chunk_step: int | None = None,
        valid_until: float | None = None,
    ) -> None:
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            raise ValueError("velocity command must be finite")
        nonzero = abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9
        if nonzero and source != GENERATOR_ACTION_SOURCE:
            raise RuntimeError("only Cosmos3 Generator may issue a nonzero command")
        now = time.monotonic()
        if valid_until is None:
            valid_until = (
                now + self.generator_config.command_ttl_sec
                if nonzero
                else math.inf
            )
        with self.data_lock:
            self.command = CommandState(
                linear_x=linear_x,
                angular_z=angular_z,
                source=source,
                issued_at=now,
                valid_until=float(valid_until),
                request_id=request_id,
                chunk_step=chunk_step,
            )
            self.command_expiry_reported = False

    def command_snapshot(self) -> tuple[float, float]:
        with self.data_lock:
            return self.command.linear_x, self.command.angular_z

    def command_state_snapshot(self) -> CommandState:
        with self.data_lock:
            return self.command

    def apply_candidate_soft_stop(
        self, scan: LidarScan, now: float, last_update_at: float
    ) -> float:
        elapsed_sec = (
            0.10
            if last_update_at <= 0.0
            else min(0.25, max(0.0, now - last_update_at))
        )
        state = self.command_state_snapshot()
        if not command_is_live(state, now):
            self.set_command(0.0, 0.0)
            return now
        linear_x, angular_z = state.linear_x, state.angular_z
        linear_x, angular_z = candidate_stop_velocity(
            linear_x, angular_z, elapsed_sec, scan, self.config
        )
        self.set_command(
            linear_x,
            angular_z,
            source=(
                state.source
                if abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9
                else SAFETY_STOP_SOURCE
            ),
            request_id=state.request_id,
            chunk_step=state.chunk_step,
            valid_until=state.valid_until if command_is_live(state, now) else now,
        )
        return now

    def publish_command(self) -> None:
        now = time.monotonic()
        with self.data_lock:
            state = self.command
            live = command_is_live(state, now)
            linear_x = state.linear_x if live else 0.0
            angular_z = state.angular_z if live else 0.0
            should_report_expiry = (
                not live
                and not self.command_expiry_reported
                and (abs(state.linear_x) > 1e-9 or abs(state.angular_z) > 1e-9)
            )
            if should_report_expiry:
                self.command_expiry_reported = True
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.velocity_publisher.publish(message)
        if should_report_expiry:
            self.get_logger().warning(
                f"generator_command_expired request_id={state.request_id} "
                f"source={state.source}"
            )

    def execute_search_action_prefix(self, generation: int) -> None:
        """Execute a selected Generator prefix stepwise at the model's 10 Hz."""

        prefix = self.pending_search_prefix
        if prefix is None or prefix.generation != generation:
            self.set_command(0.0, 0.0)
            return
        step_duration = 1.0 / float(self.generator_config.fps)
        checked_image_at = -1.0
        previous = getattr(self, "last_generator_twist", (0.0, 0.0))
        for chunk_step, (nominal_linear, nominal_yaw) in enumerate(
            prefix.nominal_commands
        ):
            current_generation, canceled = self.task_state_snapshot()
            if canceled or current_generation != generation:
                self.set_command(0.0, 0.0)
                return
            now = time.monotonic()
            image, scan, pose = self.snapshot()
            if (
                image is None
                or scan is None
                or pose is None
                or not sensor_is_fresh(image[1], now)
                or not sensor_is_fresh(scan.received_at, now)
                or not sensor_is_fresh(pose.received_at, now)
                or not posture_is_upright(pose, self.config)
            ):
                self.set_command(0.0, 0.0)
                return
            if image[1] != checked_image_at:
                checked_image_at = image[1]
                observed_marker = detect_marker(
                    image[0], self.config.marker_id
                )
                if observed_marker is not None:
                    self.pending_search_marker = (
                        observed_marker,
                        image[0],
                        image[1],
                    )
                    self.set_command(0.0, 0.0)
                    return
            limited_linear, limited_yaw = limit_generator_twist(
                nominal_linear,
                nominal_yaw,
                previous,
                "search",
                self.generator_config,
            )
            linear_x, angular_z, shield_reasons = shield_generator_action(
                limited_linear,
                limited_yaw,
                stage="search",
                scan=scan,
                pose=pose,
                now=now,
                config=self.config,
            )
            nonzero = abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9
            if nonzero:
                if not self.commit_generator_command_if_current(
                    generation,
                    linear_x,
                    angular_z,
                    request_id=prefix.request_id,
                    chunk_step=chunk_step,
                ):
                    return
            else:
                self.set_command(0.0, 0.0)
            previous = (linear_x, angular_z)
            self.last_generator_twist = previous
            self.publish_status(
                "generator_action_executing",
                stage="search",
                request_id=prefix.request_id,
                selected_seed=prefix.selected_seed,
                chunk_step=chunk_step,
                prefix_steps=len(prefix.nominal_commands),
                command_ttl_sec=self.generator_config.command_ttl_sec,
                linear_x=linear_x,
                angular_z=angular_z,
                action_source=GENERATOR_ACTION_SOURCE,
                shield=list(shield_reasons),
            )
            step_deadline = time.monotonic() + step_duration
            while time.monotonic() < step_deadline:
                current_generation, canceled = self.task_state_snapshot()
                if canceled or current_generation != generation:
                    self.set_command(0.0, 0.0)
                    return
                time.sleep(0.02)
        self.set_command(0.0, 0.0)
        self.pending_search_prefix = None

    def execute_approach_action_prefix(
        self,
        generation: int,
        marker_hint: MarkerObservation,
        last_exact_depth_m: float | None,
        last_exact_seen_at: float,
    ) -> ApproachPrefixResult | None:
        """Execute an approach prefix with per-step RGB-D, tracking, and LiDAR checks."""

        prefix = self.pending_approach_prefix
        if prefix is None or prefix.generation != generation:
            self.set_command(0.0, 0.0)
            return
        if prefix.marker_hint is not None:
            marker_hint = prefix.marker_hint
        if prefix.last_exact_seen_at > 0.0:
            last_exact_seen_at = prefix.last_exact_seen_at
        marker_tracker = (
            create_marker_tracker(prefix.marker_image, marker_hint)
            if prefix.marker_image is not None
            else None
        )
        step_duration = 1.0 / float(self.generator_config.fps)
        previous = getattr(self, "last_generator_twist", (0.0, 0.0))
        last_image = prefix.marker_image
        for chunk_step, (nominal_linear, nominal_yaw) in enumerate(
            prefix.nominal_commands
        ):
            current_generation, canceled = self.task_state_snapshot()
            if canceled or current_generation != generation:
                self.set_command(0.0, 0.0)
                return
            now = time.monotonic()
            image, scan, pose = self.snapshot()
            depth = self.depth_snapshot()
            if (
                image is None
                or scan is None
                or pose is None
                or depth is None
                or not sensor_is_fresh(image[1], now)
                or not sensor_is_fresh(scan.received_at, now)
                or not sensor_is_fresh(pose.received_at, now)
                or not sensor_is_fresh(depth.received_at, now)
                or not posture_is_upright(pose, self.config)
            ):
                self.set_command(0.0, 0.0)
                return
            last_image = image[0]
            tracked_marker = (
                update_marker_tracker(marker_tracker, image[0])
                if marker_tracker is not None
                else None
            )
            refreshed_marker = detect_marker(
                image[0],
                self.config.marker_id,
                hint=tracked_marker if tracked_marker is not None else marker_hint,
            )
            if refreshed_marker is not None:
                active_marker = refreshed_marker
                marker_hint = refreshed_marker
                marker_tracker = create_marker_tracker(image[0], refreshed_marker)
                last_exact_seen_at = now
            elif (
                tracked_marker is not None
                and tracker_hold_active(last_exact_seen_at, now, self.config)
            ):
                active_marker = tracked_marker
                marker_hint = tracked_marker
            else:
                self.set_command(0.0, 0.0)
                self.pending_approach_prefix = None
                self.publish_status(
                    "generator_action_discarded",
                    stage="approach",
                    request_id=prefix.request_id,
                    chunk_step=chunk_step,
                    reason="marker_temporarily_lost_during_approach_prefix",
                )
                return
            marker_depth_m = (
                estimate_marker_depth(
                    refreshed_marker, depth, image[1], self.config
                )
                if refreshed_marker is not None
                else None
            )
            if marker_depth_m is not None:
                last_exact_depth_m = marker_depth_m
            limited_linear, limited_yaw = limit_generator_twist(
                nominal_linear,
                nominal_yaw,
                previous,
                "approach",
                self.generator_config,
            )
            linear_x, angular_z, shield_reasons = shield_generator_action(
                limited_linear,
                limited_yaw,
                stage="approach",
                scan=scan,
                pose=pose,
                now=now,
                config=self.config,
                marker=active_marker,
                marker_depth_m=marker_depth_m,
                last_exact_depth_m=last_exact_depth_m,
            )
            nonzero = abs(linear_x) > 1e-9 or abs(angular_z) > 1e-9
            if nonzero:
                if not self.commit_generator_command_if_current(
                    generation,
                    linear_x,
                    angular_z,
                    request_id=prefix.request_id,
                    chunk_step=chunk_step,
                ):
                    return
            else:
                self.set_command(0.0, 0.0)
            previous = (linear_x, angular_z)
            self.last_generator_twist = previous
            self.publish_status(
                "generator_action_executing",
                stage="approach",
                request_id=prefix.request_id,
                selected_seed=prefix.selected_seed,
                chunk_step=chunk_step,
                prefix_steps=len(prefix.nominal_commands),
                command_ttl_sec=self.generator_config.command_ttl_sec,
                linear_x=linear_x,
                angular_z=angular_z,
                marker_depth_m=marker_depth_m,
                marker_source=(
                    "aruco" if active_marker.exact_id else "short_term_tracker"
                ),
                action_source=GENERATOR_ACTION_SOURCE,
                shield=list(shield_reasons),
            )
            step_deadline = time.monotonic() + step_duration
            while time.monotonic() < step_deadline:
                current_generation, canceled = self.task_state_snapshot()
                if canceled or current_generation != generation:
                    self.set_command(0.0, 0.0)
                    return
                time.sleep(0.02)
        self.set_command(0.0, 0.0)
        self.pending_approach_prefix = None
        if last_image is None:
            return None
        return ApproachPrefixResult(
            marker=marker_hint,
            image=last_image,
            last_exact_depth_m=last_exact_depth_m,
            last_exact_seen_at=last_exact_seen_at,
        )

    def snapshot(
        self,
    ) -> tuple[tuple[bytes, float] | None, LidarScan | None, RobotPose | None]:
        with self.data_lock:
            return self.latest_image, self.latest_scan, self.robot_pose

    def depth_snapshot(self) -> DepthFrame | None:
        with self.data_lock:
            return self.latest_depth

    def publish_status(self, state: str, **details: Any) -> None:
        payload = {"state": state, "task": self.task, **details}
        encoded = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(String(data=encoded))
        self.get_logger().info(f"mapless_charger_search_status={encoded}")

    def publish_visualization(
        self,
        state: str,
        *,
        progress: str,
        marker: MarkerObservation | None = None,
        confidence: float = 0.0,
        hazards: list[str] | None = None,
    ) -> None:
        marker_payload: dict[str, Any] | None = None
        if marker is not None:
            marker_payload = {
                "id": marker.marker_id,
                "corners": marker.corners,
                "height_ratio": marker.marker_height_ratio,
                "exact_id": marker.exact_id,
                "verification": marker.verification,
            }
        payload = {
            "instruction": self.task,
            "action": "mapless_visual_search",
            "route_id": None,
            "stage_id": state,
            "stage_index": 0,
            "stage_count": 1,
            "map_target": None,
            "marker": marker_payload,
            "confidence": confidence,
            "future_prediction": {
                "expected_observation": "A QR/ArUco marker mounted on the robot charging dock",
                "hazards": hazards or [],
                "progress": progress,
            },
            "mission_state": state,
        }
        self.prediction_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def report_marker_candidate(
        self, marker: MarkerObservation, confirm_frames: int
    ) -> None:
        self.publish_status(
            "marker_candidate",
            marker_id=marker.marker_id,
            confirm_frames=confirm_frames,
            horizontal_error=marker.horizontal_error,
            height_ratio=marker.marker_height_ratio,
            verification=marker.verification,
        )
        self.publish_visualization(
            "marker_candidate",
            progress=(
                f"Detected charging marker {marker.marker_id}; "
                "stopping for NWM-Cosmos3Edge verification"
            ),
            marker=marker,
            confidence=min(1.0, confirm_frames / self.config.confirm_frames),
        )

    def wait_for_sensors(self, timeout_sec: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not self.cancel_requested:
            image, scan, pose = self.snapshot()
            depth = self.depth_snapshot()
            now = time.monotonic()
            if (
                image is not None
                and depth is not None
                and scan is not None
                and pose is not None
                and now - image[1] < 1.5
                and now - depth.received_at < 1.0
                and now - scan.received_at < 1.0
                and now - pose.received_at < 1.0
            ):
                return True
            time.sleep(0.1)
        return False

    def generator_prompt(
        self,
        stage: str,
        scan: LidarScan,
        pose: RobotPose,
        *,
        marker: MarkerObservation | None = None,
        marker_depth_m: float | None = None,
        context: str = "",
    ) -> str:
        stage_instructions = {
            "search": (
                "Explore through the most open visible indoor corridor and search for "
                f"the charging dock carrying ArUco ID {self.config.marker_id}. Stay near "
                "the corridor center and keep generous clearance from both walls. Do not "
                "follow a memorized route, waypoint, map, or target coordinate."
            ),
            "approach": (
                "Approach the visible QR/ArUco charging dock smoothly. Center the marker "
                "in the image, move close enough for an RGB-D range of 0.20 to 0.40 m, "
                "then predict a stop. Do not pass the dock."
            ),
            "approach_hold": (
                "The verified dock disappeared for one camera frame. Hold position; only "
                "predict a small corrective rotation if the last visual evidence supports it."
            ),
            "reacquire": (
                "The verified dock is temporarily lost. Predict a cautious in-place camera "
                "rotation to reacquire the same marker; do not translate forward or reverse."
            ),
        }
        if stage not in stage_instructions:
            raise ValueError(f"unsupported Generator stage: {stage}")
        marker_context = "marker=not currently visible"
        if marker is not None:
            marker_context = (
                f"marker_id={marker.marker_id} exact_id={str(marker.exact_id).lower()} "
                f"horizontal_error={marker.horizontal_error:+.4f} "
                f"height_ratio={marker.marker_height_ratio:.4f} "
                f"rgbd_m={marker_depth_m if marker_depth_m is not None else 'unknown'}"
            )
        return (
            "You are NWM-Cosmos3Edge Generator producing egocentric AV-domain relative "
            "camera-pose actions for an experimental Unitree Go2-W adapter. The live RGB "
            "image is the authoritative visual observation. "
            f"Task: {self.task}. Stage: {stage}. {stage_instructions[stage]} "
            f"Live safety context: front={scan.front_m:.3f}m left={scan.left_m:.3f}m "
            f"right={scan.right_m:.3f}m yaw={pose.yaw:+.3f}rad; {marker_context}. "
            f"Exploration memory: {context or 'none'}. LiDAR and RGB-D are enforced by a "
            "separate veto-only shield; they are not a route planner."
        )

    def record_generator_failure(
        self, stage: str, request_id: int, error: Exception
    ) -> None:
        self.generator_failures += 1
        self.set_command(0.0, 0.0)
        self.publish_status(
            "generator_action_rejected",
            stage=stage,
            request_id=request_id,
            consecutive_failures=self.generator_failures,
            detail=str(error),
        )
        if self.generator_failures >= self.generator_config.maximum_failures:
            raise GeneratorBlocked(
                f"Cosmos3 Generator failed {self.generator_failures} consecutive requests: {error}"
            )

    def generate_and_execute(
        self,
        stage: str,
        image: tuple[bytes, float],
        scan: LidarScan,
        pose: RobotPose,
        *,
        marker: MarkerObservation | None = None,
        marker_depth_m: float | None = None,
        last_exact_depth_m: float | None = None,
        last_exact_seen_at: float = 0.0,
        context: str = "",
        preferred_yaw_sign: float | None = None,
    ) -> tuple[float, float] | None:
        generation, canceled = self.task_state_snapshot()
        if canceled:
            self.set_command(0.0, 0.0)
            return None
        self.generator_request_id += 1
        request_id = self.generator_request_id
        prompt = self.generator_prompt(
            stage,
            scan,
            pose,
            marker=marker,
            marker_depth_m=marker_depth_m,
            context=context,
        )
        candidate_seeds = generator_candidate_seeds(
            self.generator_config, stage, request_id
        )
        if stage == "search":
            self.pending_search_prefix = None
        elif stage == "approach":
            self.pending_approach_prefix = None
        self.set_command(0.0, 0.0)
        started_at = time.monotonic()
        inference_marker_tracker = (
            create_marker_tracker(image[0], marker)
            if stage == "approach" and marker is not None
            else None
        )
        self.publish_status(
            "generator_inference_started",
            stage=stage,
            request_id=request_id,
            candidate_seeds=list(candidate_seeds),
            action_source=GENERATOR_ACTION_SOURCE,
        )
        try:
            action_chunks: list[tuple[int, np.ndarray]] = []
            candidate_errors: list[str] = []
            for seed in candidate_seeds:
                try:
                    payload = self.generator_client.predict(
                        image[0], prompt, seed=seed
                    )
                    action_chunks.append(
                        (
                            seed,
                            validate_generator_action_payload(
                                payload, self.generator_config
                            ),
                        )
                    )
                except (GeneratorError, OSError, ValueError) as exc:
                    candidate_errors.append(f"seed={seed}: {exc}")
                current_generation, current_canceled = self.task_state_snapshot()
                if current_canceled or current_generation != generation:
                    break
            if not action_chunks:
                raise GeneratorError(
                    "all seeded Generator candidates failed: "
                    + "; ".join(candidate_errors)
                )
            current_image, current_scan, current_pose = self.snapshot()
            current_depth = self.depth_snapshot()
            now = time.monotonic()
            current_generation, current_canceled = self.task_state_snapshot()
            if current_canceled or generation != current_generation:
                self.set_command(0.0, 0.0)
                self.publish_status(
                    "generator_action_discarded",
                    stage=stage,
                    request_id=request_id,
                    reason="task_generation_changed",
                )
                return None
            if (
                current_image is None
                or current_scan is None
                or current_pose is None
                or current_depth is None
                or not sensor_is_fresh(current_image[1], now)
                or not sensor_is_fresh(current_scan.received_at, now)
                or not sensor_is_fresh(current_pose.received_at, now)
                or not sensor_is_fresh(current_depth.received_at, now)
            ):
                raise GeneratorError("live sensors became stale during Generator inference")
            if not observation_pose_matches(pose, current_pose, self.generator_config):
                raise GeneratorError("robot pose drifted while Generator inference was pending")
            if stage == "search":
                arrived_marker = detect_marker(
                    current_image[0], self.config.marker_id
                )
                if arrived_marker is not None:
                    self.pending_search_marker = (
                        arrived_marker,
                        current_image[0],
                        current_image[1],
                    )
                    self.set_command(0.0, 0.0)
                    self.publish_status(
                        "generator_action_discarded",
                        stage=stage,
                        request_id=request_id,
                        reason="marker_candidate_arrived_during_inference",
                        marker_id=arrived_marker.marker_id,
                    )
                    return None
            shield_marker = marker
            shield_depth_m = marker_depth_m
            if stage == "approach" and marker is not None:
                refreshed_marker = detect_marker(
                    current_image[0], self.config.marker_id, hint=marker
                )
                if refreshed_marker is None:
                    tracked_marker = (
                        update_marker_tracker(
                            inference_marker_tracker, current_image[0]
                        )
                        if inference_marker_tracker is not None
                        else None
                    )
                    if (
                        tracked_marker is None
                        or not tracker_hold_active(
                            last_exact_seen_at, now, self.config
                        )
                    ):
                        self.generator_failures = 0
                        self.set_command(0.0, 0.0)
                        self.publish_status(
                            "generator_action_discarded",
                            stage=stage,
                            request_id=request_id,
                            reason="marker_temporarily_lost_during_inference",
                        )
                        return None
                    shield_marker = tracked_marker
                    shield_depth_m = None
                else:
                    shield_marker = refreshed_marker
                    shield_depth_m = estimate_marker_depth(
                        refreshed_marker,
                        current_depth,
                        current_image[1],
                        self.config,
                    )
                    last_exact_seen_at = now
            previous = getattr(self, "last_generator_twist", (0.0, 0.0))
            candidates: list[
                tuple[
                    float,
                    int,
                    float,
                    float,
                    float,
                    float,
                    tuple[str, ...],
                ]
            ] = []
            candidate_details: list[dict[str, Any]] = []
            for seed, action_chunk in action_chunks:
                nominal_linear, nominal_yaw = adapt_generator_action_chunk(
                    action_chunk, self.generator_config
                )
                if (
                    stage in {"search", "approach", "reacquire"}
                    and abs(nominal_linear) < 1e-6
                    and abs(nominal_yaw) < 1e-6
                ):
                    candidate_details.append(
                        {"seed": seed, "rejected": "empty_nominal_action"}
                    )
                    continue
                limited_linear, limited_yaw = limit_generator_twist(
                    nominal_linear,
                    nominal_yaw,
                    previous,
                    stage,
                    self.generator_config,
                )
                linear_x, angular_z, shield_reasons = shield_generator_action(
                    limited_linear,
                    limited_yaw,
                    stage=stage,
                    scan=current_scan,
                    pose=current_pose,
                    now=now,
                    config=self.config,
                    marker=shield_marker,
                    marker_depth_m=shield_depth_m,
                    last_exact_depth_m=last_exact_depth_m,
                )
                score = score_generator_candidate(
                    linear_x,
                    angular_z,
                    stage=stage,
                    scan=current_scan,
                    config=self.config,
                    marker=shield_marker,
                    preferred_yaw_sign=preferred_yaw_sign,
                )
                candidates.append(
                    (
                        score,
                        seed,
                        nominal_linear,
                        nominal_yaw,
                        linear_x,
                        angular_z,
                        shield_reasons,
                    )
                )
                candidate_details.append(
                    {
                        "seed": seed,
                        "nominal_linear_x": nominal_linear,
                        "nominal_angular_z": nominal_yaw,
                        "linear_x": linear_x,
                        "angular_z": angular_z,
                        "score": score if math.isfinite(score) else None,
                        "shield": list(shield_reasons),
                    }
                )
            safe_candidates = [
                candidate for candidate in candidates if math.isfinite(candidate[0])
            ]
            if not safe_candidates:
                self.generator_failures = 0
                self.last_generator_twist = (0.0, 0.0)
                self.set_command(0.0, 0.0)
                self.publish_status(
                    "generator_action_vetoed",
                    stage=stage,
                    request_id=request_id,
                    reason="no_safe_seeded_candidate",
                    candidates=candidate_details,
                    action_source=GENERATOR_ACTION_SOURCE,
                )
                return None
            (
                selected_score,
                selected_seed,
                nominal_linear,
                nominal_yaw,
                linear_x,
                angular_z,
                shield_reasons,
            ) = max(safe_candidates, key=lambda candidate: candidate[0])
            execution_steps = 1
            if stage in {"search", "approach"}:
                selected_action_chunk = next(
                    action_chunk
                    for seed, action_chunk in action_chunks
                    if seed == selected_seed
                )
                execution_steps = (
                    dynamic_search_prefix_steps(current_scan, self.config)
                    if stage == "search"
                    else min(8, self.generator_config.action_chunk_size)
                )
                nominal_commands = tuple(
                    av_pose_action_to_twist(action, self.generator_config)
                    for action in selected_action_chunk[:execution_steps]
                )
                action_prefix = GeneratorActionPrefix(
                    generation=generation,
                    request_id=request_id,
                    selected_seed=selected_seed,
                    nominal_commands=nominal_commands,
                    marker_hint=shield_marker if stage == "approach" else None,
                    marker_image=current_image[0] if stage == "approach" else None,
                    last_exact_seen_at=(
                        last_exact_seen_at if stage == "approach" else 0.0
                    ),
                )
                if stage == "search":
                    self.pending_search_prefix = action_prefix
                else:
                    self.pending_approach_prefix = action_prefix
                nominal_linear, nominal_yaw = nominal_commands[0]
                limited_linear, limited_yaw = limit_generator_twist(
                    nominal_linear,
                    nominal_yaw,
                    previous,
                    stage,
                    self.generator_config,
                )
                linear_x, angular_z, shield_reasons = shield_generator_action(
                    limited_linear,
                    limited_yaw,
                    stage=stage,
                    scan=current_scan,
                    pose=current_pose,
                    now=now,
                    config=self.config,
                    marker=shield_marker,
                    marker_depth_m=shield_depth_m,
                    last_exact_depth_m=last_exact_depth_m,
                )
            self.generator_failures = 0
            self.last_generator_twist = (linear_x, angular_z)
            if not self.commit_generator_command_if_current(
                generation,
                linear_x,
                angular_z,
                request_id=request_id,
                chunk_step=0,
            ):
                self.publish_status(
                    "generator_action_discarded",
                    stage=stage,
                    request_id=request_id,
                    reason="task_generation_changed_before_commit",
                )
                return None
            self.publish_status(
                "generator_action_ready",
                stage=stage,
                request_id=request_id,
                latency_sec=now - started_at,
                selected_seed=selected_seed,
                selected_score=selected_score,
                candidate_count=len(action_chunks),
                execution_steps=execution_steps,
                nominal_linear_x=nominal_linear,
                nominal_angular_z=nominal_yaw,
                shield=list(shield_reasons),
                candidates=candidate_details,
                action_source=GENERATOR_ACTION_SOURCE,
            )
            if stage not in {"search", "approach"}:
                self.publish_status(
                    "generator_action_executing",
                    stage=stage,
                    request_id=request_id,
                    selected_seed=selected_seed,
                    chunk_step=0,
                    prefix_steps=execution_steps,
                    command_ttl_sec=self.generator_config.command_ttl_sec,
                    linear_x=linear_x,
                    angular_z=angular_z,
                    action_source=GENERATOR_ACTION_SOURCE,
                    shield=list(shield_reasons),
                )
            return linear_x, angular_z
        except GeneratorBlocked:
            raise
        except (GeneratorError, OSError, ValueError) as exc:
            self.record_generator_failure(stage, request_id, exc)
            return None

    def cosmos_prompt(self) -> str:
        return f"""Inspect this live first-person image from a Unitree Go2-W.
Task: {self.task}

Computer vision has detected ArUco DICT_4X4_1000 ID {self.config.marker_id}. Decide whether it is physically mounted on a floor-level robot charging dock. Furniture, outlets, pictures, screens, and loose markers are not docks.

OUTPUT THE COMPLETE JSON FIRST. Return exactly one JSON object, no markdown or prose, with all six fields:
{{
  "target_visible": true,
  "target_kind": "robot_charging_dock",
  "marker_visible": true,
  "safe_to_approach": true,
  "confidence": 0.9,
  "reason": "brief visual evidence"
}}

target_kind must be robot_charging_dock, other, or none. If any fact is uncertain, keep every field present and set safe_to_approach to false."""

    def run_cosmos(self, image_bytes: bytes, attempt: int) -> CosmosDetection:
        job_id = time.strftime("%Y%m%dT%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        job_dir = self.jobs_dir / job_id
        job_dir.mkdir(parents=True, exist_ok=False)
        with Image.open(BytesIO(image_bytes)) as image:
            image.load()
            image = image.convert("RGB")
            resampling = getattr(Image, "Resampling", Image)
            image.thumbnail((640, 640), resampling.LANCZOS)
            image_path = job_dir / "camera.jpg"
            image.save(image_path, format="JPEG", quality=90, optimize=True)
        request_path = job_dir / "request.json"
        log_path = job_dir / "inference.log"
        request_path.write_text(
            json.dumps(
                {
                    "model_mode": "reasoner",
                    "prompt": self.cosmos_prompt(),
                    "vision_path": str(image_path),
                    "max_new_tokens": REASONER_MAX_NEW_TOKENS,
                    "do_sample": False,
                    "server_url": self.generator_config.server_url,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        self.publish_status("cosmos_verification_started", attempt=attempt, job_id=job_id)
        started_at = time.monotonic()
        reasoner_text = self.generator_client.reason(
            image_path.read_bytes(), self.cosmos_prompt()
        )
        elapsed = time.monotonic() - started_at
        reasoner_path = job_dir / "reasoner_text.txt"
        reasoner_path.write_text(reasoner_text, encoding="utf-8")
        log_path.write_text(
            f"shared_server={self.generator_config.server_url}\n"
            f"elapsed_sec={elapsed:.6f}\n",
            encoding="ascii",
        )
        payload = extract_json_object(reasoner_text)
        return validate_cosmos_detection(payload)

    def verify_marker_with_cosmos(self, image_bytes: bytes) -> CosmosDetection | None:
        for attempt in (1, 2):
            try:
                detection = self.run_cosmos(image_bytes, attempt)
            except (GeneratorError, OSError, ValueError, RuntimeError) as exc:
                self.publish_status(
                    "cosmos_verification_rejected", attempt=attempt, detail=str(exc)
                )
                continue
            self.publish_status(
                "cosmos_verification",
                attempt=attempt,
                target_kind=detection.target_kind,
                marker_visible=detection.marker_visible,
                confidence=detection.confidence,
                reason=detection.reason,
            )
            if detection.confirmed and detection.safe_to_approach:
                return detection
        return None

    def posture_guard_failed(
        self, pose: RobotPose, now: float, unsafe_since: float
    ) -> tuple[bool, float]:
        if posture_is_upright(pose, self.config):
            return False, 0.0
        self.set_command(0.0, 0.0)
        if unsafe_since <= 0.0:
            unsafe_since = now
        if now - unsafe_since < self.config.posture_fault_delay:
            return False, unsafe_since
        self.posture_file.parent.mkdir(parents=True, exist_ok=True)
        self.posture_file.write_text("recover\n", encoding="ascii")
        self.publish_status(
            "blocked",
            reason="unexpected_low_posture",
            z=pose.z,
            roll=pose.roll,
            pitch=pose.pitch,
        )
        return True, unsafe_since

    def wait_for_cosmos_settle(
        self,
        marker: MarkerObservation,
        candidate_stop_updated_at: float,
    ) -> RobotPose | None:
        deadline = time.monotonic() + self.config.cosmos_settle_timeout
        stable_since = 0.0
        unsafe_since = 0.0
        last_pose: RobotPose | None = None
        self.publish_status(
            "cosmos_settle_started",
            marker_id=marker.marker_id,
            required_hold_sec=self.config.cosmos_settle_hold,
        )
        self.publish_visualization(
            "cosmos_settling",
            progress=(
                "Soft-stopping and checking posture before "
                "NWM-Cosmos3Edge verification"
            ),
            marker=marker,
            confidence=1.0,
        )
        while time.monotonic() < deadline and not self.cancel_requested:
            _image, scan, pose = self.snapshot()
            now = time.monotonic()
            if (
                scan is None
                or pose is None
                or now - scan.received_at > 1.0
                or now - pose.received_at > 1.0
            ):
                self.set_command(0.0, 0.0)
                stable_since = 0.0
                time.sleep(0.05)
                continue

            candidate_stop_updated_at = self.apply_candidate_soft_stop(
                scan, now, candidate_stop_updated_at
            )
            last_pose = pose
            failed, unsafe_since = self.posture_guard_failed(
                pose, now, unsafe_since
            )
            if failed:
                return None
            if cosmos_pose_is_stable(pose, self.config):
                if stable_since <= 0.0:
                    stable_since = now
                elif now - stable_since >= self.config.cosmos_settle_hold:
                    self.set_command(0.0, 0.0)
                    self.last_generator_twist = (0.0, 0.0)
                    self.publish_status(
                        "cosmos_settle_ready",
                        marker_id=marker.marker_id,
                        hold_sec=now - stable_since,
                        linear_speed=pose.linear_speed,
                        yaw_rate=pose.yaw_rate,
                    )
                    return pose
            else:
                stable_since = 0.0
            time.sleep(0.05)

        if self.cancel_requested:
            return None
        self.set_command(0.0, 0.0)
        details: dict[str, Any] = {"reason": "cosmos_precheck_not_stable"}
        if last_pose is not None:
            details.update(
                linear_speed=last_pose.linear_speed,
                yaw_rate=last_pose.yaw_rate,
                z=last_pose.z,
                roll=last_pose.roll,
                pitch=last_pose.pitch,
            )
        self.publish_status("blocked", **details)
        return None

    def stop_and_charge(
        self,
        marker: MarkerObservation,
        approach_travel_m: float,
        marker_depth_m: float,
    ) -> bool:
        self.set_command(0.0, 0.0)
        self.posture_file.parent.mkdir(parents=True, exist_ok=True)
        self.posture_file.write_text("stand\n", encoding="ascii")
        image, _scan, stop_pose = self.snapshot()
        depth = self.depth_snapshot()
        if stop_pose is None:
            self.publish_status("blocked", reason="arrival_pose_missing")
            return False
        stop_x, stop_y = stop_pose.x, stop_pose.y
        last_frame_time = image[1] if image is not None else -1.0
        last_depth_time = depth.received_at if depth is not None else -1.0
        stopped_frames = 0
        unsafe_since = 0.0
        deadline = time.monotonic() + self.config.arrival_stop_hold
        self.publish_status(
            "arrived_stopped",
            marker_id=marker.marker_id,
            approach_travel_m=approach_travel_m,
            marker_depth_m=marker_depth_m,
        )
        self.publish_visualization(
            "arrived_stopped",
            progress=(
                "Close to QR charging dock; wheels stopped for final "
                "RGB-D verification"
            ),
            marker=marker,
            confidence=1.0,
        )
        final_marker = marker
        while time.monotonic() < deadline and not self.cancel_requested:
            image, scan, pose = self.snapshot()
            depth = self.depth_snapshot()
            now = time.monotonic()
            if (
                image is None
                or scan is None
                or pose is None
                or depth is None
                or not sensor_is_fresh(image[1], now)
                or not sensor_is_fresh(scan.received_at, now)
                or not sensor_is_fresh(depth.received_at, now)
                or not sensor_is_fresh(pose.received_at, now)
            ):
                self.set_command(0.0, 0.0)
                stopped_frames = 0
                time.sleep(0.05)
                continue
            failed, unsafe_since = self.posture_guard_failed(
                pose, now, unsafe_since
            )
            if failed:
                return False
            if math.hypot(pose.x - stop_x, pose.y - stop_y) > 0.08:
                self.posture_file.write_text("recover\n", encoding="ascii")
                self.publish_status("blocked", reason="arrival_stop_drift")
                return False
            if pose.linear_speed > 0.03 or pose.yaw_rate > 0.06:
                stopped_frames = 0
                time.sleep(0.05)
                continue
            if (
                image is not None
                and image[1] != last_frame_time
                and depth.received_at != last_depth_time
                and abs(depth.received_at - image[1])
                <= self.config.depth_maximum_frame_delta
            ):
                observed = detect_marker(
                    image[0],
                    self.config.marker_id,
                    hint=final_marker,
                )
                observed_depth = estimate_marker_depth(
                    observed, depth, image[1], self.config
                ) if observed is not None else None
                if observed is None or not charge_ready(
                    observed,
                    observed_depth,
                    scan,
                    approach_travel_m,
                    self.config,
                ):
                    self.publish_status(
                        "blocked", reason="close_marker_confirmation_lost"
                    )
                    return False
                last_frame_time = image[1]
                last_depth_time = depth.received_at
                final_marker = observed
                marker_depth_m = observed_depth
                stopped_frames += 1
            time.sleep(0.05)

        if self.cancel_requested:
            return False
        if stopped_frames < self.config.final_confirm_frames:
            self.publish_status(
                "blocked",
                reason="arrival_stop_not_verified",
                stopped_frames=stopped_frames,
            )
            return False
        image, scan, pose = self.snapshot()
        depth = self.depth_snapshot()
        now = time.monotonic()
        if (
            image is None
            or scan is None
            or pose is None
            or depth is None
            or not sensor_is_fresh(image[1], now)
            or not sensor_is_fresh(scan.received_at, now)
            or not sensor_is_fresh(depth.received_at, now)
            or not sensor_is_fresh(pose.received_at, now)
            or abs(depth.received_at - image[1])
            > self.config.depth_maximum_frame_delta
        ):
            self.publish_status("blocked", reason="arrival_stop_sensors_stale")
            return False
        final_marker = detect_marker(
            image[0],
            self.config.marker_id,
            hint=final_marker,
        )
        final_depth = (
            estimate_marker_depth(final_marker, depth, image[1], self.config)
            if final_marker is not None
            else None
        )
        if final_marker is None or not charge_ready(
            final_marker,
            final_depth,
            scan,
            approach_travel_m,
            self.config,
        ):
            self.publish_status("blocked", reason="arrival_final_confirmation_lost")
            return False
        if (
            pose.linear_speed > 0.03
            or pose.yaw_rate > 0.06
            or not posture_is_upright(pose, self.config)
        ):
            self.posture_file.write_text("recover\n", encoding="ascii")
            self.publish_status("blocked", reason="charge_start_posture_invalid")
            return False

        start_z = pose.z
        marker_depth_m = final_depth
        self.posture_file.write_text("charge\n", encoding="ascii")
        self.publish_status(
            "charging",
            start_z=start_z,
            marker_id=final_marker.marker_id,
            approach_travel_m=approach_travel_m,
            marker_depth_m=marker_depth_m,
        )
        self.publish_visualization(
            "charging",
            progress="Dock reached and stopped; lowering into charge posture",
            marker=final_marker,
            confidence=1.0,
        )
        verify_deadline = time.monotonic() + 7.0
        while time.monotonic() < verify_deadline:
            _image, _scan, current = self.snapshot()
            if (
                current is not None
                and current.z <= start_z - 0.06
                and abs(current.roll) < 0.45
                and abs(current.pitch) < 0.45
            ):
                return True
            time.sleep(0.1)
        self.posture_file.write_text("recover\n", encoding="ascii")
        self.publish_status("blocked", reason="charge_posture_not_verified")
        return False

    def track_and_charge(
        self,
        initial_marker: MarkerObservation,
        approach_origin: RobotPose,
        initial_image: bytes,
    ) -> bool:
        self.last_generator_twist = (0.0, 0.0)
        deadline = time.monotonic() + self.config.approach_timeout
        last_frame_time = -1.0
        last_marker = initial_marker
        marker_tracker = create_marker_tracker(initial_image, initial_marker)
        last_exact_seen = time.monotonic()
        last_exact_depth_m: float | None = None
        last_pose = approach_origin
        reacquire_center_yaw = approach_origin.yaw
        reacquire_direction = (
            1.0 if initial_marker.horizontal_error <= 0.0 else -1.0
        )
        reacquire_next_flip_at = 0.0
        approach_travel_m = 0.0
        final_hits = 0
        last_final_depth_time = -1.0
        unsafe_since = 0.0
        next_progress_status_at = 0.0
        while time.monotonic() < deadline and not self.cancel_requested:
            image, scan, pose = self.snapshot()
            now = time.monotonic()
            if (
                image is None
                or scan is None
                or pose is None
                or not sensor_is_fresh(image[1], now)
                or not sensor_is_fresh(scan.received_at, now)
                or not sensor_is_fresh(pose.received_at, now)
            ):
                self.set_command(0.0, 0.0)
                time.sleep(0.1)
                continue
            delta = math.hypot(pose.x - last_pose.x, pose.y - last_pose.y)
            if delta <= 0.50:
                approach_travel_m += delta
            last_pose = pose
            failed, unsafe_since = self.posture_guard_failed(
                pose, now, unsafe_since
            )
            if failed:
                return False
            exact_marker = None
            exact_depth_m = None
            depth = None
            marker = None
            new_frame = image is not None and image[1] != last_frame_time
            if new_frame:
                last_frame_time = image[1]
                depth = self.depth_snapshot()
                tracked_marker = (
                    update_marker_tracker(marker_tracker, image[0])
                    if marker_tracker is not None
                    else None
                )
                exact_marker = detect_marker(
                    image[0], self.config.marker_id, hint=tracked_marker
                )
                if exact_marker is not None:
                    exact_depth_m = estimate_marker_depth(
                        exact_marker, depth, image[1], self.config
                    )
                    marker = exact_marker
                    marker_tracker = create_marker_tracker(
                        image[0], exact_marker
                    )
                    last_marker = exact_marker
                    last_exact_seen = now
                    if exact_depth_m is not None:
                        last_exact_depth_m = exact_depth_m
                    reacquire_center_yaw = pose.yaw
                    reacquire_direction = (
                        1.0 if exact_marker.horizontal_error <= 0.0 else -1.0
                    )
                    reacquire_next_flip_at = 0.0
                elif (
                    tracked_marker is not None
                    and tracker_hold_active(last_exact_seen, now, self.config)
                ):
                    marker = tracked_marker
                    last_marker = tracked_marker
                else:
                    marker_tracker = None
            else:
                time.sleep(0.05)
                continue
            if marker is None:
                final_hits = 0
                lost_for = now - last_exact_seen
                if lost_for <= self.config.approach_observation_hold:
                    generated = self.generate_and_execute(
                        "approach_hold",
                        image,
                        scan,
                        pose,
                        marker=last_marker,
                        marker_depth_m=last_exact_depth_m,
                        last_exact_depth_m=last_exact_depth_m,
                        context=f"camera_dropout_sec={lost_for:.3f}",
                    )
                    hold_speed, hold_rate = generated or (0.0, 0.0)
                    self.publish_visualization(
                        "approaching_hold",
                        progress=(
                            f"Holding marker {last_marker.marker_id} through a "
                            f"{lost_for:.2f}s camera dropout"
                        ),
                        marker=last_marker,
                        confidence=0.7,
                    )
                    if now >= next_progress_status_at:
                        self.publish_status(
                            "approaching_hold",
                            marker_id=last_marker.marker_id,
                            dropout_sec=lost_for,
                            approach_travel_m=approach_travel_m,
                            front_m=scan.front_m,
                            linear_x=hold_speed,
                            angular_z=hold_rate,
                        )
                        next_progress_status_at = now + 1.0
                    time.sleep(0.1)
                    continue
                if lost_for <= self.config.marker_lost_timeout:
                    if reacquire_next_flip_at <= 0.0:
                        reacquire_next_flip_at = (
                            now + self.config.reacquire_direction_flip
                        )
                    elif now >= reacquire_next_flip_at:
                        reacquire_direction *= -1.0
                        reacquire_next_flip_at = (
                            now + self.config.reacquire_direction_flip
                        )
                    reacquire_direction = bounded_reacquire_direction(
                        pose.yaw,
                        reacquire_center_yaw,
                        reacquire_direction,
                        self.config.reacquire_sweep_half_angle,
                    )
                    generated = self.generate_and_execute(
                        "reacquire",
                        image,
                        scan,
                        pose,
                        marker=last_marker,
                        marker_depth_m=last_exact_depth_m,
                        last_exact_depth_m=last_exact_depth_m,
                        context=(
                            f"last_bearing_sweep_direction={reacquire_direction:+.0f} "
                            f"dropout_sec={lost_for:.3f}"
                        ),
                        preferred_yaw_sign=reacquire_direction,
                    )
                    reacquire_speed, reacquire_rate = generated or (0.0, 0.0)
                    sweep_offset = wrapped_angle_delta(
                        pose.yaw, reacquire_center_yaw
                    )
                    self.publish_visualization(
                        "reacquiring",
                        progress=(
                            f"Scanning around marker {last_marker.marker_id} last "
                            f"bearing offset={sweep_offset:+.2f}rad"
                        ),
                        marker=last_marker,
                        confidence=0.5,
                    )
                    if now >= next_progress_status_at:
                        self.publish_status(
                            "reacquiring",
                            marker_id=last_marker.marker_id,
                            dropout_sec=lost_for,
                            sweep_offset_rad=sweep_offset,
                            sweep_direction=reacquire_direction,
                            approach_travel_m=approach_travel_m,
                            front_m=scan.front_m,
                            linear_x=reacquire_speed,
                            angular_z=reacquire_rate,
                        )
                        next_progress_status_at = now + 1.0
                    time.sleep(0.1)
                    continue
                self.set_command(0.0, 0.0)
                self.publish_status("blocked", reason="charging_marker_lost")
                return False

            error = marker.horizontal_error
            if exact_marker is not None and charge_ready(
                exact_marker,
                exact_depth_m,
                scan,
                approach_travel_m,
                self.config,
            ):
                self.set_command(0.0, 0.0)
                if (
                    depth is None
                    or depth.received_at == last_final_depth_time
                ):
                    time.sleep(0.05)
                    continue
                last_final_depth_time = depth.received_at
                final_hits += 1
                self.publish_status(
                    "close_marker_confirmation",
                    marker_id=exact_marker.marker_id,
                    confirm_frames=final_hits,
                    required_frames=self.config.final_confirm_frames,
                    approach_travel_m=approach_travel_m,
                    front_m=scan.front_m,
                    height_ratio=exact_marker.marker_height_ratio,
                    marker_depth_m=exact_depth_m,
                    verification=exact_marker.verification,
                )
                if final_hits >= self.config.final_confirm_frames:
                    return self.stop_and_charge(
                        exact_marker,
                        approach_travel_m,
                        exact_depth_m,
                    )
                time.sleep(0.05)
                continue
            final_hits = 0

            approach_generation, _canceled = self.task_state_snapshot()
            generated = self.generate_and_execute(
                "approach",
                image,
                scan,
                pose,
                marker=marker,
                marker_depth_m=exact_depth_m,
                last_exact_depth_m=last_exact_depth_m,
                last_exact_seen_at=last_exact_seen,
                context=(
                    f"approach_travel_m={approach_travel_m:.3f} "
                    f"exact_marker_age_sec={now - last_exact_seen:.3f}"
                ),
            )
            speed, yaw_rate = generated or (0.0, 0.0)
            if generated is not None:
                prefix_result = self.execute_approach_action_prefix(
                    approach_generation,
                    marker,
                    last_exact_depth_m,
                    last_exact_seen,
                )
                if prefix_result is not None:
                    last_marker = prefix_result.marker
                    last_exact_seen = max(
                        last_exact_seen, prefix_result.last_exact_seen_at
                    )
                    if prefix_result.last_exact_depth_m is not None:
                        last_exact_depth_m = prefix_result.last_exact_depth_m
                    marker_tracker = create_marker_tracker(
                        prefix_result.image, prefix_result.marker
                    )
            approach_state = (
                "approaching" if marker.exact_id else "approaching_tracked"
            )
            if now >= next_progress_status_at:
                self.publish_status(
                    approach_state,
                    marker_id=marker.marker_id,
                    exact_id=marker.exact_id,
                    verification=marker.verification,
                    horizontal_error=error,
                    height_ratio=marker.marker_height_ratio,
                    marker_depth_m=exact_depth_m,
                    last_exact_depth_m=last_exact_depth_m,
                    exact_age_sec=max(0.0, now - last_exact_seen),
                    approach_travel_m=approach_travel_m,
                    front_m=scan.front_m,
                    linear_x=speed,
                    angular_z=yaw_rate,
                )
                next_progress_status_at = now + 1.0
            self.publish_visualization(
                approach_state,
                progress=(
                    f"Cosmos3 Generator visual approach marker={marker.marker_id} "
                    f"source={'aruco' if marker.exact_id else 'tracker'} "
                    f"error={error:+.2f} size={marker.marker_height_ratio:.2f} "
                    f"depth={f'{exact_depth_m:.2f}m' if exact_depth_m is not None else 'unknown'} "
                    f"front={scan.front_m:.2f}m travel={approach_travel_m:.2f}m"
                ),
                marker=marker,
                confidence=1.0 if marker.exact_id else 0.75,
                hazards=["LiDAR emergency stop remains active"],
            )
            time.sleep(0.1)
        self.set_command(0.0, 0.0)
        self.publish_status("blocked", reason="charging_approach_timeout")
        return False

    def run_search(self) -> None:
        try:
            self.posture_file.parent.mkdir(parents=True, exist_ok=True)
            self.posture_file.write_text("stand\n", encoding="ascii")
            self.last_generator_twist = (0.0, 0.0)
            self.publish_status(
                "started",
                mode="mapless_visual_search",
                action_source=GENERATOR_ACTION_SOURCE,
                adapter=self.generator_config.adapter,
            )
            self.publish_visualization(
                "searching",
                progress="Cosmos3 Generator mapless exploration started",
                hazards=["Live LiDAR can only reduce or veto Generator motion"],
            )
            if not self.wait_for_sensors():
                self.publish_status("blocked", reason="mapless_sensors_not_ready")
                return
            self.publish_status("searching", mode="mapless_visual_search")

            deadline = time.monotonic() + self.config.search_timeout
            visits: dict[tuple[int, int], int] = defaultdict(int)
            pose_history: deque[RobotPose] = deque()
            processed_frame_time = -1.0
            last_generator_frame_time = -1.0
            marker_hits = 0
            last_marker: MarkerObservation | None = None
            last_marker_image: bytes | None = None
            last_marker_seen_at = 0.0
            candidate_stop_updated_at = 0.0
            escape_until = 0.0
            next_visit_update = 0.0
            next_search_status_at = time.monotonic() + 2.0
            unsafe_since = 0.0
            while time.monotonic() < deadline and not self.cancel_requested:
                image, scan, pose = self.snapshot()
                now = time.monotonic()
                if (
                    image is None
                    or scan is None
                    or pose is None
                    or not sensor_is_fresh(image[1], now)
                    or not sensor_is_fresh(scan.received_at, now)
                    or not sensor_is_fresh(pose.received_at, now)
                ):
                    self.set_command(0.0, 0.0)
                    time.sleep(0.1)
                    continue

                failed, unsafe_since = self.posture_guard_failed(
                    pose, now, unsafe_since
                )
                if failed:
                    return

                if now >= next_visit_update:
                    cell = (round(pose.x / 0.75), round(pose.y / 0.75))
                    visits[cell] += 1
                    next_visit_update = now + 1.0
                pose_history.append(pose)
                while pose_history and now - pose_history[0].received_at > 12.0:
                    pose_history.popleft()
                if (
                    len(pose_history) >= 2
                    and now - pose_history[0].received_at >= 10.0
                    and math.hypot(
                        pose.x - pose_history[0].x, pose.y - pose_history[0].y
                    ) < 0.18
                    and now >= escape_until
                ):
                    escape_until = now + 3.0
                    pose_history.clear()
                    self.publish_status(
                        "generator_context_update", reason="mapless_no_progress"
                    )

                if marker_hits > 0 and not candidate_hold_active(
                    last_marker_seen_at, now, self.config
                ):
                    marker_hits = 0
                    last_marker = None
                    last_marker_image = None
                    candidate_stop_updated_at = 0.0

                pending_marker = self.pending_search_marker
                if pending_marker is not None:
                    self.pending_search_marker = None
                    marker, marker_image, marker_frame_time = pending_marker
                    if marker_frame_time > processed_frame_time:
                        processed_frame_time = marker_frame_time
                        marker_hits = 1
                        last_marker = marker
                        last_marker_image = marker_image
                        last_marker_seen_at = now
                        candidate_stop_updated_at = now - 0.10
                        self.report_marker_candidate(marker, marker_hits)

                if image[1] != processed_frame_time:
                    processed_frame_time = image[1]
                    marker = detect_marker(
                        image[0],
                        self.config.marker_id,
                        last_marker if marker_hits > 0 else None,
                    )
                    if marker is not None:
                        if marker_hits <= 0:
                            marker_hits = 0
                            candidate_stop_updated_at = now - 0.10
                        marker_hits += 1
                        last_marker = marker
                        last_marker_image = image[0]
                        last_marker_seen_at = now
                        self.report_marker_candidate(marker, marker_hits)

                if 0 < marker_hits < self.config.confirm_frames:
                    candidate_stop_updated_at = self.apply_candidate_soft_stop(
                        scan, now, candidate_stop_updated_at
                    )
                    time.sleep(0.1)
                    continue

                if marker_hits >= self.config.confirm_frames and last_marker is not None:
                    settled_pose = self.wait_for_cosmos_settle(
                        last_marker, candidate_stop_updated_at
                    )
                    if settled_pose is None:
                        if self.cancel_requested:
                            self.publish_status("canceled")
                        return
                    if last_marker_image is None:
                        self.publish_status(
                            "blocked", reason="cosmos_marker_image_missing"
                        )
                        return
                    detection = self.verify_marker_with_cosmos(last_marker_image)
                    if detection is None:
                        marker_hits = 0
                        last_marker_image = None
                        candidate_stop_updated_at = 0.0
                        self.publish_status("marker_rejected", marker_id=last_marker.marker_id)
                        time.sleep(0.2)
                        continue
                    _image, _scan, approach_origin = self.snapshot()
                    if approach_origin is None or not cosmos_pose_is_stable(
                        approach_origin, self.config
                    ):
                        self.set_command(0.0, 0.0)
                        self.publish_status(
                            "blocked", reason="cosmos_postcheck_not_stable"
                        )
                        return
                    self.publish_status(
                        "target_confirmed",
                        marker_id=last_marker.marker_id,
                        confidence=detection.confidence,
                        reason=detection.reason,
                    )
                    if self.track_and_charge(
                        last_marker,
                        approach_origin,
                        last_marker_image,
                    ):
                        self.publish_status(
                            "succeeded",
                            marker_id=last_marker.marker_id,
                            simulated_charging=True,
                        )
                    return

                if image[1] == last_generator_frame_time:
                    time.sleep(0.05)
                    continue
                last_generator_frame_time = image[1]
                search_generation, _canceled = self.task_state_snapshot()
                generated = self.generate_and_execute(
                    "search",
                    image,
                    scan,
                    pose,
                    context=(
                        f"visited_cells={len(visits)} "
                        f"current_cell_visits={visits.get((round(pose.x / 0.75), round(pose.y / 0.75)), 0)} "
                        f"no_progress={str(now < escape_until).lower()}"
                    ),
                )
                speed, yaw_rate = generated or (0.0, 0.0)
                if generated is not None:
                    self.execute_search_action_prefix(search_generation)
                    _image, refreshed_scan, _pose = self.snapshot()
                    if refreshed_scan is not None:
                        scan = refreshed_scan
                now = time.monotonic()
                if now >= next_search_status_at:
                    self.publish_status(
                        "searching",
                        front_m=scan.front_m,
                        left_m=scan.left_m,
                        right_m=scan.right_m,
                        linear_x=speed,
                        angular_z=yaw_rate,
                        visited=len(visits),
                    )
                    next_search_status_at = now + 2.0
                self.publish_visualization(
                    "searching",
                    progress=(
                        f"Cosmos3 Generator vx={speed:+.2f} wz={yaw_rate:+.2f} "
                        f"front={scan.front_m:.2f}m left={scan.left_m:.2f}m "
                        f"right={scan.right_m:.2f}m visited={len(visits)}"
                    ),
                    hazards=["No global map or navigation goal is being used"],
                )
                time.sleep(0.1)

            self.set_command(0.0, 0.0)
            if self.cancel_requested:
                self.publish_status("canceled")
            else:
                self.publish_status("not_found", reason="mapless_search_timeout")
        except GeneratorBlocked as exc:
            self.set_command(0.0, 0.0)
            self.publish_status("blocked", reason="cosmos3_generator_unavailable", detail=str(exc))
            self.get_logger().error(f"Cosmos3 Generator task blocked: {exc}")
        except Exception as exc:
            self.set_command(0.0, 0.0)
            self.posture_file.write_text("recover\n", encoding="ascii")
            self.publish_status("failed", reason=str(exc))
            self.get_logger().error(f"Mapless charger search failed: {exc}")
        finally:
            with self.worker_lock:
                self.worker_active = False


def main() -> None:
    if RUNTIME_IMPORT_ERROR is not None:
        raise RuntimeError(
            f"ROS 2 runtime dependencies are unavailable: {RUNTIME_IMPORT_ERROR}"
        )
    rclpy.init()
    node = MaplessChargerSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.set_command(0.0, 0.0)
        if rclpy.ok():
            node.publish_command()
        node.generator_client.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
