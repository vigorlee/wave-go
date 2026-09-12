#!/usr/bin/env python3
"""Terrain-aware profile selector and safety projection for Go2W+Cosmos3Nav.

This module is an engineering interface between the semantic terrain map and a
future constrained-RL locomotion policy.  It deliberately fails closed: stale
or low-confidence observations produce a zero-velocity command.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT_DIR / "config/go2w_terrain_policy_profiles.json"
REQUIRED_PROFILES = {
    "rolling",
    "slope_up",
    "slope_down",
    "stair_up",
    "stair_down",
    "complex_obstacle",
    "narrow_passage",
    "dynamic_wait",
    "safe_stop",
}


@dataclass(frozen=True)
class TerrainObservation:
    age_s: float
    localization_confidence: float
    slope_deg: float = 0.0
    stair_probability: float = 0.0
    stair_direction: str = "none"
    corridor_clearance_m: float = math.inf
    obstacle_complexity: float = 0.0
    dynamic_time_to_collision_s: float = math.inf

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "TerrainObservation":
        allowed = {field.name for field in cls.__dataclass_fields__.values()}
        unknown = set(payload) - allowed
        if unknown:
            raise ValueError(f"unknown observation fields: {sorted(unknown)}")
        return cls(**payload)


@dataclass(frozen=True)
class PolicyCommand:
    profile: str
    locomotion_mode: str
    forward_speed_mps: float
    yaw_rate_rps: float
    body_height_m: float
    body_pitch_deg: float
    minimum_clearance_m: float
    replan_required: bool
    reason: str


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def load_policy_config(path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("version") != 1:
        raise ValueError("unsupported terrain policy configuration version")
    selector = payload.get("selector")
    profiles = payload.get("profiles")
    if not isinstance(selector, dict) or not isinstance(profiles, dict):
        raise ValueError("configuration requires selector and profiles objects")
    missing = REQUIRED_PROFILES - set(profiles)
    if missing:
        raise ValueError(f"missing required profiles: {sorted(missing)}")

    for name, profile in profiles.items():
        for field in (
            "max_forward_speed_mps",
            "max_yaw_rate_rps",
            "max_acceleration_mps2",
            "body_height_m",
            "body_pitch_deg",
            "minimum_clearance_m",
        ):
            _finite(profile[field], f"profiles.{name}.{field}")
        if profile["max_forward_speed_mps"] < 0:
            raise ValueError(f"profiles.{name}.max_forward_speed_mps is negative")
    return payload


class TerrainPolicySelector:
    def __init__(self, config: Mapping[str, Any]):
        self.selector = dict(config["selector"])
        self.profiles = dict(config["profiles"])

    def select_profile(self, observation: TerrainObservation) -> tuple[str, str]:
        s = self.selector
        if (
            observation.age_s < 0
            or observation.age_s > s["maximum_observation_age_s"]
        ):
            return "safe_stop", "terrain observation is stale or invalid"
        if not 0.0 <= observation.localization_confidence <= 1.0:
            return "safe_stop", "localization confidence is invalid"
        if observation.localization_confidence < s["minimum_localization_confidence"]:
            return "safe_stop", "localization confidence is below threshold"
        if observation.dynamic_time_to_collision_s <= s["dynamic_time_to_collision_s"]:
            return "dynamic_wait", "predicted dynamic obstacle blocks the corridor"
        if observation.stair_probability >= s["stair_probability_threshold"]:
            if observation.stair_direction == "up":
                return "stair_up", "ascending stair evidence exceeds threshold"
            if observation.stair_direction == "down":
                return "stair_down", "descending stair evidence exceeds threshold"
            return "safe_stop", "stair direction is ambiguous"
        if observation.slope_deg >= s["slope_enter_deg"]:
            return "slope_up", "uphill slope exceeds entry threshold"
        if observation.slope_deg <= -s["slope_enter_deg"]:
            return "slope_down", "downhill slope exceeds entry threshold"
        if observation.corridor_clearance_m < s["narrow_clearance_m"]:
            return "narrow_passage", "corridor clearance is limited"
        if observation.obstacle_complexity >= s["complexity_threshold"]:
            return "complex_obstacle", "obstacle complexity exceeds threshold"
        return "rolling", "terrain supports normal wheel rolling"

    def command(
        self,
        observation: TerrainObservation,
        requested_forward_speed_mps: float,
        requested_yaw_rate_rps: float = 0.0,
    ) -> PolicyCommand:
        profile_name, reason = self.select_profile(observation)
        profile = self.profiles[profile_name]
        requested_vx = _finite(requested_forward_speed_mps, "requested_forward_speed_mps")
        requested_wz = _finite(requested_yaw_rate_rps, "requested_yaw_rate_rps")
        max_vx = float(profile["max_forward_speed_mps"])
        max_wz = float(profile["max_yaw_rate_rps"])
        forward_speed = min(max(requested_vx, 0.0), max_vx)
        yaw_rate = min(max(requested_wz, -max_wz), max_wz)
        if profile_name in {"safe_stop", "dynamic_wait"}:
            forward_speed = 0.0
            yaw_rate = 0.0
        return PolicyCommand(
            profile=profile_name,
            locomotion_mode=str(profile["locomotion_mode"]),
            forward_speed_mps=forward_speed,
            yaw_rate_rps=yaw_rate,
            body_height_m=float(profile["body_height_m"]),
            body_pitch_deg=float(profile["body_pitch_deg"]),
            minimum_clearance_m=float(profile["minimum_clearance_m"]),
            replan_required=profile_name
            in {"dynamic_wait", "complex_obstacle", "safe_stop"},
            reason=reason,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--observation-json", required=True)
    parser.add_argument("--requested-vx", type=float, default=0.25)
    parser.add_argument("--requested-wz", type=float, default=0.0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()

    observation = TerrainObservation.from_mapping(json.loads(args.observation_json))
    selector = TerrainPolicySelector(load_policy_config(args.config))
    command = selector.command(observation, args.requested_vx, args.requested_wz)
    print(json.dumps(asdict(command), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
