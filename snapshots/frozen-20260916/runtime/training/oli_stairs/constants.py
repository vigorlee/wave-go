"""Shared OLI policy and stair-training constants."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Final

import numpy as np
import yaml


PROJECT_ROOT: Final = Path(__file__).resolve().parents[2]
SOURCE_MODEL: Final = (
    PROJECT_ROOT
    / "third_party/limx_humanoid_mujoco_sim/humanoid-description"
    / "HU_D04_description/xml/HU_D04_01.xml"
)
WALK_PARAM: Final = (
    PROJECT_ROOT
    / "third_party/limx_humanoid_rl_deploy_python/controllers"
    / "HU_D04_01/walk_controller/walk_param.yaml"
)
STAND_PARAM: Final = WALK_PARAM.parents[1] / "stand_controller/joint_params.yaml"
CONTROL_PROFILES: Final = ("walk", "stand")
OFFICIAL_POLICY: Final = WALK_PARAM.parent / "policy/default/policy.onnx"
ARTIFACT_ROOT: Final = PROJECT_ROOT / "artifacts/oli_stairs"


# This is the exact policy/deployment order documented by LimX. The ankle and
# waist pitch/roll entries are virtual PR-space joints, not physical A/B motors.
JOINT_NAMES: Final = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "head_pitch_joint",
    "head_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)

AUXILIARY_JOINT_NAMES: Final = (
    "left_A_achilles_joint",
    "left_A_achilles_rod_joint",
    "left_B_achilles_joint",
    "left_B_achilles_rod_joint",
    "right_A_achilles_joint",
    "right_A_achilles_rod_joint",
    "right_B_achilles_joint",
    "right_B_achilles_rod_joint",
    "waist_A_joint",
    "waist_A_rod_joint",
    "waist_B_joint",
    "waist_B_rod_joint",
)

OBSERVATION_FRAME_SIZE: Final = 102
OBSERVATION_HISTORY: Final = 5
OBSERVATION_SIZE: Final = OBSERVATION_FRAME_SIZE * OBSERVATION_HISTORY
ACTION_SIZE: Final = len(JOINT_NAMES)


@dataclass(frozen=True)
class ControlParameters:
    action_scale: np.ndarray
    torque_limit: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    default_angle: np.ndarray
    gyro_scale: float
    dof_pos_scale: float
    dof_vel_scale: float
    observation_clip: float
    action_clip: float


@dataclass(frozen=True)
class StairSpec:
    step_height: float
    step_depth: float = 0.20
    step_count: int = 10
    start_x: float = 0.75
    half_width: float = 1.50
    landing_length: float = 2.00
    riser_ramp_length: float = 0.0

    @property
    def end_x(self) -> float:
        return self.start_x + self.step_depth * self.step_count

    @property
    def top_height(self) -> float:
        return self.step_height * self.step_count


STAGE_HEIGHTS: Final = {
    "flat": 0.0,
    "h05": 0.05,
    "h10": 0.10,
    "h15": 0.15,
}


def load_control_parameters(
    path: Path = WALK_PARAM,
    profile: str = "walk",
) -> ControlParameters:
    """Load either the official walk or stand controller parameters."""
    if profile not in CONTROL_PROFILES:
        choices = ", ".join(CONTROL_PROFILES)
        raise ValueError(f"Unknown control profile {profile!r}; expected one of {choices}")
    with path.open("r", encoding="utf-8") as stream:
        robot_cfg = yaml.safe_load(stream)["HumanoidRobotCfg"]

    control = robot_cfg["control"]
    normalization = robot_cfg["normalization"]
    obs_scales = normalization["obs_scales"]
    clip_scales = normalization["clip_scales"]

    def vector(name: str) -> np.ndarray:
        value = np.asarray(control[name], dtype=np.float32)
        if value.shape != (ACTION_SIZE,):
            raise ValueError(
                f"{path}: control.{name} has shape {value.shape}, expected "
                f"({ACTION_SIZE},)"
            )
        return value

    kp = vector("kp")
    kd = vector("kd")
    default_angle = vector("default_angle")
    if profile == "stand":
        with STAND_PARAM.open("r", encoding="utf-8-sig") as stream:
            stand = yaml.safe_load(stream)

        def stand_vector(name: str) -> np.ndarray:
            value = np.asarray(stand[name], dtype=np.float32)
            if value.shape != (ACTION_SIZE,):
                raise ValueError(
                    f"{STAND_PARAM}: {name} has shape {value.shape}, expected "
                    f"({ACTION_SIZE},)"
                )
            return value

        kp = stand_vector("stand_kp")
        kd = stand_vector("stand_kd")
        default_angle = stand_vector("stand_pos")

    return ControlParameters(
        action_scale=vector("action_scale"),
        torque_limit=vector("user_torque_limit"),
        kp=kp,
        kd=kd,
        default_angle=default_angle,
        gyro_scale=float(obs_scales["ang_vel"]),
        dof_pos_scale=float(obs_scales["dof_pos"]),
        dof_vel_scale=float(obs_scales["dof_vel"]),
        observation_clip=float(clip_scales["clip_observations"]),
        action_clip=float(clip_scales["clip_actions"]),
    )


def stair_spec_for_stage(stage: str) -> StairSpec:
    try:
        height = STAGE_HEIGHTS[stage]
    except KeyError as exc:
        choices = ", ".join(STAGE_HEIGHTS)
        raise ValueError(f"Unknown stage {stage!r}; expected one of {choices}") from exc
    return StairSpec(step_height=height)
