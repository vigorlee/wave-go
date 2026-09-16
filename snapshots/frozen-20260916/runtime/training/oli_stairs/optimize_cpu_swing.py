"""Optimize a short, phase-varying residual for one OLI stair step."""

from __future__ import annotations

import argparse
import json
import multiprocessing
import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import numpy as np

from training.oli_stairs.constants import (
    ACTION_SIZE,
    CONTROL_PROFILES,
    JOINT_NAMES,
    STAGE_HEIGHTS,
    StairSpec,
    load_control_parameters,
)
from training.oli_stairs.cpu_residual_env import NumpyActor, OliCpuResidualEnv


_ENV: OliCpuResidualEnv | None = None
_INDICES: np.ndarray | None = None
_KNOT_PHASES: np.ndarray | None = None
_SWING_STEPS = 0
_SETTLE_STEPS = 0
_TARGET_STEP = 1
_HOLD_FINAL_KNOT = False
_HOLD_INITIAL_KNOT = False
_HOLD_SETTLE_ACTION = False
_SEPARATE_SETTLE_ACTION = False
_SETTLE_KNOT_COUNT = 1
_BASE_ADVANCE = -0.10
_REQUIRED_SUPPORT_LEG = "either"
_PRESERVE_SUPPORT_LEG = "none"
_REQUIRED_TREAD_FEET = "none"
_TREAD_HEIGHT_TOLERANCE = 0.05
_INITIAL_SUPPORT_RUN = 0
_SEARCH_OBJECTIVE = "balanced"
_ENDPOINT_SELECTION = "best"
_TARGET_GEOM_ID = -1
_TARGET_RAMP_GEOM_ID = -1
_PRESERVE_GEOM_ID = -1
_PRESERVE_STEP = 1
_FOOT_GEOM_IDS: tuple[frozenset[int], frozenset[int]] = (
    frozenset(),
    frozenset(),
)


def _initialize_worker(
    model_path: Path,
    policy_path: Path,
    trace_path: Path,
    spec: StairSpec,
    reset_step: int,
    residual_scale: float,
    speed: float,
    lateral_speed: float,
    lateral_hold_kp: float,
    lateral_hold_limit: float,
    yaw_rate: float,
    yaw_rate_start_step: int,
    history_lateral_speed: float,
    episode_length: int,
    indices: tuple[int, ...],
    knot_phases: tuple[float, ...],
    swing_steps: int,
    settle_steps: int,
    target_step: int,
    preserve_support_step: int,
    hold_final_knot: bool,
    hold_initial_knot: bool,
    hold_settle_action: bool,
    separate_settle_action: bool,
    settle_knot_count: int,
    control_profile: str,
    stand_gain_blend: float,
    policy_action_scale: float,
    balance_assist_scale: float,
    balance_assist_decay_start_step: int,
    balance_assist_decay_end_step: int,
    torque_limit_scale: float,
    swing_collision_mask_leg: str,
    swing_collision_mask_start_step: int,
    swing_collision_mask_end_step: int,
    balance_assist_target_x_offset: float,
    balance_assist_target_z_offset: float,
    heading_hold_kp: float,
    heading_hold_limit: float,
    heading_hold_start_x: float,
    heading_hold_start_step: int,
    base_advance: float,
    required_support_leg: str,
    preserve_support_leg: str,
    required_tread_feet: str,
    tread_height_tolerance: float,
    initial_support_run: int,
    search_objective: str,
    endpoint_selection: str,
) -> None:
    global _ENV, _INDICES, _KNOT_PHASES, _SWING_STEPS, _SETTLE_STEPS, _TARGET_STEP
    global _HOLD_FINAL_KNOT, _HOLD_INITIAL_KNOT, _HOLD_SETTLE_ACTION
    global _SEPARATE_SETTLE_ACTION, _SETTLE_KNOT_COUNT
    global _TARGET_GEOM_ID, _TARGET_RAMP_GEOM_ID, _PRESERVE_GEOM_ID
    global _PRESERVE_STEP, _FOOT_GEOM_IDS, _BASE_ADVANCE
    global _REQUIRED_SUPPORT_LEG, _PRESERVE_SUPPORT_LEG, _REQUIRED_TREAD_FEET
    global _TREAD_HEIGHT_TOLERANCE, _INITIAL_SUPPORT_RUN, _SEARCH_OBJECTIVE
    global _ENDPOINT_SELECTION
    _ENV = OliCpuResidualEnv(
        model_path,
        policy_path,
        spec,
        residual_scale=residual_scale,
        residual_start_x=-np.inf,
        episode_length=episode_length,
        speed=speed,
        lateral_speed=lateral_speed,
        lateral_hold_kp=lateral_hold_kp,
        lateral_hold_limit=lateral_hold_limit,
        yaw_rate=yaw_rate,
        yaw_rate_start_step=yaw_rate_start_step,
        history_lateral_speed=history_lateral_speed,
        history_reset_x=0.99,
        reset_trace_path=trace_path,
        reset_trace_start_step=reset_step,
        reset_trace_end_step=reset_step,
        reset_trace_probability=1.0,
        control_profile=control_profile,
        stand_gain_blend=stand_gain_blend,
        policy_action_scale=policy_action_scale,
        balance_assist_scale=balance_assist_scale,
        balance_assist_decay_start_step=balance_assist_decay_start_step,
        balance_assist_decay_end_step=balance_assist_decay_end_step,
        torque_limit_scale=torque_limit_scale,
        swing_collision_mask_leg=swing_collision_mask_leg,
        swing_collision_mask_start_step=swing_collision_mask_start_step,
        swing_collision_mask_end_step=swing_collision_mask_end_step,
        balance_assist_target_x_offset=balance_assist_target_x_offset,
        balance_assist_target_z_offset=balance_assist_target_z_offset,
        heading_hold_kp=heading_hold_kp,
        heading_hold_limit=heading_hold_limit,
        heading_hold_start_x=heading_hold_start_x,
        heading_hold_start_step=heading_hold_start_step,
    )
    _INDICES = np.asarray(indices, dtype=np.int32)
    _KNOT_PHASES = np.asarray(knot_phases, dtype=np.float64)
    _SWING_STEPS = swing_steps
    _SETTLE_STEPS = settle_steps
    _TARGET_STEP = target_step
    _PRESERVE_STEP = preserve_support_step
    _HOLD_FINAL_KNOT = hold_final_knot
    _HOLD_INITIAL_KNOT = hold_initial_knot
    _HOLD_SETTLE_ACTION = hold_settle_action
    _SEPARATE_SETTLE_ACTION = separate_settle_action
    _SETTLE_KNOT_COUNT = settle_knot_count
    _BASE_ADVANCE = base_advance
    _REQUIRED_SUPPORT_LEG = required_support_leg
    _PRESERVE_SUPPORT_LEG = preserve_support_leg
    _REQUIRED_TREAD_FEET = required_tread_feet
    _TREAD_HEIGHT_TOLERANCE = tread_height_tolerance
    _INITIAL_SUPPORT_RUN = initial_support_run
    _SEARCH_OBJECTIVE = search_objective
    _ENDPOINT_SELECTION = endpoint_selection
    _TARGET_GEOM_ID = _ENV.model.geom(f"training_stair_{target_step:02d}").id
    _TARGET_RAMP_GEOM_ID = (
        _ENV.model.geom(f"training_stair_ramp_{target_step:02d}").id
        if spec.riser_ramp_length > 0.0
        else -1
    )
    _PRESERVE_GEOM_ID = _ENV.model.geom(
        f"training_stair_{preserve_support_step:02d}"
    ).id
    _FOOT_GEOM_IDS = tuple(
        frozenset(_ENV.model.geom(name).id for name in names)
        for names in (
            (
                "left_foot",
                "contact_foot_heel_L",
                "contact_foot_center_L",
                "contact_foot_tip_L",
            ),
            (
                "right_foot",
                "contact_foot_heel_R",
                "contact_foot_center_R",
                "contact_foot_tip_R",
            ),
        )
    )


def _interpolate_knots(knots: np.ndarray, phase: float) -> np.ndarray:
    if _KNOT_PHASES is None:
        raise RuntimeError("CEM worker was not initialized")
    values = np.zeros(knots.shape[1], dtype=np.float32)
    for index in range(knots.shape[1]):
        values[index] = np.interp(phase, _KNOT_PHASES, knots[:, index])
    return values


def _evaluate(candidate: np.ndarray) -> dict:
    if _ENV is None or _INDICES is None or _KNOT_PHASES is None:
        raise RuntimeError("CEM worker was not initialized")

    internal_parameter_count = (len(_KNOT_PHASES) - 2) * _INDICES.size
    internal_knots = np.asarray(
        candidate[:internal_parameter_count], dtype=np.float32
    ).reshape(len(_KNOT_PHASES) - 2, _INDICES.size)
    final_knot = (
        internal_knots[-1:]
        if _HOLD_FINAL_KNOT
        else np.zeros((1, _INDICES.size), dtype=np.float32)
    )
    initial_knot = (
        internal_knots[:1]
        if _HOLD_INITIAL_KNOT
        else np.zeros((1, _INDICES.size), dtype=np.float32)
    )
    knots = np.vstack(
        (
            initial_knot,
            internal_knots,
            final_knot,
        )
    )
    settle_action = (
        np.asarray(candidate[internal_parameter_count:], dtype=np.float32).reshape(
            _SETTLE_KNOT_COUNT, _INDICES.size
        )
        if _SEPARATE_SETTLE_ACTION
        else knots[-1:]
    )
    _ENV.reset(seed=0, options={"use_snapshot": True, "snapshot_index": 0})
    start_x = float(_ENV.data.qpos[0])
    start_y = float(_ENV.data.qpos[1])
    target_riser = _ENV.spec.start_x + (_TARGET_STEP - 1) * _ENV.spec.step_depth
    target_height = _TARGET_STEP * _ENV.spec.step_height
    max_foot_x = np.asarray(_ENV.data.site_xpos[_ENV.foot_site_ids, 0]).copy()
    initial_foot_x = max_foot_x.copy()
    required_leg_indices = {
        "either": (0, 1),
        "left": (0,),
        "right": (1,),
        "both": (0, 1),
    }[_REQUIRED_SUPPORT_LEG]
    initial_required_foot_x = initial_foot_x[list(required_leg_indices)]
    initial_target_foot_x = (
        float(np.min(initial_required_foot_x))
        if _REQUIRED_SUPPORT_LEG == "both"
        else float(np.max(initial_required_foot_x))
    )
    max_foot_z = np.asarray(_ENV.data.site_xpos[_ENV.foot_site_ids, 2]).copy()
    stable_support_run = _INITIAL_SUPPORT_RUN
    longest_stable_support = _INITIAL_SUPPORT_RUN
    raw_support_run = _INITIAL_SUPPORT_RUN
    longest_raw_support = _INITIAL_SUPPORT_RUN
    preserve_support_run = 0
    longest_preserve_support = 0
    ramp_support_run = 0
    longest_ramp_support = 0
    support_leg = -1
    min_up_z = 1.0
    min_base_z = float(_ENV.data.qpos[2])
    start_base_z = min_base_z
    max_base_z = start_base_z
    min_target_height_error = np.inf
    terminated = truncated = False
    reward_sum = 0.0
    info: dict = {}
    best_endpoint: dict | None = None

    total_steps = _SWING_STEPS + _SETTLE_STEPS
    for step_index in range(total_steps):
        action = np.zeros(ACTION_SIZE, dtype=np.float32)
        if step_index < _SWING_STEPS:
            phase = step_index / max(_SWING_STEPS - 1, 1)
            action[_INDICES] = _interpolate_knots(knots, phase)
        elif _HOLD_SETTLE_ACTION:
            settle_phase = (step_index - _SWING_STEPS) / max(
                _SETTLE_STEPS - 1, 1
            )
            settle_phases = np.linspace(0.0, 1.0, len(settle_action))
            action[_INDICES] = np.asarray(
                [
                    np.interp(
                        settle_phase,
                        settle_phases,
                        settle_action[:, column],
                    )
                    for column in range(_INDICES.size)
                ],
                dtype=np.float32,
            )
        _, reward, terminated, truncated, info = _ENV.step(action)
        reward_sum += reward

        feet = np.asarray(_ENV.data.site_xpos[_ENV.foot_site_ids])
        max_foot_x = np.maximum(max_foot_x, feet[:, 0])
        max_foot_z = np.maximum(max_foot_z, feet[:, 2])
        beyond_target = feet[:, 0] >= target_riser
        required_beyond_target = np.zeros(2, dtype=bool)
        required_beyond_target[list(required_leg_indices)] = beyond_target[
            list(required_leg_indices)
        ]
        if np.any(required_beyond_target):
            min_target_height_error = min(
                min_target_height_error,
                float(
                    np.min(
                        np.abs(feet[required_beyond_target, 2] - target_height)
                    )
                ),
            )
        quaternion = _ENV.data.qpos[3:7]
        up_z = 1.0 - 2.0 * (quaternion[1] ** 2 + quaternion[2] ** 2)
        min_up_z = min(min_up_z, float(up_z))
        min_base_z = min(min_base_z, float(_ENV.data.qpos[2]))
        max_base_z = max(max_base_z, float(_ENV.data.qpos[2]))

        supported = []
        preserve_supported = []
        ramp_supported = []
        for leg_index, foot in enumerate(feet):
            terrain = _ENV._terrain_height(float(foot[0]))
            tread_contact_found = any(
                _ENV.last_control_contact_counts.get(
                    tuple(sorted((_TARGET_GEOM_ID, foot_geom_id))),
                    0,
                )
                >= 3
                for foot_geom_id in _FOOT_GEOM_IDS[leg_index]
            )
            ramp_contact_found = _TARGET_RAMP_GEOM_ID >= 0 and any(
                _ENV.last_control_contact_counts.get(
                    tuple(sorted((_TARGET_RAMP_GEOM_ID, foot_geom_id))),
                    0,
                )
                >= 3
                for foot_geom_id in _FOOT_GEOM_IDS[leg_index]
            )
            preserve_contact_found = any(
                _ENV.last_control_contact_counts.get(
                    tuple(sorted((_PRESERVE_GEOM_ID, foot_geom_id))),
                    0,
                )
                >= 3
                for foot_geom_id in _FOOT_GEOM_IDS[leg_index]
            )
            if (
                tread_contact_found
                and abs(terrain - target_height) < 1.0e-6
                and abs(float(foot[2]) - terrain) < 0.04
                and abs(float(_ENV.data.qvel[0])) < 3.0
            ):
                supported.append(leg_index)
            preserve_height = _PRESERVE_STEP * _ENV.spec.step_height
            if (
                preserve_contact_found
                and abs(terrain - preserve_height) < 1.0e-6
                and abs(float(foot[2]) - terrain) < 0.04
                and abs(float(_ENV.data.qvel[0])) < 3.0
            ):
                preserve_supported.append(leg_index)
            if (
                ramp_contact_found
                and target_height - _ENV.spec.step_height <= terrain <= target_height
                and abs(float(foot[2]) - terrain) < 0.04
                and abs(float(_ENV.data.qvel[0])) < 3.0
            ):
                ramp_supported.append(leg_index)
        required_support_met = {
            "either": bool(supported),
            "left": 0 in supported,
            "right": 1 in supported,
            "both": 0 in supported and 1 in supported,
        }[_REQUIRED_SUPPORT_LEG]
        preserve_support_met = {
            "none": True,
            "either": bool(preserve_supported),
            "left": 0 in preserve_supported,
            "right": 1 in preserve_supported,
            "both": 0 in preserve_supported and 1 in preserve_supported,
        }[_PRESERVE_SUPPORT_LEG]
        required_ramp_support_met = {
            "either": bool(ramp_supported),
            "left": 0 in ramp_supported,
            "right": 1 in ramp_supported,
            "both": 0 in ramp_supported and 1 in ramp_supported,
        }[_REQUIRED_SUPPORT_LEG]
        required_tread_indices = {
            "none": (),
            "left": (0,),
            "right": (1,),
            "both": (0, 1),
        }[_REQUIRED_TREAD_FEET]
        exposed_tread_depth = max(
            _ENV.spec.step_depth - _ENV.spec.riser_ramp_length,
            0.01,
        )
        tread_margin = min(0.003, 0.15 * exposed_tread_depth)
        tread_x_low = target_riser + tread_margin
        tread_x_high = target_riser + exposed_tread_depth - tread_margin
        tread_target_x = 0.5 * (tread_x_low + tread_x_high)
        tread_placement_by_leg = [
            bool(
                tread_x_low <= float(feet[leg_index, 0]) <= tread_x_high
                and abs(float(feet[leg_index, 2]) - target_height)
                < _TREAD_HEIGHT_TOLERANCE
            )
            for leg_index in required_tread_indices
        ]
        required_tread_placed = all(tread_placement_by_leg)
        raw_support_run = raw_support_run + 1 if required_support_met else 0
        longest_raw_support = max(longest_raw_support, raw_support_run)
        preserve_support_run = (
            preserve_support_run + 1 if preserve_support_met else 0
        )
        longest_preserve_support = max(
            longest_preserve_support,
            preserve_support_run,
        )
        ramp_support_run = (
            ramp_support_run + 1 if required_ramp_support_met else 0
        )
        longest_ramp_support = max(longest_ramp_support, ramp_support_run)
        if (
            required_support_met
            and required_tread_placed
            and up_z > 0.65
            and _ENV.data.qpos[2] > target_height + 0.72
        ):
            stable_support_run += 1
            support_leg = (
                -2 if _REQUIRED_SUPPORT_LEG == "both" else supported[0]
            )
            longest_stable_support = max(longest_stable_support, stable_support_run)
        else:
            stable_support_run = 0

        quaternion = _ENV.data.qpos[3:7]
        body_up_z = 1.0 - 2.0 * (quaternion[1] ** 2 + quaternion[2] ** 2)
        yaw = np.arctan2(
            2.0 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]),
            1.0 - 2.0 * (quaternion[2] ** 2 + quaternion[3] ** 2),
        )
        final_feet = np.asarray(_ENV.data.site_xpos[_ENV.foot_site_ids])
        base_velocity = np.asarray(_ENV.data.qvel[:6], dtype=np.float64)
        linear_speed = float(np.linalg.norm(base_velocity[:3]))
        angular_speed = float(np.linalg.norm(base_velocity[3:]))
        desired_foot_y = np.asarray((0.13, -0.13))
        lateral_errors = np.abs(final_feet[:, 1] - desired_foot_y)
        support_center_error = abs(
            float(_ENV.data.qpos[1]) - float(np.mean(final_feet[:, 1]))
        )
        base_lateral_drift = abs(float(_ENV.data.qpos[1]) - start_y)
        base_aligned = float(_ENV.data.qpos[0]) > target_riser + _BASE_ADVANCE
        endpoint_qualified = (
            stable_support_run >= 20
            and abs(float(yaw)) < 0.25
            and abs(float(_ENV.data.qpos[1])) < 0.18
            and body_up_z > 0.80
            and min_up_z > 0.72
            and float(np.max(lateral_errors)) < 0.15
            and abs(float(base_velocity[0])) < 0.70
            and abs(float(base_velocity[1])) < 0.40
            and abs(float(base_velocity[2])) < 0.50
            and angular_speed < 1.50
            and base_aligned
            and (_PRESERVE_SUPPORT_LEG == "none" or preserve_support_run >= 5)
        )
        required_max_foot_x = max_foot_x[list(required_leg_indices)]
        target_foot_x = (
            float(np.min(required_max_foot_x))
            if _REQUIRED_SUPPORT_LEG == "both"
            else float(np.max(required_max_foot_x))
        )
        target_crossing = max(0.0, target_foot_x - target_riser)
        tread_overshoot = max(
            0.0,
            target_foot_x - (target_riser + _ENV.spec.step_depth - 0.03),
        )
        foot_progress_by_leg = np.maximum(0.0, max_foot_x - initial_foot_x)
        foot_progress = max(0.0, target_foot_x - initial_target_foot_x)
        second_foot_progress = float(np.min(foot_progress_by_leg))
        stable_progress = foot_progress if min_up_z > 0.72 else 0.0
        base_height_progress = max(0.0, max_base_z - start_base_z)
        landing_quality = max(
            0.0,
            0.20
            - (
                min_target_height_error
                if np.isfinite(min_target_height_error)
                else 0.20
            ),
        )
        foot_lateral_quality = np.clip(
            1.0 - float(np.max(lateral_errors)) / 0.30,
            0.0,
            1.0,
        )
        tread_placement_quality = float(
            np.mean(
                [
                    np.clip(
                        1.0
                        - max(
                            tread_x_low - float(final_feet[leg_index, 0]),
                            float(final_feet[leg_index, 0]) - tread_x_high,
                            0.0,
                        )
                        / 0.20,
                        0.0,
                        1.0,
                    )
                    * np.clip(
                        1.0
                        - abs(float(final_feet[leg_index, 2]) - target_height)
                        / 0.15,
                        0.0,
                        1.0,
                    )
                    for leg_index in required_tread_indices
                ]
            )
            if required_tread_indices
            else 1.0
        )
        target_tread_placement_quality = float(
            np.mean(
                [
                    np.clip(
                        1.0
                        - abs(
                            float(final_feet[leg_index, 0]) - tread_target_x
                        )
                        / 0.20,
                        0.0,
                        1.0,
                    )
                    * np.clip(
                        1.0
                        - abs(float(final_feet[leg_index, 2]) - target_height)
                        / 0.15,
                        0.0,
                        1.0,
                    )
                    * np.clip(
                        1.0
                        - abs(
                            float(final_feet[leg_index, 1])
                            - float(desired_foot_y[leg_index])
                        )
                        / 0.30,
                        0.0,
                        1.0,
                    )
                    for leg_index in required_leg_indices
                ]
            )
        )
        velocity_quality = float(
            np.mean(
                (
                    np.clip(1.0 - abs(float(base_velocity[0])) / 0.70, 0.0, 1.0),
                    np.clip(1.0 - abs(float(base_velocity[1])) / 0.40, 0.0, 1.0),
                    np.clip(1.0 - abs(float(base_velocity[2])) / 0.50, 0.0, 1.0),
                    np.clip(1.0 - angular_speed / 1.50, 0.0, 1.0),
                )
            )
        )
        support_fraction = min(stable_support_run / 20.0, 1.0)
        target_height_quality = (
            float(np.clip(1.0 - min_target_height_error / 0.08, 0.0, 1.0))
            if np.isfinite(min_target_height_error)
            else 0.0
        )
        reach_progress_fraction = min(foot_progress / 0.16, 1.0)
        reach_stability_quality = float(
            np.clip(
                1.0 - max(0.0, start_x - float(_ENV.data.qpos[0])) / 0.15,
                0.0,
                1.0,
            )
            * np.clip(
                1.0 - max(0.0, start_base_z - float(_ENV.data.qpos[2])) / 0.15,
                0.0,
                1.0,
            )
            * np.clip(
                1.0 - base_lateral_drift / 0.20,
                0.0,
                1.0,
            )
            * np.clip((min_up_z - 0.72) / 0.20, 0.0, 1.0)
            * np.clip(1.0 - linear_speed / 2.0, 0.0, 1.0)
        )
        reach_objective_bonus = (
            reach_stability_quality
            * (
                10_000_000.0 * min(foot_progress, 0.40)
                + 8_000_000.0 * min(target_crossing, 0.20)
                + 3_000_000.0 * landing_quality
                + 7_000_000.0
                * reach_progress_fraction
                * tread_placement_quality
                + 4_000_000.0
                * reach_progress_fraction
                * target_height_quality
                + 12_000_000.0 * target_tread_placement_quality
                + 8_000_000.0 * target_height_quality
                + 12_000_000.0 * min(raw_support_run / 5.0, 1.0)
                + 30_000_000.0 * min(raw_support_run / 20.0, 1.0)
                + 20_000_000.0 * min(preserve_support_run / 5.0, 1.0)
            )
            if _SEARCH_OBJECTIVE == "reach"
            else 0.0
        )
        contact_stability_scale = (
            reach_stability_quality if _SEARCH_OBJECTIVE == "reach" else 1.0
        )
        scored_support_fraction = support_fraction * contact_stability_scale
        alignment_quality = float(
            np.mean(
                (
                    np.clip(1.0 - abs(float(yaw)) / 0.25, 0.0, 1.0),
                    np.clip(
                        1.0 - abs(float(_ENV.data.qpos[1])) / 0.18,
                        0.0,
                        1.0,
                    ),
                    foot_lateral_quality,
                    np.clip((body_up_z - 0.80) / 0.20, 0.0, 1.0),
                    float(base_aligned),
                )
            )
        )
        support_center_quality = float(
            np.clip(1.0 - support_center_error / 0.18, 0.0, 1.0)
        )
        settle_objective_bonus = (
            support_fraction
            * (
                20_000_000.0 * velocity_quality
                + 10_000_000.0 * alignment_quality
                + 20_000_000.0 * support_center_quality
                - 5_000_000.0 * linear_speed
                - 3_000_000.0 * angular_speed
                - 8_000_000.0 * support_center_error
            )
            if _SEARCH_OBJECTIVE == "settle"
            else 0.0
        )
        endpoint_score = (
            20_000_000.0 * float(endpoint_qualified)
            + 4_000_000.0
            * float(ramp_support_run > 0)
            * contact_stability_scale
            + 250_000.0
            * min(ramp_support_run, 20)
            * contact_stability_scale
            + 4_000_000.0
            * float(stable_support_run > 0)
            * contact_stability_scale
            + 250_000.0
            * min(stable_support_run, 20)
            * contact_stability_scale
            + 8_000_000.0
            * scored_support_fraction
            * np.clip(1.0 - abs(float(yaw)) / 0.50, 0.0, 1.0)
            + 8_000_000.0
            * scored_support_fraction
            * np.clip(
                1.0 - abs(float(_ENV.data.qpos[1])) / 0.40,
                0.0,
                1.0,
            )
            + 6_000_000.0 * scored_support_fraction * foot_lateral_quality
            + 6_000_000.0 * scored_support_fraction * velocity_quality
            + 6_000_000.0
            * float(stable_support_run > 0 and base_aligned)
            * contact_stability_scale
            + 3_000_000.0
            * np.clip(float(_ENV.data.qpos[0]) - (target_riser - 0.45), 0.0, 0.60)
            + 330_000.0 * min(target_crossing, 0.20)
            + 300_000.0 * min(stable_progress, 0.40)
            + 900_000.0 * min(second_foot_progress, 0.30)
            + 600_000.0 * landing_quality
            + 1_500_000.0 * min(base_height_progress, 0.20)
            + reach_objective_bonus
            + (
                25_000_000.0 * reach_progress_fraction
                + 15_000_000.0 * min(target_crossing / 0.10, 1.0)
                if _SEARCH_OBJECTIVE == "reach"
                else 0.0
            )
            + settle_objective_bonus
            + 10_000.0 * float(_ENV.data.qpos[0] - start_x)
            + 500.0 * reward_sum
            + 20_000.0 * body_up_z
            + 10_000.0 * min_up_z
            - 1_000_000.0 * abs(float(_ENV.data.qpos[1]))
            - 12_000_000.0 * base_lateral_drift
            - 1_200_000.0 * abs(float(yaw))
            - 4_000_000.0 * float(np.sum(lateral_errors))
            - 2_000_000.0 * tread_overshoot
            - 500_000.0 * linear_speed
            - 400_000.0 * angular_speed
            - 20_000.0 * max(0.0, 0.72 - min_base_z)
            - 250.0 * float(np.sum(np.square(candidate)))
            - 1_000_000.0 * float(terminated)
            - 1_000_000.0 * float(body_up_z < 0.80 or min_up_z < 0.72)
        )
        endpoint = {
            "score": float(endpoint_score),
            "qualified": bool(endpoint_qualified),
            "target_step": int(_TARGET_STEP),
            "required_support_leg": _REQUIRED_SUPPORT_LEG,
            "preserve_support_leg": _PRESERVE_SUPPORT_LEG,
            "preserve_support_step": int(_PRESERVE_STEP),
            "required_tread_feet": _REQUIRED_TREAD_FEET,
            "tread_height_tolerance": _TREAD_HEIGHT_TOLERANCE,
            "initial_support_run": int(_INITIAL_SUPPORT_RUN),
            "search_objective": _SEARCH_OBJECTIVE,
            "target_support": bool(stable_support_run > 0),
            "target_ramp_support": bool(ramp_support_run > 0),
            "required_tread_placed": bool(required_tread_placed),
            "tread_placement_quality": float(tread_placement_quality),
            "target_tread_placement_quality": float(
                target_tread_placement_quality
            ),
            "base_aligned": bool(base_aligned),
            "steps": int(_ENV.steps),
            "selected_step": int(_ENV.steps),
            "longest_stable_support": int(longest_stable_support),
            "terminal_stable_support": int(stable_support_run),
            "longest_raw_support": int(longest_raw_support),
            "terminal_raw_support": int(raw_support_run),
            "longest_preserve_support": int(longest_preserve_support),
            "terminal_preserve_support": int(preserve_support_run),
            "longest_ramp_support": int(longest_ramp_support),
            "terminal_ramp_support": int(ramp_support_run),
            "support_leg": int(support_leg),
            "max_foot_x": max_foot_x.tolist(),
            "tread_overshoot": float(tread_overshoot),
            "foot_progress": float(foot_progress),
            "reach_stability_quality": float(reach_stability_quality),
            "second_foot_progress": float(second_foot_progress),
            "min_target_height_error": (
                float(min_target_height_error)
                if np.isfinite(min_target_height_error)
                else None
            ),
            "terminated_early": bool(terminated),
            "unstable": bool(body_up_z < 0.80 or min_up_z < 0.72),
            "max_foot_z": max_foot_z.tolist(),
            "final_x": float(_ENV.data.qpos[0]),
            "final_y": float(_ENV.data.qpos[1]),
            "final_z": float(_ENV.data.qpos[2]),
            "max_base_z": float(max_base_z),
            "base_height_progress": float(base_height_progress),
            "body_up_z": float(body_up_z),
            "min_up_z": float(min_up_z),
            "yaw_deg": float(np.degrees(yaw)),
            "foot_lateral_error": lateral_errors.tolist(),
            "support_center_error": float(support_center_error),
            "base_lateral_drift": float(base_lateral_drift),
            "final_feet": final_feet.tolist(),
            "base_velocity": base_velocity.tolist(),
            "linear_speed": float(linear_speed),
            "angular_speed": float(angular_speed),
            "action": np.asarray(candidate, dtype=float).tolist(),
        }
        if _ENDPOINT_SELECTION == "final":
            best_endpoint = endpoint
        elif _SEARCH_OBJECTIVE == "settle":
            # A settle search must optimize the state it can actually hold at
            # the end of the rollout.  Keep an earlier endpoint only when it
            # already passes every qualification gate; otherwise the newest
            # endpoint replaces it, even if a transient contact scored higher.
            if endpoint_qualified:
                if (
                    best_endpoint is None
                    or not best_endpoint["qualified"]
                    or endpoint_score > best_endpoint["score"]
                ):
                    best_endpoint = endpoint
            elif best_endpoint is None or not best_endpoint["qualified"]:
                best_endpoint = endpoint
        elif best_endpoint is None or endpoint_score > best_endpoint["score"]:
            best_endpoint = endpoint
        if terminated or truncated:
            break
    if best_endpoint is None:
        raise RuntimeError("candidate rollout produced no endpoint")
    return best_endpoint


def _default_mean(indices: list[int], internal_knot_count: int) -> np.ndarray:
    # A conservative sagittal seed: flex the left swing leg, extend it onto the
    # tread, and keep the right support leg slightly extended.
    lift = {0: -0.80, 3: 0.20, 4: -0.20, 6: 0.05, 9: -0.10, 10: 0.05, 14: -0.05}
    reach = {0: -0.90, 3: -0.15, 4: 0.00, 6: 0.00, 9: -0.15, 10: 0.05, 14: -0.05}
    land = {0: -0.40, 3: -0.30, 4: 0.10, 6: 0.00, 9: -0.10, 10: 0.00, 14: 0.00}
    templates = (lift, reach, land)
    knots = []
    for knot_index in range(internal_knot_count):
        template = templates[min(knot_index, len(templates) - 1)]
        knots.extend(template.get(index, 0.0) for index in indices)
    return np.asarray(knots, dtype=np.float64)


def _mirror_mean(mean: np.ndarray, indices: list[int], knot_count: int) -> np.ndarray:
    pair_map = {
        0: (6, 1.0), 1: (7, -1.0), 2: (8, -1.0),
        3: (9, 1.0), 4: (10, 1.0), 5: (11, -1.0),
        6: (0, 1.0), 7: (1, -1.0), 8: (2, -1.0),
        9: (3, 1.0), 10: (4, 1.0), 11: (5, -1.0),
        12: (12, -1.0), 13: (13, -1.0), 14: (14, 1.0),
    }
    lookup = {joint_index: column for column, joint_index in enumerate(indices)}
    knots = np.asarray(mean, dtype=np.float64).reshape(knot_count, len(indices))
    mirrored = np.zeros_like(knots)
    for output_column, joint_index in enumerate(indices):
        source_joint, sign = pair_map.get(joint_index, (joint_index, 1.0))
        if source_joint not in lookup:
            raise ValueError(
                f"cannot mirror joint {joint_index}: paired joint {source_joint} is absent"
            )
        mirrored[:, output_column] = sign * knots[:, lookup[source_joint]]
    return mirrored.reshape(-1)


def _pose_hold_mean(
    model_path: Path,
    policy_path: Path,
    trace_path: Path,
    reset_step: int,
    indices: list[int],
    knot_count: int,
    residual_scale: float,
    control_profile: str,
    policy_action_scale: float,
) -> np.ndarray:
    import mujoco

    records = json.loads(trace_path.expanduser().resolve().read_text(encoding="utf-8"))
    snapshot = next(
        (record for record in records if int(record["step"]) == reset_step),
        None,
    )
    if snapshot is None:
        raise ValueError(f"no step {reset_step} in {trace_path}")
    model = mujoco.MjModel.from_xml_path(str(model_path.expanduser().resolve()))
    qpos_ids = np.asarray(
        [model.joint(name).qposadr[0] for name in JOINT_NAMES], dtype=np.int32
    )
    control = load_control_parameters(profile=control_profile)
    joint_position = np.asarray(snapshot["state_qpos"], dtype=np.float64)[qpos_ids]
    base_action = policy_action_scale * NumpyActor(policy_path)(
        np.asarray(snapshot["observation"], dtype=np.float32)
    )
    hold_action = (
        (joint_position - control.default_angle)
        / control.action_scale
        - base_action
    ) / residual_scale
    return np.tile(hold_action[np.asarray(indices)], knot_count)


def _foot_target_mean(
    model_path: Path,
    policy_path: Path,
    trace_path: Path,
    reset_step: int,
    indices: list[int],
    knot_count: int,
    residual_scale: float,
    control_profile: str,
    policy_action_scale: float,
    spec: StairSpec,
    target_step: int,
    leg: str,
    clearance: float,
    preload_knots: int,
    preload_shift: float,
    preload_roll: float,
    sagittal_only: bool,
) -> np.ndarray:
    import mujoco

    records = json.loads(trace_path.expanduser().resolve().read_text(encoding="utf-8"))
    snapshot = next(
        (record for record in records if int(record["step"]) == reset_step),
        None,
    )
    if snapshot is None:
        raise ValueError(f"no step {reset_step} in {trace_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path.expanduser().resolve()))
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(snapshot["state_qpos"], dtype=np.float64)
    data.qvel[:] = np.asarray(snapshot["state_qvel"], dtype=np.float64)
    data.qacc_warmstart[:] = np.asarray(
        snapshot.get("state_qacc_warmstart", np.zeros(model.nv)),
        dtype=np.float64,
    )
    data.time = float(snapshot.get("state_time", 0.0))
    mujoco.mj_forward(model, data)

    control = load_control_parameters(profile=control_profile)
    actor_action = policy_action_scale * NumpyActor(policy_path)(
        np.asarray(snapshot["observation"], dtype=np.float32)
    )
    all_qpos_ids = np.asarray(
        [model.joint(name).qposadr[0] for name in JOINT_NAMES], dtype=np.int32
    )
    if "residual_action" in snapshot:
        source_residual = np.asarray(
            snapshot["residual_action"], dtype=np.float64
        )
        if source_residual.shape != (ACTION_SIZE,):
            raise ValueError("snapshot residual_action has the wrong shape")
        hold_total_action = actor_action + residual_scale * source_residual
    else:
        hold_total_action = (
            data.qpos[all_qpos_ids] - control.default_angle
        ) / control.action_scale

    def solve_leg(target_leg: str, target: np.ndarray) -> None:
        target_offset = 0 if target_leg == "left" else 6
        target_joint_names = JOINT_NAMES[target_offset : target_offset + 6]
        target_qpos_ids = np.asarray(
            [model.joint(name).qposadr[0] for name in target_joint_names],
            dtype=np.int32,
        )
        target_dof_ids = np.asarray(
            [model.joint(name).dofadr[0] for name in target_joint_names],
            dtype=np.int32,
        )
        target_joint_ids = np.asarray(
            [model.joint(name).id for name in target_joint_names],
            dtype=np.int32,
        )
        target_lower = model.jnt_range[target_joint_ids, 0]
        target_upper = model.jnt_range[target_joint_ids, 1]
        target_site_id = model.site(f"{target_leg}_foot_training_site").id
        for _ in range(80):
            jacobian_position = np.zeros((3, model.nv), dtype=np.float64)
            jacobian_rotation = np.zeros((3, model.nv), dtype=np.float64)
            mujoco.mj_jacSite(
                model,
                data,
                jacobian_position,
                jacobian_rotation,
                target_site_id,
            )
            error = target - data.site_xpos[target_site_id]
            if np.linalg.norm(error) < 1.0e-4:
                return
            jacobian = jacobian_position[:, target_dof_ids]
            delta = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + 2.0e-4 * np.eye(3),
                error,
            )
            data.qpos[target_qpos_ids] = np.clip(
                data.qpos[target_qpos_ids]
                + 0.6 * np.clip(delta, -0.08, 0.08),
                target_lower,
                target_upper,
            )
            mujoco.mj_forward(model, data)

    leg_offset = 0 if leg == "left" else 6
    leg_joint_names = JOINT_NAMES[leg_offset : leg_offset + 6]
    leg_qpos_ids = np.asarray(
        [model.joint(name).qposadr[0] for name in leg_joint_names],
        dtype=np.int32,
    )
    site_id = model.site(f"{leg}_foot_training_site").id
    initial_foot = np.asarray(data.site_xpos[site_id], dtype=np.float64).copy()
    initial_feet = {
        target_leg: np.asarray(
            data.site_xpos[model.site(f"{target_leg}_foot_training_site").id],
            dtype=np.float64,
        ).copy()
        for target_leg in ("left", "right")
    }

    target_riser = spec.start_x + (target_step - 1) * spec.step_depth
    exposed_tread_depth = max(
        spec.step_depth - spec.riser_ramp_length,
        0.01,
    )
    side = 1.0 if leg == "left" else -1.0
    final_target = np.asarray(
        [
            target_riser + 0.5 * exposed_tread_depth,
            float(data.qpos[1]) + side * 0.13,
            target_step * spec.step_height - 0.012,
        ],
        dtype=np.float64,
    )

    if preload_knots > 0 and preload_shift > 0.0:
        # The feet are fixed by contact in the rollout.  Solving the leg pose
        # for an opposite virtual pelvis displacement makes the joint target
        # generate the desired support-side body shift when applied dynamically.
        data.qpos[1] += side * preload_shift
        mujoco.mj_forward(model, data)
        for _ in range(3):
            solve_leg("left", initial_feet["left"])
            solve_leg("right", initial_feet["right"])
        preload_total_action = (
            data.qpos[all_qpos_ids] - control.default_angle
        ) / control.action_scale
    else:
        preload_total_action = hold_total_action
    hold_residual = (preload_total_action - actor_action) / residual_scale
    if preload_knots > 0 and preload_roll != 0.0:
        hold_residual = hold_residual.copy()
        for joint_index in (1, 7):
            hold_residual[joint_index] += side * preload_roll
        for joint_index in (5, 11):
            hold_residual[joint_index] -= side * preload_roll
    knot_actions = [
        hold_residual[np.asarray(indices)].copy()
        for _ in range(preload_knots)
    ]
    swing_knot_count = knot_count - preload_knots
    for knot_index in range(swing_knot_count):
        progress = (knot_index + 1) / swing_knot_count
        smooth_progress = progress * progress * (3.0 - 2.0 * progress)
        target = initial_foot + smooth_progress * (final_target - initial_foot)
        target[2] += clearance * np.sin(np.pi * progress)

        solve_leg(leg, target)

        total_action = hold_total_action.copy()
        leg_total_action = (
            data.qpos[leg_qpos_ids]
            - control.default_angle[leg_offset : leg_offset + 6]
        ) / control.action_scale[leg_offset : leg_offset + 6]
        leg_action_indices = (0, 3, 4) if sagittal_only else range(6)
        for local_index in leg_action_indices:
            total_action[leg_offset + local_index] = leg_total_action[local_index]
        residual = (total_action - actor_action) / residual_scale
        if preload_roll != 0.0:
            for joint_index in (1, 7):
                residual[joint_index] += side * preload_roll
            for joint_index in (5, 11):
                residual[joint_index] -= side * preload_roll
        knot_actions.append(residual[np.asarray(indices)])

    return np.clip(np.asarray(knot_actions), -1.0, 1.0).reshape(-1)


def _replay_best(
    *,
    model_path: Path,
    policy_path: Path,
    source_trace: Path,
    spec: StairSpec,
    reset_step: int,
    indices: list[int],
    knot_phases: list[float],
    swing_steps: int,
    settle_steps: int,
    residual_scale: float,
    speed: float,
    lateral_speed: float,
    lateral_hold_kp: float,
    lateral_hold_limit: float,
    yaw_rate: float,
    yaw_rate_start_step: int,
    history_lateral_speed: float,
    candidate: list[float],
    hold_final_knot: bool,
    hold_initial_knot: bool,
    hold_settle_action: bool,
    separate_settle_action: bool,
    settle_knot_count: int,
    trace_output: Path | None,
    video_output: Path | None,
    render_stride: int,
    control_profile: str,
    stand_gain_blend: float,
    policy_action_scale: float,
    balance_assist_scale: float,
    balance_assist_decay_start_step: int,
    balance_assist_decay_end_step: int,
    torque_limit_scale: float,
    swing_collision_mask_leg: str,
    swing_collision_mask_start_step: int,
    swing_collision_mask_end_step: int,
    balance_assist_target_x_offset: float,
    balance_assist_target_z_offset: float,
    selected_step: int | None,
    heading_hold_kp: float,
    heading_hold_limit: float,
    heading_hold_start_x: float,
    heading_hold_start_step: int,
) -> None:
    env = OliCpuResidualEnv(
        model_path,
        policy_path,
        spec,
        residual_scale=residual_scale,
        residual_start_x=-np.inf,
        episode_length=swing_steps + settle_steps,
        speed=speed,
        lateral_speed=lateral_speed,
        lateral_hold_kp=lateral_hold_kp,
        lateral_hold_limit=lateral_hold_limit,
        yaw_rate=yaw_rate,
        yaw_rate_start_step=yaw_rate_start_step,
        history_lateral_speed=history_lateral_speed,
        history_reset_x=0.99,
        reset_trace_path=source_trace,
        reset_trace_start_step=reset_step,
        reset_trace_end_step=reset_step,
        reset_trace_probability=1.0,
        control_profile=control_profile,
        stand_gain_blend=stand_gain_blend,
        policy_action_scale=policy_action_scale,
        balance_assist_scale=balance_assist_scale,
        balance_assist_decay_start_step=balance_assist_decay_start_step,
        balance_assist_decay_end_step=balance_assist_decay_end_step,
        torque_limit_scale=torque_limit_scale,
        swing_collision_mask_leg=swing_collision_mask_leg,
        swing_collision_mask_start_step=swing_collision_mask_start_step,
        swing_collision_mask_end_step=swing_collision_mask_end_step,
        balance_assist_target_x_offset=balance_assist_target_x_offset,
        balance_assist_target_z_offset=balance_assist_target_z_offset,
        heading_hold_kp=heading_hold_kp,
        heading_hold_limit=heading_hold_limit,
        heading_hold_start_x=heading_hold_start_x,
        heading_hold_start_step=heading_hold_start_step,
    )
    env.reset(seed=0, options={"use_snapshot": True, "snapshot_index": 0})
    selected = np.asarray(indices, dtype=np.int32)
    internal_parameter_count = (len(knot_phases) - 2) * selected.size
    internal = np.asarray(
        candidate[:internal_parameter_count], dtype=np.float32
    ).reshape(len(knot_phases) - 2, selected.size)
    final_knot = (
        internal[-1:]
        if hold_final_knot
        else np.zeros((1, selected.size), dtype=np.float32)
    )
    initial_knot = (
        internal[:1]
        if hold_initial_knot
        else np.zeros((1, selected.size), dtype=np.float32)
    )
    knots = np.vstack(
        (
            initial_knot,
            internal,
            final_knot,
        )
    )
    settle_action = (
        np.asarray(candidate[internal_parameter_count:], dtype=np.float32).reshape(
            settle_knot_count, selected.size
        )
        if separate_settle_action
        else knots[-1:]
    )
    phases = np.asarray(knot_phases, dtype=np.float64)
    records = []
    frames = []
    renderer = None
    if video_output is not None:
        import mujoco

        renderer = mujoco.Renderer(env.model, height=360, width=640)

    def record(step_index: int, residual: np.ndarray) -> None:
        records.append(
            {
                "step": step_index,
                "source_step": reset_step,
                "time": float(env.data.time),
                "base": env.data.qpos[:3].tolist(),
                "quaternion": env.data.qpos[3:7].tolist(),
                "feet": env.data.site_xpos[env.foot_site_ids].tolist(),
                "support_height": float(env._support_height()),
                "residual_action": residual.tolist(),
                "state_qpos": env.data.qpos.tolist(),
                "state_qvel": env.data.qvel.tolist(),
                "state_qacc_warmstart": env.data.qacc_warmstart.tolist(),
                "state_time": float(env.data.time),
                "observation": env.observation.tolist(),
            }
        )

    total_steps = min(
        swing_steps + settle_steps,
        selected_step if selected_step is not None else swing_steps + settle_steps,
    )
    last_residual = np.zeros(ACTION_SIZE, dtype=np.float32)
    for step_index in range(total_steps):
        residual = np.zeros(ACTION_SIZE, dtype=np.float32)
        if step_index < swing_steps:
            phase = step_index / max(swing_steps - 1, 1)
            residual[selected] = np.asarray(
                [np.interp(phase, phases, knots[:, index]) for index in range(selected.size)],
                dtype=np.float32,
            )
        elif hold_settle_action:
            settle_phase = (step_index - swing_steps) / max(settle_steps - 1, 1)
            settle_phases = np.linspace(0.0, 1.0, len(settle_action))
            residual[selected] = np.asarray(
                [
                    np.interp(
                        settle_phase,
                        settle_phases,
                        settle_action[:, column],
                    )
                    for column in range(selected.size)
                ],
                dtype=np.float32,
            )
        record(step_index, residual)
        _, _, terminated, truncated, _ = env.step(residual)
        last_residual = residual
        if renderer is not None and step_index % render_stride == 0:
            renderer.update_scene(env.data, camera="track")
            frames.append(renderer.render())
        if terminated or truncated:
            break
    record(env.steps, last_residual)

    if trace_output is not None:
        resolved_trace = trace_output.expanduser().resolve()
        resolved_trace.parent.mkdir(parents=True, exist_ok=True)
        resolved_trace.write_text(
            json.dumps(records, separators=(",", ":")) + "\n", encoding="utf-8"
        )
        print(resolved_trace)
    if renderer is not None and video_output is not None:
        import mediapy as media

        renderer.close()
        resolved_video = video_output.expanduser().resolve()
        resolved_video.parent.mkdir(parents=True, exist_ok=True)
        media.write_video(
            resolved_video,
            frames,
            fps=round(1.0 / env.control_dt / render_stride),
        )
        print(resolved_video)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h15")
    parser.add_argument("--step-count", type=int, default=10)
    parser.add_argument("--step-depth", type=float, default=0.28)
    parser.add_argument("--landing-length", type=float, default=3.20)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--riser-ramp-length", type=float, default=0.0)
    parser.add_argument("--reset-step", type=int, default=286)
    parser.add_argument("--target-step", type=int, default=1)
    parser.add_argument("--base-advance", type=float, default=-0.10)
    parser.add_argument(
        "--required-support-leg",
        choices=("either", "left", "right", "both"),
        default="either",
    )
    parser.add_argument(
        "--preserve-support-leg",
        choices=("none", "either", "left", "right", "both"),
        default="none",
    )
    parser.add_argument("--preserve-support-step", type=int)
    parser.add_argument(
        "--required-tread-feet",
        choices=("none", "left", "right", "both"),
        default="none",
    )
    parser.add_argument("--tread-height-tolerance", type=float, default=0.05)
    parser.add_argument("--initial-support-run", type=int, default=0)
    parser.add_argument(
        "--search-objective",
        choices=("balanced", "reach", "settle"),
        default="balanced",
    )
    parser.add_argument(
        "--endpoint-selection",
        choices=("best", "final"),
        default="best",
    )
    parser.add_argument("--indices", type=int, nargs="+", default=[0, 3, 4, 6, 9, 10, 14])
    parser.add_argument("--optimize-indices", type=int, nargs="+")
    parser.add_argument("--optimize-swing-knots", type=int, nargs="+")
    parser.add_argument("--knot-phases", type=float, nargs="+", default=[0.0, 0.25, 0.60, 0.82, 1.0])
    parser.add_argument("--swing-steps", type=int, default=60)
    parser.add_argument("--settle-steps", type=int, default=70)
    parser.add_argument("--residual-scale", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=0.30)
    parser.add_argument("--lateral-speed", type=float, default=0.05)
    parser.add_argument("--lateral-hold-kp", type=float, default=0.0)
    parser.add_argument("--lateral-hold-limit", type=float, default=0.05)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--yaw-rate-start-step", type=int, default=0)
    parser.add_argument("--history-lateral-speed", type=float, default=0.08)
    parser.add_argument("--heading-hold-kp", type=float, default=0.0)
    parser.add_argument("--heading-hold-limit", type=float, default=0.15)
    parser.add_argument("--heading-hold-start-x", type=float, default=-1.0e9)
    parser.add_argument("--heading-hold-start-step", type=int, default=0)
    parser.add_argument("--control-profile", choices=CONTROL_PROFILES, default="walk")
    parser.add_argument("--stand-gain-blend", type=float, default=0.0)
    parser.add_argument("--policy-action-scale", type=float, default=1.0)
    parser.add_argument("--balance-assist-scale", type=float, default=0.0)
    parser.add_argument(
        "--balance-assist-decay-start-step", type=int, default=2**31 - 1
    )
    parser.add_argument(
        "--balance-assist-decay-end-step", type=int, default=2**31 - 1
    )
    parser.add_argument("--torque-limit-scale", type=float, default=1.0)
    parser.add_argument(
        "--swing-collision-mask-leg",
        choices=("none", "left", "right"),
        default="none",
    )
    parser.add_argument(
        "--swing-collision-mask-start-step", type=int, default=2**31 - 1
    )
    parser.add_argument(
        "--swing-collision-mask-end-step", type=int, default=2**31 - 1
    )
    parser.add_argument("--balance-assist-target-x-offset", type=float, default=0.0)
    parser.add_argument("--balance-assist-target-z-offset", type=float, default=0.0)
    parser.add_argument("--hold-final-knot", action="store_true")
    parser.add_argument("--hold-initial-knot", action="store_true")
    parser.add_argument("--hold-settle-action", action="store_true")
    parser.add_argument("--separate-settle-action", action="store_true")
    parser.add_argument("--settle-knot-count", type=int, default=1)
    parser.add_argument("--optimize-settle-knots", type=int, nargs="+")
    parser.add_argument("--optimize-settle-only", action="store_true")
    parser.add_argument("--population", type=int, default=96)
    parser.add_argument("--elite", type=int, default=12)
    parser.add_argument("--iterations", type=int, default=15)
    parser.add_argument("--minimum-iterations", type=int, default=1)
    parser.add_argument("--initial-std", type=float, default=0.20)
    parser.add_argument("--minimum-std", type=float, default=0.015)
    parser.add_argument("--maximum-action", type=float, default=1.0)
    parser.add_argument("--smoothing", type=float, default=0.35)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--initial-search", type=Path)
    parser.add_argument("--initial-history-iteration", type=int)
    parser.add_argument("--mirror-initial", action="store_true")
    parser.add_argument("--zero-initial", action="store_true")
    parser.add_argument("--initial-pose-hold", action="store_true")
    parser.add_argument(
        "--initial-foot-target-leg",
        choices=("left", "right"),
    )
    parser.add_argument("--initial-foot-target-clearance", type=float, default=0.12)
    parser.add_argument("--initial-foot-target-preload-knots", type=int, default=0)
    parser.add_argument("--initial-foot-target-preload-shift", type=float, default=0.0)
    parser.add_argument("--initial-foot-target-preload-roll", type=float, default=0.0)
    parser.add_argument("--initial-foot-target-sagittal-only", action="store_true")
    parser.add_argument("--pose-hold-indices", type=int, nargs="+")
    parser.add_argument("--best-trace-output", type=Path)
    parser.add_argument("--best-video", type=Path)
    parser.add_argument("--replay-through-end", action="store_true")
    parser.add_argument("--render-stride", type=int, default=3)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if args.step_count < 1:
        parser.error("--step-count must be positive")
    if args.tread_height_tolerance <= 0.0:
        parser.error("--tread-height-tolerance must be positive")
    if args.initial_support_run < 0:
        parser.error("--initial-support-run must be non-negative")
    if args.initial_foot_target_clearance < 0.0:
        parser.error("--initial-foot-target-clearance must be non-negative")
    if args.initial_foot_target_preload_shift < 0.0:
        parser.error("--initial-foot-target-preload-shift must be non-negative")
    if not 1 <= args.target_step <= args.step_count:
        parser.error("--target-step must be within the staircase")
    preserve_support_step = (
        args.target_step
        if args.preserve_support_step is None
        else args.preserve_support_step
    )
    if not 1 <= preserve_support_step <= args.step_count:
        parser.error("--preserve-support-step must be within the staircase")
    if not 1 <= args.elite <= args.population:
        parser.error("--elite must be within the population")
    if not 1 <= args.minimum_iterations <= args.iterations:
        parser.error("--minimum-iterations must be within --iterations")
    if len(args.knot_phases) < 3:
        parser.error("--knot-phases must contain at least start, one internal, and end")
    if args.knot_phases[0] != 0.0 or args.knot_phases[-1] != 1.0:
        parser.error("--knot-phases must start at 0 and end at 1")
    if any(b <= a for a, b in zip(args.knot_phases, args.knot_phases[1:])):
        parser.error("--knot-phases must be strictly increasing")
    if len(set(args.indices)) != len(args.indices):
        parser.error("--indices must be unique")
    if any(index < 0 or index >= ACTION_SIZE for index in args.indices):
        parser.error(f"--indices must be in [0, {ACTION_SIZE - 1}]")
    if args.optimize_indices is not None:
        if len(set(args.optimize_indices)) != len(args.optimize_indices):
            parser.error("--optimize-indices must be unique")
        if not set(args.optimize_indices).issubset(args.indices):
            parser.error("--optimize-indices must be a subset of --indices")
    if args.pose_hold_indices is not None:
        if len(set(args.pose_hold_indices)) != len(args.pose_hold_indices):
            parser.error("--pose-hold-indices must be unique")
        if not set(args.pose_hold_indices).issubset(args.indices):
            parser.error("--pose-hold-indices must be a subset of --indices")
    if args.render_stride < 1:
        parser.error("--render-stride must be positive")
    if not 0.0 <= args.stand_gain_blend <= 1.0:
        parser.error("--stand-gain-blend must be in [0, 1]")
    if args.balance_assist_scale < 0.0:
        parser.error("--balance-assist-scale must be non-negative")
    if args.torque_limit_scale <= 0.0:
        parser.error("--torque-limit-scale must be positive")
    if args.swing_collision_mask_end_step < args.swing_collision_mask_start_step:
        parser.error("swing collision mask end must not precede its start")
    if args.balance_assist_decay_end_step < args.balance_assist_decay_start_step:
        parser.error("balance-assist decay end must not precede its start")
    if args.separate_settle_action and not args.hold_settle_action:
        parser.error("--separate-settle-action requires --hold-settle-action")
    if args.optimize_settle_only and not args.separate_settle_action:
        parser.error("--optimize-settle-only requires --separate-settle-action")
    if args.settle_knot_count < 1:
        parser.error("--settle-knot-count must be positive")
    if args.settle_knot_count != 1 and not args.separate_settle_action:
        parser.error("--settle-knot-count requires --separate-settle-action")
    if args.optimize_settle_knots is not None:
        if not args.separate_settle_action:
            parser.error(
                "--optimize-settle-knots requires --separate-settle-action"
            )
        if len(set(args.optimize_settle_knots)) != len(args.optimize_settle_knots):
            parser.error("--optimize-settle-knots must be unique")
        if any(
            index < 0 or index >= args.settle_knot_count
            for index in args.optimize_settle_knots
        ):
            parser.error("--optimize-settle-knots contains an invalid knot index")
    internal_knot_count = len(args.knot_phases) - 2
    if not 0 <= args.initial_foot_target_preload_knots < internal_knot_count:
        parser.error(
            "--initial-foot-target-preload-knots must be smaller than the internal knot count"
        )
    if args.optimize_swing_knots is not None:
        if len(set(args.optimize_swing_knots)) != len(args.optimize_swing_knots):
            parser.error("--optimize-swing-knots must be unique")
        if any(
            index < 0 or index >= internal_knot_count
            for index in args.optimize_swing_knots
        ):
            parser.error("--optimize-swing-knots contains an invalid knot index")

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        half_width=args.stair_half_width,
        landing_length=args.landing_length,
        riser_ramp_length=args.riser_ramp_length,
    )
    rng = np.random.default_rng(args.seed)
    internal_parameter_count = internal_knot_count * len(args.indices)
    settle_parameter_count = (
        args.settle_knot_count * len(args.indices)
        if args.separate_settle_action
        else 0
    )
    parameter_count = internal_parameter_count + settle_parameter_count
    mean = np.concatenate(
        (
            _default_mean(args.indices, internal_knot_count),
            np.zeros(settle_parameter_count, dtype=np.float64),
        )
    )
    std = np.full(parameter_count, args.initial_std, dtype=np.float64)
    initial_modes = sum(
        (
            args.initial_search is not None,
            args.zero_initial,
            args.initial_pose_hold,
            args.initial_foot_target_leg is not None,
        )
    )
    if initial_modes > 1:
        parser.error(
            "use only one explicit initial-search mode"
        )
    if args.initial_history_iteration is not None and args.initial_search is None:
        parser.error("--initial-history-iteration requires --initial-search")
    if args.initial_pose_hold:
        try:
            internal_mean = _pose_hold_mean(
                args.model,
                args.policy,
                args.trace,
                args.reset_step,
                args.indices,
                internal_knot_count,
                args.residual_scale,
                args.control_profile,
                args.policy_action_scale,
            )
        except ValueError as exc:
            parser.error(str(exc))
        mean[:internal_parameter_count] = internal_mean
        mean = np.clip(mean, -args.maximum_action, args.maximum_action)
    elif args.initial_foot_target_leg is not None:
        try:
            internal_mean = _foot_target_mean(
                args.model,
                args.policy,
                args.trace,
                args.reset_step,
                args.indices,
                internal_knot_count,
                args.residual_scale,
                args.control_profile,
                args.policy_action_scale,
                spec,
                args.target_step,
                args.initial_foot_target_leg,
                args.initial_foot_target_clearance,
                args.initial_foot_target_preload_knots,
                args.initial_foot_target_preload_shift,
                args.initial_foot_target_preload_roll,
                args.initial_foot_target_sagittal_only,
            )
        except (ValueError, np.linalg.LinAlgError) as exc:
            parser.error(str(exc))
        mean[:internal_parameter_count] = internal_mean
        mean = np.clip(mean, -args.maximum_action, args.maximum_action)
    elif args.zero_initial:
        mean = np.zeros(parameter_count, dtype=np.float64)
    elif args.initial_search is not None:
        payload = json.loads(args.initial_search.expanduser().resolve().read_text())
        source_indices = list(payload.get("indices", args.indices))
        source_record = payload["best"]
        if args.initial_history_iteration is not None:
            source_record = next(
                (
                    record
                    for record in payload.get("history", ())
                    if int(record.get("iteration", -1))
                    == args.initial_history_iteration
                ),
                None,
            )
            if source_record is None:
                parser.error(
                    "--initial-history-iteration is absent from --initial-search"
                )
        source_mean = np.asarray(source_record["action"], dtype=np.float64)
        source_internal_count = internal_knot_count * len(source_indices)
        source_has_settle = bool(payload.get("separate_settle_action", False))
        source_settle_knot_count = (
            int(payload.get("settle_knot_count", 1)) if source_has_settle else 0
        )
        expected_source_shape = (
            source_internal_count
            + source_settle_knot_count * len(source_indices),
        )
        if source_mean.shape != expected_source_shape:
            parser.error("--initial-search action shape does not match its indices/knots")
        source_knots = source_mean[:source_internal_count].reshape(
            internal_knot_count, len(source_indices)
        )
        target_knots = mean[:internal_parameter_count].reshape(
            internal_knot_count, len(args.indices)
        )
        source_settle = (
            source_mean[source_internal_count:].reshape(
                source_settle_knot_count, len(source_indices)
            )
            if source_has_settle
            else source_knots[-1:].copy()
        )
        target_settle = (
            mean[internal_parameter_count:].reshape(
                args.settle_knot_count, len(args.indices)
            )
            if args.separate_settle_action
            else np.zeros((0, len(args.indices)), dtype=np.float64)
        )
        source_settle_phases = np.linspace(0.0, 1.0, len(source_settle))
        target_settle_phases = np.linspace(0.0, 1.0, len(target_settle))
        source_lookup = {
            joint_index: column for column, joint_index in enumerate(source_indices)
        }
        for target_column, joint_index in enumerate(args.indices):
            if joint_index in source_lookup:
                source_column = source_lookup[joint_index]
                target_knots[:, target_column] = source_knots[:, source_column]
                if args.separate_settle_action:
                    target_settle[:, target_column] = np.interp(
                        target_settle_phases,
                        source_settle_phases,
                        source_settle[:, source_column],
                    )
        if args.mirror_initial:
            try:
                mean = _mirror_mean(
                    mean,
                    args.indices,
                    internal_knot_count
                    + args.settle_knot_count * int(args.separate_settle_action),
                )
            except ValueError as exc:
                parser.error(str(exc))
    elif args.mirror_initial:
        try:
            mean = _mirror_mean(
                mean,
                args.indices,
                internal_knot_count
                + args.settle_knot_count * int(args.separate_settle_action),
            )
        except ValueError as exc:
            parser.error(str(exc))

    if args.pose_hold_indices is not None:
        try:
            pose_hold = _pose_hold_mean(
                args.model,
                args.policy,
                args.trace,
                args.reset_step,
                args.indices,
                internal_knot_count,
                args.residual_scale,
                args.control_profile,
                args.policy_action_scale,
            ).reshape(internal_knot_count, len(args.indices))
        except ValueError as exc:
            parser.error(str(exc))
        mean_knots = mean[:internal_parameter_count].reshape(
            internal_knot_count, len(args.indices)
        )
        pose_hold_columns = [
            args.indices.index(joint_index)
            for joint_index in args.pose_hold_indices
        ]
        mean_knots[:, pose_hold_columns] = pose_hold[:, pose_hold_columns]
        if args.separate_settle_action:
            mean_settle = mean[internal_parameter_count:].reshape(
                args.settle_knot_count, len(args.indices)
            )
            mean_settle[:, pose_hold_columns] = pose_hold[
                -1:, pose_hold_columns
            ]
        mean = np.clip(mean, -args.maximum_action, args.maximum_action)

    optimized_joints = set(
        args.optimize_indices if args.optimize_indices is not None else args.indices
    )
    joint_optimize_mask = np.asarray(
        [index in optimized_joints for index in args.indices], dtype=bool
    )
    optimize_mask = np.concatenate(
        (
            np.concatenate(
                [
                    joint_optimize_mask
                    if not args.optimize_settle_only
                    and (
                        args.optimize_swing_knots is None
                        or knot_index in args.optimize_swing_knots
                    )
                    else np.zeros_like(joint_optimize_mask)
                    for knot_index in range(internal_knot_count)
                ]
            ),
            np.concatenate(
                [
                    joint_optimize_mask
                    if args.optimize_settle_knots is None
                    or knot_index in args.optimize_settle_knots
                    else np.zeros_like(joint_optimize_mask)
                    for knot_index in range(args.settle_knot_count)
                ]
            )
            if args.separate_settle_action
            else np.zeros(0, dtype=bool),
        )
    )
    fixed_mean = mean.copy()
    std[~optimize_mask] = 0.0

    best: dict | None = None
    history = []
    context = multiprocessing.get_context("spawn")
    episode_length = args.swing_steps + args.settle_steps
    with context.Pool(
        processes=args.workers,
        initializer=_initialize_worker,
        initargs=(
            args.model.expanduser().resolve(),
            args.policy.expanduser().resolve(),
            args.trace.expanduser().resolve(),
            spec,
            args.reset_step,
            args.residual_scale,
            args.speed,
            args.lateral_speed,
            args.lateral_hold_kp,
            args.lateral_hold_limit,
            args.yaw_rate,
            args.yaw_rate_start_step,
            args.history_lateral_speed,
            episode_length,
            tuple(args.indices),
            tuple(args.knot_phases),
            args.swing_steps,
            args.settle_steps,
            args.target_step,
            preserve_support_step,
            args.hold_final_knot,
            args.hold_initial_knot,
            args.hold_settle_action,
            args.separate_settle_action,
            args.settle_knot_count,
            args.control_profile,
            args.stand_gain_blend,
            args.policy_action_scale,
            args.balance_assist_scale,
            args.balance_assist_decay_start_step,
            args.balance_assist_decay_end_step,
            args.torque_limit_scale,
            args.swing_collision_mask_leg,
            args.swing_collision_mask_start_step,
            args.swing_collision_mask_end_step,
            args.balance_assist_target_x_offset,
            args.balance_assist_target_z_offset,
            args.heading_hold_kp,
            args.heading_hold_limit,
            args.heading_hold_start_x,
            args.heading_hold_start_step,
            args.base_advance,
            args.required_support_leg,
            args.preserve_support_leg,
            args.required_tread_feet,
            args.tread_height_tolerance,
            args.initial_support_run,
            args.search_objective,
            args.endpoint_selection,
        ),
    ) as pool:
        for iteration in range(args.iterations):
            candidates = np.clip(
                rng.normal(mean, std, size=(args.population, parameter_count)),
                -args.maximum_action,
                args.maximum_action,
            )
            candidates[0] = (
                np.asarray(best["action"], dtype=np.float64)
                if best is not None
                else mean
            )
            if args.population > 1:
                candidates[1] = mean
            if args.population > 2:
                candidates[2] = mean
                candidates[2, optimize_mask] = 0.0
            results = pool.map(_evaluate, list(candidates))
            order = np.argsort([item["score"] for item in results])[::-1]
            elites = candidates[order[: args.elite]]
            mean = args.smoothing * mean + (1.0 - args.smoothing) * elites.mean(axis=0)
            std = np.maximum(
                args.minimum_std,
                args.smoothing * std + (1.0 - args.smoothing) * elites.std(axis=0),
            )
            mean[~optimize_mask] = fixed_mean[~optimize_mask]
            std[~optimize_mask] = 0.0
            generation_best = results[int(order[0])]
            generation_best["iteration"] = iteration + 1
            history.append(generation_best)
            if best is None or generation_best["score"] > best["score"]:
                best = generation_best.copy()
            payload = {
                "model": str(args.model.expanduser().resolve()),
                "policy": str(args.policy.expanduser().resolve()),
                "source_trace": str(args.trace.expanduser().resolve()),
                "stair_spec": spec.__dict__,
                "reset_step": args.reset_step,
                "target_step": args.target_step,
                "base_advance": args.base_advance,
                "required_support_leg": args.required_support_leg,
                "preserve_support_leg": args.preserve_support_leg,
                "preserve_support_step": preserve_support_step,
                "required_tread_feet": args.required_tread_feet,
                "tread_height_tolerance": args.tread_height_tolerance,
                "initial_support_run": args.initial_support_run,
                "search_objective": args.search_objective,
                "endpoint_selection": args.endpoint_selection,
                "minimum_iterations": args.minimum_iterations,
                "indices": args.indices,
                "optimize_indices": sorted(optimized_joints),
                "pose_hold_indices": args.pose_hold_indices,
                "initial_foot_target_leg": args.initial_foot_target_leg,
                "initial_foot_target_clearance": args.initial_foot_target_clearance,
                "initial_foot_target_preload_knots": args.initial_foot_target_preload_knots,
                "initial_foot_target_preload_shift": args.initial_foot_target_preload_shift,
                "initial_foot_target_preload_roll": args.initial_foot_target_preload_roll,
                "initial_foot_target_sagittal_only": args.initial_foot_target_sagittal_only,
                "optimize_swing_knots": args.optimize_swing_knots,
                "knot_phases": args.knot_phases,
                "swing_steps": args.swing_steps,
                "settle_steps": args.settle_steps,
                "residual_scale": args.residual_scale,
                "speed": args.speed,
                "lateral_speed": args.lateral_speed,
                "lateral_hold_kp": args.lateral_hold_kp,
                "lateral_hold_limit": args.lateral_hold_limit,
                "yaw_rate": args.yaw_rate,
                "yaw_rate_start_step": args.yaw_rate_start_step,
                "history_lateral_speed": args.history_lateral_speed,
                "heading_hold_kp": args.heading_hold_kp,
                "heading_hold_limit": args.heading_hold_limit,
                "heading_hold_start_x": args.heading_hold_start_x,
                "heading_hold_start_step": args.heading_hold_start_step,
                "hold_final_knot": args.hold_final_knot,
                "hold_initial_knot": args.hold_initial_knot,
                "hold_settle_action": args.hold_settle_action,
                "separate_settle_action": args.separate_settle_action,
                "settle_knot_count": args.settle_knot_count,
                "optimize_settle_knots": args.optimize_settle_knots,
                "optimize_settle_only": args.optimize_settle_only,
                "control_profile": args.control_profile,
                "stand_gain_blend": args.stand_gain_blend,
                "policy_action_scale": args.policy_action_scale,
                "balance_assist_scale": args.balance_assist_scale,
                "balance_assist_decay_start_step": args.balance_assist_decay_start_step,
                "balance_assist_decay_end_step": args.balance_assist_decay_end_step,
                "torque_limit_scale": args.torque_limit_scale,
                "swing_collision_mask_leg": args.swing_collision_mask_leg,
                "swing_collision_mask_start_step": args.swing_collision_mask_start_step,
                "swing_collision_mask_end_step": args.swing_collision_mask_end_step,
                "balance_assist_target_x_offset": args.balance_assist_target_x_offset,
                "balance_assist_target_z_offset": args.balance_assist_target_z_offset,
                "mean": mean.tolist(),
                "std": std.tolist(),
                "best": best,
                "history": history,
            }
            (output_dir / "search.json").write_text(
                json.dumps(payload, indent=2) + "\n", encoding="utf-8"
            )
            print(
                f"iteration={iteration + 1} score={generation_best['score']:.1f} "
                f"qualified={generation_best['qualified']} "
                f"support={generation_best['target_support']} "
                f"hold={generation_best['terminal_stable_support']} "
                f"foot_x={generation_best['max_foot_x']} "
                f"x={generation_best['final_x']:.3f} y={generation_best['final_y']:.3f} "
                f"yaw={generation_best['yaw_deg']:.1f} "
                f"up={generation_best['body_up_z']:.3f} std={std.mean():.3f}",
                flush=True,
            )
            if (
                generation_best["qualified"]
                and iteration + 1 >= args.minimum_iterations
            ):
                break

    if best is None:
        raise RuntimeError("search completed without evaluating a candidate")
    profile = {
        "format_version": 1,
        "model": str(args.model.expanduser().resolve()),
        "policy": str(args.policy.expanduser().resolve()),
        "source_trace": str(args.trace.expanduser().resolve()),
        "target_step": args.target_step,
        "base_advance": args.base_advance,
        "required_support_leg": args.required_support_leg,
        "preserve_support_leg": args.preserve_support_leg,
        "preserve_support_step": preserve_support_step,
        "required_tread_feet": args.required_tread_feet,
        "tread_height_tolerance": args.tread_height_tolerance,
        "initial_support_run": args.initial_support_run,
        "search_objective": args.search_objective,
        "endpoint_selection": args.endpoint_selection,
        "minimum_iterations": args.minimum_iterations,
        "indices": args.indices,
        "pose_hold_indices": args.pose_hold_indices,
        "initial_foot_target_leg": args.initial_foot_target_leg,
        "initial_foot_target_clearance": args.initial_foot_target_clearance,
        "initial_foot_target_preload_knots": args.initial_foot_target_preload_knots,
        "initial_foot_target_preload_shift": args.initial_foot_target_preload_shift,
        "initial_foot_target_preload_roll": args.initial_foot_target_preload_roll,
        "initial_foot_target_sagittal_only": args.initial_foot_target_sagittal_only,
        "knot_phases": args.knot_phases,
        "swing_steps": args.swing_steps,
        "settle_steps": args.settle_steps,
        "residual_scale": args.residual_scale,
        "speed": args.speed,
        "lateral_speed": args.lateral_speed,
        "lateral_hold_kp": args.lateral_hold_kp,
        "lateral_hold_limit": args.lateral_hold_limit,
        "yaw_rate": args.yaw_rate,
        "yaw_rate_start_step": args.yaw_rate_start_step,
        "history_lateral_speed": args.history_lateral_speed,
        "heading_hold_kp": args.heading_hold_kp,
        "heading_hold_limit": args.heading_hold_limit,
        "heading_hold_start_x": args.heading_hold_start_x,
        "heading_hold_start_step": args.heading_hold_start_step,
        "hold_final_knot": args.hold_final_knot,
        "hold_initial_knot": args.hold_initial_knot,
        "hold_settle_action": args.hold_settle_action,
        "separate_settle_action": args.separate_settle_action,
        "settle_knot_count": args.settle_knot_count,
        "optimize_swing_knots": args.optimize_swing_knots,
        "optimize_settle_knots": args.optimize_settle_knots,
        "optimize_settle_only": args.optimize_settle_only,
        "control_profile": args.control_profile,
        "stand_gain_blend": args.stand_gain_blend,
        "policy_action_scale": args.policy_action_scale,
        "balance_assist_scale": args.balance_assist_scale,
        "balance_assist_decay_start_step": args.balance_assist_decay_start_step,
        "balance_assist_decay_end_step": args.balance_assist_decay_end_step,
        "torque_limit_scale": args.torque_limit_scale,
        "swing_collision_mask_leg": args.swing_collision_mask_leg,
        "swing_collision_mask_start_step": args.swing_collision_mask_start_step,
        "swing_collision_mask_end_step": args.swing_collision_mask_end_step,
        "balance_assist_target_x_offset": args.balance_assist_target_x_offset,
        "balance_assist_target_z_offset": args.balance_assist_target_z_offset,
        "selected_step": best.get("selected_step"),
        "replay_through_end": args.replay_through_end,
        "action": best["action"],
    }
    (output_dir / "best_profile.json").write_text(
        json.dumps(profile, indent=2) + "\n", encoding="utf-8"
    )
    if args.best_trace_output is not None or args.best_video is not None:
        _replay_best(
            model_path=args.model.expanduser().resolve(),
            policy_path=args.policy.expanduser().resolve(),
            source_trace=args.trace.expanduser().resolve(),
            spec=spec,
            reset_step=args.reset_step,
            indices=args.indices,
            knot_phases=args.knot_phases,
            swing_steps=args.swing_steps,
            settle_steps=args.settle_steps,
            residual_scale=args.residual_scale,
            speed=args.speed,
            lateral_speed=args.lateral_speed,
            lateral_hold_kp=args.lateral_hold_kp,
            lateral_hold_limit=args.lateral_hold_limit,
            yaw_rate=args.yaw_rate,
            yaw_rate_start_step=args.yaw_rate_start_step,
            history_lateral_speed=args.history_lateral_speed,
            candidate=best["action"],
            hold_final_knot=args.hold_final_knot,
            hold_initial_knot=args.hold_initial_knot,
            hold_settle_action=args.hold_settle_action,
            separate_settle_action=args.separate_settle_action,
            settle_knot_count=args.settle_knot_count,
            trace_output=args.best_trace_output,
            video_output=args.best_video,
            render_stride=args.render_stride,
            control_profile=args.control_profile,
            stand_gain_blend=args.stand_gain_blend,
            policy_action_scale=args.policy_action_scale,
            balance_assist_scale=args.balance_assist_scale,
            balance_assist_decay_start_step=args.balance_assist_decay_start_step,
            balance_assist_decay_end_step=args.balance_assist_decay_end_step,
            torque_limit_scale=args.torque_limit_scale,
            swing_collision_mask_leg=args.swing_collision_mask_leg,
            swing_collision_mask_start_step=args.swing_collision_mask_start_step,
            swing_collision_mask_end_step=args.swing_collision_mask_end_step,
            balance_assist_target_x_offset=args.balance_assist_target_x_offset,
            balance_assist_target_z_offset=args.balance_assist_target_z_offset,
            selected_step=(
                None if args.replay_through_end else best.get("selected_step")
            ),
            heading_hold_kp=args.heading_hold_kp,
            heading_hold_limit=args.heading_hold_limit,
            heading_hold_start_x=args.heading_hold_start_x,
            heading_hold_start_step=args.heading_hold_start_step,
        )
    print(output_dir)


if __name__ == "__main__":
    main()
