"""Evaluate an OLI ONNX policy with the deployment-equivalent CPU solver."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import os
from pathlib import Path

# This evaluator is the CPU MuJoCo reference path.  Keeping JAX on CPU avoids
# reserving training GPU memory for the small parallel-kinematics calculations.
os.environ.setdefault("JAX_PLATFORMS", "cpu")

import jax.numpy as jp
import mediapy as media
import mujoco
import numpy as np
import onnx
from onnx import numpy_helper

from training.oli_stairs.constants import (
    CONTROL_PROFILES,
    OFFICIAL_POLICY,
    STAGE_HEIGHTS,
    load_control_parameters,
    stair_spec_for_stage,
)
from training.oli_stairs.cpu_residual_policy import NumpyResidualActor
from training.oli_stairs.environment import OliStairEnv, default_config
from training.oli_stairs.parallel_kinematics import (
    left_ankle_pr_to_ab,
    pr_torque_to_ab,
    right_ankle_pr_to_ab,
    waist_pr_to_ab,
)


class NumpyOnnxActor:
    """Fast evaluator for the common 510-512-256-128-31 OLI actor graph."""

    def __init__(self, path: Path) -> None:
        model = onnx.load(str(path.expanduser().resolve()))
        tensors = {
            item.name: numpy_helper.to_array(item) for item in model.graph.initializer
        }
        self._layers = [
            (tensors[f"actor.{index}.weight"], tensors[f"actor.{index}.bias"])
            for index in (0, 2, 4, 6)
        ]
        self._bounded_output = any(node.name == "action_scale" for node in model.graph.node)
        self._output_scale = (
            float(np.asarray(tensors["clip_scale"]).reshape(()))
            if self._bounded_output
            else None
        )

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        value = self.features(observation)[None]
        weight, bias = self._layers[-1]
        value = value @ weight.T + bias
        if self._bounded_output:
            value = self._output_scale * value / (1.0 + np.abs(value))
        return value[0].astype(np.float64)

    def features(self, observation: np.ndarray) -> np.ndarray:
        """Return the actor's 128-value penultimate hidden representation."""
        value = np.asarray(observation, dtype=np.float32)[None]
        for weight, bias in self._layers[:-1]:
            value = value @ weight.T + bias
            value = np.where(value > 0.0, value, np.expm1(value))
        return value[0].astype(np.float64)


def _projected_gravity(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    rotation = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )
    return rotation.T @ np.asarray([0.0, 0.0, -1.0])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="flat")
    parser.add_argument("--step-count", type=int)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--riser-ramp-length", type=float, default=0.0)
    parser.add_argument("--policy", type=Path, default=OFFICIAL_POLICY)
    parser.add_argument("--residual-policy", type=Path)
    parser.add_argument("--mid-residual-policy", type=Path)
    parser.add_argument("--mid-residual-start-x", type=float, default=1.0e9)
    parser.add_argument("--landing-residual-policy", type=Path)
    parser.add_argument("--landing-residual-start-x", type=float, default=-1.0e9)
    parser.add_argument("--fixed-residual-policy", type=Path)
    parser.add_argument("--residual-scale", type=float)
    parser.add_argument("--residual-start-x", type=float)
    parser.add_argument("--late-policy", type=Path)
    parser.add_argument("--late-policy-start-x", type=float, default=1.0e9)
    parser.add_argument("--late-policy-blend", type=float, default=1.0)
    parser.add_argument("--control-profile", choices=CONTROL_PROFILES, default="walk")
    parser.add_argument("--steps", type=int, default=1800)
    parser.add_argument("--speed", type=float, default=0.30)
    parser.add_argument("--late-speed", type=float)
    parser.add_argument("--late-speed-start-x", type=float, default=1.0e9)
    parser.add_argument("--lateral-speed", type=float, default=0.0)
    parser.add_argument("--history-lateral-speed", type=float)
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--lateral-hold-kp", type=float, default=0.0)
    parser.add_argument("--lateral-hold-limit", type=float, default=0.05)
    parser.add_argument("--late-lateral-speed", type=float)
    parser.add_argument("--late-lateral-start-x", type=float, default=1.0e9)
    parser.add_argument("--heading-hold-kp", type=float, default=0.0)
    parser.add_argument("--heading-hold-limit", type=float, default=0.15)
    parser.add_argument("--heading-hold-start-x", type=float, default=-1.0e9)
    parser.add_argument("--initial-yaw-deg", type=float, default=0.0)
    parser.add_argument("--initial-x-offset", type=float, default=0.0)
    parser.add_argument("--initial-y-offset", type=float, default=0.0)
    parser.add_argument("--action-filter-alpha", type=float, default=1.0)
    parser.add_argument("--history-reset-x", type=float, action="append", default=[])
    parser.add_argument("--history-reset-preserve-action", action="store_true")
    parser.add_argument("--history-reset-reuse-first", action="store_true")
    parser.add_argument(
        "--history-reset-support-step",
        type=int,
        action="append",
        default=[],
    )
    parser.add_argument("--action-replay-trace", type=Path)
    parser.add_argument("--action-replay-start-step", type=int, default=300)
    parser.add_argument("--action-replay-end-step", type=int, default=380)
    parser.add_argument("--action-replay-trigger-support-step", type=int)
    parser.add_argument("--action-replay-start-x", type=float, default=np.inf)
    parser.add_argument("--action-replay-residual-scale", type=float, default=0.0)
    parser.add_argument("--action-replay-mirror", action="store_true")
    parser.add_argument("--policy-action-clip", type=float, default=100.0)
    parser.add_argument("--policy-action-clip-start-x", type=float, default=-1.0e9)
    parser.add_argument("--late-knee-action-clip", type=float, default=100.0)
    parser.add_argument("--late-knee-action-clip-start-x", type=float, default=1.0e9)
    parser.add_argument("--landing-leg-action-scale", type=float, default=1.0)
    parser.add_argument("--landing-leg-action-start-x", type=float, default=1.0e9)
    parser.add_argument("--landing-stand-start-x", type=float, default=1.0e9)
    parser.add_argument("--landing-stand-blend", type=float, default=1.0)
    parser.add_argument("--landing-stand-trigger-top-support", action="store_true")
    parser.add_argument("--landing-stand-ramp-duration", type=float, default=0.0)
    parser.add_argument("--landing-captured-pose-control", action="store_true")
    parser.add_argument("--landing-capture-start-x", type=float, default=-1.0e9)
    parser.add_argument("--landing-captured-pose-ramp-duration", type=float, default=0.5)
    parser.add_argument("--landing-captured-use-stand-gains", action="store_true")
    parser.add_argument("--landing-follow-step", action="store_true")
    parser.add_argument("--landing-follow-step-duration", type=float, default=0.50)
    parser.add_argument("--landing-follow-step-depth", type=float, default=0.15)
    parser.add_argument("--landing-follow-step-clearance", type=float, default=0.10)
    parser.add_argument("--landing-follow-step-gain", type=float, default=0.70)
    parser.add_argument("--landing-follow-step-max-correction", type=float, default=0.50)
    parser.add_argument("--height-knee-kp", type=float, default=0.0)
    parser.add_argument("--height-knee-target", type=float, default=0.82)
    parser.add_argument("--height-knee-max", type=float, default=0.40)
    parser.add_argument("--height-knee-start-x", type=float, default=1.0e9)
    parser.add_argument("--top-support-leg-lock", action="store_true")
    parser.add_argument("--top-support-lock-all-joints", action="store_true")
    parser.add_argument("--top-support-knee-extension", type=float, default=0.0)
    parser.add_argument("--landing-ankle-pitch-kp", type=float, default=0.0)
    parser.add_argument("--landing-ankle-pitch-kd", type=float, default=0.0)
    parser.add_argument("--landing-ankle-pitch-limit", type=float, default=0.40)
    parser.add_argument("--landing-hip-pitch-kp", type=float, default=0.0)
    parser.add_argument("--landing-hip-pitch-kd", type=float, default=0.0)
    parser.add_argument("--landing-hip-pitch-limit", type=float, default=0.40)
    parser.add_argument("--single-support-roll-gain", type=float, default=0.0)
    parser.add_argument("--single-support-roll-limit", type=float, default=0.15)
    parser.add_argument("--single-support-roll-start-x", type=float, default=1.0e9)
    parser.add_argument("--stair-residual", action="store_true")
    parser.add_argument(
        "--stair-residual-mode",
        choices=("phase", "terrain", "terrain_ik"),
        default="terrain",
    )
    parser.add_argument("--stair-hip-lift", type=float, default=0.036)
    parser.add_argument("--stair-knee-lift", type=float, default=0.096)
    parser.add_argument("--stair-ankle-lift", type=float, default=0.048)
    parser.add_argument("--stair-trigger-min-distance", type=float, default=0.05)
    parser.add_argument("--stair-trigger-max-distance", type=float, default=0.25)
    parser.add_argument("--stair-trigger-min-vx", type=float, default=0.05)
    parser.add_argument("--stair-trigger-min-vz", type=float, default=0.03)
    parser.add_argument("--stair-trigger-min-clearance", type=float, default=0.004)
    parser.add_argument("--stair-residual-start-step", type=int, default=1)
    parser.add_argument("--stair-residual-max-duration", type=float, default=0.45)
    parser.add_argument("--stair-riser-cross-margin", type=float, default=0.04)
    parser.add_argument("--stair-foot-lateral-limit", type=float, default=0.30)
    parser.add_argument("--stair-hip-roll-limit", type=float, default=0.10)
    parser.add_argument("--stair-settle-duration", type=float, default=0.0)
    parser.add_argument("--stair-settle-speed", type=float, default=0.0)
    parser.add_argument("--stair-ik-clearance", type=float, default=-1.0)
    parser.add_argument("--stair-ik-landing-depth", type=float, default=0.10)
    parser.add_argument("--stair-ik-gain", type=float, default=0.70)
    parser.add_argument("--stair-ik-damping", type=float, default=0.03)
    parser.add_argument("--stair-ik-max-correction", type=float, default=0.35)
    parser.add_argument("--video", type=Path)
    parser.add_argument("--trace-output", type=Path)
    parser.add_argument("--render-stride", type=int, default=3)
    parser.add_argument("--render-width", type=int, default=640)
    parser.add_argument("--render-height", type=int, default=360)
    parser.add_argument("--fall-height-clearance", type=float, default=0.62)
    parser.add_argument("--require-top-support", action="store_true")
    parser.add_argument("--success-hold-steps", type=int, default=1)
    args = parser.parse_args()
    if args.step_count is not None and args.step_count < 1:
        parser.error("--step-count must be positive")
    for name in ("step_depth", "landing_length", "stair_half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.riser_ramp_length < 0.0:
        parser.error("--riser-ramp-length cannot be negative")
    if args.success_hold_steps < 1:
        parser.error("--success-hold-steps must be positive")
    if args.landing_stand_ramp_duration < 0.0:
        parser.error("--landing-stand-ramp-duration cannot be negative")
    if args.landing_captured_pose_ramp_duration < 0.0:
        parser.error("--landing-captured-pose-ramp-duration cannot be negative")
    if args.landing_follow_step_duration <= 0.0:
        parser.error("--landing-follow-step-duration must be positive")

    config = default_config()
    config.control_profile = args.control_profile
    config.parallel_control_mode = "projected_torque"
    stair_spec = stair_spec_for_stage(args.stage)
    stair_spec = replace(
        stair_spec,
        step_count=(args.step_count if args.step_count is not None else stair_spec.step_count),
        step_depth=args.step_depth,
        landing_length=args.landing_length,
        half_width=args.stair_half_width,
        riser_ramp_length=args.riser_ramp_length,
    )
    env = OliStairEnv(stair_spec, config=config)
    model = env.mj_model
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(env._init_q)
    data.qpos[0] = (
        env.spec.start_x - 0.72 if env.spec.step_height > 0 else -0.2
    ) + args.initial_x_offset
    data.qpos[1] = args.initial_y_offset
    initial_yaw = np.radians(args.initial_yaw_deg)
    data.qpos[3:7] = [
        np.cos(initial_yaw * 0.5),
        0.0,
        0.0,
        np.sin(initial_yaw * 0.5),
    ]
    mujoco.mj_forward(model, data)

    actor = NumpyOnnxActor(args.policy)
    residual_actor = (
        NumpyResidualActor(args.residual_policy) if args.residual_policy else None
    )
    mid_residual_actor = (
        NumpyResidualActor(args.mid_residual_policy)
        if args.mid_residual_policy
        else None
    )
    landing_residual_actor = (
        NumpyResidualActor(args.landing_residual_policy)
        if args.landing_residual_policy else None
    )
    fixed_residual_actor = (
        NumpyResidualActor(args.fixed_residual_policy)
        if args.fixed_residual_policy else None
    )
    residual_scale = (
        args.residual_scale
        if args.residual_scale is not None
        else residual_actor.residual_scale if residual_actor is not None else 0.0
    )
    residual_start_x = (
        args.residual_start_x
        if args.residual_start_x is not None
        else residual_actor.residual_start_x if residual_actor is not None else np.inf
    )
    late_actor = NumpyOnnxActor(args.late_policy) if args.late_policy else None
    replay_actions = None
    if args.action_replay_trace is not None:
        replay_records = json.loads(
            args.action_replay_trace.expanduser().resolve().read_text(
                encoding="utf-8"
            )
        )
        replay_actions = []
        for item in replay_records[
            args.action_replay_start_step - 1 : args.action_replay_end_step
        ]:
            if "action" in item:
                replay_action = np.asarray(item["action"], dtype=np.float64)
            elif "residual_action" in item and "observation" in item:
                replay_action = actor(
                    np.asarray(item["observation"], dtype=np.float32)
                ) + args.action_replay_residual_scale * np.asarray(
                    item["residual_action"], dtype=np.float64
                )
            else:
                raise ValueError(
                    "action replay records must contain either action or "
                    "observation plus residual_action"
                )
            replay_actions.append(replay_action)
        replay_actions = np.asarray(replay_actions, dtype=np.float64)
        if args.action_replay_mirror:
            mirrored = replay_actions.copy()
            mirrored[:, :6] = replay_actions[:, 6:12]
            mirrored[:, 6:12] = replay_actions[:, :6]
            replay_actions = mirrored
    default_pose = np.asarray(env._default_pose)
    action_scale = np.asarray(env._action_scale)
    lower = np.asarray(env._soft_lowers)
    upper = np.asarray(env._soft_uppers)
    kp = np.asarray(env._kp)
    kd = np.asarray(env._kd)
    pr_limits = np.asarray(env._torque_limit)
    motor_limits = np.asarray(env._actuator_torque_limit)
    stand_control = load_control_parameters(profile="stand")
    stand_default_pose = stand_control.default_angle.astype(np.float64)
    stand_kp = stand_control.kp.astype(np.float64)
    stand_kd = stand_control.kd.astype(np.float64)
    command = np.asarray([args.speed, args.lateral_speed, args.yaw_rate])
    pairs = (
        ((4, 5), left_ankle_pr_to_ab),
        ((10, 11), right_ankle_pr_to_ab),
        ((13, 14), waist_pr_to_ab),
    )

    def observation_frame(
        last_action: np.ndarray,
        frame_command: np.ndarray | None = None,
    ) -> np.ndarray:
        observed_command = command if frame_command is None else frame_command
        return np.concatenate(
            [
                data.sensordata[env._gyro_sensor] * env._gyro_scale,
                _projected_gravity(data.qpos[3:7]),
                observed_command,
                (data.qpos[env._qpos_ids] - default_pose) * env._dof_pos_scale,
                data.qvel[env._qvel_ids] * env._dof_vel_scale,
                last_action,
            ]
        ).astype(np.float32)

    last_action = np.zeros(env.action_size)
    history_command = command.copy()
    if args.history_lateral_speed is not None:
        history_command[1] = args.history_lateral_speed
    observation = np.tile(observation_frame(last_action, history_command), 5)
    start = data.qpos.copy()
    frames = []
    trace = []
    renderer = None
    if args.video is not None:
        renderer = mujoco.Renderer(
            model,
            height=args.render_height,
            width=args.render_width,
        )

    result = "timeout"
    max_support_height = 0.0
    max_base_height = float(data.qpos[2])
    gait_phase = 0.0
    active_risers = np.full(2, np.nan)
    residual_ages = np.zeros(2)
    swing_starts = np.full((2, 3), np.nan)
    settle_remaining = 0.0
    history_reset_points = sorted(args.history_reset_x)
    history_reset_index = 0
    history_reset_template = None
    pending_support_resets = set(args.history_reset_support_step)
    replay_index = None
    replay_started = False
    success_hold_count = 0
    landing_stand_active = False
    landing_stand_elapsed = 0.0
    locked_leg_targets = np.full((2, 6), np.nan)
    top_support_seen = False
    top_landed_legs = np.zeros(2, dtype=bool)
    captured_landing_pose = None
    captured_landing_elapsed = 0.0
    landing_swing_leg = None
    landing_swing_start = None
    landing_swing_target_y = None
    for step_index in range(args.steps):
        yaw_error = np.arctan2(
            2.0 * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
            1.0 - 2.0 * (data.qpos[5] ** 2 + data.qpos[6] ** 2),
        )
        nominal_speed = (
            args.late_speed
            if args.late_speed is not None
            and data.qpos[0] >= args.late_speed_start_x
            else args.speed
        )
        command[0] = (
            args.stair_settle_speed if settle_remaining > 0.0 else nominal_speed
        )
        nominal_lateral = (
            args.late_lateral_speed
            if args.late_lateral_speed is not None
            and data.qpos[0] >= args.late_lateral_start_x
            else args.lateral_speed
        )
        command[1] = np.clip(
            nominal_lateral - args.lateral_hold_kp * data.qpos[1],
            -args.lateral_hold_limit,
            args.lateral_hold_limit,
        )
        heading_correction = (
            args.heading_hold_kp * yaw_error
            if data.qpos[0] >= args.heading_hold_start_x
            else 0.0
        )
        command[2] = np.clip(
            args.yaw_rate - heading_correction,
            -args.heading_hold_limit,
            args.heading_hold_limit,
        )
        support_level = 0
        top_contact_legs = []
        for leg_index, foot in enumerate(data.site_xpos[env._foot_site_ids]):
            foot_height = float(env._terrain_height(jp.asarray(foot[0])))
            if foot[2] - foot_height < 0.035 and env.spec.step_height > 0.0:
                support_level = max(
                    support_level,
                    int(round(foot_height / env.spec.step_height)),
                )
                if foot_height >= env.spec.top_height - 1.0e-6:
                    top_contact_legs.append(leg_index)
        if args.top_support_leg_lock:
            current_position = data.qpos[env._qpos_ids]
            for leg_index in top_contact_legs:
                if not np.all(np.isfinite(locked_leg_targets[leg_index])):
                    start_index = leg_index * 6
                    locked_leg_targets[leg_index] = current_position[
                        start_index : start_index + 6
                    ]
        top_support_seen = top_support_seen or bool(top_contact_legs)
        top_landed_legs[top_contact_legs] = True
        if (
            (args.landing_captured_pose_control or args.landing_follow_step)
            and captured_landing_pose is None
            and top_contact_legs
            and data.qpos[0] >= args.landing_capture_start_x
        ):
            captured_landing_pose = data.qpos[env._qpos_ids].copy()
            landing_support_leg = top_contact_legs[0]
            landing_swing_leg = 1 - landing_support_leg
            landing_swing_start = data.site_xpos[
                env._foot_site_ids[landing_swing_leg]
            ].copy()
            landing_swing_target_y = data.qpos[1] + (
                0.13 if landing_swing_leg == 0 else -0.13
            )
        position_reset = (
            history_reset_index < len(history_reset_points)
            and data.qpos[0] >= history_reset_points[history_reset_index]
        )
        support_reset = support_level in pending_support_resets
        support_replay_trigger = (
            args.action_replay_trigger_support_step is not None
            and support_level == args.action_replay_trigger_support_step
        )
        position_replay_trigger = data.qpos[0] >= args.action_replay_start_x
        if (
            replay_actions is not None
            and not replay_started
            and (support_replay_trigger or position_replay_trigger)
        ):
            replay_index = 0
            replay_started = True
        if position_reset or support_reset:
            if not args.history_reset_preserve_action:
                last_action = np.zeros(env.action_size)
            fresh_history = np.tile(observation_frame(last_action), 5)
            if args.history_reset_reuse_first and history_reset_template is not None:
                observation = history_reset_template.copy()
            else:
                observation = fresh_history
                history_reset_template = fresh_history.copy()
            if position_reset:
                history_reset_index += 1
            if support_reset:
                pending_support_resets.remove(support_level)
        actor_observation = observation.copy()
        actor_qpos = data.qpos.copy()
        actor_qvel = data.qvel.copy()
        actor_qacc_warmstart = data.qacc_warmstart.copy()
        actor_time = float(data.time)
        action_clip = (
            args.policy_action_clip
            if data.qpos[0] >= args.policy_action_clip_start_x
            else 100.0
        )
        base_policy_action = actor(observation)
        if (
            fixed_residual_actor is not None
            and data.qpos[0] >= fixed_residual_actor.residual_start_x
        ):
            base_policy_action = base_policy_action + (
                fixed_residual_actor.residual_scale
                * fixed_residual_actor(observation)
            )
        if (
            landing_residual_actor is not None
            and top_support_seen
            and data.qpos[0] >= args.landing_residual_start_x
        ):
            active_residual_actor = landing_residual_actor
        elif (
            mid_residual_actor is not None
            and data.qpos[0] >= args.mid_residual_start_x
        ):
            active_residual_actor = mid_residual_actor
        else:
            active_residual_actor = residual_actor
        active_residual_scale = (
            active_residual_actor.residual_scale
            if active_residual_actor is not None
            else residual_scale
        )
        active_residual_start_x = (
            active_residual_actor.residual_start_x
            if active_residual_actor is not None
            else residual_start_x
        )
        if (
            active_residual_actor is not None
            and data.qpos[0] >= active_residual_start_x
        ):
            base_policy_action = (
                base_policy_action
                + active_residual_scale * active_residual_actor(observation)
            )
        if late_actor is not None and data.qpos[0] >= args.late_policy_start_x:
            late_policy_action = late_actor(observation)
            base_policy_action = base_policy_action + args.late_policy_blend * (
                late_policy_action - base_policy_action
            )
        raw_action = np.clip(
            base_policy_action,
            -action_clip,
            action_clip,
        )
        if data.qpos[0] >= args.late_knee_action_clip_start_x:
            raw_action[[3, 9]] = np.clip(
                raw_action[[3, 9]],
                -args.late_knee_action_clip,
                args.late_knee_action_clip,
            )
        policy_action = raw_action.copy()
        gait_frequency = 0.8 + 0.15 * abs(command[0])
        gait_phase = np.fmod(gait_phase + env.dt * gait_frequency, 1.0)
        residual_lifts = np.zeros(2)
        residual_rolls = np.zeros(2)
        ik_desired = [None, None]
        if args.stair_residual:
            if args.stair_residual_mode == "phase":
                phase = np.sin(2.0 * np.pi * gait_phase)
                residual_lifts[:] = (max(0.0, phase), max(0.0, -phase))
            elif env.spec.step_height > 0.0:
                feet = data.site_xpos[env._foot_site_ids]
                foot_velocity = data.sensordata[env._foot_velocity_sensors]
                riser_indices = np.floor(
                    (feet[:, 0] - env.spec.start_x) / env.spec.step_depth
                ).astype(np.int32) + 1
                riser_indices = np.clip(riser_indices, 0, env.spec.step_count - 1)
                next_risers = env.spec.start_x + riser_indices * env.spec.step_depth
                distances = next_risers - feet[:, 0]
                current_heights = np.asarray(
                    [
                        float(env._terrain_height(jp.asarray(foot[0])))
                        for foot in feet
                    ]
                )
                clearances = feet[:, 2] - current_heights
                trigger_candidates = (
                    (distances >= args.stair_trigger_min_distance)
                    & (distances <= args.stair_trigger_max_distance)
                    & (riser_indices >= args.stair_residual_start_step - 1)
                    & (foot_velocity[:, 0] > args.stair_trigger_min_vx)
                    & (foot_velocity[:, 2] > args.stair_trigger_min_vz)
                    & (clearances > args.stair_trigger_min_clearance)
                    & np.isnan(active_risers)
                )
                triggers = np.zeros(2, dtype=bool)
                if (
                    settle_remaining <= 0.0
                    and not np.any(np.isfinite(active_risers))
                    and np.any(
                    trigger_candidates
                    )
                ):
                    candidates = np.flatnonzero(trigger_candidates)
                    selected = candidates[
                        np.argmax(foot_velocity[candidates, 0])
                    ]
                    triggers[selected] = True
                active_risers[triggers] = next_risers[triggers]
                residual_ages[triggers] = 0.0
                swing_starts[triggers] = feet[triggers]

                active = np.isfinite(active_risers)
                residual_ages[active] += env.dt
                crossed = feet[:, 0] >= (
                    active_risers + args.stair_riser_cross_margin
                )
                timed_out = residual_ages >= args.stair_residual_max_duration
                finished = active & (crossed | timed_out)
                if np.any(finished):
                    settle_remaining = max(
                        settle_remaining,
                        args.stair_settle_duration,
                    )
                active_risers[finished] = np.nan
                residual_ages[finished] = 0.0
                swing_starts[finished] = np.nan
                active = np.isfinite(active_risers)
                residual_lifts[active] = np.minimum(
                    residual_ages[active] / 0.03,
                    1.0,
                )
                if args.stair_residual_mode == "terrain":
                    bounded_foot_y = np.clip(
                        feet[:, 1],
                        -args.stair_foot_lateral_limit,
                        args.stair_foot_lateral_limit,
                    )
                    residual_rolls[active] = np.clip(
                        (bounded_foot_y[active] - feet[active, 1]) / 0.8,
                        -args.stair_hip_roll_limit,
                        args.stair_hip_roll_limit,
                    )

            if args.stair_residual_mode != "terrain_ik":
                for lift, roll_delta, hip, roll, knee, ankle in (
                    (residual_lifts[0], residual_rolls[0], 0, 1, 3, 4),
                    (residual_lifts[1], residual_rolls[1], 6, 7, 9, 10),
                ):
                    yaw = np.arctan2(
                        2.0
                        * (
                            data.qpos[3] * data.qpos[6]
                            + data.qpos[4] * data.qpos[5]
                        ),
                        1.0 - 2.0 * (data.qpos[5] ** 2 + data.qpos[6] ** 2),
                    )
                    hip_direction = (
                        -1.0
                        if args.stair_residual_mode == "phase"
                        else -np.sign(np.cos(yaw))
                    )
                    raw_action[hip] += (
                        hip_direction * args.stair_hip_lift * lift / action_scale[hip]
                    )
                    raw_action[roll] += roll_delta / action_scale[roll]
                    raw_action[knee] += (
                        args.stair_knee_lift * lift / action_scale[knee]
                    )
                    raw_action[ankle] -= (
                        args.stair_ankle_lift * lift / action_scale[ankle]
                    )
        if data.qpos[0] >= args.landing_leg_action_start_x:
            raw_action[:12] *= args.landing_leg_action_scale
        if args.height_knee_kp > 0.0 and data.qpos[0] >= args.height_knee_start_x:
            feet_for_support = data.site_xpos[env._foot_site_ids]
            support_for_control = 0.0
            for foot in feet_for_support:
                terrain_for_foot = float(
                    env._terrain_height(jp.asarray(foot[0]))
                )
                if foot[2] - terrain_for_foot < 0.035:
                    support_for_control = max(
                        support_for_control,
                        terrain_for_foot,
                    )
            height_error = max(
                0.0,
                args.height_knee_target
                - (data.qpos[2] - support_for_control),
            )
            knee_extension = min(
                args.height_knee_kp * height_error,
                args.height_knee_max,
            )
            raw_action[[3, 9]] -= knee_extension / action_scale[[3, 9]]
        if (
            args.single_support_roll_gain != 0.0
            and data.qpos[0] >= args.single_support_roll_start_x
        ):
            support_feet = data.site_xpos[env._foot_site_ids]
            contacts = []
            for foot in support_feet:
                foot_height = float(env._terrain_height(jp.asarray(foot[0])))
                contacts.append(foot[2] - foot_height < 0.025)
            if sum(contacts) == 1:
                support_leg = int(np.flatnonzero(contacts)[0])
                inside_offset = -0.13 if support_leg == 0 else 0.13
                target_base_y = support_feet[support_leg, 1] + inside_offset
                base_error = target_base_y - data.qpos[1]
                roll_delta = np.clip(
                    -args.single_support_roll_gain * base_error / 0.8,
                    -args.single_support_roll_limit,
                    args.single_support_roll_limit,
                )
                roll_index = 1 if support_leg == 0 else 7
                raw_action[roll_index] += roll_delta / action_scale[roll_index]
        if replay_index is not None:
            raw_action = replay_actions[replay_index].copy()
            replay_index += 1
            if replay_index >= len(replay_actions):
                replay_index = None
        action = args.action_filter_alpha * raw_action + (
            1.0 - args.action_filter_alpha
        ) * last_action
        target = np.clip(default_pose + action * action_scale, lower, upper)
        position = data.qpos[env._qpos_ids]
        velocity = data.qvel[env._qvel_ids]
        if args.stair_residual and args.stair_residual_mode == "terrain_ik":
            ik_applied = False
            clearance = (
                env.spec.step_height + 0.03
                if args.stair_ik_clearance < 0.0
                else args.stair_ik_clearance
            )
            for leg_index, joint_indices in enumerate(
                ((0, 1, 3), (6, 7, 9))
            ):
                if not np.isfinite(active_risers[leg_index]):
                    continue
                ik_applied = True
                phase = np.clip(
                    residual_ages[leg_index] / args.stair_residual_max_duration,
                    0.0,
                    1.0,
                )
                blend = phase * phase * (3.0 - 2.0 * phase)
                start_foot = swing_starts[leg_index]
                landing_height = float(
                    env._terrain_height(
                        jp.asarray(active_risers[leg_index] + 1.0e-4)
                    )
                )
                desired = np.asarray(
                    [
                        start_foot[0]
                        + (
                            active_risers[leg_index]
                            + args.stair_ik_landing_depth
                            - start_foot[0]
                        )
                        * blend,
                        np.clip(
                            start_foot[1],
                            -args.stair_foot_lateral_limit,
                            args.stair_foot_lateral_limit,
                        ),
                        start_foot[2]
                        + (landing_height + 0.003 - start_foot[2]) * blend
                        + clearance * np.sin(np.pi * phase),
                    ]
                )
                ik_desired[leg_index] = desired.tolist()
                jacobian = np.zeros((3, model.nv))
                mujoco.mj_jacSite(
                    model,
                    data,
                    jacobian,
                    None,
                    int(env._foot_site_ids[leg_index]),
                )
                policy_indices = np.asarray(joint_indices)
                columns = env._qvel_ids[policy_indices]
                reduced = jacobian[:, columns]
                error = desired - data.site_xpos[env._foot_site_ids[leg_index]]
                regularized = (
                    reduced @ reduced.T
                    + np.eye(3) * args.stair_ik_damping**2
                )
                correction = (
                    reduced.T @ np.linalg.solve(regularized, error)
                ) * args.stair_ik_gain
                limits = np.asarray(
                    [
                        args.stair_ik_max_correction,
                        args.stair_hip_roll_limit,
                        args.stair_ik_max_correction,
                    ]
                )
                correction = np.clip(correction, -limits, limits)
                target[policy_indices] += correction
            if ik_applied:
                target = np.clip(target, lower, upper)
                action = (target - default_pose) / action_scale
        if args.top_support_leg_lock:
            for leg_index in range(2):
                if np.all(np.isfinite(locked_leg_targets[leg_index])):
                    start_index = leg_index * 6
                    if args.top_support_lock_all_joints:
                        target[start_index : start_index + 6] = (
                            locked_leg_targets[leg_index]
                        )
                        target[start_index + 3] -= args.top_support_knee_extension
                        continue
                    sagittal_indices = np.asarray(
                        (start_index, start_index + 3, start_index + 4)
                    )
                    locked_sagittal = locked_leg_targets[leg_index][
                        np.asarray((0, 3, 4))
                    ].copy()
                    locked_sagittal[1] -= args.top_support_knee_extension
                    target[sagittal_indices] = locked_sagittal
        elif args.top_support_knee_extension != 0.0:
            for leg_index in np.flatnonzero(top_landed_legs):
                target[leg_index * 6 + 3] -= args.top_support_knee_extension
        landing_stand_active = landing_stand_active or (
            support_level >= env.spec.step_count
            if args.landing_stand_trigger_top_support
            else data.qpos[0] >= args.landing_stand_start_x
        )
        if landing_stand_active:
            landing_stand_elapsed += env.dt
        landing_ramp = (
            min(landing_stand_elapsed / args.landing_stand_ramp_duration, 1.0)
            if args.landing_stand_ramp_duration > 0.0
            else float(landing_stand_active)
        )
        landing_blend = args.landing_stand_blend * landing_ramp
        landing_target_blend = landing_blend
        landing_gain_blend = landing_blend
        if captured_landing_pose is not None:
            captured_landing_elapsed += env.dt
        if captured_landing_pose is not None and args.landing_captured_pose_control:
            captured_ramp = (
                min(
                    captured_landing_elapsed
                    / args.landing_captured_pose_ramp_duration,
                    1.0,
                )
                if args.landing_captured_pose_ramp_duration > 0.0
                else 0.0
            )
            target = np.clip(
                captured_landing_pose
                + captured_ramp * (stand_default_pose - captured_landing_pose),
                lower,
                upper,
            )
            landing_target_blend = 0.0
            landing_gain_blend = float(args.landing_captured_use_stand_gains)
            if args.top_support_knee_extension != 0.0:
                target[np.asarray((3, 9))] -= args.top_support_knee_extension
        if top_support_seen and (
            args.landing_ankle_pitch_kp != 0.0
            or args.landing_ankle_pitch_kd != 0.0
            or args.landing_hip_pitch_kp != 0.0
            or args.landing_hip_pitch_kd != 0.0
        ):
            quaternion = data.qpos[3:7]
            pitch = np.arcsin(
                np.clip(
                    2.0
                    * (
                        quaternion[0] * quaternion[2]
                        - quaternion[3] * quaternion[1]
                    ),
                    -1.0,
                    1.0,
                )
            )
            pitch_delta = np.clip(
                args.landing_ankle_pitch_kp * pitch
                + args.landing_ankle_pitch_kd * data.qvel[4],
                -args.landing_ankle_pitch_limit,
                args.landing_ankle_pitch_limit,
            )
            target[np.asarray((4, 10))] += pitch_delta
            hip_pitch_delta = np.clip(
                args.landing_hip_pitch_kp * pitch
                + args.landing_hip_pitch_kd * data.qvel[4],
                -args.landing_hip_pitch_limit,
                args.landing_hip_pitch_limit,
            )
            target[np.asarray((0, 6))] += hip_pitch_delta
            target = np.clip(target, lower, upper)
        if (
            captured_landing_pose is not None
            and args.landing_follow_step
            and landing_swing_leg is not None
        ):
            phase = np.clip(
                captured_landing_elapsed / args.landing_follow_step_duration,
                0.0,
                1.0,
            )
            blend = phase * phase * (3.0 - 2.0 * phase)
            desired = np.asarray(
                [
                    landing_swing_start[0]
                    + (
                        env.spec.end_x
                        + args.landing_follow_step_depth
                        - landing_swing_start[0]
                    )
                    * blend,
                    landing_swing_start[1]
                    + (landing_swing_target_y - landing_swing_start[1]) * blend,
                    landing_swing_start[2]
                    + (env.spec.top_height + 0.003 - landing_swing_start[2])
                    * blend
                    + args.landing_follow_step_clearance * np.sin(np.pi * phase),
                ]
            )
            jacobian = np.zeros((3, model.nv))
            mujoco.mj_jacSite(
                model,
                data,
                jacobian,
                None,
                int(env._foot_site_ids[landing_swing_leg]),
            )
            policy_indices = np.arange(
                landing_swing_leg * 6,
                landing_swing_leg * 6 + 6,
            )
            reduced = jacobian[:, env._qvel_ids[policy_indices]]
            error = desired - data.site_xpos[env._foot_site_ids[landing_swing_leg]]
            regularized = (
                reduced @ reduced.T + np.eye(3) * args.stair_ik_damping**2
            )
            correction = (
                reduced.T @ np.linalg.solve(regularized, error)
            ) * args.landing_follow_step_gain
            correction = np.clip(
                correction,
                -args.landing_follow_step_max_correction,
                args.landing_follow_step_max_correction,
            )
            target[policy_indices] += correction
            target = np.clip(target, lower, upper)
            ik_desired[landing_swing_leg] = desired.tolist()
        active_kp = (
            (1.0 - landing_gain_blend) * kp
            + landing_gain_blend * stand_kp
        )
        active_kd = (
            (1.0 - landing_gain_blend) * kd
            + landing_gain_blend * stand_kd
        )
        target = np.clip(
            (1.0 - landing_target_blend) * target
            + landing_target_blend * stand_default_pose,
            lower,
            upper,
        )
        control = np.clip(
            active_kp * (target - position) - active_kd * velocity,
            -pr_limits,
            pr_limits,
        )
        for indices, mapping in pairs:
            pair = np.asarray(indices)
            control[pair] = np.asarray(
                pr_torque_to_ab(
                    mapping,
                    jp.asarray(position[pair]),
                    jp.asarray(control[pair]),
                )
            )
        data.ctrl[:] = np.clip(control, -motor_limits, motor_limits)
        for _ in range(round(env.dt / model.opt.timestep)):
            mujoco.mj_step(model, data)
        settle_remaining = max(0.0, settle_remaining - env.dt)

        last_action = action
        observation = np.roll(observation, 102)
        observation[:102] = observation_frame(last_action)

        if args.trace_output is not None:
            feet = data.site_xpos[env._foot_site_ids]
            foot_velocity = data.sensordata[env._foot_velocity_sensors]
            trace.append(
                {
                    "step": step_index + 1,
                    "time": float(data.time),
                    "base": [float(value) for value in data.qpos[:3]],
                    "quaternion": [float(value) for value in data.qpos[3:7]],
                    "base_velocity": [float(value) for value in data.qvel[:6]],
                    "joint_position": position.tolist(),
                    "joint_target": target.tolist(),
                    "joint_control": control.tolist(),
                    "feet": feet.tolist(),
                    "foot_velocity": foot_velocity.tolist(),
                    "stair_residual_lift": residual_lifts.tolist(),
                    "stair_residual_roll": residual_rolls.tolist(),
                    "stair_residual_riser": [
                        None if np.isnan(value) else float(value)
                        for value in active_risers
                    ],
                    "command": command.tolist(),
                    "stair_settle_remaining": float(settle_remaining),
                    "observation": actor_observation.tolist(),
                    "state_qpos": actor_qpos.tolist(),
                    "state_qvel": actor_qvel.tolist(),
                    "state_qacc_warmstart": actor_qacc_warmstart.tolist(),
                    "state_time": actor_time,
                    "policy_action": policy_action.tolist(),
                    "action": action.tolist(),
                    "leg_action": action[:12].tolist(),
                }
            )

        if renderer is not None and step_index % args.render_stride == 0:
            renderer.update_scene(data, camera="track")
            frames.append(renderer.render())

        body_up_z = 1.0 - 2.0 * (data.qpos[4] ** 2 + data.qpos[5] ** 2)
        support_height = 0.0
        for foot in data.site_xpos[env._foot_site_ids]:
            foot_terrain_height = float(
                env._terrain_height(jp.asarray(foot[0]))
            )
            if foot[2] - foot_terrain_height < 0.035:
                support_height = max(support_height, foot_terrain_height)
        max_support_height = max(max_support_height, support_height)
        max_base_height = max(max_base_height, float(data.qpos[2]))
        if not np.all(np.isfinite(data.qpos)):
            result = "invalid"
            break
        if (
            data.qpos[2] < support_height + args.fall_height_clearance
            or body_up_z < 0.45
        ):
            result = "fallen"
            break
        reached_landing = data.qpos[0] > env.spec.end_x + 0.35
        high_enough = data.qpos[2] > env.spec.top_height + 0.72
        top_supported = support_height >= (
            env.spec.top_height - 0.5 * env.spec.step_height
        )
        qualifies_for_success = (
            env.spec.step_height > 0
            and reached_landing
            and high_enough
            and (top_supported or not args.require_top_support)
        )
        success_hold_count = (
            success_hold_count + 1 if qualifies_for_success else 0
        )
        if success_hold_count >= args.success_hold_steps:
            result = "success"
            break

    yaw = np.degrees(
        np.arctan2(
            2.0 * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
            1.0 - 2.0 * (data.qpos[5] ** 2 + data.qpos[6] ** 2),
        )
    )
    print(
        f"steps={step_index + 1} result={result} x={data.qpos[0]:.3f} "
        f"dx={data.qpos[0] - start[0]:.3f} y={data.qpos[1]:.3f} "
        f"z={data.qpos[2]:.3f} yaw={yaw:.1f}deg "
        f"support_z={support_height:.3f} up_z={body_up_z:.3f} "
        f"max_support_z={max_support_height:.3f} max_base_z={max_base_height:.3f}"
    )

    if renderer is not None:
        renderer.close()
        video = args.video.expanduser().resolve()
        video.parent.mkdir(parents=True, exist_ok=True)
        media.write_video(
            video,
            frames,
            fps=round(1.0 / env.dt / args.render_stride),
        )
        print(video)

    if args.trace_output is not None:
        trace_output = args.trace_output.expanduser().resolve()
        trace_output.parent.mkdir(parents=True, exist_ok=True)
        trace_output.write_text(
            json.dumps(trace, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        print(trace_output)


if __name__ == "__main__":
    main()
