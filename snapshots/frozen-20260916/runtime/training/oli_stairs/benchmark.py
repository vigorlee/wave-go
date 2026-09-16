"""Batch robustness benchmark for OLI locomotion checkpoints."""

from __future__ import annotations

import os

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import json
from pathlib import Path
from typing import Any

from brax.training.agents.ppo import checkpoint as ppo_checkpoint
import jax
import jax.numpy as jp
import numpy as np

from training.oli_stairs.checkpointing import resolve_checkpoint
from training.oli_stairs.constants import (
    CONTROL_PROFILES,
    STAGE_HEIGHTS,
    stair_spec_for_stage,
)
from training.oli_stairs.environment import OliStairEnv, default_config
from training.oli_stairs.network import make_network_factory


def _stats(values: np.ndarray) -> dict[str, float]:
    return {
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="flat")
    parser.add_argument("--checkpoint", type=Path, nargs="+", required=True)
    parser.add_argument("--steps", type=int, default=1800)
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.01, 0.03, 0.06])
    parser.add_argument("--yaw-rate", type=float, default=0.0)
    parser.add_argument("--episodes-per-speed", type=int, default=32)
    parser.add_argument("--seed", type=int, default=20260719)
    parser.add_argument(
        "--control-profile", choices=CONTROL_PROFILES, default="walk"
    )
    parser.add_argument(
        "--parallel-control-mode",
        choices=("motor_pd", "projected_torque"),
        default="motor_pd",
    )
    parser.add_argument("--teacher-policy", type=Path)
    parser.add_argument("--teacher-blend", type=float, default=1.0)
    parser.add_argument("--balance-assist", type=float, default=0.0)
    parser.add_argument("--heading-hold-kp", type=float, default=0.0)
    parser.add_argument("--heading-hold-max-yaw-rate", type=float, default=0.30)
    parser.add_argument("--heading-action-kp", type=float, default=0.0)
    parser.add_argument("--heading-action-limit", type=float, default=0.5)
    parser.add_argument("--action-gain", type=float, default=1.0)
    parser.add_argument("--leg-action-gain", type=float, default=-1.0)
    parser.add_argument("--action-filter-alpha", type=float, default=0.10)
    parser.add_argument("--leg-action-filter-alpha", type=float, default=-1.0)
    parser.add_argument("--sagittal-action-filter-alpha", type=float, default=-1.0)
    parser.add_argument("--observation-noise", type=float, default=0.0)
    parser.add_argument("--reset-joint-noise", type=float, default=0.02)
    parser.add_argument("--reset-velocity-noise", type=float, default=0.08)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    speeds = np.asarray(args.speeds, dtype=np.float32)
    if np.any(speeds < 0.0):
        parser.error("--speeds must be non-negative")
    if args.episodes_per_speed < 1:
        parser.error("--episodes-per-speed must be positive")
    if args.steps < 1:
        parser.error("--steps must be positive")

    num_envs = len(speeds) * args.episodes_per_speed
    config = default_config()
    config.control_profile = args.control_profile
    config.parallel_control_mode = args.parallel_control_mode
    config.command_speed = [float(speeds.min()), float(speeds.max())]
    config.command_lateral = [0.0, 0.0]
    config.command_yaw = [0.0, 0.0]
    config.stand_probability = 0.0
    config.observation_noise = args.observation_noise
    config.reset_joint_noise = args.reset_joint_noise
    config.reset_velocity_noise = args.reset_velocity_noise
    config.balance_assist_min = args.balance_assist
    config.balance_assist_max = args.balance_assist
    config.heading_hold_kp = args.heading_hold_kp
    config.heading_hold_max_yaw_rate = args.heading_hold_max_yaw_rate
    config.heading_action_kp = args.heading_action_kp
    config.heading_action_limit = args.heading_action_limit
    config.action_gain = args.action_gain
    config.leg_action_gain = args.leg_action_gain
    config.action_filter_alpha = args.action_filter_alpha
    config.leg_action_filter_alpha = args.leg_action_filter_alpha
    config.sagittal_action_filter_alpha = args.sagittal_action_filter_alpha
    if args.teacher_policy is not None:
        config.teacher_policy = str(args.teacher_policy.expanduser().resolve())
    config.teacher_blend = args.teacher_blend
    env = OliStairEnv(stair_spec_for_stage(args.stage), config=config)

    networks = make_network_factory()(env.observation_size, env.action_size)
    distribution = networks.parametric_action_distribution
    reset_batch = jax.jit(jax.vmap(env.reset))
    command_speeds = np.repeat(speeds, args.episodes_per_speed)
    commands = jp.stack(
        [
            jp.asarray(command_speeds),
            jp.zeros(num_envs),
            jp.full((num_envs,), args.yaw_rate),
        ],
        axis=1,
    )

    def rollout(state, params):
        alive = jp.ones((num_envs,), dtype=jp.bool_)
        first_done = jp.full((num_envs,), args.steps, dtype=jp.int32)
        failure_reason = jp.zeros((num_envs,), dtype=jp.int32)

        def body(carry, _):
            current, still_alive, failure_step, reason = carry
            logits = networks.policy_network.apply(
                params[0], params[1], current.obs
            )
            action = distribution.mode(logits)
            next_state = jax.vmap(env.step)(current, action)
            done = next_state.done > 0.5
            first_failure = still_alive & done
            support_height = jax.vmap(env._support_height)(next_state.data)
            qpos = next_state.data.qpos
            fallen = qpos[:, 2] < support_height + 0.62
            body_up_z = 1.0 - 2.0 * (
                jp.square(qpos[:, 4]) + jp.square(qpos[:, 5])
            )
            tipped = body_up_z < 0.45
            forward_x = 1.0 - 2.0 * (
                jp.square(qpos[:, 5]) + jp.square(qpos[:, 6])
            )
            wrong_heading = forward_x < 0.707
            left_course = jp.abs(qpos[:, 1]) > env.spec.half_width - 0.25
            invalid = ~jp.all(jp.isfinite(qpos), axis=1) | ~jp.all(
                jp.isfinite(next_state.data.qvel), axis=1
            )
            current_reason = (
                fallen.astype(jp.int32)
                + 2 * tipped.astype(jp.int32)
                + 4 * wrong_heading.astype(jp.int32)
                + 8 * left_course.astype(jp.int32)
                + 16 * invalid.astype(jp.int32)
            )
            reason = jp.where(first_failure, current_reason, reason)
            failure_step = jp.where(
                first_failure, next_state.info["step"], failure_step
            )
            return (
                next_state,
                still_alive & ~done,
                failure_step,
                reason,
            ), None

        return jax.lax.scan(
            body,
            (state, alive, first_done, failure_reason),
            None,
            length=args.steps,
        )[0]

    rollout_batch = jax.jit(rollout)
    keys = jax.random.split(jax.random.PRNGKey(args.seed), num_envs)
    duration = args.steps * env.dt
    results: list[dict[str, Any]] = []

    for checkpoint_arg in args.checkpoint:
        checkpoint = resolve_checkpoint(checkpoint_arg)
        params = ppo_checkpoint.load(str(checkpoint))
        initial = reset_batch(keys)
        initial.info["command"] = commands
        initial.info["nominal_command"] = commands
        start_qpos = np.asarray(initial.data.qpos)
        final, alive, failure_step, failure_reason = rollout_batch(initial, params)

        alive = np.asarray(alive)
        failure_step = np.asarray(failure_step)
        failure_reason = np.asarray(failure_reason)
        qpos = np.asarray(final.data.qpos)
        dx = qpos[:, 0] - start_qpos[:, 0]
        yaw = np.degrees(
            np.arctan2(
                2.0 * (qpos[:, 3] * qpos[:, 6] + qpos[:, 4] * qpos[:, 5]),
                1.0 - 2.0 * (qpos[:, 5] ** 2 + qpos[:, 6] ** 2),
            )
        )

        speed_results = []
        for speed_index, speed in enumerate(speeds):
            begin = speed_index * args.episodes_per_speed
            end = begin + args.episodes_per_speed
            group_alive = alive[begin:end]
            group_dx = dx[begin:end]
            group_y = qpos[begin:end, 1]
            group_z = qpos[begin:end, 2]
            group_yaw = yaw[begin:end]
            expected_dx = float(speed * duration)
            straight_pass = (
                group_alive
                & (group_dx >= 0.0)
                & (np.abs(group_y) <= 0.25)
                & (np.abs(group_yaw) <= 10.0)
            )
            tracking_pass = straight_pass & (group_dx >= 0.5 * expected_dx)
            item: dict[str, Any] = {
                "speed": float(speed),
                "expected_dx": expected_dx,
                "survived": int(group_alive.sum()),
                "survival_rate": float(group_alive.mean()),
                "straight_pass_rate": float(straight_pass.mean()),
                "tracking_pass_rate": float(tracking_pass.mean()),
            }
            if group_alive.any():
                item.update(
                    {
                        "dx": _stats(group_dx[group_alive]),
                        "abs_y": _stats(np.abs(group_y[group_alive])),
                        "z": _stats(group_z[group_alive]),
                        "abs_yaw_deg": _stats(np.abs(group_yaw[group_alive])),
                        "yaw_deg": _stats(group_yaw[group_alive]),
                    }
                )
            if (~group_alive).any():
                group_reason = failure_reason[begin:end][~group_alive]
                item["failure_step"] = _stats(
                    failure_step[begin:end][~group_alive]
                )
                item["failure_reasons"] = {
                    "fallen": int(np.count_nonzero(group_reason & 1)),
                    "tipped": int(np.count_nonzero(group_reason & 2)),
                    "wrong_heading": int(np.count_nonzero(group_reason & 4)),
                    "left_course": int(np.count_nonzero(group_reason & 8)),
                    "invalid": int(np.count_nonzero(group_reason & 16)),
                }
            speed_results.append(item)

        result = {
            "checkpoint": str(checkpoint),
            "speeds": speed_results,
        }
        results.append(result)
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)

    report = {
        "stage": args.stage,
        "steps": args.steps,
        "duration_seconds": duration,
        "episodes_per_speed": args.episodes_per_speed,
        "seed": args.seed,
        "control_profile": args.control_profile,
        "parallel_control_mode": args.parallel_control_mode,
        "teacher_policy": config.teacher_policy or None,
        "teacher_blend": args.teacher_blend,
        "balance_assist": args.balance_assist,
        "yaw_rate": args.yaw_rate,
        "heading_hold_kp": args.heading_hold_kp,
        "heading_hold_max_yaw_rate": args.heading_hold_max_yaw_rate,
        "heading_action_kp": args.heading_action_kp,
        "heading_action_limit": args.heading_action_limit,
        "action_gain": args.action_gain,
        "leg_action_gain": args.leg_action_gain,
        "action_filter_alpha": args.action_filter_alpha,
        "leg_action_filter_alpha": args.leg_action_filter_alpha,
        "sagittal_action_filter_alpha": args.sagittal_action_filter_alpha,
        "results": results,
    }
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(output)


if __name__ == "__main__":
    main()
