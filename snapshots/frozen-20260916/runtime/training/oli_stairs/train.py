"""Train the OLI HU_D04_01 stair policy with GPU MJX and Brax PPO."""

from __future__ import annotations

import os

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("JAX_DEFAULT_MATMUL_PRECISION", "highest")

import argparse
from dataclasses import replace
import functools
import json
from pathlib import Path
import time
from typing import Any

from brax.training.agents.ppo import train as ppo
import jax
import jax.numpy as jp
import numpy as np
from mujoco_playground import wrapper

from training.oli_stairs.constants import (
    ARTIFACT_ROOT,
    CONTROL_PROFILES,
    OFFICIAL_POLICY,
    STAGE_HEIGHTS,
    stair_spec_for_stage,
)
from training.oli_stairs.checkpointing import resolve_checkpoint
from training.oli_stairs.environment import OliStairEnv, default_config
from training.oli_stairs.export_onnx import export_actor
from training.oli_stairs.network import (
    make_network_factory,
    make_official_restore_params,
    make_zero_actor_restore_params,
)


def _optional_positive_float(value: str) -> float | None:
    if value.strip().lower() in {"none", "null", "unbounded"}:
        return None
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a positive number or 'none': {value}") from exc
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def _scalar_metrics(metrics: dict[str, Any]) -> dict[str, float]:
    result = {}
    for name, value in metrics.items():
        array = np.asarray(value)
        if array.size == 1:
            result[name] = float(array.reshape(()))
    return result


def _install_single_gpu_jax_compatibility() -> None:
    """Restore the one removed JAX helper still used by this Brax release."""
    if "device_put_replicated" in jax.__dict__:
        return
    devices = jax.local_devices()
    if len(devices) != 1:
        raise RuntimeError(
            "The installed Brax/JAX compatibility shim only supports one GPU"
        )

    def device_put_replicated(value: Any, target_devices: list[Any]) -> Any:
        if len(target_devices) != 1:
            raise RuntimeError("OLI stair training currently supports one GPU")
        return jax.tree_util.tree_map(
            lambda item: jax.device_put(jp.expand_dims(item, axis=0), target_devices[0]),
            value,
        )

    jax.device_put_replicated = device_put_replicated


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h05")
    parser.add_argument("--step-count", type=int)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--riser-ramp-length", type=float, default=0.0)
    parser.add_argument("--approach-distance", type=float)
    parser.add_argument("--timesteps", type=int, default=20_000_000)
    parser.add_argument("--num-envs", type=int, default=2048)
    parser.add_argument("--episode-length", type=int, default=1800)
    parser.add_argument("--sim-dt", type=float)
    parser.add_argument("--impl", choices=("jax", "warp"), default="jax")
    parser.add_argument("--naconmax", type=int)
    parser.add_argument("--njmax", type=int)
    parser.add_argument("--num-evals", type=int, default=8)
    parser.add_argument("--num-eval-envs", type=int, default=128)
    parser.add_argument(
        "--run-evals",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument(
        "--mean-clip-scale",
        type=_optional_positive_float,
        default=4.0,
        help="Brax soft-bound scale, or 'none' to preserve unbounded actor outputs",
    )
    parser.add_argument("--entropy-cost", type=float, default=0.001)
    parser.add_argument("--clipping-epsilon", type=float, default=0.2)
    parser.add_argument("--updates-per-batch", type=int, default=2)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--num-minibatches", type=int, default=32)
    parser.add_argument("--unroll-length", type=int, default=20)
    parser.add_argument("--balance-assist-min", type=float, default=0.0)
    parser.add_argument("--balance-assist-max", type=float, default=0.0)
    parser.add_argument("--heading-hold-kp", type=float, default=0.0)
    parser.add_argument("--heading-hold-max-yaw-rate", type=float, default=0.30)
    parser.add_argument("--heading-action-kp", type=float, default=0.0)
    parser.add_argument("--heading-action-limit", type=float, default=0.5)
    parser.add_argument("--action-gain", type=float, default=1.0)
    parser.add_argument("--action-clip", type=float)
    parser.add_argument("--leg-action-gain", type=float, default=-1.0)
    parser.add_argument("--action-filter-alpha", type=float, default=0.10)
    parser.add_argument("--leg-action-filter-alpha", type=float, default=-1.0)
    parser.add_argument("--sagittal-action-filter-alpha", type=float, default=-1.0)
    parser.add_argument("--observation-noise", type=float)
    parser.add_argument("--reset-joint-noise", type=float)
    parser.add_argument("--reset-velocity-noise", type=float)
    parser.add_argument("--reset-x-noise", type=float)
    parser.add_argument("--reset-y-noise", type=float)
    parser.add_argument("--reset-yaw-noise", type=float)
    parser.add_argument("--command-speed-min", type=float)
    parser.add_argument("--command-speed-max", type=float)
    parser.add_argument("--command-lateral-min", type=float)
    parser.add_argument("--command-lateral-max", type=float)
    parser.add_argument("--history-lateral-speed", type=float)
    parser.add_argument("--stand-probability", type=float)
    parser.add_argument("--tracking-sigma", type=float)
    parser.add_argument("--swing-clearance", type=float)
    parser.add_argument("--tracking-lin-vel-scale", type=float)
    parser.add_argument("--tracking-yaw-scale", type=float)
    parser.add_argument("--forward-progress-scale", type=float)
    parser.add_argument("--backward-velocity-scale", type=float)
    parser.add_argument("--heading-alignment-scale", type=float)
    parser.add_argument("--lateral-drift-scale", type=float)
    parser.add_argument("--upward-progress-scale", type=float)
    parser.add_argument("--base-height-scale", type=float)
    parser.add_argument("--stair-height-scale", type=float)
    parser.add_argument("--stair-foot-progress-scale", type=float)
    parser.add_argument("--stair-foot-target-scale", type=float)
    parser.add_argument("--stair-success-scale", type=float)
    parser.add_argument("--feet-clearance-scale", type=float)
    parser.add_argument(
        "--control-profile", choices=CONTROL_PROFILES, default="walk"
    )
    parser.add_argument(
        "--parallel-control-mode",
        choices=("motor_pd", "projected_torque"),
        default="motor_pd",
    )
    parser.add_argument("--reduced-model", action="store_true")
    parser.add_argument("--teacher-policy", type=Path)
    parser.add_argument("--teacher-blend", type=float, default=1.0)
    parser.add_argument("--teacher-deviation-scale", type=float)
    parser.add_argument(
        "--adaptive-kl",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--restore", type=Path)
    parser.add_argument("--reset-value-function", action="store_true")
    parser.add_argument("--freeze-policy-backbone", action="store_true")
    parser.add_argument("--no-official-init", action="store_true")
    parser.add_argument("--init-policy", type=Path)
    parser.add_argument("--zero-actor-init", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.restore is not None and args.zero_actor_init:
        parser.error("--zero-actor-init cannot be combined with --restore")
    if args.step_count is not None and args.step_count < 1:
        parser.error("--step-count must be positive")
    for name in ("step_depth", "landing_length", "stair_half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.riser_ramp_length < 0.0:
        parser.error("--riser-ramp-length cannot be negative")
    if args.approach_distance is not None and args.approach_distance <= 0.0:
        parser.error("--approach-distance must be positive")
    if args.sim_dt is not None and args.sim_dt <= 0.0:
        parser.error("--sim-dt must be positive")
    if args.num_eval_envs < 1:
        parser.error("--num-eval-envs must be positive")
    if args.naconmax is not None and args.naconmax < 1:
        parser.error("--naconmax must be positive")
    if args.njmax is not None and args.njmax < 1:
        parser.error("--njmax must be positive")
    if args.batch_size < 1 or args.num_minibatches < 1 or args.unroll_length < 1:
        parser.error("PPO batch and unroll sizes must be positive")
    if args.batch_size * args.num_minibatches % args.num_envs != 0:
        parser.error("--batch-size * --num-minibatches must divide by --num-envs")
    if jax.default_backend() != "gpu":
        raise RuntimeError(
            f"OLI stair training requires GPU JAX, got {jax.default_backend()!r}"
        )
    _install_single_gpu_jax_compatibility()

    output_dir = (
        args.output_dir or ARTIFACT_ROOT / "runs" / args.stage
    ).expanduser().resolve()
    restore_path = resolve_checkpoint(args.restore) if args.restore else None
    checkpoint_dir = output_dir / "checkpoints"
    output_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = output_dir / "metrics.jsonl"

    env_config = default_config()
    env_config.episode_length = args.episode_length
    env_config.impl = args.impl
    if args.naconmax is not None:
        env_config.naconmax = args.naconmax
    elif args.impl == "warp":
        env_config.naconmax = 4096
    if args.njmax is not None:
        env_config.njmax = args.njmax
    elif args.impl == "warp":
        env_config.njmax = 8192
    if args.sim_dt is not None:
        env_config.sim_dt = args.sim_dt
    env_config.control_profile = args.control_profile
    env_config.parallel_control_mode = args.parallel_control_mode
    env_config.full_parallel_model = not args.reduced_model
    if args.approach_distance is not None:
        env_config.stair_approach_distance = args.approach_distance
    env_config.balance_assist_min = args.balance_assist_min
    env_config.balance_assist_max = args.balance_assist_max
    env_config.heading_hold_kp = args.heading_hold_kp
    env_config.heading_hold_max_yaw_rate = args.heading_hold_max_yaw_rate
    env_config.heading_action_kp = args.heading_action_kp
    env_config.heading_action_limit = args.heading_action_limit
    env_config.action_gain = args.action_gain
    if args.action_clip is not None:
        env_config.action_clip = args.action_clip
    env_config.leg_action_gain = args.leg_action_gain
    env_config.action_filter_alpha = args.action_filter_alpha
    env_config.leg_action_filter_alpha = args.leg_action_filter_alpha
    env_config.sagittal_action_filter_alpha = args.sagittal_action_filter_alpha
    if args.stage == "flat":
        env_config.command_speed = [0.18, 0.35]
        env_config.stand_probability = 0.05
    else:
        env_config.command_speed = [0.20, 0.35]
        env_config.stand_probability = 0.0
    env_config.command_lateral = [0.0, 0.0]
    env_config.command_yaw = [0.0, 0.0]
    if args.observation_noise is not None:
        env_config.observation_noise = args.observation_noise
    if args.reset_joint_noise is not None:
        env_config.reset_joint_noise = args.reset_joint_noise
    if args.reset_velocity_noise is not None:
        env_config.reset_velocity_noise = args.reset_velocity_noise
    if args.reset_x_noise is not None:
        env_config.reset_x_noise = args.reset_x_noise
    if args.reset_y_noise is not None:
        env_config.reset_y_noise = args.reset_y_noise
    if args.reset_yaw_noise is not None:
        env_config.reset_yaw_noise = args.reset_yaw_noise
    if args.command_speed_min is not None:
        env_config.command_speed[0] = args.command_speed_min
    if args.command_speed_max is not None:
        env_config.command_speed[1] = args.command_speed_max
    if args.command_lateral_min is not None:
        env_config.command_lateral[0] = args.command_lateral_min
    if args.command_lateral_max is not None:
        env_config.command_lateral[1] = args.command_lateral_max
    if args.history_lateral_speed is not None:
        env_config.history_lateral_speed = args.history_lateral_speed
    if args.stand_probability is not None:
        env_config.stand_probability = args.stand_probability
    if args.tracking_sigma is not None:
        env_config.reward_config.tracking_sigma = args.tracking_sigma
    if args.swing_clearance is not None:
        env_config.reward_config.swing_clearance = args.swing_clearance
    if args.tracking_lin_vel_scale is not None:
        env_config.reward_config.scales.tracking_lin_vel = (
            args.tracking_lin_vel_scale
        )
    if args.tracking_yaw_scale is not None:
        env_config.reward_config.scales.tracking_yaw = args.tracking_yaw_scale
    if args.forward_progress_scale is not None:
        env_config.reward_config.scales.forward_progress = (
            args.forward_progress_scale
        )
    if args.backward_velocity_scale is not None:
        env_config.reward_config.scales.backward_velocity = (
            args.backward_velocity_scale
        )
    if args.heading_alignment_scale is not None:
        env_config.reward_config.scales.heading_alignment = (
            args.heading_alignment_scale
        )
    if args.lateral_drift_scale is not None:
        env_config.reward_config.scales.lateral_drift = args.lateral_drift_scale
    if args.upward_progress_scale is not None:
        env_config.reward_config.scales.upward_progress = args.upward_progress_scale
    if args.base_height_scale is not None:
        env_config.reward_config.scales.base_height = args.base_height_scale
    if args.stair_height_scale is not None:
        env_config.reward_config.scales.stair_height = args.stair_height_scale
    if args.stair_foot_progress_scale is not None:
        env_config.reward_config.scales.stair_foot_progress = (
            args.stair_foot_progress_scale
        )
    if args.stair_foot_target_scale is not None:
        env_config.reward_config.scales.stair_foot_target = (
            args.stair_foot_target_scale
        )
    if args.stair_success_scale is not None:
        env_config.reward_config.scales.stair_success = args.stair_success_scale
    if args.feet_clearance_scale is not None:
        env_config.reward_config.scales.feet_clearance = args.feet_clearance_scale
    if args.teacher_deviation_scale is not None:
        env_config.reward_config.scales.teacher_deviation = (
            args.teacher_deviation_scale
        )
    if args.teacher_policy is not None:
        env_config.teacher_policy = str(args.teacher_policy.expanduser().resolve())
    env_config.teacher_blend = args.teacher_blend
    stair_spec = stair_spec_for_stage(args.stage)
    stair_spec = replace(
        stair_spec,
        step_count=(args.step_count if args.step_count is not None else stair_spec.step_count),
        step_depth=args.step_depth,
        landing_length=args.landing_length,
        half_width=args.stair_half_width,
        riser_ramp_length=args.riser_ramp_length,
    )
    env = OliStairEnv(stair_spec, config=env_config)
    network_factory = make_network_factory(
        freeze_policy_backbone=args.freeze_policy_backbone,
        mean_clip_scale=args.mean_clip_scale,
    )

    restore_params = None
    if restore_path is None:
        if args.zero_actor_init:
            restore_params = make_zero_actor_restore_params(args.seed)
        elif not args.no_official_init:
            init_policy = args.init_policy or OFFICIAL_POLICY
            restore_params = make_official_restore_params(
                args.seed,
                policy_path=init_policy.expanduser().resolve(),
                mean_clip_scale=args.mean_clip_scale,
            )

    started = time.monotonic()

    def progress(step: int, metrics: dict[str, Any]) -> None:
        record = {
            "step": step,
            "elapsed_seconds": time.monotonic() - started,
            **_scalar_metrics(metrics),
        }
        with metrics_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, sort_keys=True) + "\n")
        success = record.get("eval/episode_stair/success", 0.0)
        reward = record.get("eval/episode_reward", float("nan"))
        print(
            f"step={step:,} reward={reward:.4f} success={success:.3f} "
            f"elapsed={record['elapsed_seconds']:.1f}s",
            flush=True,
        )

    run_config = {
        "stage": args.stage,
        "step_count": stair_spec.step_count,
        "approach_distance": env_config.stair_approach_distance,
        "timesteps": args.timesteps,
        "num_envs": args.num_envs,
        "num_eval_envs": args.num_eval_envs,
        "run_evals": args.run_evals,
        "episode_length": args.episode_length,
        "sim_dt": env_config.sim_dt,
        "impl": env_config.impl,
        "naconmax": env_config.naconmax,
        "njmax": env_config.njmax,
        "seed": args.seed,
        "learning_rate": args.learning_rate,
        "mean_clip_scale": args.mean_clip_scale,
        "entropy_cost": args.entropy_cost,
        "clipping_epsilon": args.clipping_epsilon,
        "updates_per_batch": args.updates_per_batch,
        "batch_size": args.batch_size,
        "num_minibatches": args.num_minibatches,
        "unroll_length": args.unroll_length,
        "adaptive_kl": args.adaptive_kl,
        "balance_assist_min": args.balance_assist_min,
        "balance_assist_max": args.balance_assist_max,
        "heading_hold_kp": env_config.heading_hold_kp,
        "heading_hold_max_yaw_rate": env_config.heading_hold_max_yaw_rate,
        "heading_action_kp": env_config.heading_action_kp,
        "heading_action_limit": env_config.heading_action_limit,
        "action_gain": env_config.action_gain,
        "action_clip": env_config.action_clip,
        "leg_action_gain": env_config.leg_action_gain,
        "action_filter_alpha": env_config.action_filter_alpha,
        "leg_action_filter_alpha": env_config.leg_action_filter_alpha,
        "sagittal_action_filter_alpha": env_config.sagittal_action_filter_alpha,
        "observation_noise": env_config.observation_noise,
        "reset_joint_noise": env_config.reset_joint_noise,
        "reset_velocity_noise": env_config.reset_velocity_noise,
        "reset_x_noise": env_config.reset_x_noise,
        "reset_y_noise": env_config.reset_y_noise,
        "reset_yaw_noise": env_config.reset_yaw_noise,
        "command_speed": list(env_config.command_speed),
        "command_lateral": list(env_config.command_lateral),
        "history_lateral_speed": env_config.history_lateral_speed,
        "stand_probability": env_config.stand_probability,
        "control_profile": env_config.control_profile,
        "parallel_control_mode": env_config.parallel_control_mode,
        "full_parallel_model": env_config.full_parallel_model,
        "teacher_policy": env_config.teacher_policy or None,
        "teacher_blend": env_config.teacher_blend,
        "restore": str(restore_path) if restore_path else None,
        "restore_value_function": bool(
            restore_path is not None and not args.reset_value_function
        ),
        "freeze_policy_backbone": args.freeze_policy_backbone,
        "reward_scales": dict(env_config.reward_config.scales),
        "tracking_sigma": env_config.reward_config.tracking_sigma,
        "official_init": restore_params is not None and not args.zero_actor_init,
        "zero_actor_init": restore_params is not None and args.zero_actor_init,
        "jax_backend": jax.default_backend(),
        "jax_devices": [str(device) for device in jax.devices()],
        "xml_path": env.xml_path,
    }
    (output_dir / "run_config.json").write_text(
        json.dumps(run_config, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    _, params, metrics = ppo.train(
        environment=env,
        num_timesteps=args.timesteps,
        num_envs=args.num_envs,
        episode_length=args.episode_length,
        action_repeat=1,
        learning_rate=args.learning_rate,
        entropy_cost=args.entropy_cost,
        discounting=0.99,
        unroll_length=args.unroll_length,
        batch_size=args.batch_size,
        num_minibatches=args.num_minibatches,
        num_updates_per_batch=args.updates_per_batch,
        normalize_observations=False,
        reward_scaling=1.0,
        clipping_epsilon=args.clipping_epsilon,
        gae_lambda=0.95,
        max_grad_norm=1.0,
        desired_kl=0.02,
        learning_rate_schedule="ADAPTIVE_KL" if args.adaptive_kl else None,
        learning_rate_schedule_min_lr=1e-6,
        learning_rate_schedule_max_lr=3e-4,
        network_factory=network_factory,
        seed=args.seed,
        num_evals=args.num_evals,
        num_eval_envs=min(args.num_eval_envs, args.num_envs),
        deterministic_eval=True,
        run_evals=args.run_evals,
        progress_fn=progress,
        save_checkpoint_path=str(checkpoint_dir),
        restore_checkpoint_path=str(restore_path) if restore_path else None,
        restore_params=restore_params,
        restore_value_fn=restore_path is not None and not args.reset_value_function,
        wrap_env_fn=functools.partial(
            wrapper.wrap_for_brax_training, full_reset=True
        ),
        use_pmap_on_reset=args.impl == "warp",
    )

    export_path = output_dir / "policy.onnx"
    export_actor(
        params[1],
        export_path,
        mean_clip_scale=args.mean_clip_scale,
    )
    summary = {
        "final_metrics": _scalar_metrics(metrics),
        "policy": str(export_path),
        "checkpoints": str(checkpoint_dir),
    }
    (output_dir / "result.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
