"""Train a small residual policy in authoritative CPU MuJoCo."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Callable

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

from training.oli_stairs.constants import (
    CONTROL_PROFILES,
    OFFICIAL_POLICY,
    STAGE_HEIGHTS,
    StairSpec,
)
from training.oli_stairs.cpu_residual_env import OliCpuResidualEnv
from training.oli_stairs.cpu_residual_policy import (
    NumpyResidualActor,
    export_sb3_residual,
)
from training.oli_stairs.generate_mjcf import generate_full_training_model


class ExportCheckpointCallback(BaseCallback):
    def __init__(
        self,
        output_dir: Path,
        save_interval: int,
        residual_scale: float,
        residual_start_x: float,
    ) -> None:
        super().__init__()
        self.output_dir = output_dir
        self.save_interval = save_interval
        self.residual_scale = residual_scale
        self.residual_start_x = residual_start_x
        self.next_save = save_interval

    def _on_training_start(self) -> None:
        if self.num_timesteps > 0:
            self.next_save = (
                self.num_timesteps // self.save_interval + 1
            ) * self.save_interval

    def _save(self, step: int) -> None:
        stem = self.output_dir / f"step_{step:09d}"
        self.model.save(stem.with_suffix(".zip"))
        export_sb3_residual(
            self.model,
            stem.with_suffix(".npz"),
            self.residual_scale,
            self.residual_start_x,
        )

    def _on_step(self) -> bool:
        while self.num_timesteps >= self.next_save:
            self._save(self.num_timesteps)
            self.next_save += self.save_interval
        return True


def _environment_factory(
    rank: int,
    model_path: Path,
    policy_path: Path,
    spec: StairSpec,
    args: argparse.Namespace,
) -> Callable[[], gym.Env]:
    def create() -> gym.Env:
        env = OliCpuResidualEnv(
            model_path,
            policy_path,
            spec,
            residual_scale=args.residual_scale,
            episode_length=args.episode_length,
            speed=args.speed,
            lateral_speed=args.lateral_speed,
            history_lateral_speed=args.history_lateral_speed,
            history_reset_x=args.history_reset_x,
            residual_start_x=args.residual_start_x,
            fixed_residual_policy=args.fixed_residual_policy,
            reset_trace_path=tuple(args.reset_trace) if args.reset_trace else None,
            reset_trace_start_step=args.reset_trace_start_step,
            reset_trace_end_step=args.reset_trace_end_step,
            reset_trace_start_x=args.reset_trace_start_x,
            reset_trace_end_x=args.reset_trace_end_x,
            reset_trace_probability=args.reset_trace_probability,
            reset_x_noise=args.reset_x_noise,
            reset_y_noise=args.reset_y_noise,
            reset_yaw_noise=args.reset_yaw_noise,
            control_profile=args.control_profile,
            stand_gain_blend=args.stand_gain_blend,
            policy_action_scale=args.policy_action_scale,
            target_step=args.target_step,
            target_shaping_scale=args.target_shaping_scale,
        )
        env.reset(seed=args.seed + rank)
        return Monitor(env, info_keywords=("is_success", "max_support_height"))

    return create


def _initialize_residual(
    model: PPO,
    log_std: float,
    initial_policy: Path | None,
) -> None:
    with torch.no_grad():
        model.policy.action_net.weight.zero_()
        model.policy.action_net.bias.zero_()
        model.policy.log_std.fill_(log_std)
        if initial_policy is None:
            return
        residual = NumpyResidualActor(initial_policy)
        linear_layers = [
            module
            for module in model.policy.mlp_extractor.policy_net
            if isinstance(module, torch.nn.Linear)
        ] + [model.policy.action_net]
        if len(residual.layers) == len(linear_layers) and all(
            source_weight.shape == tuple(target.weight.shape)
            for (source_weight, _), target in zip(residual.layers, linear_layers)
        ):
            for (source_weight, source_bias), target in zip(
                residual.layers, linear_layers
            ):
                target.weight.copy_(torch.from_numpy(source_weight))
                target.bias.copy_(torch.from_numpy(source_bias))
        else:
            bias = residual.layers[-1][1]
            if bias.shape != tuple(model.policy.action_net.bias.shape):
                raise ValueError("Initial residual action size does not match PPO")
            model.policy.action_net.bias.copy_(torch.from_numpy(bias))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h05")
    parser.add_argument("--step-count", type=int, default=5)
    parser.add_argument("--target-step", type=int)
    parser.add_argument("--target-shaping-scale", type=float, default=0.0)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--riser-ramp-length", type=float, default=0.0)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--policy", type=Path, default=OFFICIAL_POLICY)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--total-timesteps", type=int, default=500_000)
    parser.add_argument("--save-interval", type=int, default=50_000)
    parser.add_argument("--num-envs", type=int, default=16)
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--n-epochs", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument("--target-kl", type=float, default=0.02)
    parser.add_argument("--residual-scale", type=float, default=0.25)
    parser.add_argument("--residual-start-x", type=float, default=0.95)
    parser.add_argument("--initial-log-std", type=float, default=-2.5)
    parser.add_argument("--episode-length", type=int, default=1000)
    parser.add_argument("--speed", type=float, default=0.30)
    parser.add_argument("--lateral-speed", type=float, default=0.05)
    parser.add_argument("--history-lateral-speed", type=float, default=0.08)
    parser.add_argument("--history-reset-x", type=float, default=0.99)
    parser.add_argument("--reset-x-noise", type=float, default=0.01)
    parser.add_argument("--reset-y-noise", type=float, default=0.01)
    parser.add_argument("--reset-yaw-noise", type=float, default=0.01)
    parser.add_argument("--reset-trace", type=Path, action="append", default=[])
    parser.add_argument("--reset-trace-start-step", type=int, default=540)
    parser.add_argument("--reset-trace-end-step", type=int, default=590)
    parser.add_argument("--reset-trace-start-x", type=float, default=-np.inf)
    parser.add_argument("--reset-trace-end-x", type=float, default=np.inf)
    parser.add_argument("--reset-trace-probability", type=float, default=0.0)
    parser.add_argument("--init-residual-policy", type=Path)
    parser.add_argument("--fixed-residual-policy", type=Path)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--torch-threads", type=int, default=4)
    parser.add_argument("--control-profile", choices=CONTROL_PROFILES, default="walk")
    parser.add_argument("--stand-gain-blend", type=float, default=0.0)
    parser.add_argument("--policy-action-scale", type=float, default=1.0)
    args = parser.parse_args()

    if args.step_count < 1:
        parser.error("--step-count must be positive")
    if args.target_step is not None and not 1 <= args.target_step <= args.step_count:
        parser.error("--target-step must be within the staircase")
    if args.num_envs < 1:
        parser.error("--num-envs must be positive")
    if args.batch_size > args.num_envs * args.n_steps:
        parser.error("--batch-size cannot exceed num-envs * n-steps")
    if args.total_timesteps < 1 or args.save_interval < 1:
        parser.error("training and checkpoint intervals must be positive")
    if not 0.0 <= args.reset_trace_probability <= 1.0:
        parser.error("--reset-trace-probability must be in [0, 1]")
    for name in ("step_depth", "landing_length", "stair_half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.riser_ramp_length < 0.0:
        parser.error("--riser-ramp-length cannot be negative")
    if not 0.0 <= args.stand_gain_blend <= 1.0:
        parser.error("--stand-gain-blend must be in [0, 1]")

    torch.set_num_threads(args.torch_threads)
    torch.set_num_interop_threads(1)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    run_dir = args.run_dir.expanduser().resolve()
    checkpoints = run_dir / "checkpoints"
    checkpoints.mkdir(parents=True, exist_ok=True)
    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        half_width=args.stair_half_width,
        landing_length=args.landing_length,
        riser_ramp_length=args.riser_ramp_length,
    )
    if args.model is not None:
        model_path = args.model.expanduser().resolve()
    else:
        depth_tag = round(args.step_depth * 100)
        ramp_tag = round(args.riser_ramp_length * 100)
        model_path = (
            Path("/home/unitree/oli_stairs_training/models")
            / f"oli_full_{args.stage}_{args.step_count}_d{depth_tag}_r{ramp_tag}.xml"
        )
        generate_full_training_model(model_path, spec)

    config = vars(args).copy()
    config.update(
        {
            "run_dir": str(run_dir),
            "policy": str(args.policy.expanduser().resolve()),
            "resume": str(args.resume.expanduser().resolve()) if args.resume else None,
            "reset_trace": [
                str(path.expanduser().resolve()) for path in args.reset_trace
            ],
            "init_residual_policy": (
                str(args.init_residual_policy.expanduser().resolve())
                if args.init_residual_policy else None
            ),
            "fixed_residual_policy": (
                str(args.fixed_residual_policy.expanduser().resolve())
                if args.fixed_residual_policy else None
            ),
            "model": str(model_path.resolve()),
        }
    )
    (run_dir / "run_config.json").write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    vector_env = SubprocVecEnv(
        [
            _environment_factory(
                rank,
                model_path,
                args.policy.expanduser().resolve(),
                spec,
                args,
            )
            for rank in range(args.num_envs)
        ],
        start_method="spawn",
    )
    try:
        if args.resume is not None:
            model = PPO.load(
                args.resume.expanduser().resolve(),
                env=vector_env,
                device="cpu",
            )
        else:
            model = PPO(
                "MlpPolicy",
                vector_env,
                learning_rate=args.learning_rate,
                n_steps=args.n_steps,
                batch_size=args.batch_size,
                n_epochs=args.n_epochs,
                gamma=0.995,
                gae_lambda=0.95,
                clip_range=0.15,
                normalize_advantage=True,
                ent_coef=0.0,
                vf_coef=0.5,
                max_grad_norm=0.5,
                target_kl=args.target_kl,
                policy_kwargs={
                    "activation_fn": torch.nn.Tanh,
                    "net_arch": {"pi": [128, 128], "vf": [256, 128]},
                    "log_std_init": args.initial_log_std,
                    "ortho_init": True,
                },
                verbose=1,
                seed=args.seed,
                device="cpu",
            )
            _initialize_residual(
                model,
                args.initial_log_std,
                args.init_residual_policy,
            )

        callback = ExportCheckpointCallback(
            checkpoints,
            args.save_interval,
            args.residual_scale,
            args.residual_start_x,
        )
        model.learn(
            total_timesteps=args.total_timesteps,
            callback=callback,
            reset_num_timesteps=args.resume is None,
            progress_bar=False,
        )
        model.save(run_dir / "final.zip")
        export_sb3_residual(
            model,
            run_dir / "residual_policy.npz",
            args.residual_scale,
            args.residual_start_x,
        )
    finally:
        vector_env.close()

    print(run_dir / "residual_policy.npz")


if __name__ == "__main__":
    main()
