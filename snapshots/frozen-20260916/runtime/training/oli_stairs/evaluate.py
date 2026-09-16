"""Roll out an OLI stair checkpoint and optionally render a video."""

from __future__ import annotations

import os

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
from pathlib import Path

from brax.training.agents.ppo import checkpoint as ppo_checkpoint
from brax.training.agents.ppo import networks as ppo_networks
import jax
import jax.numpy as jp
import mediapy as media
import numpy as np

from training.oli_stairs.checkpointing import resolve_checkpoint
from training.oli_stairs.constants import (
    CONTROL_PROFILES,
    STAGE_HEIGHTS,
    stair_spec_for_stage,
)
from training.oli_stairs.environment import OliStairEnv, default_config
from training.oli_stairs.network import make_network_factory


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h15")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--zero-action", action="store_true")
    parser.add_argument(
        "--control-profile", choices=CONTROL_PROFILES, default="walk"
    )
    parser.add_argument(
        "--parallel-control-mode",
        choices=("motor_pd", "projected_torque"),
        default="motor_pd",
    )
    parser.add_argument("--steps", type=int, default=1800)
    parser.add_argument("--speed", type=float, default=0.32)
    parser.add_argument("--video", type=Path)
    parser.add_argument("--render-stride", type=int, default=3)
    parser.add_argument("--render-width", type=int, default=640)
    parser.add_argument("--render-height", type=int, default=360)
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
    parser.add_argument("--teacher-policy", type=Path)
    parser.add_argument("--teacher-blend", type=float, default=1.0)
    args = parser.parse_args()
    if args.checkpoint is None and not args.zero_action:
        parser.error("--checkpoint is required unless --zero-action is used")

    config = default_config()
    config.observation_noise = 0.0
    config.reset_joint_noise = 0.0
    config.reset_velocity_noise = 0.0
    config.reset_y_noise = 0.0
    config.reset_yaw_noise = 0.0
    config.command_speed = [args.speed, args.speed]
    config.command_lateral = [0.0, 0.0]
    config.command_yaw = [0.0, 0.0]
    config.control_profile = args.control_profile
    config.parallel_control_mode = args.parallel_control_mode
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

    reset = jax.jit(env.reset)
    step = jax.jit(env.step)
    policy = None
    if not args.zero_action:
        checkpoint = resolve_checkpoint(args.checkpoint)
        params = ppo_checkpoint.load(str(checkpoint))
        ppo_network = make_network_factory()(env.observation_size, env.action_size)
        policy = jax.jit(
            ppo_networks.make_inference_fn(ppo_network)(params, deterministic=True)
        )

    key = jax.random.PRNGKey(0)
    state = reset(key)
    start_qpos = np.asarray(state.data.qpos)
    trajectory = []
    for index in range(args.steps):
        key, action_key = jax.random.split(key)
        if policy is None:
            action = jp.zeros(env.action_size)
        else:
            action, _ = policy(state.obs, action_key)
        state = step(state, action)
        if args.video and index % args.render_stride == 0:
            trajectory.append(state)
        if bool(np.asarray(state.done)):
            break

    qpos = np.asarray(state.data.qpos)
    yaw = np.degrees(
        np.arctan2(
            2.0 * (qpos[3] * qpos[6] + qpos[4] * qpos[5]),
            1.0 - 2.0 * (qpos[5] ** 2 + qpos[6] ** 2),
        )
    )
    print(
        f"steps={index + 1} done={float(np.asarray(state.done)):.0f} "
        f"success={float(np.asarray(state.info['success'])):.0f} "
        f"x={qpos[0]:.3f} dx={qpos[0] - start_qpos[0]:.3f} "
        f"y={qpos[1]:.3f} z={qpos[2]:.3f} "
        f"yaw={yaw:.1f}deg"
    )
    if args.video:
        video = args.video.expanduser().resolve()
        video.parent.mkdir(parents=True, exist_ok=True)
        frames = env.render(
            trajectory,
            height=args.render_height,
            width=args.render_width,
            camera="track",
        )
        media.write_video(
            video,
            frames,
            fps=round(1.0 / env.dt / args.render_stride),
        )
        print(video)


if __name__ == "__main__":
    main()
