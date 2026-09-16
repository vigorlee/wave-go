"""Create a kinematically consistent double-support OLI stair snapshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import jax.numpy as jp
import mujoco
import numpy as np

from training.oli_stairs.constants import JOINT_NAMES, STAGE_HEIGHTS, StairSpec
from training.oli_stairs.cpu_residual_env import OliCpuResidualEnv
from training.oli_stairs.parallel_kinematics import (
    left_ankle_pr_to_ab,
    right_ankle_pr_to_ab,
    waist_pr_to_ab,
)


def _solve_foot(
    env: OliCpuResidualEnv,
    leg_index: int,
    target: np.ndarray,
) -> None:
    offset = leg_index * 6
    qpos_ids = env.qpos_ids[offset : offset + 6]
    dof_ids = env.qvel_ids[offset : offset + 6]
    joint_ids = np.asarray(
        [env.model.joint(name).id for name in JOINT_NAMES[offset : offset + 6]],
        dtype=np.int32,
    )
    lower = env.model.jnt_range[joint_ids, 0]
    upper = env.model.jnt_range[joint_ids, 1]
    site_id = int(env.foot_site_ids[leg_index])
    for _ in range(160):
        jacobian = np.zeros((3, env.model.nv), dtype=np.float64)
        mujoco.mj_jacSite(env.model, env.data, jacobian, None, site_id)
        error = target - env.data.site_xpos[site_id]
        if np.linalg.norm(error) < 1.0e-5:
            return
        reduced = jacobian[:, dof_ids]
        delta = reduced.T @ np.linalg.solve(
            reduced @ reduced.T + 2.0e-4 * np.eye(3),
            error,
        )
        env.data.qpos[qpos_ids] = np.clip(
            env.data.qpos[qpos_ids] + 0.5 * np.clip(delta, -0.06, 0.06),
            lower,
            upper,
        )
        mujoco.mj_forward(env.model, env.data)
    raise RuntimeError(f"IK did not converge for leg {leg_index}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h15")
    parser.add_argument("--step-count", type=int, default=10)
    parser.add_argument("--step-depth", type=float, default=0.28)
    parser.add_argument("--riser-ramp-length", type=float, default=0.26)
    parser.add_argument("--target-step", type=int, default=1)
    parser.add_argument("--base-x", type=float, default=0.62)
    parser.add_argument("--base-z", type=float, default=0.94)
    parser.add_argument("--stance-half-width", type=float, default=0.13)
    parser.add_argument("--control-profile", choices=("walk", "stand"), default="stand")
    args = parser.parse_args()

    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        half_width=1.50,
        landing_length=3.20,
        riser_ramp_length=args.riser_ramp_length,
    )
    env = OliCpuResidualEnv(
        args.model,
        args.policy,
        spec,
        episode_length=1,
        speed=0.0,
        control_profile=args.control_profile,
    )
    env.data.qpos[:] = env.init_qpos
    env.data.qpos[:3] = [args.base_x, 0.0, args.base_z]
    env.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    env.data.qvel[:] = 0.0
    mujoco.mj_forward(env.model, env.data)

    riser_x = spec.start_x + (args.target_step - 1) * spec.step_depth
    exposed_depth = max(spec.step_depth - spec.riser_ramp_length, 0.01)
    foot_x = riser_x + 0.5 * exposed_depth
    foot_z = args.target_step * spec.step_height - 0.012
    for _ in range(3):
        _solve_foot(
            env,
            0,
            np.asarray([foot_x, args.stance_half_width, foot_z]),
        )
        _solve_foot(
            env,
            1,
            np.asarray([foot_x, -args.stance_half_width, foot_z]),
        )

    for indices, mapping in (
        ((4, 5), left_ankle_pr_to_ab),
        ((10, 11), right_ankle_pr_to_ab),
        ((13, 14), waist_pr_to_ab),
    ):
        pair = np.asarray(indices)
        env.data.qpos[env.physical_qpos_ids[pair]] = np.asarray(
            mapping(jp.asarray(env.data.qpos[env.qpos_ids[pair]]))
        )
    env.data.qpos[:] = env._close_parallel_constraints(env.data.qpos.copy())
    env.data.qvel[:] = 0.0
    env.data.qacc_warmstart[:] = 0.0
    env.data.time = 0.0
    mujoco.mj_forward(env.model, env.data)

    joint_position = env.data.qpos[env.qpos_ids]
    action = (
        joint_position - env.control.default_angle
    ) / env.control.action_scale
    env.command[:] = 0.0
    observation = np.tile(env._frame(action), 5)
    record = {
        "step": 0,
        "time": 0.0,
        "base": env.data.qpos[:3].tolist(),
        "quaternion": env.data.qpos[3:7].tolist(),
        "feet": env.data.site_xpos[env.foot_site_ids].tolist(),
        "state_qpos": env.data.qpos.tolist(),
        "state_qvel": env.data.qvel.tolist(),
        "state_qacc_warmstart": env.data.qacc_warmstart.tolist(),
        "state_time": 0.0,
        "observation": observation.tolist(),
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps([record], separators=(",", ":")) + "\n")
    print(output)
    print(f"base={record['base']} feet={record['feet']}")


if __name__ == "__main__":
    main()
