"""Rank CPU residual checkpoints with deterministic full-model rollouts."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
import json
import multiprocessing
from pathlib import Path

import numpy as np

from training.oli_stairs.constants import ACTION_SIZE, STAGE_HEIGHTS, StairSpec
from training.oli_stairs.cpu_residual_env import OliCpuResidualEnv
from training.oli_stairs.cpu_residual_policy import NumpyResidualActor


@dataclass(frozen=True)
class RolloutResult:
    policy: str
    landing_policy: str | None
    initial_x: float
    initial_y: float
    initial_yaw: float
    success: bool
    steps: int
    max_support_height: float
    final_x: float
    final_y: float
    final_z: float


def _rollout(task: tuple) -> RolloutResult:
    (
        model_path,
        base_policy_path,
        residual_policy_path,
        landing_residual_policy_path,
        landing_residual_start_x,
        fixed_residual_policy_path,
        spec,
        episode_length,
        speed,
        lateral_speed,
        history_lateral_speed,
        history_reset_x,
        lateral_hold_kp,
        lateral_hold_limit,
        heading_hold_kp,
        heading_hold_limit,
        heading_hold_start_x,
        initial,
    ) = task
    residual = NumpyResidualActor(residual_policy_path)
    landing_residual = (
        NumpyResidualActor(landing_residual_policy_path)
        if landing_residual_policy_path is not None
        else None
    )
    env = OliCpuResidualEnv(
        model_path,
        base_policy_path,
        spec,
        residual_scale=residual.residual_scale,
        episode_length=episode_length,
        speed=speed,
        lateral_speed=lateral_speed,
        history_lateral_speed=history_lateral_speed,
        history_reset_x=history_reset_x,
        lateral_hold_kp=lateral_hold_kp,
        lateral_hold_limit=lateral_hold_limit,
        heading_hold_kp=heading_hold_kp,
        heading_hold_limit=heading_hold_limit,
        heading_hold_start_x=heading_hold_start_x,
        residual_start_x=residual.residual_start_x,
        fixed_residual_policy=fixed_residual_policy_path,
    )
    x_offset, y_offset, yaw_offset = initial
    env.reset(
        seed=0,
        options={
            "x_offset": x_offset,
            "y_offset": y_offset,
            "yaw_offset": yaw_offset,
        },
    )
    terminated = truncated = False
    info: dict = {}
    while not (terminated or truncated):
        active_residual = (
            landing_residual
            if landing_residual is not None
            and env.max_support_height >= spec.top_height - 1.0e-6
            and env.data.qpos[0] >= landing_residual_start_x
            else residual
        )
        action = active_residual(env.observation)
        _, _, terminated, truncated, info = env.step(action)
    return RolloutResult(
        policy=str(residual_policy_path),
        landing_policy=(
            str(landing_residual_policy_path)
            if landing_residual_policy_path is not None
            else None
        ),
        initial_x=x_offset,
        initial_y=y_offset,
        initial_yaw=yaw_offset,
        success=bool(info["is_success"]),
        steps=env.steps,
        max_support_height=float(info["max_support_height"]),
        final_x=float(env.data.qpos[0]),
        final_y=float(env.data.qpos[1]),
        final_z=float(env.data.qpos[2]),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policies", nargs="+", type=Path)
    parser.add_argument("--landing-policies", nargs="+", type=Path, default=[])
    parser.add_argument("--landing-residual-start-x", type=float, default=-np.inf)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--base-policy", type=Path, required=True)
    parser.add_argument("--fixed-residual-policy", type=Path)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, required=True)
    parser.add_argument("--step-count", type=int, required=True)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--episode-length", type=int, default=1200)
    parser.add_argument("--speed", type=float, default=0.30)
    parser.add_argument("--lateral-speed", type=float, default=0.05)
    parser.add_argument("--history-lateral-speed", type=float, default=0.08)
    parser.add_argument("--history-reset-x", type=float, default=0.99)
    parser.add_argument("--lateral-hold-kp", type=float, default=0.0)
    parser.add_argument("--lateral-hold-limit", type=float, default=0.05)
    parser.add_argument("--heading-hold-kp", type=float, default=0.0)
    parser.add_argument("--heading-hold-limit", type=float, default=0.15)
    parser.add_argument("--heading-hold-start-x", type=float, default=-np.inf)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nominal-only", action="store_true")
    parser.add_argument("--require-all-success", action="store_true")
    args = parser.parse_args()

    if args.step_count < 1:
        parser.error("--step-count must be positive")
    for name in ("step_depth", "landing_length", "stair_half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")

    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        half_width=args.stair_half_width,
        landing_length=args.landing_length,
    )
    initial_states = [(0.0, 0.0, 0.0)]
    if not args.nominal_only:
        initial_states.extend(
            [
                (-0.01, 0.0, 0.0),
                (0.01, 0.0, 0.0),
                (0.0, -0.015, 0.0),
                (0.0, 0.015, 0.0),
                (0.0, 0.0, -0.015),
                (0.0, 0.0, 0.015),
            ]
        )
    policy_pairs = (
        [(policy, landing) for policy in args.policies for landing in args.landing_policies]
        if args.landing_policies
        else [(policy, None) for policy in args.policies]
    )
    tasks = [
        (
            args.model.expanduser().resolve(),
            args.base_policy.expanduser().resolve(),
            policy.expanduser().resolve(),
            landing.expanduser().resolve() if landing is not None else None,
            args.landing_residual_start_x,
            (
                args.fixed_residual_policy.expanduser().resolve()
                if args.fixed_residual_policy else None
            ),
            spec,
            args.episode_length,
            args.speed,
            args.lateral_speed,
            args.history_lateral_speed,
            args.history_reset_x,
            args.lateral_hold_kp,
            args.lateral_hold_limit,
            args.heading_hold_kp,
            args.heading_hold_limit,
            args.heading_hold_start_x,
            initial,
        )
        for policy, landing in policy_pairs
        for initial in initial_states
    ]
    results: list[RolloutResult] = []
    with ProcessPoolExecutor(
        max_workers=args.workers,
        mp_context=multiprocessing.get_context("spawn"),
    ) as executor:
        futures = [executor.submit(_rollout, task) for task in tasks]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(
                f"success={result.success} support={result.max_support_height:.3f} "
                f"steps={result.steps} policy={result.policy} "
                f"landing={result.landing_policy} "
                f"init=({result.initial_x:+.3f},{result.initial_y:+.3f},"
                f"{result.initial_yaw:+.3f})"
            )

    results.sort(
        key=lambda item: (
            item.policy,
            item.landing_policy or "",
            item.initial_x,
            item.initial_y,
            item.initial_yaw,
        )
    )
    summaries = []
    controller_keys = sorted(
        {(item.policy, item.landing_policy) for item in results},
        key=lambda item: (item[0], item[1] or ""),
    )
    for policy, landing_policy in controller_keys:
        selected = [
            item
            for item in results
            if item.policy == policy and item.landing_policy == landing_policy
        ]
        summaries.append(
            {
                "policy": policy,
                "landing_policy": landing_policy,
                "successes": sum(item.success for item in selected),
                "rollouts": len(selected),
                "minimum_support_height": min(
                    item.max_support_height for item in selected
                ),
                "mean_support_height": float(
                    np.mean([item.max_support_height for item in selected])
                ),
            }
        )
    payload = {
        "stage": args.stage,
        "step_count": args.step_count,
        "step_depth": args.step_depth,
        "landing_length": args.landing_length,
        "stair_half_width": args.stair_half_width,
        "lateral_hold_kp": args.lateral_hold_kp,
        "lateral_hold_limit": args.lateral_hold_limit,
        "heading_hold_kp": args.heading_hold_kp,
        "heading_hold_limit": args.heading_hold_limit,
        "heading_hold_start_x": args.heading_hold_start_x,
        "summaries": summaries,
        "rollouts": [asdict(item) for item in results],
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(output)

    if args.require_all_success and any(
        summary["successes"] != summary["rollouts"] for summary in summaries
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
