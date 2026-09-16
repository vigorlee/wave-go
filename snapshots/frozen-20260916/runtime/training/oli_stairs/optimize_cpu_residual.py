"""Optimize a low-dimensional constant stair residual with CPU MuJoCo CEM."""

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

from training.oli_stairs.constants import ACTION_SIZE, STAGE_HEIGHTS, StairSpec
from training.oli_stairs.cpu_residual_env import OliCpuResidualEnv
from training.oli_stairs.cpu_residual_policy import (
    NumpyResidualActor,
    export_constant_residual,
)


_ENV: OliCpuResidualEnv | None = None
_INDICES: np.ndarray | None = None
_SEGMENTS = 1
_MID_SEGMENT_SWITCH_X = np.inf
_SEGMENT_SWITCH_X = -np.inf


def _initialize_worker(
    model_path: Path,
    base_policy_path: Path,
    trace_path: Path,
    spec: StairSpec,
    reset_step: int,
    residual_start_x: float,
    episode_length: int,
    indices: tuple[int, ...],
    segments: int,
    mid_segment_switch_x: float,
    segment_switch_x: float,
) -> None:
    global _ENV, _INDICES, _SEGMENTS, _MID_SEGMENT_SWITCH_X, _SEGMENT_SWITCH_X
    _ENV = OliCpuResidualEnv(
        model_path,
        base_policy_path,
        spec,
        residual_scale=1.0,
        residual_start_x=residual_start_x,
        episode_length=episode_length,
        speed=0.30,
        lateral_speed=0.05,
        history_lateral_speed=0.08,
        history_reset_x=0.99,
        reset_trace_path=trace_path,
        reset_trace_start_step=reset_step,
        reset_trace_end_step=reset_step,
        reset_trace_probability=1.0,
    )
    _INDICES = np.asarray(indices, dtype=np.int32)
    _SEGMENTS = segments
    _MID_SEGMENT_SWITCH_X = mid_segment_switch_x
    _SEGMENT_SWITCH_X = segment_switch_x


def _evaluate(candidate: np.ndarray) -> dict:
    if _ENV is None or _INDICES is None:
        raise RuntimeError("CEM worker was not initialized")
    segment_actions = np.asarray(candidate, dtype=np.float32).reshape(
        _SEGMENTS, _INDICES.size
    )
    _ENV.reset(seed=0, options={"use_snapshot": True, "snapshot_index": 0})
    terminated = truncated = False
    reward_sum = 0.0
    info: dict = {}
    while not (terminated or truncated):
        top_supported = (
            bool(info)
            and info["max_support_height"] >= _ENV.spec.top_height - 1.0e-6
            and _ENV.data.qpos[0] >= _SEGMENT_SWITCH_X
        )
        if _SEGMENTS == 3 and top_supported:
            segment = 2
        elif _SEGMENTS == 3 and _ENV.data.qpos[0] >= _MID_SEGMENT_SWITCH_X:
            segment = 1
        elif _SEGMENTS == 2 and top_supported:
            segment = 1
        else:
            segment = 0
        action = np.zeros(ACTION_SIZE, dtype=np.float32)
        action[_INDICES] = segment_actions[segment]
        _, reward, terminated, truncated, info = _ENV.step(action)
        reward_sum += reward
    quaternion = _ENV.data.qpos[3:7]
    body_up_z = 1.0 - 2.0 * (quaternion[1] ** 2 + quaternion[2] ** 2)
    score = (
        reward_sum
        + 5.0 * _ENV.steps
        + 20_000.0 * float(_ENV.data.qpos[0])
        + 500_000.0 * float(info["max_support_height"])
        + 100.0 * min(float(info["longest_top_support"]), 50.0)
        + 2_000.0 * min(float(info["longest_success_hold"]), 20.0)
        + 100.0 * body_up_z
        - 1_000.0 * abs(float(_ENV.data.qpos[1]))
        - 10.0 * float(np.sum(np.square(candidate)))
    )
    if info["is_success"]:
        score += 1_000_000.0
    return {
        "score": float(score),
        "success": bool(info["is_success"]),
        "steps": _ENV.steps,
        "reward": float(reward_sum),
        "max_support_height": float(info["max_support_height"]),
        "longest_top_support": int(info["longest_top_support"]),
        "longest_success_hold": int(info["longest_success_hold"]),
        "final_x": float(_ENV.data.qpos[0]),
        "final_y": float(_ENV.data.qpos[1]),
        "final_z": float(_ENV.data.qpos[2]),
        "action": np.asarray(candidate, dtype=float).tolist(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--base-policy", type=Path, required=True)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--stage", choices=STAGE_HEIGHTS, default="h05")
    parser.add_argument("--step-count", type=int, default=5)
    parser.add_argument("--step-depth", type=float, default=0.20)
    parser.add_argument("--landing-length", type=float, default=2.00)
    parser.add_argument("--stair-half-width", type=float, default=1.50)
    parser.add_argument("--reset-step", type=int, default=500)
    parser.add_argument("--residual-start-x", type=float, default=1.0)
    parser.add_argument("--episode-length", type=int, default=400)
    parser.add_argument("--indices", type=int, nargs="+", default=list(range(15)))
    parser.add_argument("--segments", type=int, choices=(1, 2, 3), default=1)
    parser.add_argument("--freeze-first-segment", action="store_true")
    parser.add_argument("--mid-segment-switch-x", type=float, default=np.inf)
    parser.add_argument("--segment-switch-x", type=float, default=-np.inf)
    parser.add_argument("--population", type=int, default=64)
    parser.add_argument("--elite", type=int, default=8)
    parser.add_argument("--iterations", type=int, default=15)
    parser.add_argument("--initial-std", type=float, default=0.05)
    parser.add_argument("--minimum-std", type=float, default=0.002)
    parser.add_argument("--maximum-action", type=float, default=0.50)
    parser.add_argument("--smoothing", type=float, default=0.35)
    parser.add_argument("--workers", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-search", type=Path, action="append", default=[])
    parser.add_argument("--initial-mean-search", type=Path)
    parser.add_argument("--initial-mean-policy", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    if args.initial_mean_search is not None and args.initial_mean_policy is not None:
        parser.error("use only one of --initial-mean-search and --initial-mean-policy")

    if not 1 <= args.elite <= args.population:
        parser.error("--elite must be within the population")
    if args.step_count < 1:
        parser.error("--step-count must be positive")
    for name in ("step_depth", "landing_length", "stair_half_width"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if len(set(args.indices)) != len(args.indices):
        parser.error("--indices must be unique")
    if any(index < 0 or index >= ACTION_SIZE for index in args.indices):
        parser.error(f"--indices must be in [0, {ACTION_SIZE - 1}]")

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    spec = StairSpec(
        step_height=STAGE_HEIGHTS[args.stage],
        step_depth=args.step_depth,
        step_count=args.step_count,
        half_width=args.stair_half_width,
        landing_length=args.landing_length,
    )
    rng = np.random.default_rng(args.seed)
    parameter_count = len(args.indices) * args.segments
    mean = np.zeros(parameter_count, dtype=np.float64)
    std = np.full(parameter_count, args.initial_std, dtype=np.float64)
    if args.initial_mean_search is not None:
        initial_payload = json.loads(
            args.initial_mean_search.expanduser().resolve().read_text(encoding="utf-8")
        )
        mean = np.asarray(initial_payload["best"]["action"], dtype=np.float64)
        if mean.shape == (len(args.indices),) and args.segments > 1:
            mean = np.tile(mean, args.segments)
        elif mean.shape == (2 * len(args.indices),) and args.segments == 3:
            first, landing = np.split(mean, 2)
            mean = np.concatenate((first, first, landing))
        if mean.shape != (parameter_count,):
            parser.error("--initial-mean-search action shape does not match --indices")
    elif args.initial_mean_policy is not None:
        actor = NumpyResidualActor(args.initial_mean_policy)
        full_action = actor(np.zeros(actor.observation_size, dtype=np.float32))
        selected_action = full_action[np.asarray(args.indices)]
        mean = np.tile(selected_action, args.segments).astype(np.float64)
    best: dict | None = None
    history = []
    seeded_candidates = []
    for seed_path in args.seed_search:
        seed_payload = json.loads(
            seed_path.expanduser().resolve().read_text(encoding="utf-8")
        )
        seeded_candidates.extend(
            np.asarray(item["action"], dtype=np.float64)
            for item in seed_payload.get("history", [])
        )
    if args.segments > 1:
        seeded_candidates = [
            np.tile(candidate, args.segments)
            if candidate.shape == (len(args.indices),)
            else np.concatenate(
                (
                    candidate[: len(args.indices)],
                    candidate[: len(args.indices)],
                    candidate[-len(args.indices) :],
                )
            )
            if candidate.shape == (2 * len(args.indices),) and args.segments == 3
            else candidate
            for candidate in seeded_candidates
        ]

    context = multiprocessing.get_context("spawn")
    with context.Pool(
        processes=args.workers,
        initializer=_initialize_worker,
        initargs=(
            args.model.expanduser().resolve(),
            args.base_policy.expanduser().resolve(),
            args.trace.expanduser().resolve(),
            spec,
            args.reset_step,
            args.residual_start_x,
            args.episode_length,
            tuple(args.indices),
            args.segments,
            args.mid_segment_switch_x,
            args.segment_switch_x,
        ),
    ) as pool:
        for iteration in range(args.iterations):
            candidates = rng.normal(mean, std, size=(args.population, mean.size))
            candidates = np.clip(
                candidates,
                -args.maximum_action,
                args.maximum_action,
            )
            if args.freeze_first_segment:
                if args.segments < 2:
                    parser.error("--freeze-first-segment requires multiple segments")
                candidates[:, : len(args.indices)] = mean[: len(args.indices)]
            candidates[0] = mean
            candidates[1] = 0.0
            for index, candidate in enumerate(
                seeded_candidates[: max(0, args.population - 2)], start=2
            ):
                if candidate.shape == mean.shape:
                    candidates[index] = candidate
            results = pool.map(_evaluate, list(candidates))
            order = np.argsort([item["score"] for item in results])[::-1]
            elites = candidates[order[: args.elite]]
            elite_mean = elites.mean(axis=0)
            elite_std = elites.std(axis=0)
            mean = args.smoothing * mean + (1.0 - args.smoothing) * elite_mean
            std = np.maximum(
                args.minimum_std,
                args.smoothing * std + (1.0 - args.smoothing) * elite_std,
            )
            if args.freeze_first_segment:
                mean[: len(args.indices)] = candidates[0, : len(args.indices)]
                std[: len(args.indices)] = 0.0
            generation_best = results[int(order[0])]
            generation_best["iteration"] = iteration + 1
            history.append(generation_best)
            if best is None or generation_best["score"] > best["score"]:
                best = generation_best.copy()
                best_segments = np.asarray(best["action"]).reshape(
                    args.segments, len(args.indices)
                )
                for segment_index, segment_action in enumerate(best_segments):
                    full_action = np.zeros(ACTION_SIZE, dtype=np.float32)
                    full_action[np.asarray(args.indices)] = segment_action
                    suffix = (
                        "best_residual.npz"
                        if args.segments == 1
                        else f"best_residual_segment_{segment_index}.npz"
                    )
                    export_constant_residual(
                        full_action,
                        output_dir / suffix,
                        residual_scale=1.0,
                        residual_start_x=args.residual_start_x,
                    )
            (output_dir / "search.json").write_text(
                json.dumps(
                    {
                        "stair_spec": {
                            "step_height": spec.step_height,
                            "step_depth": spec.step_depth,
                            "step_count": spec.step_count,
                            "start_x": spec.start_x,
                            "half_width": spec.half_width,
                            "landing_length": spec.landing_length,
                        },
                        "indices": args.indices,
                        "mean": mean.tolist(),
                        "std": std.tolist(),
                        "best": best,
                        "history": history,
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            print(
                f"iteration={iteration + 1} score={generation_best['score']:.1f} "
                f"success={generation_best['success']} steps={generation_best['steps']} "
                f"support={generation_best['max_support_height']:.3f} "
                f"top_hold={generation_best['longest_top_support']} "
                f"qualified={generation_best['longest_success_hold']} "
                f"x={generation_best['final_x']:.3f} y={generation_best['final_y']:.3f} "
                f"std={std.mean():.4f}",
                flush=True,
            )
            if generation_best["success"]:
                break
            seeded_candidates = []

    print(output_dir)


if __name__ == "__main__":
    main()
