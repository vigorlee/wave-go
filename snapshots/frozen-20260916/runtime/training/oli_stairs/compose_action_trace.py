"""Compose exact action replay traces from rollout and CEM residual segments."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from training.oli_stairs.constants import OFFICIAL_POLICY
from training.oli_stairs.evaluate_cpu import NumpyOnnxActor


def _load(path: Path) -> list[dict]:
    return json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix-trace", type=Path, required=True)
    parser.add_argument("--prefix-end-step", type=int, required=True)
    parser.add_argument("--segment-trace", type=Path, action="append", default=[])
    parser.add_argument("--segment-end-step", type=int, action="append", default=[])
    parser.add_argument("--base-policy", type=Path, default=OFFICIAL_POLICY)
    parser.add_argument("--residual-scale", type=float, default=4.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.prefix_end_step < 0:
        parser.error("--prefix-end-step must be non-negative")
    if len(args.segment_trace) != len(args.segment_end_step):
        parser.error("each --segment-trace requires one --segment-end-step")
    if any(step < 1 for step in args.segment_end_step):
        parser.error("--segment-end-step values must be positive")

    actor = NumpyOnnxActor(args.base_policy)
    actions: list[dict] = []
    prefix = _load(args.prefix_trace)
    for record in prefix:
        if int(record["step"]) > args.prefix_end_step:
            break
        actions.append(
            {
                "action": np.asarray(record["action"], dtype=float).tolist(),
                "source": str(args.prefix_trace.expanduser().resolve()),
                "source_step": int(record["step"]),
            }
        )

    for path, end_step in zip(args.segment_trace, args.segment_end_step):
        for record in _load(path):
            if int(record["step"]) >= end_step:
                break
            observation = np.asarray(record["observation"], dtype=np.float32)
            residual = np.asarray(record["residual_action"], dtype=np.float64)
            action = actor(observation) + args.residual_scale * residual
            actions.append(
                {
                    "action": action.tolist(),
                    "source": str(path.expanduser().resolve()),
                    "source_step": int(record["step"]),
                }
            )

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(actions, separators=(",", ":")) + "\n", encoding="utf-8")
    print(f"actions={len(actions)}")
    print(output)


if __name__ == "__main__":
    main()
