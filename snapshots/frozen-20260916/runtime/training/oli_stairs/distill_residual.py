"""Distill terrain-triggered CPU residual traces into an OLI ONNX actor."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")

import numpy as np
import onnx
from onnx import numpy_helper

from training.oli_stairs.constants import OFFICIAL_POLICY
from training.oli_stairs.evaluate_cpu import NumpyOnnxActor


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, action="append", required=True)
    parser.add_argument("--base-policy", type=Path, default=OFFICIAL_POLICY)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--active-weight", type=float, default=8.0)
    parser.add_argument("--ridge", type=float, default=10.0)
    args = parser.parse_args()

    actor = NumpyOnnxActor(args.base_policy)
    observations = []
    targets = []
    sample_weights = []
    active_mask = []
    for path in args.trace:
        records = json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))
        for record in records:
            active = any(value > 0.0 for value in record["stair_residual_lift"])
            observations.append(record["observation"])
            targets.append(record["action"])
            sample_weights.append(args.active_weight if active else 1.0)
            active_mask.append(active)

    observations_array = np.asarray(observations, dtype=np.float32)
    target_array = np.asarray(targets, dtype=np.float64)
    weights = np.asarray(sample_weights, dtype=np.float64)
    active = np.asarray(active_mask, dtype=bool)
    features = np.stack([actor.features(item) for item in observations_array])
    design = np.concatenate([features, np.ones((len(features), 1))], axis=1)

    model = onnx.load(str(args.base_policy.expanduser().resolve()))
    tensors = {
        tensor.name: numpy_helper.to_array(tensor) for tensor in model.graph.initializer
    }
    initial = np.concatenate(
        [tensors["actor.6.weight"].T, tensors["actor.6.bias"][None]],
        axis=0,
    ).astype(np.float64)
    weighted_design = design * weights[:, None]
    normal = design.T @ weighted_design + args.ridge * np.eye(design.shape[1])
    right_hand = design.T @ (target_array * weights[:, None]) + args.ridge * initial
    fitted = np.linalg.solve(normal, right_hand)

    baseline = design @ initial
    prediction = design @ fitted
    for name, mask in (("all", np.ones(len(active), dtype=bool)), ("active", active)):
        if not np.any(mask):
            continue
        baseline_mse = np.mean(np.square(baseline[mask] - target_array[mask]))
        fitted_mse = np.mean(np.square(prediction[mask] - target_array[mask]))
        print(f"{name}_samples={int(mask.sum())} baseline_mse={baseline_mse:.6g} fitted_mse={fitted_mse:.6g}")

    replacements = {
        "actor.6.weight": fitted[:-1].T.astype(np.float32),
        "actor.6.bias": fitted[-1].astype(np.float32),
    }
    for tensor in model.graph.initializer:
        if tensor.name in replacements:
            tensor.CopyFrom(numpy_helper.from_array(replacements[tensor.name], tensor.name))

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.checker.check_model(model)
    onnx.save(model, str(output))
    print(f"max_parameter_delta={np.max(np.abs(fitted - initial)):.6g}")
    print(output)


if __name__ == "__main__":
    main()
