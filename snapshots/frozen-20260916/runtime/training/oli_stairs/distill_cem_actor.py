"""Distill CPU CEM residual traces into a deployment-compatible OLI actor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import numpy_helper
import torch
from torch import nn

from training.oli_stairs.constants import OFFICIAL_POLICY
from training.oli_stairs.evaluate_cpu import NumpyOnnxActor


LAYER_INDICES = (0, 2, 4, 6)


class Actor(nn.Module):
    def __init__(self, tensors: dict[str, np.ndarray]) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        for position, index in enumerate(LAYER_INDICES):
            weight = tensors[f"actor.{index}.weight"]
            bias = tensors[f"actor.{index}.bias"]
            layer = nn.Linear(weight.shape[1], weight.shape[0])
            layer.weight.data.copy_(torch.from_numpy(weight.copy()))
            layer.bias.data.copy_(torch.from_numpy(bias.copy()))
            layers.append(layer)
            if position != len(LAYER_INDICES) - 1:
                layers.append(nn.ELU())
        self.network = nn.Sequential(*layers)

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return self.network(observation)


def _records(paths: list[Path]) -> list[dict]:
    records: list[dict] = []
    for path in paths:
        records.extend(
            json.loads(path.expanduser().resolve().read_text(encoding="utf-8"))
        )
    return records


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, action="append", required=True)
    parser.add_argument("--anchor-trace", type=Path, action="append", default=[])
    parser.add_argument("--base-policy", type=Path, default=OFFICIAL_POLICY)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--residual-scale", type=float, default=4.0)
    parser.add_argument("--trace-weight", type=float, default=8.0)
    parser.add_argument("--anchor-weight", type=float, default=2.0)
    parser.add_argument("--parameter-l2", type=float, default=1.0e-4)
    parser.add_argument("--noise-std", type=float, default=0.002)
    parser.add_argument("--epochs", type=int, default=5000)
    parser.add_argument("--learning-rate", type=float, default=1.0e-4)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.epochs < 1:
        parser.error("--epochs must be positive")
    if args.learning_rate <= 0.0:
        parser.error("--learning-rate must be positive")
    if args.threads < 1:
        parser.error("--threads must be positive")

    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    source = args.base_policy.expanduser().resolve()
    model = onnx.load(str(source))
    tensors = {
        item.name: numpy_helper.to_array(item).astype(np.float32, copy=True)
        for item in model.graph.initializer
    }
    base_actor = NumpyOnnxActor(source)

    observations: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    weights: list[float] = []
    for record in _records(args.trace):
        observation = np.asarray(record["observation"], dtype=np.float32)
        residual = np.asarray(record["residual_action"], dtype=np.float32)
        target = base_actor(observation) + args.residual_scale * residual
        observations.append(observation)
        targets.append(target.astype(np.float32))
        weights.append(args.trace_weight)

    for record in _records(args.anchor_trace):
        observation = np.asarray(record["observation"], dtype=np.float32)
        observations.append(observation)
        targets.append(base_actor(observation).astype(np.float32))
        weights.append(args.anchor_weight)

    observation_tensor = torch.from_numpy(np.stack(observations))
    target_tensor = torch.from_numpy(np.stack(targets))
    weight_tensor = torch.from_numpy(np.asarray(weights, dtype=np.float32))
    actor = Actor(tensors)
    initial_parameters = [
        parameter.detach().clone() for parameter in actor.parameters()
    ]
    optimizer = torch.optim.AdamW(
        actor.parameters(), lr=args.learning_rate, weight_decay=0.0
    )

    with torch.no_grad():
        baseline_mse = torch.mean(
            torch.square(actor(observation_tensor) - target_tensor)
        ).item()
    for epoch in range(args.epochs):
        noisy_observation = observation_tensor
        if args.noise_std > 0.0:
            noisy_observation = noisy_observation + torch.randn_like(
                noisy_observation
            ) * args.noise_std
        prediction = actor(noisy_observation)
        sample_loss = torch.mean(torch.square(prediction - target_tensor), dim=1)
        imitation_loss = torch.sum(sample_loss * weight_tensor) / torch.sum(
            weight_tensor
        )
        parameter_loss = sum(
            torch.mean(torch.square(parameter - initial))
            for parameter, initial in zip(actor.parameters(), initial_parameters)
        )
        loss = imitation_loss + args.parameter_l2 * parameter_loss
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
        optimizer.step()
        if epoch == 0 or (epoch + 1) % 500 == 0:
            print(
                f"epoch={epoch + 1} imitation={imitation_loss.item():.6g} "
                f"parameter={parameter_loss.item():.6g}",
                flush=True,
            )

    with torch.no_grad():
        fitted_mse = torch.mean(
            torch.square(actor(observation_tensor) - target_tensor)
        ).item()

    trained_layers = [module for module in actor.network if isinstance(module, nn.Linear)]
    replacements: dict[str, np.ndarray] = {}
    for index, layer in zip(LAYER_INDICES, trained_layers):
        replacements[f"actor.{index}.weight"] = (
            layer.weight.detach().numpy().astype(np.float32, copy=True)
        )
        replacements[f"actor.{index}.bias"] = (
            layer.bias.detach().numpy().astype(np.float32, copy=True)
        )
    for initializer in model.graph.initializer:
        replacement = replacements.get(initializer.name)
        if replacement is not None:
            initializer.CopyFrom(
                numpy_helper.from_array(replacement, initializer.name)
            )

    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    onnx.checker.check_model(model)
    onnx.save(model, str(output))
    print(
        f"samples={len(observations)} baseline_mse={baseline_mse:.6g} "
        f"fitted_mse={fitted_mse:.6g}"
    )
    print(output)


if __name__ == "__main__":
    main()
