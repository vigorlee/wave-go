"""Portable NumPy representation of an SB3 OLI residual actor."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


class NumpyResidualActor:
    """Evaluate a deterministic tanh MLP exported by the CPU PPO trainer."""

    def __init__(self, path: Path) -> None:
        source = path.expanduser().resolve()
        with np.load(source, allow_pickle=False) as archive:
            version = int(np.asarray(archive["format_version"]).reshape(()))
            if version != 1:
                raise ValueError(f"Unsupported residual policy version {version}: {source}")
            layer_count = int(np.asarray(archive["layer_count"]).reshape(()))
            self.layers = tuple(
                (
                    np.asarray(archive[f"weight_{index}"], dtype=np.float32),
                    np.asarray(archive[f"bias_{index}"], dtype=np.float32),
                )
                for index in range(layer_count)
            )
            self.residual_scale = float(
                np.asarray(archive["residual_scale"]).reshape(())
            )
            self.residual_start_x = float(
                np.asarray(archive["residual_start_x"]).reshape(())
            ) if "residual_start_x" in archive.files else -np.inf
            self.observation_size = int(
                np.asarray(archive["observation_size"]).reshape(())
            )
            self.action_size = int(np.asarray(archive["action_size"]).reshape(()))

        if not self.layers:
            raise ValueError(f"Residual policy has no layers: {source}")
        if self.layers[0][0].shape[1] != self.observation_size:
            raise ValueError(f"Residual policy input metadata mismatch: {source}")
        if self.layers[-1][0].shape[0] != self.action_size:
            raise ValueError(f"Residual policy output metadata mismatch: {source}")

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float32)[None]
        for weight, bias in self.layers[:-1]:
            value = np.tanh(value @ weight.T + bias)
        weight, bias = self.layers[-1]
        value = value @ weight.T + bias
        return np.clip(value[0], -1.0, 1.0).astype(np.float64)


def export_sb3_residual(
    model: Any,
    output_path: Path,
    residual_scale: float,
    residual_start_x: float,
) -> Path:
    """Export the deterministic policy mean from an SB3 PPO model."""
    import torch

    policy_layers = [
        module
        for module in model.policy.mlp_extractor.policy_net
        if isinstance(module, torch.nn.Linear)
    ]
    layers = [*policy_layers, model.policy.action_net]
    arrays: dict[str, np.ndarray] = {
        "format_version": np.asarray(1, dtype=np.int32),
        "layer_count": np.asarray(len(layers), dtype=np.int32),
        "residual_scale": np.asarray(residual_scale, dtype=np.float64),
        "residual_start_x": np.asarray(residual_start_x, dtype=np.float64),
        "observation_size": np.asarray(
            model.observation_space.shape[0], dtype=np.int32
        ),
        "action_size": np.asarray(model.action_space.shape[0], dtype=np.int32),
    }
    for index, layer in enumerate(layers):
        arrays[f"weight_{index}"] = (
            layer.weight.detach().cpu().numpy().astype(np.float32, copy=True)
        )
        arrays[f"bias_{index}"] = (
            layer.bias.detach().cpu().numpy().astype(np.float32, copy=True)
        )

    output = output_path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **arrays)

    residual = NumpyResidualActor(output)
    probe = np.zeros(residual.observation_size, dtype=np.float32)
    with torch.inference_mode():
        tensor = torch.from_numpy(probe[None]).to(model.policy.device)
        torch_action = model.policy._predict(tensor, deterministic=True)[0]
    np.testing.assert_allclose(
        residual(probe),
        torch_action.detach().cpu().numpy(),
        rtol=2.0e-5,
        atol=2.0e-5,
    )
    return output


def export_constant_residual(
    action: np.ndarray,
    output_path: Path,
    residual_scale: float,
    residual_start_x: float,
    observation_size: int = 510,
) -> Path:
    """Write a constant residual using the same portable policy format."""
    value = np.asarray(action, dtype=np.float32)
    output = output_path.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        format_version=np.asarray(1, dtype=np.int32),
        layer_count=np.asarray(1, dtype=np.int32),
        residual_scale=np.asarray(residual_scale, dtype=np.float64),
        residual_start_x=np.asarray(residual_start_x, dtype=np.float64),
        observation_size=np.asarray(observation_size, dtype=np.int32),
        action_size=np.asarray(value.size, dtype=np.int32),
        weight_0=np.zeros((value.size, observation_size), dtype=np.float32),
        bias_0=value,
    )
    NumpyResidualActor(output)
    return output
