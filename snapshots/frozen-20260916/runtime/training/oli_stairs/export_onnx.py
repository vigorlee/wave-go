"""Export a trained Brax OLI actor as a deployment-compatible ONNX model."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from brax.training.agents.ppo import checkpoint as ppo_checkpoint
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
from onnx.reference import ReferenceEvaluator

from training.oli_stairs.checkpointing import resolve_checkpoint
from training.oli_stairs.constants import ACTION_SIZE, OBSERVATION_SIZE
from training.oli_stairs.network import MEAN_CLIP_SCALE


def _actor_arrays(actor_params: Any) -> list[tuple[np.ndarray, np.ndarray]]:
    params = actor_params["params"]
    hidden = params["MLP_0"]
    return [
        (
            np.asarray(hidden["hidden_0"]["kernel"], dtype=np.float32),
            np.asarray(hidden["hidden_0"]["bias"], dtype=np.float32),
        ),
        (
            np.asarray(hidden["hidden_1"]["kernel"], dtype=np.float32),
            np.asarray(hidden["hidden_1"]["bias"], dtype=np.float32),
        ),
        (
            np.asarray(hidden["hidden_2"]["kernel"], dtype=np.float32),
            np.asarray(hidden["hidden_2"]["bias"], dtype=np.float32),
        ),
        (
            np.asarray(params["Dense_0"]["kernel"], dtype=np.float32),
            np.asarray(params["Dense_0"]["bias"], dtype=np.float32),
        ),
    ]


def evaluate_numpy(
    actor_params: Any,
    observation: np.ndarray,
    mean_clip_scale: float | None = MEAN_CLIP_SCALE,
) -> np.ndarray:
    output = np.asarray(observation, dtype=np.float32)
    for index, (kernel, bias) in enumerate(_actor_arrays(actor_params)):
        output = output @ kernel + bias
        if index < 3:
            output = np.where(output > 0.0, output, np.expm1(output))
    if mean_clip_scale is not None:
        output = mean_clip_scale * output / (1.0 + np.abs(output))
    return output.astype(np.float32)


def export_actor(
    actor_params: Any,
    output_path: Path,
    mean_clip_scale: float | None = MEAN_CLIP_SCALE,
) -> Path:
    """Write a 510 -> 512 -> 256 -> 128 -> 31 ELU ONNX actor."""
    layers = _actor_arrays(actor_params)
    nodes = []
    initializers = []
    input_name = "obs"
    for index, (kernel, bias) in enumerate(layers):
        weight_name = f"actor.{index * 2}.weight"
        bias_name = f"actor.{index * 2}.bias"
        if index == len(layers) - 1:
            gemm_output = "actions" if mean_clip_scale is None else "raw_actions"
        else:
            gemm_output = f"linear_{index}"
        initializers.extend(
            [
                numpy_helper.from_array(kernel.T, weight_name),
                numpy_helper.from_array(bias, bias_name),
            ]
        )
        nodes.append(
            helper.make_node(
                "Gemm",
                [input_name, weight_name, bias_name],
                [gemm_output],
                name=f"actor_gemm_{index}",
                transB=1,
            )
        )
        if index < len(layers) - 1:
            elu_output = f"elu_{index}"
            nodes.append(
                helper.make_node(
                    "Elu",
                    [gemm_output],
                    [elu_output],
                    name=f"actor_elu_{index}",
                    alpha=1.0,
                )
            )
            input_name = elu_output

    if mean_clip_scale is not None:
        initializers.extend(
            [
                numpy_helper.from_array(
                    np.asarray(1.0, dtype=np.float32), "clip_one"
                ),
                numpy_helper.from_array(
                    np.asarray(mean_clip_scale, dtype=np.float32), "clip_scale"
                ),
            ]
        )
        nodes.extend(
            [
                helper.make_node(
                    "Abs", ["raw_actions"], ["raw_actions_abs"], name="action_abs"
                ),
                helper.make_node(
                    "Add",
                    ["raw_actions_abs", "clip_one"],
                    ["action_denominator"],
                    name="action_add_one",
                ),
                helper.make_node(
                    "Div",
                    ["raw_actions", "action_denominator"],
                    ["normalized_actions"],
                    name="action_divide",
                ),
                helper.make_node(
                    "Mul",
                    ["normalized_actions", "clip_scale"],
                    ["actions"],
                    name="action_scale",
                ),
            ]
        )

    graph = helper.make_graph(
        nodes,
        "oli_hu_d04_01_stair_actor",
        [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, OBSERVATION_SIZE])],
        [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, ACTION_SIZE])],
        initializer=initializers,
    )
    model = helper.make_model(
        graph,
        producer_name="matrix_go2w_lcm_demo.oli_stairs",
        opset_imports=[helper.make_opsetid("", 17)],
    )
    model.ir_version = min(model.ir_version, 10)
    onnx.checker.check_model(model)

    sample = np.random.default_rng(0).normal(
        size=(1, OBSERVATION_SIZE)
    ).astype(np.float32)
    expected = evaluate_numpy(actor_params, sample, mean_clip_scale=mean_clip_scale)
    actual = ReferenceEvaluator(model).run(None, {"obs": sample})[0]
    np.testing.assert_allclose(actual, expected, rtol=2e-5, atol=2e-5)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output_path))
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    checkpoint = resolve_checkpoint(args.checkpoint)
    output = args.output.expanduser().resolve()
    params = ppo_checkpoint.load(str(checkpoint))
    print(export_actor(params[1], output))


if __name__ == "__main__":
    main()
