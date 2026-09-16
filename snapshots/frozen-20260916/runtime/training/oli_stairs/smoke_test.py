"""GPU/JIT smoke test for the generated OLI stair environment."""

from __future__ import annotations

import tempfile
from pathlib import Path

import jax
import jax.numpy as jp
import numpy as np
import onnx
from onnx.reference import ReferenceEvaluator

from training.oli_stairs.constants import (
    ACTION_SIZE,
    OBSERVATION_SIZE,
    OFFICIAL_POLICY,
    stair_spec_for_stage,
)
from training.oli_stairs.environment import OliStairEnv, default_config
from training.oli_stairs.export_onnx import export_actor
from training.oli_stairs.network import (
    MEAN_CLIP_SCALE,
    make_network_factory,
    make_official_restore_params,
)


def main() -> None:
    if jax.default_backend() != "gpu":
        raise RuntimeError(f"Expected GPU backend, got {jax.default_backend()}")

    config = default_config()
    config.observation_noise = 0.0
    env = OliStairEnv(stair_spec_for_stage("h15"), config=config)
    reset = jax.jit(env.reset)
    step = jax.jit(env.step)
    state = reset(jax.random.PRNGKey(0))
    state = step(state, jp.zeros(ACTION_SIZE))
    if state.obs.shape != (OBSERVATION_SIZE,):
        raise AssertionError(f"Unexpected observation shape: {state.obs.shape}")
    if not bool(np.asarray(jp.all(jp.isfinite(state.obs)))):
        raise AssertionError("Non-finite observation after one MJX step")

    normalizer, actor, critic = make_official_restore_params()
    del critic
    network = make_network_factory()(OBSERVATION_SIZE, ACTION_SIZE)
    mean, _ = network.policy_network.apply(normalizer, actor, state.obs[None, :])
    official = ReferenceEvaluator(onnx.load(str(OFFICIAL_POLICY))).run(
        None, {"obs": np.asarray(state.obs[None, :], dtype=np.float32)}
    )[0]
    bounded_official = official / (1.0 + np.abs(official) / MEAN_CLIP_SCALE)
    np.testing.assert_allclose(
        np.asarray(mean), bounded_official, rtol=2e-5, atol=2e-5
    )

    with tempfile.TemporaryDirectory() as temp_dir:
        exported_path = export_actor(actor, Path(temp_dir) / "policy.onnx")
        exported = ReferenceEvaluator(onnx.load(str(exported_path))).run(
            None, {"obs": np.asarray(state.obs[None, :], dtype=np.float32)}
        )[0]
        np.testing.assert_allclose(
            exported, bounded_official, rtol=2e-5, atol=2e-5
        )

        unbounded_normalizer, unbounded_actor, _ = make_official_restore_params(
            policy_path=OFFICIAL_POLICY,
            mean_clip_scale=None,
        )
        unbounded_network = make_network_factory(mean_clip_scale=None)(
            OBSERVATION_SIZE, ACTION_SIZE
        )
        unbounded_mean, _ = unbounded_network.policy_network.apply(
            unbounded_normalizer,
            unbounded_actor,
            state.obs[None, :],
        )
        np.testing.assert_allclose(
            np.asarray(unbounded_mean), official, rtol=2e-5, atol=2e-5
        )
        unbounded_path = export_actor(
            unbounded_actor,
            Path(temp_dir) / "policy_unbounded.onnx",
            mean_clip_scale=None,
        )
        unbounded_export = ReferenceEvaluator(onnx.load(str(unbounded_path))).run(
            None, {"obs": np.asarray(state.obs[None, :], dtype=np.float32)}
        )[0]
        np.testing.assert_allclose(
            unbounded_export, official, rtol=2e-5, atol=2e-5
        )

    print(
        "OK "
        f"backend={jax.default_backend()} device={jax.devices()[0]} "
        f"nq={env.mj_model.nq} nv={env.mj_model.nv} nu={env.mj_model.nu} "
        f"obs={state.obs.shape} action={env.action_size}"
    )


if __name__ == "__main__":
    main()
