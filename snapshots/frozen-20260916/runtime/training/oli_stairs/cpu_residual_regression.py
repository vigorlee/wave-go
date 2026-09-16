"""Verify that zero residual exactly follows the authoritative CPU rollout."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from training.oli_stairs.constants import ACTION_SIZE, StairSpec
from training.oli_stairs.cpu_residual_env import OliCpuResidualEnv


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--policy", type=Path, required=True)
    args = parser.parse_args()

    env = OliCpuResidualEnv(
        args.model,
        args.policy,
        StairSpec(step_height=0.05, step_count=5),
        speed=0.30,
        lateral_speed=0.05,
        history_lateral_speed=0.08,
        history_reset_x=0.99,
    )
    env.reset(seed=0)
    terminated = truncated = False
    info: dict = {}
    while not (terminated or truncated):
        _, _, terminated, truncated, info = env.step(
            np.zeros(ACTION_SIZE, dtype=np.float32)
        )

    expected = np.asarray([2.3393563437450404, -0.21321063026084633, 0.60379865705612])
    np.testing.assert_allclose(env.data.qpos[:3], expected, rtol=0.0, atol=1.0e-11)
    if env.steps != 631:
        raise AssertionError(f"Expected 631 steps, got {env.steps}")
    if info["max_support_height"] != 0.25:
        raise AssertionError(
            f"Expected 0.25 m max support, got {info['max_support_height']}"
        )
    print(
        "OK "
        f"steps={env.steps} x={env.data.qpos[0]:.6f} "
        f"z={env.data.qpos[2]:.6f} "
        f"max_support_z={info['max_support_height']:.3f}"
    )


if __name__ == "__main__":
    main()
