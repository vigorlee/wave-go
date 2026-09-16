"""JAX PR-to-AB kinematics for the OLI ankle and waist mechanisms."""

from __future__ import annotations

from functools import lru_cache

import jax
import jax.numpy as jp


_LEFT_ANKLE_A = jp.asarray(
    [[-0.035, 0.0, 0.270], [-0.035, 0.0, 0.180]], dtype=jp.float32
)
_LEFT_ANKLE_B = jp.asarray(
    [[-0.035, 0.032692, 0.2745], [-0.035, -0.032692, 0.1845]],
    dtype=jp.float32,
)
_LEFT_ANKLE_C = jp.asarray(
    [[-0.035, 0.032692, 0.0045], [-0.035, -0.032692, 0.0045]],
    dtype=jp.float32,
)
_RIGHT_ANKLE_A = jp.asarray(
    [[-0.035, 0.0, 0.180], [-0.035, 0.0, 0.270]], dtype=jp.float32
)
_RIGHT_ANKLE_B = jp.asarray(
    [[-0.035, 0.032692, 0.1845], [-0.035, -0.032692, 0.2745]],
    dtype=jp.float32,
)
_RIGHT_ANKLE_C = _LEFT_ANKLE_C
_ANKLE_ZERO_OFFSET = jp.asarray(
    [-0.000045238867, 0.000045238867], dtype=jp.float32
)

_WAIST_A = jp.asarray(
    [[0.0, 0.060, 0.064], [0.0, -0.060, 0.064]], dtype=jp.float32
)
_WAIST_B = jp.asarray(
    [
        [-0.063899, 0.060, 0.040743],
        [-0.063899, -0.060, 0.040743],
    ],
    dtype=jp.float32,
)
_WAIST_C = jp.asarray(
    [
        [-0.063899, 0.060, -0.023257],
        [-0.063899, -0.060, -0.023257],
    ],
    dtype=jp.float32,
)
_WAIST_ZERO_OFFSET = jp.asarray([0.000004765156, 0.000004765156])


def _rotation(first: jax.Array, second: jax.Array) -> jax.Array:
    """Return the SDK mechanism rotation, `Ry(second) @ Rx(first)`."""
    c0, s0 = jp.cos(first), jp.sin(first)
    c1, s1 = jp.cos(second), jp.sin(second)
    rx = jp.asarray(
        [[1.0, 0.0, 0.0], [0.0, c0, -s0], [0.0, s0, c0]]
    )
    ry = jp.asarray(
        [[c1, 0.0, s1], [0.0, 1.0, 0.0], [-s1, 0.0, c1]]
    )
    return ry @ rx


def _inverse_pair(
    pr: jax.Array,
    pivot: jax.Array,
    crank_end: jax.Array,
    platform_point: jax.Array,
    axis: str,
) -> jax.Array:
    rotation = _rotation(pr[0], pr[1])
    target = jp.einsum("ij,kj->ki", rotation, platform_point)
    displacement = target - pivot
    crank = crank_end - pivot
    rod_length = jp.linalg.norm(platform_point - crank_end, axis=1)
    constraint = (
        jp.sum(displacement * displacement, axis=1)
        + jp.sum(crank * crank, axis=1)
        - rod_length * rod_length
    ) * 0.5

    if axis == "x":
        cosine_term = displacement[:, 1] * crank[:, 1] + displacement[:, 2] * crank[:, 2]
        sine_term = -displacement[:, 1] * crank[:, 2] + displacement[:, 2] * crank[:, 1]
    elif axis == "y":
        cosine_term = displacement[:, 0] * crank[:, 0] + displacement[:, 2] * crank[:, 2]
        sine_term = displacement[:, 0] * crank[:, 2] - displacement[:, 2] * crank[:, 0]
    else:
        raise ValueError(f"Unsupported motor axis: {axis}")

    phase = jp.arctan2(sine_term, cosine_term)
    radius = jp.maximum(
        jp.sqrt(cosine_term * cosine_term + sine_term * sine_term),
        1.0e-8,
    )
    delta = jp.arccos(jp.clip(constraint / radius, -1.0 + 1.0e-6, 1.0 - 1.0e-6))
    plus = jp.arctan2(jp.sin(phase + delta), jp.cos(phase + delta))
    minus = jp.arctan2(jp.sin(phase - delta), jp.cos(phase - delta))
    if axis == "x":
        return jp.stack([plus[0], minus[1]])
    return plus


def left_ankle_pr_to_ab(pr: jax.Array) -> jax.Array:
    mechanism_pr = jp.stack([pr[1], pr[0]])
    return _inverse_pair(
        mechanism_pr, _LEFT_ANKLE_A, _LEFT_ANKLE_B, _LEFT_ANKLE_C, "x"
    ) + _ANKLE_ZERO_OFFSET


def right_ankle_pr_to_ab(pr: jax.Array) -> jax.Array:
    mechanism_pr = jp.stack([pr[1], pr[0]])
    generic_ab = _inverse_pair(
        mechanism_pr, _RIGHT_ANKLE_A, _RIGHT_ANKLE_B, _RIGHT_ANKLE_C, "x"
    ) + _ANKLE_ZERO_OFFSET
    return generic_ab[jp.asarray([1, 0])]


def waist_pr_to_ab(pr: jax.Array) -> jax.Array:
    return -(
        _inverse_pair(pr, _WAIST_A, _WAIST_B, _WAIST_C, "y")
        + _WAIST_ZERO_OFFSET
    )


@lru_cache(maxsize=None)
def _compiled_torque_projection(mapping):
    jacobian_fn = jax.jacfwd(mapping)

    @jax.jit
    def project(pr_position: jax.Array, pr_torque: jax.Array) -> jax.Array:
        jacobian = jp.nan_to_num(jacobian_fn(pr_position))
        torque = jp.nan_to_num(pr_torque)
        damping_squared = jp.asarray(1.0e-6, dtype=jacobian.dtype)
        regularized = jacobian @ jacobian.T + damping_squared * jp.eye(2)
        projected = jp.linalg.solve(regularized, jacobian @ torque)
        return jp.nan_to_num(projected)

    return project


def pr_torque_to_ab(
    mapping,
    pr_position: jax.Array,
    pr_torque: jax.Array,
) -> jax.Array:
    """Map virtual PR torque through a cached IK Jacobian projection."""
    return _compiled_torque_projection(mapping)(pr_position, pr_torque)
