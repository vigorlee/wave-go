"""PPO network construction and official OLI actor initialization."""

from __future__ import annotations

import functools
from pathlib import Path
from typing import Any, Callable

from brax.training.acme import running_statistics
from brax.training.acme import specs
from brax.training.agents.ppo import networks as ppo_networks
from brax.training import networks as brax_networks
from flax import linen
from flax.core import FrozenDict, freeze, unfreeze
import jax
import jax.numpy as jp
import onnx
from onnx import numpy_helper

from training.oli_stairs.constants import (
    ACTION_SIZE,
    OBSERVATION_SIZE,
    OFFICIAL_POLICY,
)


POLICY_HIDDEN_SIZES = (512, 256, 128)
VALUE_HIDDEN_SIZES = (512, 256, 128)
MEAN_CLIP_SCALE = 4.0


def make_network_factory(
    freeze_policy_backbone: bool = False,
    mean_clip_scale: float | None = MEAN_CLIP_SCALE,
) -> Callable[..., ppo_networks.PPONetworks]:
    """Return a Brax network factory matching the official OLI actor MLP."""
    base_factory = functools.partial(
        ppo_networks.make_ppo_networks,
        policy_hidden_layer_sizes=POLICY_HIDDEN_SIZES,
        value_hidden_layer_sizes=VALUE_HIDDEN_SIZES,
        activation=linen.elu,
        distribution_type="normal",
        noise_std_type="scalar",
        init_noise_std=0.20,
        mean_clip_scale=mean_clip_scale,
    )
    if not freeze_policy_backbone:
        return base_factory

    def frozen_backbone_factory(*args, **kwargs) -> ppo_networks.PPONetworks:
        networks = base_factory(*args, **kwargs)
        base_policy = networks.policy_network

        def apply(normalizer_params, policy_params, observation):
            was_frozen = isinstance(policy_params, FrozenDict)
            live_params = unfreeze(policy_params)
            mixed_params = jax.tree_util.tree_map(
                jax.lax.stop_gradient, live_params
            )
            mixed_params["params"]["Dense_0"] = live_params["params"]["Dense_0"]
            if was_frozen:
                mixed_params = freeze(mixed_params)
            return base_policy.apply(
                normalizer_params, mixed_params, observation
            )

        policy = brax_networks.FeedForwardNetwork(
            init=base_policy.init,
            apply=apply,
        )
        return ppo_networks.PPONetworks(
            policy_network=policy,
            value_network=networks.value_network,
            parametric_action_distribution=networks.parametric_action_distribution,
        )

    return frozen_backbone_factory


def load_official_actor_params(
    initialized_params: Any,
    policy_path: Path = OFFICIAL_POLICY,
    initial_std: float = 0.20,
    mean_clip_scale: float | None = MEAN_CLIP_SCALE,
) -> Any:
    """Copy the official flat-ground ONNX actor into a Brax policy tree."""
    model = onnx.load(str(policy_path))
    tensors = {
        tensor.name: numpy_helper.to_array(tensor) for tensor in model.graph.initializer
    }
    expected = {
        "actor.0.weight",
        "actor.0.bias",
        "actor.2.weight",
        "actor.2.bias",
        "actor.4.weight",
        "actor.4.bias",
        "actor.6.weight",
        "actor.6.bias",
    }
    missing = expected - tensors.keys()
    if missing:
        raise ValueError(f"Official OLI policy is missing tensors: {sorted(missing)}")

    was_frozen = isinstance(initialized_params, FrozenDict)
    params = unfreeze(initialized_params)
    hidden = params["params"]["MLP_0"]
    for flax_name, onnx_index in (
        ("hidden_0", 0),
        ("hidden_1", 2),
        ("hidden_2", 4),
    ):
        hidden[flax_name]["kernel"] = jp.asarray(
            tensors[f"actor.{onnx_index}.weight"].T
        )
        hidden[flax_name]["bias"] = jp.asarray(
            tensors[f"actor.{onnx_index}.bias"]
        )

    # A bounded Brax mean expects the final layer in normalized units.  With an
    # unbounded mean, retain the source actor's final layer exactly.
    final_scale = mean_clip_scale if mean_clip_scale is not None else 1.0
    params["params"]["Dense_0"]["kernel"] = jp.asarray(
        tensors["actor.6.weight"].T / final_scale
    )
    params["params"]["Dense_0"]["bias"] = jp.asarray(
        tensors["actor.6.bias"] / final_scale
    )
    params["params"]["std_param"]["value"] = jp.full(
        (ACTION_SIZE,), initial_std, dtype=jp.float32
    )
    return freeze(params) if was_frozen else params


def make_official_restore_params(
    seed: int = 0,
    policy_path: Path = OFFICIAL_POLICY,
    mean_clip_scale: float | None = MEAN_CLIP_SCALE,
) -> tuple[Any, Any, Any]:
    """Create the `(normalizer, actor, critic)` tuple accepted by Brax PPO."""
    factory = make_network_factory(mean_clip_scale=mean_clip_scale)
    networks = factory(OBSERVATION_SIZE, ACTION_SIZE)
    policy_key, value_key = jax.random.split(jax.random.PRNGKey(seed))
    actor = load_official_actor_params(
        networks.policy_network.init(policy_key),
        policy_path=policy_path,
        mean_clip_scale=mean_clip_scale,
    )
    critic = networks.value_network.init(value_key)
    normalizer = running_statistics.init_state(
        specs.Array((OBSERVATION_SIZE,), jp.float32)
    )
    return normalizer, actor, critic


def make_zero_actor_restore_params(seed: int = 0) -> tuple[Any, Any, Any]:
    """Create PPO parameters whose deterministic action is exactly zero."""
    factory = make_network_factory()
    networks = factory(OBSERVATION_SIZE, ACTION_SIZE)
    policy_key, value_key = jax.random.split(jax.random.PRNGKey(seed))
    actor = unfreeze(networks.policy_network.init(policy_key))
    actor["params"]["Dense_0"]["kernel"] = jp.zeros_like(
        actor["params"]["Dense_0"]["kernel"]
    )
    actor["params"]["Dense_0"]["bias"] = jp.zeros_like(
        actor["params"]["Dense_0"]["bias"]
    )
    actor["params"]["std_param"]["value"] = jp.full(
        (ACTION_SIZE,), 0.10, dtype=jp.float32
    )
    critic = networks.value_network.init(value_key)
    normalizer = running_statistics.init_state(
        specs.Array((OBSERVATION_SIZE,), jp.float32)
    )
    return normalizer, actor, critic
