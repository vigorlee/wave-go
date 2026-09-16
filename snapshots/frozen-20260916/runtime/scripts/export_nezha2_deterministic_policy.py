#!/usr/bin/env python3
"""Export a deterministic Nezha2 policy with the deployed two-input ABI."""

from __future__ import annotations

import argparse
from pathlib import Path

import torch
from torch import nn


class DeterministicNezha2Policy(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(76, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 16),
        )
        self.encoder = nn.Sequential(
            nn.Linear(285, 128),
            nn.ELU(),
            nn.Linear(128, 64),
        )
        self.latent_mu = nn.Linear(64, 16)
        self.vel_mu = nn.Linear(64, 3)

    def forward(
        self, observations: torch.Tensor, history_observations: torch.Tensor
    ) -> torch.Tensor:
        encoded = self.encoder(history_observations)
        latent = self.latent_mu(encoded)
        velocity = self.vel_mu(encoded)
        return self.actor(torch.cat((latent, velocity, observations), dim=-1))


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[1]
    model_dir = (
        root
        / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/rl_sar/models/nezha/robot_lab"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", type=Path, default=model_dir / "policy_zyl_0115_1.pt"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=model_dir / "policy_zyl_0115_1_deterministic.pt",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    source = torch.jit.load(str(args.source), map_location="cpu").eval()
    source_state = source.state_dict()

    remapped = {}
    for key, value in source_state.items():
        if key.startswith("actor."):
            remapped[key] = value
        elif key.startswith("vae.encoder.encoder."):
            remapped[key.replace("vae.encoder.encoder.", "encoder.")] = value
        elif key.startswith("vae.latent_mu."):
            remapped[key.replace("vae.", "")] = value
        elif key.startswith("vae.vel_mu."):
            remapped[key.replace("vae.", "")] = value

    policy = DeterministicNezha2Policy().eval()
    policy.load_state_dict(remapped, strict=True)
    scripted = torch.jit.script(policy)

    observations = torch.zeros((1, 57), dtype=torch.float32)
    history = torch.zeros((1, 285), dtype=torch.float32)
    expected = policy(observations, history)
    repeated = torch.stack([scripted(observations, history) for _ in range(20)])
    if not torch.equal(repeated, expected.unsqueeze(0).expand_as(repeated)):
        raise RuntimeError("deterministic export produced non-repeatable actions")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(args.output))
    print(f"[OK] deterministic Nezha2 policy: {args.output}")
    print(f"[OK] ABI: observation=57 history=285 action={expected.shape[-1]}")


if __name__ == "__main__":
    main()
