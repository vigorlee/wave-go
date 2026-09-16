"""Checkpoint path helpers shared by OLI training tools."""

from __future__ import annotations

from pathlib import Path


def resolve_checkpoint(path: Path) -> Path:
    """Resolve a checkpoint root to its latest numeric Brax checkpoint."""
    resolved = path.expanduser().resolve()
    if (resolved / "_CHECKPOINT_METADATA").exists():
        return resolved
    if not resolved.is_dir():
        raise FileNotFoundError(f"Checkpoint path does not exist: {resolved}")

    candidates = [
        child
        for child in resolved.iterdir()
        if child.is_dir()
        and child.name.isdigit()
        and (child / "_CHECKPOINT_METADATA").exists()
    ]
    if not candidates:
        raise FileNotFoundError(f"No numeric Brax checkpoints found under {resolved}")
    return max(candidates, key=lambda child: int(child.name))

