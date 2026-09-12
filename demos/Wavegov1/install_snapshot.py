#!/usr/bin/env python3
"""Verify or restore the Wavegov1 source overlay onto a prepared runtime."""
import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def verified_files():
    manifest = json.loads((ROOT / "files.sha256.json").read_text())
    for name, expected in manifest.items():
        path = ROOT / name
        if not path.resolve().is_relative_to(ROOT):
            raise RuntimeError(f"Invalid manifest path: {name}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"Snapshot checksum mismatch: {name}")
    return [ROOT / name for name in manifest if name.startswith("runtime/")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--apply", action="store_true", help="Copy files, backing up changes first")
    args = parser.parse_args()
    target = args.runtime.expanduser().resolve()
    files = verified_files()
    required = ["matrix/src/robot_mujoco/zsibot_robots/go2w/go2w.xml",
                "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w",
                "genisom_roamerx_open/install/setup.bash"]
    for name in required:
        if not (target / name).exists():
            raise RuntimeError(f"Prepared runtime dependency missing: {target / name}")
    changes = []
    for source in files:
        relative = source.relative_to(ROOT / "runtime")
        destination = target / relative
        if not destination.exists() or source.read_bytes() != destination.read_bytes():
            changes.append((source, relative, destination))
    print(f"Verified {len(files)} runtime files; {len(changes)} files differ at {target}")
    if not args.apply:
        print("Check only. Use --apply to restore this version after stopping the demo.")
        return
    for unit in ["go2w-course.service", "go2w-navigation.service", "go2w-anything-models.service", "go2w-anything-view.service"]:
        if subprocess.run(["systemctl", "--user", "is-active", "--quiet", unit]).returncode == 0:
            raise RuntimeError(f"Stop {unit} before replacing runtime files")
    backup = target / ".backups" / ("Wavegov1_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    for source, relative, destination in changes:
        if destination.exists():
            saved = backup / relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(destination, saved)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    # Preserve executable bits even if file contents were identical.
    for source in files:
        destination = target / source.relative_to(ROOT / "runtime")
        shutil.copymode(source, destination)
    print(f"Restored {len(changes)} files. Previous changed files: {backup}")
    print("Rebuild the bridge/ROS workspace before starting; see README.md.")


if __name__ == "__main__":
    main()
