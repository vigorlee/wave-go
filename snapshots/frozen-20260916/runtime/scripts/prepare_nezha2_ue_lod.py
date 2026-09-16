#!/usr/bin/env python3
"""Build a renderer-safe Nezha2 visual LOD without touching physics assets.

The packaged UE MJCF renderer corrupts the high-facet Nezha2 dynamic meshes.
This utility installs the project's precomputed low-polygon wheel-leg mesh
variant only after checking each part's bounds against the canonical Nezha2
UE mesh.  The result is visual-only: the MuJoCo model, collision geoms,
joint order and RL controller remain the canonical Nezha2 versions.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import trimesh


ROOT = Path(__file__).resolve().parents[1]
UE_MODELS = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
CANONICAL_DIR = UE_MODELS / "nezha2"
# This cache is the repository's proven low-facet wheel-leg visual set.  Its
# individual part dimensions are checked below; it is not used for physics.
LOD_SOURCE_DIR = UE_MODELS / "custom/_cache/nezha3/assets"
TARGET_DIR = UE_MODELS / "nezha2_lod"
PARTS = (
    "trunk.STL", "Lidar.STL",
    "FL_hip.STL", "FL_thigh.STL", "FL_calf.STL", "FL_footmotor.STL", "FL_foot.STL",
    "FR_hip.STL", "FR_thigh.STL", "FR_calf.STL", "FR_footmotor.STL", "FR_foot.STL",
    "RL_hip.STL", "RL_thigh.STL", "RL_calf.STL", "RL_footmotor.STL", "RL_foot.STL",
    "RR_hip.STL", "RR_thigh.STL", "RR_calf.STL", "RR_footmotor.STL", "RR_foot.STL",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def describe(path: Path) -> tuple[int, list[float], list[float]]:
    mesh = trimesh.load_mesh(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh) or mesh.is_empty:
        raise RuntimeError(f"invalid STL mesh: {path}")
    return (
        int(len(mesh.faces)),
        [float(value) for value in mesh.extents],
        [float(value) for value in mesh.bounds.mean(axis=0)],
    )


def main() -> None:
    if not (CANONICAL_DIR / "nezha2.xml").is_file():
        raise SystemExit(f"missing canonical Nezha2 UE model: {CANONICAL_DIR}")
    if not LOD_SOURCE_DIR.is_dir():
        raise SystemExit(f"missing checked renderer LOD source: {LOD_SOURCE_DIR}")

    assets = TARGET_DIR / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "purpose": "renderer-only Nezha2 low-polygon visual LOD",
        "physics_model": str(ROOT / "matrix/src/robot_mujoco/zsibot_robots/nezha2/nezha2.xml"),
        "canonical_visual_model": str(CANONICAL_DIR / "nezha2.xml"),
        "source_visual_meshes": str(LOD_SOURCE_DIR),
        "parts": {},
    }
    part_report: dict[str, object] = {}
    for name in PARTS:
        canonical = CANONICAL_DIR / "assets" / name
        lod = LOD_SOURCE_DIR / name
        if not canonical.is_file() or not lod.is_file():
            raise SystemExit(f"missing visual mesh {name}")
        canonical_faces, canonical_extent, canonical_center = describe(canonical)
        lod_faces, lod_extent, lod_center = describe(lod)
        extent_error = max(abs(a - b) for a, b in zip(canonical_extent, lod_extent))
        center_error = max(abs(a - b) for a, b in zip(canonical_center, lod_center))
        # At most 3 cm on a visual shell; coordinate frames remain identical.
        if extent_error > 0.03 or center_error > 0.03:
            raise SystemExit(
                f"LOD geometry mismatch for {name}: extent={extent_error:.5f}, center={center_error:.5f}"
            )
        shutil.copy2(lod, assets / name)
        part_report[name] = {
            "canonical_faces": canonical_faces,
            "lod_faces": lod_faces,
            "extent_max_error_m": extent_error,
            "center_max_error_m": center_error,
            "canonical_sha256": sha256(canonical),
            "lod_sha256": sha256(lod),
        }

    tree = ET.parse(CANONICAL_DIR / "nezha2.xml")
    compiler = tree.getroot().find("compiler")
    if compiler is None:
        raise SystemExit("canonical Nezha2 model has no compiler element")
    compiler.set("meshdir", "assets")
    ET.indent(tree, space="  ")
    tree.write(TARGET_DIR / "nezha2_lod.xml", encoding="utf-8", xml_declaration=True)
    report["parts"] = part_report
    report["max_extent_error_m"] = max(
        value["extent_max_error_m"] for value in part_report.values()  # type: ignore[index]
    )
    report["max_center_error_m"] = max(
        value["center_max_error_m"] for value in part_report.values()  # type: ignore[index]
    )
    (TARGET_DIR / "manifest.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
