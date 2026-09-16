#!/usr/bin/env python3
"""Build and compile a standalone Nezha2 MJCF without changing MATRiX state."""

import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import trimesh
from urdf2mjcf.convert import convert_urdf_to_mjcf


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = (
    PROJECT_ROOT
    / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/robots/nezha_description"
)
SOURCE_URDF = PROJECT_ROOT / "third_party/nezha2_rl_sar/view/nezha2_view.urdf"
OUTPUT_ROOT = PROJECT_ROOT / "third_party/nezha2_rl_sar/view/mujoco"
OUTPUT_URDF = OUTPUT_ROOT / "nezha2_mujoco.urdf"
OUTPUT_MJCF = OUTPUT_ROOT / "nezha2_mujoco.xml"
ASSET_ROOT = OUTPUT_ROOT / "assets"


def simplify_stl(path: Path, target_triangles: int = 180_000) -> int:
    # Binary STL repeats vertices per triangle. Merge coincident vertices so
    # quadric simplification sees connected surfaces instead of loose faces.
    mesh = trimesh.load_mesh(path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"STL did not load as one mesh: {path}")
    if len(mesh.faces) <= target_triangles:
        return len(mesh.faces)
    try:
        simplified = mesh.simplify_quadric_decimation(face_count=target_triangles)
    except BaseException as error:
        print(f"[WARN] keeping full mesh {path.name}: simplification failed: {error}")
        return len(mesh.faces)
    if len(simplified.faces) == 0:
        raise ValueError(f"simplification produced an empty mesh: {path}")
    simplified.export(path, file_type="stl")
    return len(simplified.faces)


def main() -> None:
    ASSET_ROOT.mkdir(parents=True, exist_ok=True)
    source_assets = PACKAGE_ROOT / "meshes"
    for mesh in source_assets.glob("*.STL"):
        target = ASSET_ROOT / mesh.name
        if target.exists():
            target.unlink()
        shutil.copy2(mesh, target)
        target.chmod(0o644)
        triangles = simplify_stl(target)
        print(f"mesh {mesh.name}: {triangles} triangles")

    tree = ET.parse(SOURCE_URDF)
    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        mesh.set("filename", str((ASSET_ROOT / Path(filename).name).resolve()))
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)

    convert_urdf_to_mjcf(OUTPUT_URDF, OUTPUT_MJCF, copy_meshes=False)
    model = mujoco.MjModel.from_xml_path(str(OUTPUT_MJCF))
    print(
        "MUJOCO_COMPILE_OK "
        f"nq={model.nq} nv={model.nv} nu={model.nu} "
        f"nbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom}"
    )


if __name__ == "__main__":
    main()
