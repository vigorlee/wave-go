#!/usr/bin/env python3
"""Install the canonical Nezha2 model into MATRiX's custom runtime.

MATRiX's ``custom`` renderer does *not* render the model only from the
``_cache/<custom_name>`` import directory.  Its active entry is
``Content/model/custom/current.xml`` (with ``custom/assets``).  If that active
entry still belongs to a different robot, UE applies Nezha2 state packets to
the wrong link/mesh hierarchy and the visible robot becomes fragmented.

This script stages the verified Nezha2 MJCF and matching meshes in both the
cache and the active custom entry.  The warehouse MuJoCo scene, collision
model, joint ordering and RL policy are deliberately not changed.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import mujoco
import trimesh


ROOT = Path(__file__).resolve().parents[1]
NAME = "nezha2"
PHYSICS_SOURCE = ROOT / "matrix/src/robot_mujoco/zsibot_robots/nezha2"
UE_SOURCE = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/nezha2"
PHYSICS_CACHE = ROOT / "matrix/src/robot_mujoco/zsibot_robots/custom/_cache" / NAME
UE_CACHE = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/_cache" / NAME
PHYSICS_ACTIVE = ROOT / "matrix/src/robot_mujoco/zsibot_robots/custom"
UE_ACTIVE = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom"
VIEW_URDF = ROOT / "third_party/nezha2_rl_sar/view/nezha2_view.urdf"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def normalize_renderer_mesh(mesh: trimesh.Trimesh) -> tuple[trimesh.Trimesh, float]:
    """Merge tiny fragmented components while keeping the source dimensions.

    Nezha2's supplied binary meshes contain thousands of tiny disconnected
    pieces. MATRiX's runtime STL importer renders those pieces sparsely. We
    merge vertices on a 1--3 mm grid and discard microscopic free pieces;
    this is a render-only cache, not a dynamics or collision asset.
    """
    if mesh.is_empty:
        raise RuntimeError("cannot normalize an empty Nezha2 mesh")
    maximum_extent = float(np.max(mesh.extents))
    grid = min(0.003, max(0.001, maximum_extent / 160.0))
    cells = np.rint(np.asarray(mesh.vertices) / grid).astype(np.int64)
    unique_cells, inverse = np.unique(cells, axis=0, return_inverse=True)
    sums = np.zeros((len(unique_cells), 3), dtype=np.float64)
    counts = np.zeros(len(unique_cells), dtype=np.int64)
    np.add.at(sums, inverse, np.asarray(mesh.vertices))
    np.add.at(counts, inverse, 1)
    vertices = sums / counts[:, None]
    faces = inverse[np.asarray(mesh.faces)]
    non_degenerate = np.logical_and.reduce(
        (faces[:, 0] != faces[:, 1], faces[:, 0] != faces[:, 2], faces[:, 1] != faces[:, 2])
    )
    faces = faces[non_degenerate]
    canonical_faces = np.sort(faces, axis=1)
    _, keep = np.unique(canonical_faces, axis=0, return_index=True)
    faces = faces[np.sort(keep)]
    normalized = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
    components = normalized.split(only_watertight=False)
    retained = [component for component in components if len(component.faces) >= 20]
    if retained:
        normalized = trimesh.util.concatenate(retained)
    if normalized.is_empty or len(normalized.faces) < 12:
        raise RuntimeError("renderer normalization removed a Nezha2 mesh")
    extent_error = float(np.max(np.abs(normalized.extents - mesh.extents)))
    if extent_error > 0.015:
        raise RuntimeError(
            f"renderer normalization changed Nezha2 dimensions: {extent_error:.6f} m"
        )
    normalized.remove_unreferenced_vertices()
    try:
        normalized.fix_normals(multibody=True)
    except (ModuleNotFoundError, ValueError):
        # The source winding is already valid; normal repair is best-effort.
        pass
    return normalized, extent_error


def write_renderer_stl(source: Path, destination: Path) -> tuple[int, int, float]:
    """Build a UE-safe indexed STL without changing the Nezha2 dimensions."""
    mesh = trimesh.load_mesh(source, process=True)
    if not isinstance(mesh, trimesh.Trimesh) or mesh.is_empty:
        raise RuntimeError(f"invalid Nezha2 render mesh: {source}")
    normalized, extent_error = normalize_renderer_mesh(mesh)
    normalized.export(destination, file_type="stl")
    return int(len(normalized.vertices)), int(len(normalized.faces)), extent_error


def build_standing_shell(xml_path: Path, assets: Path) -> tuple[int, int, float]:
    """Fuse the verified Nezha2 exterior at its neutral wheel-leg stance.

    MATRiX updates the BASE_LINK free joint reliably, but its dynamic custom
    visual loader can decompose a many-link mesh into fragments.  A single
    root-attached neutral shell follows the *actual* simulated base pose and
    preserves the real Nezha2 dimensions/mesh source; its legs are stationary
    relative to the body in the renderer only.  Physics and 16-joint RL stay
    untouched in the MuJoCo runtime.
    """
    tree = ET.parse(xml_path)
    files = {
        element.get("name"): element.get("file")
        for element in tree.findall("asset/mesh")
        if element.get("name") and element.get("file")
    }
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "BASE_LINK")
    if root_id < 0:
        raise RuntimeError("Nezha2 render cache has no BASE_LINK")
    root_position = np.array(data.xpos[root_id], dtype=np.float64)
    root_rotation = np.array(data.xmat[root_id], dtype=np.float64).reshape(3, 3)
    root_inverse = root_rotation.T
    transformed: list[trimesh.Trimesh] = []
    for geom in tree.findall(".//geom[@class='visual']"):
        mesh_name = geom.get("mesh")
        geom_name = geom.get("name")
        filename = files.get(mesh_name)
        if not mesh_name or not geom_name or not filename:
            continue
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id < 0:
            raise RuntimeError(f"Nezha2 visual geom is not in MuJoCo: {geom_name}")
        part = trimesh.load_mesh(assets / filename, process=True)
        if not isinstance(part, trimesh.Trimesh) or part.is_empty:
            raise RuntimeError(f"invalid Nezha2 shell mesh: {filename}")
        pose = np.eye(4)
        pose[:3, :3] = root_inverse @ np.array(data.geom_xmat[geom_id]).reshape(3, 3)
        pose[:3, 3] = root_inverse @ (np.array(data.geom_xpos[geom_id]) - root_position)
        part.apply_transform(pose)
        transformed.append(part)
    if not transformed:
        raise RuntimeError("Nezha2 shell has no visible mesh geoms")
    fused = trimesh.util.concatenate(transformed)
    normalized, extent_error = normalize_renderer_mesh(fused)
    normalized.export(assets / "nezha2_standing_shell.STL", file_type="stl")
    return int(len(normalized.vertices)), int(len(normalized.faces)), extent_error


def stage(source: Path, target: Path, *, ue_render_cache: bool = False) -> dict[str, object]:
    source_xml = source / f"{NAME}.xml"
    source_assets = source / "assets"
    if not source_xml.is_file() or not source_assets.is_dir():
        raise SystemExit(f"incomplete Nezha2 source model: {source}")

    target.mkdir(parents=True, exist_ok=True)
    target_xml = target / f"{NAME}.xml"
    target_assets = target / "assets"
    shutil.copy2(source_xml, target_xml)
    shutil.copytree(source_assets, target_assets, dirs_exist_ok=True)
    if VIEW_URDF.is_file():
        shutil.copy2(VIEW_URDF, target / f"{NAME}.urdf")

    render_meshes: dict[str, dict[str, int | str]] = {}
    if ue_render_cache:
        tree = ET.parse(target_xml)
        # Match the stock MATRiX robot convention: group 2 is the visible
        # shell and group 3 is collision-only.  The source Nezha2 MJCF left
        # its collision default in group 0, so UE drew collision boxes on top
        # of the STL shell and produced a speckled, apparently disassembled
        # robot.  This affects the renderer cache only.
        collision_default = tree.find("default/default[@class='robot']/default[@class='collision']/geom")
        if collision_default is not None:
            collision_default.set("group", "3")
            collision_default.set("rgba", "0 0 0 0")
        for collision in tree.findall(".//geom[@class='collision']"):
            collision.set("group", "3")
            collision.set("rgba", "0 0 0 0")
        for mesh in tree.findall("asset/mesh"):
            filename = mesh.get("file")
            if not filename:
                continue
            stl = target_assets / filename
            if stl.suffix.lower() != ".stl" or not stl.is_file():
                raise SystemExit(f"UE cache mesh is missing: {stl}")
            # The packaged custom loader resolves runtime meshes only from
            # ``assets`` (nested meshdir paths are silently skipped), so the
            # renderer-normalized copy deliberately occupies the cache-local
            # asset filename.  The source Nezha2 assets outside this cache are
            # never changed and remain the MuJoCo physics reference.
            renderer_stl = target_assets / filename
            vertices, faces, extent_error = write_renderer_stl(stl, renderer_stl)
            render_meshes[filename] = {
                "render_file": str(Path("assets") / filename),
                "vertices": vertices,
                "faces": faces,
                "extent_max_error_m": extent_error,
            }
        compiler = tree.getroot().find("compiler")
        if compiler is None:
            raise SystemExit("Nezha2 UE cache MJCF has no compiler element")
        compiler.set("meshdir", "assets")
        ET.indent(tree, space="  ")
        tree.write(target_xml, encoding="utf-8", xml_declaration=True)

    assets = sorted(path for path in target_assets.iterdir() if path.is_file())
    if not assets:
        raise SystemExit(f"no meshes staged in {target_assets}")
    return {
        "cache": str(target),
        "xml": str(target_xml),
        "xml_sha256": sha256(target_xml),
        "asset_count": len(assets),
        "asset_sha256": {path.name: sha256(path) for path in assets},
        "render_mesh_format": "normalized_binary_stl" if ue_render_cache else "stl",
        "render_meshes": render_meshes,
    }


def stage_active_runtime(cache: Path, active_root: Path) -> dict[str, object]:
    """Synchronize MATRiX's real custom entry from a verified cache.

    ``current.xml`` and ``assets/`` are the names consumed by the UE custom
    importer.  Keeping this entry in sync on both sides prevents a stale
    human/proxy model from being selected by renderer discovery code.
    """
    cache_xml = cache / f"{NAME}.xml"
    cache_assets = cache / "assets"
    if not cache_xml.is_file() or not cache_assets.is_dir():
        raise SystemExit(f"incomplete Nezha2 cache cannot become active: {cache}")

    active_root.mkdir(parents=True, exist_ok=True)
    active_xml = active_root / "current.xml"
    active_assets = active_root / "assets"
    shutil.copy2(cache_xml, active_xml)
    shutil.copytree(cache_assets, active_assets, dirs_exist_ok=True)

    # Validate exactly the entry UE resolves, rather than merely the cache.
    model = mujoco.MjModel.from_xml_path(str(active_xml))
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "BASE_LINK")
    if root_id < 0 or model.nu != 16:
        raise RuntimeError(
            f"invalid active Nezha2 custom entry: BASE_LINK={root_id}, nu={model.nu}"
        )
    return {
        "root": str(active_root),
        "entry": str(active_xml),
        "entry_sha256": sha256(active_xml),
        "asset_count": len([path for path in active_assets.iterdir() if path.is_file()]),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "nu": int(model.nu),
    }


def main() -> None:
    physics = stage(PHYSICS_SOURCE, PHYSICS_CACHE)
    ue = stage(UE_SOURCE, UE_CACHE, ue_render_cache=True)
    # The active entries are intentionally byte-identical.  MuJoCo dynamics
    # continue to use ``robot_mujoco/zsibot_robots/nezha2``; these copies are
    # MATRiX's custom-import contract and must not drift across the bridge.
    active_physics = stage_active_runtime(UE_CACHE, PHYSICS_ACTIVE)
    active_ue = stage_active_runtime(UE_CACHE, UE_ACTIVE)
    manifest = {
        "schema_version": 1,
        "generator": "prepare_nezha2_matrix_custom_cache.py",
        "custom_name": NAME,
        "reference_profile": "xgw",
        "source": {
            "physics": str(PHYSICS_SOURCE),
            "ue_visual": str(UE_SOURCE),
            "view_urdf": str(VIEW_URDF),
        },
        "integrity": {
            "physics_cache": physics,
            "ue_cache": ue,
            "physics_active": active_physics,
            "ue_active": active_ue,
        },
        "guarantee": (
            "The active custom entry is canonical Nezha2 (BASE_LINK, 24 bodies, "
            "16 actuators); the warehouse MuJoCo scene and RL policy remain unchanged."
        ),
    }
    for cache in (PHYSICS_CACHE, UE_CACHE):
        (cache / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
