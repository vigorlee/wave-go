#!/usr/bin/env python3
"""Build Nezha2's warehouse scene and its truthful static navigation map.

The source is MATRiX's ``xgw/scene_terrain_wh.xml``.  Only its robot include
is replaced; world collision geometry is not redrawn, simplified, or
hand-authored.  The PGM is rasterized from every collidable world primitive in
that source at the navigation stack's native 5 cm resolution.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


RESOLUTION = 0.05
MARGIN_METERS = 2.0


def floats(text: str, count: int, default: Iterable[float]) -> list[float]:
    values = [float(value) for value in text.split()] if text else list(default)
    if len(values) < count:
        values.extend(list(default)[len(values):count])
    return values[:count]


def yaw_from_quaternion(quat: str | None) -> float:
    w, x, y, z = floats(quat or "", 4, (1.0, 0.0, 0.0, 0.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def collidable_world_geoms(scene: Path) -> list[ET.Element]:
    root = ET.parse(scene).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError(f"worldbody missing from {scene}")
    geoms = []
    for geom in worldbody.findall("geom"):
        geom_type = geom.get("type", "sphere")
        if geom_type == "plane" or geom.get("contype") == "0":
            continue
        if geom_type not in {"box", "cylinder", "sphere", "capsule"}:
            raise RuntimeError(
                f"Cannot truthfully rasterize world geom '{geom.get('name')}' "
                f"of type '{geom_type}'. Add an exact rasterizer before using it."
            )
        geoms.append(geom)
    if not geoms:
        raise RuntimeError(f"no collidable world geometry found in {scene}")
    return geoms


def bounds_for_geom(geom: ET.Element) -> tuple[float, float, float, float]:
    geom_type = geom.get("type", "sphere")
    x, y, _ = floats(geom.get("pos", ""), 3, (0.0, 0.0, 0.0))
    size = floats(geom.get("size", ""), 3, (0.0, 0.0, 0.0))
    if geom_type == "box":
        half_x, half_y = size[0], size[1]
        yaw = yaw_from_quaternion(geom.get("quat"))
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
    elif geom_type in {"cylinder", "sphere", "capsule"}:
        extent_x = extent_y = size[0]
    else:  # Protected by collidable_world_geoms; leave for static analyzers.
        raise AssertionError(geom_type)
    return x - extent_x, x + extent_x, y - extent_y, y + extent_y


def set_cell(grid: bytearray, width: int, height: int, ix: int, iy: int) -> None:
    if 0 <= ix < width and 0 <= iy < height:
        # PGM rows are top-to-bottom; map coordinates are bottom-to-top.
        grid[(height - 1 - iy) * width + ix] = 0


def rasterize_geom(
    grid: bytearray, width: int, height: int, origin_x: float, origin_y: float, geom: ET.Element
) -> None:
    min_x, max_x, min_y, max_y = bounds_for_geom(geom)
    ix0 = max(0, int(math.floor((min_x - origin_x) / RESOLUTION)) - 1)
    ix1 = min(width - 1, int(math.ceil((max_x - origin_x) / RESOLUTION)) + 1)
    iy0 = max(0, int(math.floor((min_y - origin_y) / RESOLUTION)) - 1)
    iy1 = min(height - 1, int(math.ceil((max_y - origin_y) / RESOLUTION)) + 1)
    geom_type = geom.get("type", "sphere")
    x, y, _ = floats(geom.get("pos", ""), 3, (0.0, 0.0, 0.0))
    size = floats(geom.get("size", ""), 3, (0.0, 0.0, 0.0))
    yaw = yaw_from_quaternion(geom.get("quat"))
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_cell = RESOLUTION * math.sqrt(2.0) / 2.0

    for iy in range(iy0, iy1 + 1):
        cell_y = origin_y + (iy + 0.5) * RESOLUTION
        for ix in range(ix0, ix1 + 1):
            cell_x = origin_x + (ix + 0.5) * RESOLUTION
            dx, dy = cell_x - x, cell_y - y
            if geom_type == "box":
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                occupied = abs(local_x) <= size[0] + half_cell and abs(local_y) <= size[1] + half_cell
            else:
                occupied = dx * dx + dy * dy <= (size[0] + half_cell) ** 2
            if occupied:
                set_cell(grid, width, height, ix, iy)


def write_scene(source: Path, output: Path, *, model_name: str = "Nezha2 warehouse scene") -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", model_name)
    include = root.find("include")
    if include is None:
        raise RuntimeError(f"robot include missing from {source}")
    include.set("file", "nezha2.xml")
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def write_map(source: Path, output_dir: Path, *, map_stem: str = "nezha2_warehouse") -> dict[str, object]:
    geoms = collidable_world_geoms(source)
    bounds = [bounds_for_geom(geom) for geom in geoms]
    min_x = min(bound[0] for bound in bounds) - MARGIN_METERS
    max_x = max(bound[1] for bound in bounds) + MARGIN_METERS
    min_y = min(bound[2] for bound in bounds) - MARGIN_METERS
    max_y = max(bound[3] for bound in bounds) + MARGIN_METERS
    origin_x = math.floor(min_x / RESOLUTION) * RESOLUTION
    origin_y = math.floor(min_y / RESOLUTION) * RESOLUTION
    width = int(math.ceil((max_x - origin_x) / RESOLUTION))
    height = int(math.ceil((max_y - origin_y) / RESOLUTION))
    grid = bytearray([254]) * (width * height)
    for geom in geoms:
        rasterize_geom(grid, width, height, origin_x, origin_y, geom)

    output_dir.mkdir(parents=True, exist_ok=True)
    pgm = output_dir / f"{map_stem}.pgm"
    with pgm.open("wb") as stream:
        stream.write(f"P5\n# collision geometry rasterized from {source.name}\n{width} {height}\n255\n".encode())
        stream.write(grid)
    yaml = output_dir / f"{map_stem}.yaml"
    yaml.write_text(
        "\n".join(
            [
                f"image: {pgm.name}",
                f"resolution: {RESOLUTION:.2f}",
                f"origin: [{origin_x:.2f}, {origin_y:.2f}, 0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )
    occupied = grid.count(0)
    return {
        "source_scene": str(source),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "method": "direct rasterization of collidable worldbody primitives; no synthetic obstacles",
        "resolution_m": RESOLUTION,
        "origin_m": [origin_x, origin_y, 0.0],
        "size_cells": [width, height],
        "size_m": [round(width * RESOLUTION, 3), round(height * RESOLUTION, 3)],
        "world_collision_geometries": len(geoms),
        "geometry_types": dict(sorted(Counter(geom.get("type", "sphere") for geom in geoms).items())),
        "occupied_cells": occupied,
        "free_cells": width * height - occupied,
        "map_file": str(pgm),
        "map_yaml": str(yaml),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    source = root / "matrix/src/robot_mujoco/zsibot_robots/xgw/scene_terrain_wh.xml"
    output_scene = root / "matrix/src/robot_mujoco/zsibot_robots/nezha2/scene_terrain_warehouse.xml"
    map_dir = root / "genisom_roamerx_open/map"
    if not source.is_file():
        raise SystemExit(f"missing MATRiX Warehouse scene: {source}")

    write_scene(source, output_scene)
    manifest = write_map(source, map_dir)
    manifest["nezha_scene"] = str(output_scene)
    (map_dir / "nezha2_warehouse_map_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
