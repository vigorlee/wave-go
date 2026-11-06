#!/usr/bin/env python3
"""Build a G1 OfficeWorld scene and matching navigation map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

from build_nezha2_warehouse_navigation_assets import write_map


def write_g1_scene(source: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "G1 MATRiX OfficeWorld scene")
    include = root.find("include")
    if include is None:
        raise RuntimeError(f"robot include missing from {source}")
    include.set("file", "g1_12dof.xml")
    ET.indent(tree, space="  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    source = root / "matrix/src/robot_mujoco/zsibot_robots/xgw/scene_terrain_office.xml"
    destination = root / "matrix/src/robot_mujoco/zsibot_robots/g1/scene_terrain_office.xml"
    if not source.is_file():
        raise FileNotFoundError(source)
    if not (destination.parent / "g1_12dof.xml").is_file():
        raise FileNotFoundError(destination.parent / "g1_12dof.xml")
    write_g1_scene(source, destination)
    for image_name in ("height_field.png", "unitree_hfield.png"):
        image_source = source.parent / image_name
        if not image_source.is_file():
            raise FileNotFoundError(image_source)
        shutil.copy2(image_source, destination.parent / image_name)
    manifest = write_map(source, root / "genisom_roamerx_open/map", map_stem="g1_office")
    manifest.update(
        {
            "g1_scene": str(destination),
            "guarantee": (
                "OfficeWorld world collision geometry is unchanged; only the XGW robot "
                "include is replaced by the validated G1 12-DOF model."
            ),
        }
    )
    path = root / "genisom_roamerx_open/map/g1_office_map_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
