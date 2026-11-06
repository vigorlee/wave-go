#!/usr/bin/env python3
"""Build Nezha2's OfficeWorld physics entry and Nav2 map from MATRiX geometry.

No walls or obstacles are redrawn here.  The generated PGM is a direct 5 cm
rasterization of every collidable primitive in the OfficeWorld XGW scene; only
the robot include is exchanged for the validated Nezha2 model.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from build_nezha2_warehouse_navigation_assets import write_map, write_scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    source = root / "matrix/src/robot_mujoco/zsibot_robots/xgw/scene_terrain_office.xml"
    output_scene = root / "matrix/src/robot_mujoco/zsibot_robots/nezha2/scene_terrain_office.xml"
    map_dir = root / "genisom_roamerx_open/map"
    if not source.is_file():
        raise SystemExit(f"missing MATRiX OfficeWorld scene: {source}")
    write_scene(source, output_scene, model_name="Nezha2 MATRiX OfficeWorld scene")
    # OfficeWorld's hfield declarations are relative to the active robot
    # directory after the XGW include is replaced.  Preserve those two source
    # images so the Nezha2 XML is independently loadable by robot_mujoco.
    for image_name in ("height_field.png", "unitree_hfield.png"):
        image_source = source.parent / image_name
        if not image_source.is_file():
            raise SystemExit(f"missing OfficeWorld height-field asset: {image_source}")
        shutil.copy2(image_source, output_scene.parent / image_name)
    manifest = write_map(source, map_dir, map_stem="nezha2_office")
    manifest["nezha_scene"] = str(output_scene)
    manifest["guarantee"] = (
        "OfficeWorld collision geometry is copied/rasterized directly; "
        "only the XGW robot include is replaced by Nezha2."
    )
    (map_dir / "nezha2_office_map_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
