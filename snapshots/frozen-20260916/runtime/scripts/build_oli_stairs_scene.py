#!/usr/bin/env python3
"""Merge the MATRIX YardWorld terrain into the full LimX OLI model."""

import copy
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = (
    ROOT
    / "third_party/limx_humanoid_mujoco_sim/humanoid-description"
    / "HU_D04_description/xml"
)
SOURCE = MODEL_DIR / "HU_D04_01.xml"
OUTPUT = MODEL_DIR / "HU_D04_01_matrix_yard.xml"
MATRIX_SCENE = (
    ROOT
    / "matrix/src/robot_mujoco/zsibot_robots/g1/scene_terrain_yard.xml"
)


def build_scene():
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("The OLI model has no worldbody")

    matrix_root = ET.parse(MATRIX_SCENE).getroot()
    matrix_worldbody = matrix_root.find("worldbody")
    if matrix_worldbody is None:
        raise RuntimeError("The MATRIX YardWorld scene has no worldbody")

    # Preserve OLI's floor/light/camera and import only MATRIX terrain geometry.
    # This excludes the scene's <include>, so no G1 or Go2-W model is introduced.
    for element in matrix_worldbody:
        if element.tag != "geom" or element.get("name") == "floor":
            continue
        terrain = copy.deepcopy(element)
        terrain.set("friction", terrain.get("friction", "1.2 0.02 0.002"))
        terrain.set("contype", "1")
        terrain.set("conaffinity", "1")
        terrain.set("group", "3")
        worldbody.append(terrain)

    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    build_scene()
    print(OUTPUT)
