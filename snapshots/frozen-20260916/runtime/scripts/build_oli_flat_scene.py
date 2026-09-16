#!/usr/bin/env python3
"""Build OLI YardWorld navigation physics without either staircase."""

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
OUTPUT = MODEL_DIR / "HU_D04_01_matrix_flat.xml"
MATRIX_SCENE = (
    ROOT
    / "matrix/src/robot_mujoco/zsibot_robots/g1/scene_terrain_yard.xml"
)


def is_stair_geom(name: str) -> bool:
    return name.startswith("box_OBS") or name.startswith("box2_OBS")


def build_scene() -> None:
    root = ET.parse(SOURCE).getroot()
    root.set("model", "OLI HU_D04_01 flat navigation")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("The OLI model has no worldbody")

    floor = worldbody.find("geom[@name='floor']")
    if floor is None:
        raise RuntimeError("The OLI model has no floor geometry")
    floor.set("friction", "1.2 0.02 0.002")
    floor.set("contype", "1")
    floor.set("conaffinity", "1")

    matrix_root = ET.parse(MATRIX_SCENE).getroot()
    matrix_worldbody = matrix_root.find("worldbody")
    if matrix_worldbody is None:
        raise RuntimeError("The MATRIX YardWorld scene has no worldbody")

    for element in matrix_worldbody.findall("geom"):
        name = element.get("name", "")
        if name == "floor" or is_stair_geom(name):
            continue
        obstacle = copy.deepcopy(element)
        obstacle.set("friction", obstacle.get("friction", "1.2 0.02 0.002"))
        obstacle.set("contype", "1")
        obstacle.set("conaffinity", "1")
        obstacle.set("group", "3")
        obstacle.set("rgba", "0.28 0.32 0.36 0.18")
        worldbody.append(obstacle)

    ET.ElementTree(root).write(OUTPUT, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    build_scene()
    print(OUTPUT)
