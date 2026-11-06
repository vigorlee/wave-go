#!/usr/bin/env python3
"""Generate the packaged UE rendering variant from the physics scene.

YardWorld renders roll-tilted world geoms with the opposite slope to MuJoCo.
Only the two visible ramp skins need the measured roll-sign compensation;
collision geometry, geom ordering, robot and pedestrian state remain intact.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PHYSICS = ROOT / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml"
UE = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_extended_nav.xml"
COMPENSATED_ROLL_GEOMS = {"box_OBS15", "box_OBS16"}


def ue_scene_root(physics: ET.Element) -> ET.Element:
    result = ET.fromstring(ET.tostring(physics))
    for name in COMPENSATED_ROLL_GEOMS:
        geom = result.find(f".//worldbody/geom[@name='{name}']")
        if geom is None:
            raise ValueError(f"Missing ramp skin: {name}")
        angles = [float(x) for x in geom.attrib["euler"].split()]
        if angles[1:] != [0.0, 0.0]:
            raise ValueError(f"Unsupported compound ramp rotation: {name}")
        geom.set("euler", f"{-angles[0]:.8f} 0 0")
    return result


def main() -> None:
    root = ue_scene_root(ET.parse(PHYSICS).getroot())
    ET.indent(root, space="  ")
    UE.write_text(ET.tostring(root, encoding="unicode") + "\n")
    print(f"[OK] UE variant generated; compensated ramp skins: {sorted(COMPENSATED_ROLL_GEOMS)}")


if __name__ == "__main__":
    main()
