#!/usr/bin/env python3
"""Create a traceable RViz copy of the Nezha2 URDF.

The source archive is missing meshes/imu_fixed_joint.STL. The IMU link remains
in the kinematic tree, but its unavailable visual is removed from the view copy.
"""

from pathlib import Path
from xml.etree import ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = (
    PROJECT_ROOT
    / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/robots/nezha_description"
)
# Use the archive's active default model: complete wheel-leg base and LiDAR,
# without the optional tops payload assembly.
SOURCE_URDF = PACKAGE_ROOT / "urdf/nezha_description.urdf"
OUTPUT_URDF = PROJECT_ROOT / "third_party/nezha2_rl_sar/view/nezha2_view.urdf"


def main() -> None:
    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()

    # Older archive variants use a tiny Gazebo-only base wrapper. MATRiX needs
    # the physical trunk to be the floating root when that wrapper is present.
    for joint in list(root.findall("joint")):
        if joint.get("name") == "floating_base":
            root.remove(joint)
    for link in list(root.findall("link")):
        if link.get("name") == "base":
            root.remove(link)

    removed = 0
    material_index = 0
    for link in root.findall("link"):
        for visual in list(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename", "")
            if filename.endswith("/imu_fixed_joint.STL"):
                link.remove(visual)
                removed += 1

            material = visual.find("material")
            if material is not None and not material.get("name"):
                material.set("name", f"nezha2_material_{material_index}")
                material_index += 1

    if removed != 1:
        raise RuntimeError(f"expected one missing IMU visual, removed {removed}")

    OUTPUT_URDF.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUTPUT_URDF} from {SOURCE_URDF.name}")


if __name__ == "__main__":
    main()
