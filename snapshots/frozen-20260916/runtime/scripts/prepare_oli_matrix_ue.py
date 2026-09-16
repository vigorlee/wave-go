#!/usr/bin/env python3
"""Install the full OLI model into MATRIX with flat navigation geometry."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = (
    ROOT
    / "third_party/limx_humanoid_mujoco_sim/humanoid-description"
    / "HU_D04_description"
)
OLI_XML = DESCRIPTION / "xml/HU_D04_01.xml"
OLI_MESHES = DESCRIPTION / "meshes/HU_D04_01"
MATRIX_G1 = ROOT / "matrix/src/robot_mujoco/zsibot_robots/g1"
UE_ROOT = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
UE_GO2W = UE_ROOT / "go2w"
UE_OLI = UE_ROOT / "oli"
UE_CUSTOM = UE_ROOT / "custom"


def strip_to_flat_floor(root: ET.Element) -> None:
    """Remove imported YardWorld collision geometry, including both stairs."""
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MATRIX scene has no worldbody")
    for geom in list(worldbody.findall("geom")):
        if geom.get("name") != "floor":
            worldbody.remove(geom)


def remove_closed_chain_branches(root: ET.Element) -> None:
    """Keep only the visible 31-joint OLI kinematic tree for UE rendering."""
    branch_names = {
        "left_A_achilles_link",
        "left_B_achilles_link",
        "right_A_achilles_link",
        "right_B_achilles_link",
        "waist_A_link",
        "waist_B_link",
    }
    for parent in root.iter():
        for body in list(parent.findall("body")):
            if body.get("name") in branch_names:
                parent.remove(body)

    for tag in ("equality", "actuator", "sensor", "tendon", "keyframe"):
        element = root.find(tag)
        if element is not None:
            root.remove(element)

    actuator = ET.SubElement(root, "actuator")
    for joint in root.findall(".//joint"):
        joint_name = joint.get("name")
        if not joint_name or joint.get("type") in {"free", "ball"}:
            continue
        ET.SubElement(
            actuator,
            "motor",
            {"name": f"render_{joint_name}", "joint": joint_name, "gear": "1"},
        )


def build_oli_proxy() -> Path:
    # OLI source assets live in their own namespace.  The packaged UE binary
    # still dispatches its MuJoCo actor through `go2w`, so the launcher stages
    # these files into that runtime directory only for the lifetime of OLI.
    # In particular, never overwrite Go2-W's canonical YardWorld source.
    UE_OLI.mkdir(parents=True, exist_ok=True)
    mesh_target = UE_OLI / "oli_meshes/HU_D04_01"
    shutil.copytree(OLI_MESHES, mesh_target, dirs_exist_ok=True)

    root = ET.parse(OLI_XML).getroot()
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("OLI MJCF has no compiler element")
    compiler.set("meshdir", "oli_meshes/HU_D04_01/")

    # MATRIX's UE MuJoCo renderer follows its bundled robot convention: group
    # 2 contains the visible meshes and the collision groups stay hidden.
    for default in root.findall("default/default"):
        if default.get("class") != "visual":
            continue
        visual_geom = default.find("geom")
        if visual_geom is not None:
            visual_geom.set("group", "2")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("OLI MJCF has no worldbody")
    for geom in list(worldbody.findall("geom")):
        if geom.get("name") == "floor":
            worldbody.remove(geom)
    remove_closed_chain_branches(root)
    root.set("model", "OLI HU_D04_01 render proxy")

    model_target = UE_OLI / "oli_HU_D04_01_proxy.xml"
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(model_target, encoding="unicode")

    yard_root = ET.parse(MATRIX_G1 / "scene_terrain_yard.xml").getroot()
    include = yard_root.find("include")
    if include is None:
        raise RuntimeError("MATRIX YardWorld MJCF has no robot include")
    include.set("file", model_target.name)
    strip_to_flat_floor(yard_root)
    yard_root.set("model", "OLI flat navigation scene")
    yard_target = UE_OLI / "scene_terrain_oli_flat.xml"
    ET.indent(yard_root, space="  ")
    ET.ElementTree(yard_root).write(yard_target, encoding="unicode")
    # Install the same model through MATRIX's official arbitrary-robot entry.
    # YardWorld remains the selected UE map; only the robot renderer changes
    # from the fixed Go2W actor to the custom MJCF actor.
    UE_CUSTOM.mkdir(parents=True, exist_ok=True)
    custom_mesh_target = UE_CUSTOM / "oli_meshes/HU_D04_01"
    shutil.copytree(OLI_MESHES, custom_mesh_target, dirs_exist_ok=True)

    custom_root = ET.parse(OLI_XML).getroot()
    custom_compiler = custom_root.find("compiler")
    if custom_compiler is None:
        raise RuntimeError("OLI custom MJCF has no compiler element")
    custom_compiler.set("meshdir", "oli_meshes/HU_D04_01/")
    custom_worldbody = custom_root.find("worldbody")
    if custom_worldbody is None:
        raise RuntimeError("OLI custom MJCF has no worldbody")
    for geom in list(custom_worldbody.findall("geom")):
        if geom.get("name") == "floor":
            custom_worldbody.remove(geom)
    remove_closed_chain_branches(custom_root)
    custom_root.set("model", "OLI HU_D04_01 render proxy")

    ET.indent(custom_root, space="  ")
    ET.ElementTree(custom_root).write(UE_CUSTOM / "current.xml", encoding="unicode")

    custom_scene_root = ET.parse(MATRIX_G1 / "scene_terrain_yard.xml").getroot()
    custom_include = custom_scene_root.find("include")
    if custom_include is None:
        raise RuntimeError("MATRIX custom YardWorld MJCF has no robot include")
    custom_include.set("file", "current.xml")
    strip_to_flat_floor(custom_scene_root)
    custom_scene_root.set("model", "OLI flat navigation scene")

    ET.indent(custom_scene_root, space="  ")
    custom_scene = UE_CUSTOM / "scene_terrain_custom.xml"
    ET.ElementTree(custom_scene_root).write(custom_scene, encoding="unicode")
    # The packaged multi-robot renderer currently ignores ``custom_urdf`` for
    # its MuJoCo render actor and derives this fixed name from YardWorld.
    # Keep the configured name and the packaged-runtime alias identical.
    shutil.copy2(custom_scene, UE_CUSTOM / "scene_terrain_yard.xml")
    return yard_target


def write_runtime_config() -> Path:
    source = ROOT / "matrix/config/config.json"
    source_config = json.loads(source.read_text(encoding="utf-8"))
    robot = source_config.get("robot", {})
    robot.update(
        {
            "robot_type": "custom",
            "mujoco_running": True,
            "state_port": 9999,
            "cmd_port": 25002,
            "StatePort": 9999,
            "CMDPort": 25002,
            "EgoView": True,
            "map_key": "yard",
            "map_asset": "/Game/Maps/YardWorld",
            "use_custom_urdf": True,
            "custom_urdf": "custom/scene_terrain_custom.xml",
            "custom_name": "oli_HU_D04_01",
        }
    )
    robot.setdefault("position", {"x": 0, "y": 0, "z": 0})
    # The streamed free-joint quaternion already contains the simulator's
    # initial heading. Applying another UE actor rotation corrupts the custom
    # MJCF root transform, so keep the renderer actor at identity.
    robot["rotation"] = {"roll": 0, "pitch": 0, "yaw": 0}
    sensors = robot.setdefault("sensors", {})
    sensors.pop("camera", None)
    sensors.pop("depth_sensor", None)

    # Keep the primary key named ``robot`` for the packaged custom-MJCF
    # renderer. The former robot2 entry existed only to draw stair geometry.
    config = {"robot": robot}

    target_dir = UE_ROOT / "config"
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / "config.json"
    target.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return target


def main() -> None:
    for required in (OLI_XML, OLI_MESHES, MATRIX_G1 / "scene_terrain_yard.xml"):
        if not required.exists():
            raise FileNotFoundError(required)
    scene = build_oli_proxy()
    config = write_runtime_config()
    print(scene)
    print(config)


if __name__ == "__main__":
    main()
