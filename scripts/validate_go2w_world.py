#!/usr/bin/env python3
"""Fail closed when the Go2-W physics, UE world, and navigation contract diverge."""

from __future__ import annotations

import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MUJOCO = ROOT / "matrix/src/robot_mujoco/zsibot_robots/go2w"
UE = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w"
UE_CONFIG = ROOT / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json"
NAV_CONFIG = (
    ROOT
    / "genisom_roamerx_open/src/navigation/src/robot_navigo/params/navigo_params.yaml"
)


def fail(message: str) -> None:
    print(f"[WORLD_MODEL_FAIL] {message}", file=sys.stderr)
    raise SystemExit(1)


def parse(path: Path) -> ET.Element:
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        fail(f"cannot parse {path}: {exc}")


def included_model(path: Path) -> str:
    include = parse(path).find("include")
    if include is None or not include.get("file"):
        fail(f"scene has no robot include: {path}")
    return include.get("file", "")


def main() -> None:
    physics_scene = MUJOCO / "scene_terrain_yard.xml"
    ue_source_scene = UE / "scene_terrain_yard_go2w.xml"
    ue_runtime_scene = UE / "scene_terrain.xml"

    if included_model(physics_scene) != "go2w.xml":
        fail("MuJoCo YardWorld is not bound to go2w.xml")
    if included_model(ue_source_scene) != "go2w.xml":
        fail("isolated UE Go2-W world is not bound to go2w.xml")
    if included_model(ue_runtime_scene) != "go2w.xml":
        fail("active UE runtime was replaced by another robot")

    robot = parse(MUJOCO / "go2w.xml")
    required_joints = {
        f"{leg}_{joint}_joint"
        for leg in ("FL", "FR", "RL", "RR")
        for joint in ("hip", "thigh", "calf", "wheel")
    }
    joints = {node.get("name") for node in robot.findall(".//joint")}
    missing = sorted(required_joints - joints)
    if missing:
        fail(f"Go2-W joint contract is incomplete: {missing}")

    actuators = robot.find("actuator")
    if actuators is None or len(list(actuators)) != 16:
        fail("Go2-W must expose exactly 16 actuators")
    sensor_names = {node.get("name") for node in robot.findall("sensor/*")}
    for required in ("imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel"):
        if required not in sensor_names:
            fail(f"missing required state sensor: {required}")

    try:
        config = json.loads(UE_CONFIG.read_text(encoding="utf-8"))["robot"]
    except (OSError, KeyError, json.JSONDecodeError) as exc:
        fail(f"invalid UE runtime config: {exc}")
    if config.get("robot_type") != "go2w" or int(config.get("state_port", -1)) != 25001:
        fail("UE runtime config is not isolated Go2-W/25001")

    try:
        nav = NAV_CONFIG.read_text(encoding="utf-8")
    except OSError as exc:
        fail(f"cannot read navigation world model: {exc}")
    navigation_contract = {
        "LiDAR topic": r"topic:\s*/livox/lidar\b",
        "PointCloud2 type": r"data_type:\s*[\"']?PointCloud2[\"']?",
        "0.10 m obstacle floor": r"min_obstacle_height:\s*0\.10\b",
        "1.20 m obstacle ceiling": r"max_obstacle_height:\s*1\.20\b",
        "MPPI footprint collision critic": (
            r"CostCritic:\s*\n(?:.*\n){0,5}?\s*enabled:\s*true"
        ),
    }
    for label, pattern in navigation_contract.items():
        if re.search(pattern, nav) is None:
            fail(f"navigation contract missing {label}")

    print(
        "[WORLD_MODEL_OK] robot=go2w joints=16 actuators=16 "
        "world=YardWorld state_port=25001 local_obstacles=PointCloud2/MPPI"
    )


if __name__ == "__main__":
    main()
