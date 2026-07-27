#!/usr/bin/env python3
"""Validate the isolated Go2-W HouseWorld simulator/runtime contract."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Mapping
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
HOUSE_SCENE_SHA256 = "47b8a865eeef18c946061d840bda20b178c23ccf2dc63a7eaafae4961ac1fcd7"
HOUSE_PAK_SHA256 = {
    "pakchunk17-Linux.pak": "7dd98239f13209bcfe244174669417ba50874c500359e7660dc9e137943dd9c7",
    "pakchunk17-Linux.utoc": "95291502519f044663919b90ed8ce8edd93e70de9082026ab3026fee8d52079a",
    "pakchunk17-Linux.ucas": "1da6fec873d13e087d0e0832dfba122150dca896b6388f025d205f4457b927eb",
}
EXPECTED_FOOTPRINT = "[[0.30,0.15],[0.30,-0.15],[-0.30,-0.15],[-0.30,0.15]]"


@dataclass(frozen=True)
class HousePaths:
    root: Path
    physics_scene: Path
    ue_scene: Path
    robot_xml: Path
    ue_config: Path
    paks_dir: Path
    map_yaml: Path
    map_pgm: Path
    nav_config: Path

    @classmethod
    def from_root(cls, root: Path) -> "HousePaths":
        root = root.resolve()
        map_yaml = Path(
            os.environ.get(
                "GO2W_HOUSE_MAP_YAML",
                ".run/go2w_house/map/house_map.yaml",
            )
        )
        map_pgm = Path(
            os.environ.get(
                "GO2W_HOUSE_MAP_PGM",
                ".run/go2w_house/map/house_map.pgm",
            )
        )
        if not map_yaml.is_absolute():
            map_yaml = root / map_yaml
        if not map_pgm.is_absolute():
            map_pgm = root / map_pgm
        return cls(
            root=root,
            physics_scene=(
                root
                / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_house.xml"
            ),
            ue_scene=(
                root
                / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_house.xml"
            ),
            robot_xml=(
                root / "matrix/src/robot_mujoco/zsibot_robots/go2w/go2w.xml"
            ),
            ue_config=(
                root
                / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json"
            ),
            paks_dir=(
                root / "matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks"
            ),
            map_yaml=map_yaml,
            map_pgm=map_pgm,
            nav_config=root / "config/go2w_house_navigo_params.yaml",
        )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_xml(path: Path, errors: list[str]) -> ET.Element | None:
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        errors.append(f"cannot parse XML {path}: {exc}")
        return None


def _read_yaml(path: Path, errors: list[str]) -> Any:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        errors.append(f"cannot parse YAML {path}: {exc}")
        return None


def _normalized_footprint(value: Any) -> str:
    return "".join(str(value).split())


def _is_true(value: Any) -> bool:
    return value is True or (isinstance(value, str) and value.lower() == "true")


def _is_false(value: Any) -> bool:
    return value is False or (isinstance(value, str) and value.lower() == "false")


def _float(value: Any, label: str, errors: list[str]) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        errors.append(f"{label} must be numeric, got {value!r}")
        return None


def _pointcloud_contract(
    layer: Mapping[str, Any] | None,
    label: str,
    errors: list[str],
    *,
    require_near_returns: bool,
) -> None:
    if not isinstance(layer, Mapping):
        errors.append(f"{label} obstacle_layer is missing")
        return
    if not _is_true(layer.get("enabled")):
        errors.append(f"{label} obstacle_layer must be enabled")
    sources = str(layer.get("observation_sources", "")).split()
    if "pointcloud" not in sources:
        errors.append(f"{label} obstacle_layer must observe pointcloud")
    pointcloud = layer.get("pointcloud")
    if not isinstance(pointcloud, Mapping):
        errors.append(f"{label} pointcloud source is missing")
        return
    if pointcloud.get("topic") != "/livox/lidar":
        errors.append(f"{label} pointcloud topic must be /livox/lidar")
    if pointcloud.get("data_type") != "PointCloud2":
        errors.append(f"{label} pointcloud data_type must be PointCloud2")
    if not _is_true(pointcloud.get("marking")):
        errors.append(f"{label} pointcloud marking must be enabled")
    if not _is_true(pointcloud.get("clearing")):
        errors.append(f"{label} pointcloud clearing must be enabled")
    minimum = _float(
        pointcloud.get("obstacle_min_range"),
        f"{label} obstacle_min_range",
        errors,
    )
    if require_near_returns and minimum is not None and minimum > 0.20:
        errors.append(
            f"{label} obstacle_min_range must be <= 0.20 m, got {minimum:.3f}"
        )
    min_height = _float(
        pointcloud.get("min_obstacle_height"),
        f"{label} min_obstacle_height",
        errors,
    )
    max_height = _float(
        pointcloud.get("max_obstacle_height"),
        f"{label} max_obstacle_height",
        errors,
    )
    if min_height is not None and min_height > 0.15:
        errors.append(f"{label} min_obstacle_height is too high: {min_height:.3f}")
    if max_height is not None and max_height < 1.0:
        errors.append(f"{label} max_obstacle_height is too low: {max_height:.3f}")


def validate_navigation_config(path: Path) -> list[str]:
    errors: list[str] = []
    document = _read_yaml(path, errors)
    if not isinstance(document, Mapping):
        if not errors:
            errors.append(f"navigation config is not a mapping: {path}")
        return errors

    try:
        local = document["local_costmap"]["local_costmap"]["ros__parameters"]
        global_costmap = document["global_costmap"]["global_costmap"]["ros__parameters"]
        planner = document["planner_server"]["ros__parameters"]["GridBased"]
        controller = document["controller_server"]["ros__parameters"]["FollowPath"]
        optimizer = document["velocity_optimizer"]["ros__parameters"]
        map_server = document["map_server"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        errors.append(f"navigation config is incomplete: missing {exc}")
        return errors

    expected_footprint = _normalized_footprint(EXPECTED_FOOTPRINT)
    for label, parameters in (("local", local), ("global", global_costmap)):
        if _normalized_footprint(parameters.get("footprint")) != expected_footprint:
            errors.append(f"{label} costmap changed the Go2-W footprint")

    local_plugins = local.get("plugins", [])
    if local_plugins != ["obstacle_layer", "inflation_layer"]:
        errors.append("local costmap plugins must be obstacle_layer then inflation_layer")
    _pointcloud_contract(
        local.get("obstacle_layer"), "local", errors, require_near_returns=True
    )

    global_plugins = global_costmap.get("plugins", [])
    required_global_plugins = {"static_layer", "obstacle_layer", "inflation_layer"}
    if set(global_plugins) != required_global_plugins:
        errors.append(
            "global costmap must contain static_layer, obstacle_layer, and inflation_layer"
        )
    _pointcloud_contract(
        global_costmap.get("obstacle_layer"),
        "global",
        errors,
        require_near_returns=False,
    )
    if not _is_false(global_costmap.get("track_unknown_space")):
        errors.append("global costmap track_unknown_space must be false indoors")

    for label, parameters in (("local", local), ("global", global_costmap)):
        inflation = parameters.get("inflation_layer", {})
        radius = _float(
            inflation.get("inflation_radius"), f"{label} inflation_radius", errors
        )
        if radius is not None and not 0.30 <= radius <= 0.35:
            errors.append(
                f"{label} inflation_radius must be in [0.30, 0.35] m, got {radius:.3f}"
            )

    if not _is_false(planner.get("allow_unknown")):
        errors.append("planner GridBased.allow_unknown must be false indoors")
    vx_max = _float(controller.get("vx_max"), "FollowPath.vx_max", errors)
    if vx_max is not None and vx_max > 0.35:
        errors.append(f"FollowPath.vx_max exceeds 0.35 m/s: {vx_max:.3f}")
    max_velocity = optimizer.get("max_velocity")
    if not isinstance(max_velocity, list) or not max_velocity:
        errors.append("velocity_optimizer.max_velocity is missing")
    else:
        optimizer_vx = _float(
            max_velocity[0], "velocity_optimizer.max_velocity[0]", errors
        )
        if optimizer_vx is not None and optimizer_vx > 0.35:
            errors.append(
                f"velocity_optimizer max forward speed exceeds 0.35 m/s: {optimizer_vx:.3f}"
            )

    map_name = str(map_server.get("yaml_filename", ""))
    if Path(map_name).name != "house_map.yaml" or "yard" in map_name.lower():
        errors.append(
            "map_server must reference the dedicated "
            ".run/go2w_house/map/house_map.yaml"
        )
    return errors


def _validate_map(paths: HousePaths, errors: list[str]) -> None:
    map_config = _read_yaml(paths.map_yaml, errors)
    if not isinstance(map_config, Mapping):
        if not errors:
            errors.append("House map YAML is not a mapping")
        return
    image_value = map_config.get("image")
    if not isinstance(image_value, str) or Path(image_value).name != paths.map_pgm.name:
        errors.append(f"House map YAML must reference {paths.map_pgm.name}")
        return
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = paths.map_yaml.parent / image_path
    if image_path.resolve() != paths.map_pgm.resolve():
        errors.append(
            f"House map YAML image {image_path} does not match "
            f"GO2W_HOUSE_MAP_PGM={paths.map_pgm}"
        )
        return
    if not image_path.is_file():
        errors.append(f"House occupancy image is missing: {image_path}")
        return
    try:
        prefix = image_path.read_bytes()[:2]
    except OSError as exc:
        errors.append(f"cannot read House occupancy image: {exc}")
        return
    if prefix not in {b"P2", b"P5"} or image_path.stat().st_size < 64:
        errors.append(f"House occupancy image is not a valid nonempty PGM: {image_path}")
    resolution = _float(map_config.get("resolution"), "House map resolution", errors)
    if resolution is not None and not 0.02 <= resolution <= 0.10:
        errors.append(f"House map resolution is unreasonable: {resolution:.3f}")


def validate_house_world(
    root: Path = ROOT,
    *,
    map_source: str = "known_map",
    expected_scene_sha: str = HOUSE_SCENE_SHA256,
    expected_pak_sha: Mapping[str, str] = HOUSE_PAK_SHA256,
) -> list[str]:
    if map_source not in {"known_map", "online_slam"}:
        return [f"unsupported HouseWorld map source: {map_source}"]
    paths = HousePaths.from_root(root)
    errors: list[str] = []
    required_files = (
        paths.physics_scene,
        paths.ue_scene,
        paths.robot_xml,
        paths.ue_config,
    )
    if map_source == "known_map":
        required_files += (paths.map_yaml, paths.nav_config)
    for path in required_files:
        if not path.is_file():
            errors.append(f"required HouseWorld file is missing: {path}")

    physics_root = _read_xml(paths.physics_scene, errors) if paths.physics_scene.is_file() else None
    ue_root = _read_xml(paths.ue_scene, errors) if paths.ue_scene.is_file() else None
    for label, scene_root in (("MuJoCo", physics_root), ("UE", ue_root)):
        if scene_root is None:
            continue
        include = scene_root.find("include")
        if include is None or include.get("file") != "go2w.xml":
            errors.append(f"{label} House scene is not bound to go2w.xml")
        supported = [
            geom
            for geom in scene_root.findall(".//worldbody//geom")
            if geom.get("type") in {"box", "cylinder"}
        ]
        types = {geom.get("type") for geom in supported}
        if len(supported) < 90 or types != {"box", "cylinder"}:
            errors.append(
                f"{label} House collision model is incomplete: "
                f"geometries={len(supported)} types={sorted(item for item in types if item)}"
            )

    if paths.physics_scene.is_file() and paths.ue_scene.is_file():
        physics_sha = sha256_file(paths.physics_scene)
        ue_sha = sha256_file(paths.ue_scene)
        if physics_sha != ue_sha:
            errors.append(
                f"House MuJoCo/UE scene SHA mismatch: {physics_sha} != {ue_sha}"
            )
        if expected_scene_sha and physics_sha != expected_scene_sha:
            errors.append(
                f"House scene SHA is not the approved package value: {physics_sha}"
            )

    robot_root = _read_xml(paths.robot_xml, errors) if paths.robot_xml.is_file() else None
    if robot_root is not None:
        required_joints = {
            f"{leg}_{joint}_joint"
            for leg in ("FL", "FR", "RL", "RR")
            for joint in ("hip", "thigh", "calf", "wheel")
        }
        joint_names = {joint.get("name") for joint in robot_root.findall(".//joint")}
        missing_joints = sorted(required_joints - joint_names)
        if missing_joints:
            errors.append(f"Go2-W joint contract is incomplete: {missing_joints}")
        actuators = robot_root.find("actuator")
        if actuators is None or len(list(actuators)) != 16:
            errors.append("Go2-W must expose exactly 16 actuators")
        sensor_names = {sensor.get("name") for sensor in robot_root.findall("sensor/*")}
        required_sensors = {"imu_quat", "imu_gyro", "imu_acc", "frame_pos", "frame_vel"}
        missing_sensors = sorted(required_sensors - sensor_names)
        if missing_sensors:
            errors.append(f"Go2-W state sensors are incomplete: {missing_sensors}")
        lidar_site = robot_root.find(".//site[@name='livox_imu']")
        if lidar_site is None or lidar_site.get("pos") != "0.13011 0.02329 0.17598":
            errors.append("Go2-W livox_imu offset does not match base_link -> lidar TF")

    if paths.ue_config.is_file():
        try:
            robot_config = json.loads(paths.ue_config.read_text(encoding="utf-8"))["robot"]
        except (OSError, KeyError, json.JSONDecodeError) as exc:
            errors.append(f"invalid UE robot config: {exc}")
        else:
            if robot_config.get("robot_type") != "go2w":
                errors.append("UE robot_type must be go2w")
            if robot_config.get("state_port") != 25001 or robot_config.get("cmd_port") != 25002:
                errors.append("UE House runtime must use state/cmd ports 25001/25002")
            lidar = robot_config.get("sensors", {}).get("lidar", {})
            if lidar.get("topic") != "/livox/lidar_raw":
                errors.append("UE House LiDAR must publish only /livox/lidar_raw")

    for filename, expected_hash in expected_pak_sha.items():
        path = paths.paks_dir / filename
        if not path.is_file():
            errors.append(f"HouseWorld UE chunk is missing: {path}")
            continue
        if path.stat().st_size == 0:
            errors.append(f"HouseWorld UE chunk is empty: {path}")
            continue
        actual_hash = sha256_file(path)
        if expected_hash and actual_hash != expected_hash:
            errors.append(f"HouseWorld UE chunk SHA mismatch for {filename}: {actual_hash}")

    if map_source == "known_map":
        if paths.map_yaml.is_file():
            _validate_map(paths, errors)
        if paths.nav_config.is_file():
            errors.extend(validate_navigation_config(paths.nav_config))
    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument(
        "--map-source",
        choices=("known_map", "online_slam"),
        default=os.environ.get("GO2W_HOUSE_MAP_SOURCE", "online_slam"),
        help="validate static-map/Nav2 artifacts or the online SLAM runtime contract",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    errors = validate_house_world(args.root, map_source=args.map_source)
    if errors:
        for error in errors:
            print(f"[HOUSE_WORLD_FAIL] {error}", file=sys.stderr)
        return 1
    if args.map_source == "online_slam":
        runtime_contract = "scan=/scan slam=slam_toolbox nav2=disabled"
    else:
        runtime_contract = "global_obstacles=enabled speed_cap=0.35"
    print(
        "[HOUSE_WORLD_OK] robot=go2w joints=16 actuators=16 "
        f"world=HouseWorld chunk=17 map_source={args.map_source} "
        f"lidar=/livox/lidar {runtime_contract}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
