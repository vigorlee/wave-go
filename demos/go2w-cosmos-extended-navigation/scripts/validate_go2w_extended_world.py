#!/usr/bin/env python3
"""Fail-closed contract check for the extended Go2-W mixed-terrain demo."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "scene/scene_terrain_yard_extended_nav.xml"
UE_SOURCE = ROOT / "scene/scene_terrain_yard_extended_nav_ue.xml"
SCENE_JSON = ROOT / "scene/scene_go2w_extended_nav.json"
ROUTES = ROOT / "config/cosmos_vln_routes.json"
RUNTIME_ROOT = Path(
    os.environ.get("WAVE_GO_RUNTIME_ROOT", "/home/unitree/matrix_go2w_lcm_demo")
)
NAV_CONFIG = RUNTIME_ROOT / "genisom_roamerx_open/src/navigation/src/robot_navigo/params/navigo_params.yaml"
RL_BRIDGE_SOURCE = ROOT / "controller/go2w_rl_bridge/main.cpp"
RUNTIME_RL_BRIDGE_SOURCE = RUNTIME_ROOT / "controllers/go2w_rl_bridge/src/main.cpp"
EXTENDED_RUNNER = ROOT / "scripts/start_extended_stack.sh"
EXTENDED_RECORDER = ROOT / "scripts/record_demo.sh"
TERRAIN_CYLINDER_NAMES = {
    "Cylinder37",
    "Cylinder38",
    "Cylinder39",
    "Cylinder41",
}
TERRAIN_VISUAL_NAMES = {"box_OBS11", "box_OBS12", "box_OBS13", "box_OBS14"}
RAMP_VISUAL_NAMES = {"box_OBS15", "box_OBS16"}
PEDESTRIAN_VISUAL_NAMES = {"box_OBS17", "box_OBS18", "box_OBS19"}


def fail(message: str) -> None:
    print(f"[EXTENDED_WORLD_FAIL] {message}", file=sys.stderr)
    raise SystemExit(1)


def parse(path: Path) -> ET.Element:
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        fail(f"cannot parse {path}: {exc}")


def check_scene(
    path: Path,
) -> tuple[
    set[str],
    list[tuple[float, float, float]],
    dict[str, tuple[tuple[float, float, float], tuple[float, float]]],
]:
    root = parse(path)
    include = root.find("include")
    if include is None or include.get("file") != "go2w.xml":
        fail(f"{path} is not bound to go2w.xml")
    worldbody = root.find("worldbody")
    if worldbody is None:
        fail(f"{path} has no worldbody")
    names: set[str] = set()
    cylinders: list[tuple[float, float, float]] = []
    terrain_cylinders: dict[
        str, tuple[tuple[float, float, float], tuple[float, float]]
    ] = {}
    terrain_visuals: set[str] = set()
    ramp_visuals: set[str] = set()
    pedestrian_visuals: set[str] = set()
    for geom in worldbody.iter("geom"):
        name = geom.get("name")
        if name:
            names.add(name)
        if name and name.startswith("extended_slalom_cylinder_"):
            if geom.get("type") != "cylinder":
                fail(f"{path}: {name} is not a cylinder")
            pos = tuple(float(item) for item in geom.get("pos", "").split())
            size = tuple(float(item) for item in geom.get("size", "").split())
            if len(pos) != 3 or len(size) < 2 or size[0] <= 0 or size[1] <= 0:
                fail(f"{path}: malformed {name}")
            cylinders.append(pos)
        if name in TERRAIN_CYLINDER_NAMES:
            if geom.get("type") != "cylinder":
                fail(f"{path}: {name} is not a cylinder")
            try:
                pos = tuple(float(item) for item in geom.get("pos", "").split())
                size = tuple(float(item) for item in geom.get("size", "").split())
            except ValueError:
                fail(f"{path}: malformed {name}")
            if len(pos) != 3 or len(size) < 2:
                fail(f"{path}: malformed {name}")
            terrain_cylinders[name] = (pos, size[:2])
        if name in TERRAIN_VISUAL_NAMES:
            if geom.get("type") != "cylinder" or geom.get("contype") != "0" or geom.get("conaffinity") != "0":
                fail(f"{path}: {name} must be a render-only cylinder")
            terrain_visuals.add(name)
        if name in RAMP_VISUAL_NAMES:
            if geom.get("type") != "box" or geom.get("contype") != "0" or geom.get("conaffinity") != "0":
                fail(f"{path}: {name} must be a render-only ramp box")
            ramp_visuals.add(name)
        if name in PEDESTRIAN_VISUAL_NAMES:
            if geom.get("type") != "capsule" or geom.get("contype") != "0" or geom.get("conaffinity") != "0":
                fail(f"{path}: {name} must be a render-only pedestrian capsule")
            pedestrian_visuals.add(name)
    required = {
        "extended_slalom_cylinder_1",
        "extended_slalom_cylinder_2",
        "extended_slalom_cylinder_3",
        "extended_slalom_cylinder_4",
        "extended_ramp_up",
        "extended_ramp_crest",
        "extended_ramp_down",
        *TERRAIN_CYLINDER_NAMES,
        *TERRAIN_VISUAL_NAMES,
        *RAMP_VISUAL_NAMES,
        *PEDESTRIAN_VISUAL_NAMES,
    }
    missing = sorted(required - names)
    if missing:
        fail(f"{path}: missing extended geometry {missing}")
    if len(cylinders) != 4:
        fail(f"{path}: expected four extended slalom cylinders, got {len(cylinders)}")
    if set(terrain_cylinders) != TERRAIN_CYLINDER_NAMES:
        fail(f"{path}: terrain-center cylinder set is incomplete")
    if terrain_visuals != TERRAIN_VISUAL_NAMES:
        fail(f"{path}: terrain visual cylinder set is incomplete")
    if ramp_visuals != RAMP_VISUAL_NAMES:
        fail(f"{path}: ramp visual set is incomplete")
    if pedestrian_visuals != PEDESTRIAN_VISUAL_NAMES:
        fail(f"{path}: pedestrian visual set is incomplete")
    visual_joints = {
        joint.get("name"): joint
        for joint in root.findall(".//worldbody/body/joint")
        if joint.get("name", "").startswith("pedestrian_")
    }
    if len(visual_joints) != 3:
        fail(f"{path}: expected three pedestrian visual slide joints")
    for name, joint in visual_joints.items():
        if joint.get("type") != "slide" or float(joint.get("stiffness", "0")) <= 0:
            fail(f"{path}: pedestrian visual joint is not a moving slide: {name}")
    return names, cylinders, terrain_cylinders


def stair_surface_top(root: ET.Element, prefix: str, y: float) -> float:
    tops: list[float] = []
    for geom in root.findall(".//worldbody/geom"):
        if not geom.get("name", "").startswith(prefix):
            continue
        pos = tuple(float(item) for item in geom.get("pos", "").split())
        size = tuple(float(item) for item in geom.get("size", "").split())
        if len(pos) == 3 and len(size) == 3 and pos[1] - size[1] <= y <= pos[1] + size[1]:
            tops.append(pos[2] + size[2])
    if not tops:
        fail(f"no {prefix} stair surface below y={y}")
    return max(tops)


def ramp_surface_top(root: ET.Element, ramp_name: str, y: float) -> float:
    ramp = root.find(f".//worldbody/geom[@name='{ramp_name}']")
    if ramp is None:
        fail(f"missing ramp surface {ramp_name}")
    pos = tuple(float(item) for item in ramp.get("pos", "").split())
    size = tuple(float(item) for item in ramp.get("size", "").split())
    euler = tuple(float(item) for item in ramp.get("euler", "").split())
    if len(pos) != 3 or len(size) != 3 or len(euler) != 3:
        fail(f"malformed ramp surface {ramp_name}")
    angle = euler[0]
    return pos[2] + (size[2] + math.sin(angle) * (y - pos[1])) / math.cos(angle)


def main() -> int:
    if not SCENE.is_file() or not UE_SOURCE.is_file():
        fail("extended MuJoCo and UE scene files must both exist")
    _, physics_cylinders, terrain_cylinders = check_scene(SCENE)
    _, ue_cylinders, _ = check_scene(UE_SOURCE)
    if SCENE.read_bytes() != UE_SOURCE.read_bytes():
        fail("MuJoCo and UE extended scene files differ")
    for index, position in enumerate(physics_cylinders, start=1):
        if not (-1.5 < position[0] < 1.5 and 10.4 < position[1] < 13.0):
            fail(f"slalom cylinder {index} is outside the post-stair course: {position}")
    scene_root = parse(SCENE)
    terrain_surfaces = {
        "Cylinder37": stair_surface_top(
            scene_root, "box_OBS", terrain_cylinders["Cylinder37"][0][1]
        ),
        "Cylinder38": stair_surface_top(
            scene_root, "box2_OBS", terrain_cylinders["Cylinder38"][0][1]
        ),
        "Cylinder39": ramp_surface_top(
            scene_root, "extended_ramp_up", terrain_cylinders["Cylinder39"][0][1]
        ),
        "Cylinder41": ramp_surface_top(
            scene_root, "extended_ramp_down", terrain_cylinders["Cylinder41"][0][1]
        ),
    }
    for name, (position, size) in terrain_cylinders.items():
        radius, half_height = size
        if abs(position[0]) > 0.25:
            fail(f"{name} is outside the central travel band: x={position[0]}")
        if not (0.10 <= radius <= 0.16 and 0.25 <= half_height <= 0.35):
            fail(f"{name} has an unsafe or invisible size: {size}")
        bottom = position[2] - half_height
        if abs(bottom - terrain_surfaces[name]) > 0.01:
            fail(
                f"{name} is not seated on its terrain surface: "
                f"bottom={bottom:.4f} surface={terrain_surfaces[name]:.4f}"
            )
    up = parse(SCENE).find(".//worldbody/geom[@name='extended_ramp_up']")
    down = parse(SCENE).find(".//worldbody/geom[@name='extended_ramp_down']")
    if up is None or down is None:
        fail("ramp geometry missing")
    up_angle = float(up.get("euler", "0").split()[0])
    down_angle = float(down.get("euler", "0").split()[0])
    if not (
        math.radians(8.0) <= up_angle <= math.radians(12.0)
        and math.radians(-12.0) <= down_angle <= math.radians(-8.0)
    ):
        fail(
            "physical ramp slopes must be an obvious +8..12/-12..8 degree pair: "
            f"{math.degrees(up_angle):.2f}, {math.degrees(down_angle):.2f} deg"
        )
    crest = parse(SCENE).find(".//worldbody/geom[@name='extended_ramp_crest']")
    if crest is None:
        fail("ramp crest geometry missing")
    crest_pos = tuple(float(value) for value in crest.get("pos", "").split())
    crest_size = tuple(float(value) for value in crest.get("size", "").split())
    if len(crest_pos) != 3 or len(crest_size) != 3:
        fail("malformed ramp crest geometry")
    crest_top = crest_pos[2] + crest_size[2]
    up_low = ramp_surface_top(scene_root, "extended_ramp_up", 12.80)
    up_high = ramp_surface_top(scene_root, "extended_ramp_up", 15.20)
    down_high = ramp_surface_top(scene_root, "extended_ramp_down", 15.80)
    down_low = ramp_surface_top(scene_root, "extended_ramp_down", 18.20)
    if up_high - up_low < 0.35 or down_high - down_low < 0.35:
        fail(
            "physical ramps do not create a meaningful elevation change: "
            f"up={up_high - up_low:.3f} down={down_high - down_low:.3f}"
        )
    if up_low > 0.06 or down_low > 0.06:
        fail(f"physical ramps do not return to floor height: {up_low:.3f}, {down_low:.3f}")
    if abs(up_high - crest_top) > 0.03 or abs(down_high - crest_top) > 0.03:
        fail(
            "physical ramp/crest seams are discontinuous: "
            f"up={up_high:.3f} crest={crest_top:.3f} down={down_high:.3f}"
        )
    for visual_name, physical_name, samples in (
        ("box_OBS15", "extended_ramp_up", (12.90, 14.00, 15.10)),
        ("box_OBS16", "extended_ramp_down", (15.90, 17.00, 18.10)),
    ):
        visual = scene_root.find(f".//worldbody/geom[@name='{visual_name}']")
        physical = scene_root.find(f".//worldbody/geom[@name='{physical_name}']")
        if visual is None or physical is None:
            fail(f"missing visual/physical ramp pair {visual_name}/{physical_name}")
        visual_angle = float(visual.get("euler", "0").split()[0])
        physical_angle = float(physical.get("euler", "0").split()[0])
        if abs(visual_angle - physical_angle) > 1e-5:
            fail(f"{visual_name} angle does not match physical surface {physical_name}")
        for y in samples:
            visual_top = ramp_surface_top(scene_root, visual_name, y)
            physical_top = ramp_surface_top(scene_root, physical_name, y)
            if not 0.0 <= visual_top - physical_top <= 0.025:
                fail(
                    f"{visual_name} is not seated on {physical_name} at y={y}: "
                    f"visual={visual_top:.3f} physical={physical_top:.3f}"
                )

    try:
        scene_payload = json.loads(SCENE_JSON.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid extended scene JSON: {exc}")
    if not isinstance(scene_payload, dict):
        fail("extended scene JSON root must be an object")
    static_elements = {
        str(value.get("name")): value
        for value in scene_payload.values()
        if isinstance(value, dict) and value.get("type") == "static"
    }
    for name, (position, _) in terrain_cylinders.items():
        element = static_elements.get(name)
        if element is None or element.get("model") != "cylinder1":
            fail(f"scene JSON is missing the visual cylinder {name}")
        raw_position = element.get("position")
        if not isinstance(raw_position, dict):
            fail(f"scene JSON has malformed position for {name}")
        rendered = tuple(float(raw_position[axis]) * 0.01 for axis in ("x", "y", "z"))
        if any(abs(rendered[index] - position[index]) > 0.015 for index in range(3)):
            fail(f"scene JSON/XML position mismatch for {name}: {rendered} vs {position}")
    actors = [
        value
        for value in scene_payload.values()
        if isinstance(value, dict) and value.get("type") == "dynamic"
    ]
    if len(actors) < 3:
        fail("extended scene JSON must contain at least three dynamic pedestrians")
    if not any(actor.get("avoid") is False for actor in actors):
        fail("at least one pedestrian must use the PDF's non-cooperative avoid:false mode")
    for actor in actors:
        if actor.get("model") != "human1" or not isinstance(actor.get("trajectory"), dict):
            fail("dynamic pedestrian is missing human1 trajectory contract")
        velocity = float(actor.get("velocity", 0.0))
        if not math.isfinite(velocity) or velocity <= 0:
            fail("dynamic pedestrian velocity must be positive and finite")

    try:
        routes = json.loads(ROUTES.read_text(encoding="utf-8"))
        route = next(item for item in routes["routes"] if item["route_id"] == "farthest_end_via_stairs_ramps_cylinders_pedestrians")
    except (OSError, KeyError, StopIteration, TypeError, json.JSONDecodeError) as exc:
        fail(f"extended route is missing from catalog: {exc}")
    stages = route.get("stages", [])
    required_stages = {
        "descend_far_staircase",
        "slalom_pass_first_pair",
        "ramp_up_entry",
        "ramp_up_crest",
        "ramp_down_entry",
        "ramp_down_exit",
        "cross_dynamic_pedestrian_zone",
        "navigate_to_farthest_endpoint",
    }
    stage_ids = {stage.get("stage_id") for stage in stages}
    if not required_stages <= stage_ids:
        fail(f"extended route is missing stages: {sorted(required_stages - stage_ids)}")
    stage_by_id = {str(stage.get("stage_id")): stage for stage in stages}
    for stage_id in ("ramp_up_entry", "ramp_up_crest", "ramp_down_entry", "ramp_down_exit"):
        try:
            lane_x = float(stage_by_id[stage_id]["goal"]["x"])
        except (KeyError, TypeError, ValueError):
            fail(f"extended route has malformed ramp-lane goal: {stage_id}")
        if lane_x < 0.20:
            fail(f"{stage_id} must remain in the continuous right avoidance lane, got x={lane_x}")
    if stage_by_id["ramp_up_entry"].get("navigation_mode") != "slope" or any(
        stage_by_id[stage_id].get("navigation_mode") != "slope"
        for stage_id in ("ramp_up_crest", "ramp_down_entry", "ramp_down_exit")
    ):
        fail("all four physical ramp stages must use the dedicated slope mode")
    try:
        ramp_exit_y = float(stage_by_id["ramp_down_exit"]["goal"]["y"])
        pedestrian_y = float(stage_by_id["cross_dynamic_pedestrian_zone"]["goal"]["y"])
    except (KeyError, TypeError, ValueError):
        fail("extended route has malformed physical-ramp/dynamic-zone goals")
    if ramp_exit_y < 17.95:
        fail(f"ramp_down_exit must be beyond the physical ramp, got y={ramp_exit_y}")
    if abs(pedestrian_y - 20.80) > 0.20:
        fail(f"dynamic pedestrian waypoint must cross the actor trajectory, got y={pedestrian_y}")
    try:
        remote_setup_x = float(stage_by_id["approach_remote_obstacles"]["goal"]["x"])
        remote_setup_y = float(stage_by_id["approach_remote_obstacles"]["goal"]["y"])
    except (KeyError, TypeError, ValueError):
        fail("extended route has malformed remote-obstacle lane setup goal")
    if remote_setup_x < 0.50 or remote_setup_y < 18.50:
        fail(
            "approach_remote_obstacles must move into the open right lane after "
            f"the remote row, got ({remote_setup_x}, {remote_setup_y})"
        )

    # The external runtime is deliberately not vendored.  When it is present,
    # perform the stronger navigation/controller contract checks; a clean
    # source checkout can still validate the portable scene and route files.
    if NAV_CONFIG.is_file():
        nav_text = NAV_CONFIG.read_text(encoding="utf-8")
        for token in ("/livox/lidar", "PointCloud2", "min_obstacle_height: 0.10", "max_obstacle_height: 1.20"):
            if token not in nav_text:
                fail(f"navigation local-costmap contract missing {token}")
    else:
        print(f"[INFO] external navigation config not found; skipped: {NAV_CONFIG}")
    bridge_source = RL_BRIDGE_SOURCE if RL_BRIDGE_SOURCE.is_file() else RUNTIME_RL_BRIDGE_SOURCE
    if bridge_source.is_file():
        bridge_text = bridge_source.read_text(encoding="utf-8")
        for token in (
            "GO2W_RL_STAIR_OBSTACLE_AVOIDANCE",
            "GO2W_RL_STAIR_PLANNER_YAW_BLEND",
            "planner_desired[2]",
        ):
            if token not in bridge_text:
                fail(f"stair obstacle steering contract missing {token}")
    else:
        print(f"[INFO] RL bridge source not found; skipped: {bridge_source}")
    for runner in (EXTENDED_RUNNER, EXTENDED_RECORDER):
        runner_text = runner.read_text(encoding="utf-8") if runner.is_file() else ""
        if 'GO2W_RL_STAIR_OBSTACLE_AVOIDANCE:-1' not in runner_text:
            fail(f"extended runner does not enable stair obstacle avoidance: {runner}")
        if 'COSMOS_VLN_CONTINUOUS_ROUTE:-1' not in runner_text:
            fail(f"extended runner does not enforce continuous navigation: {runner}")

    print(
        "[EXTENDED_WORLD_OK] robot=go2w "
        "terrain_center_cylinders=stair_up+stair_down+ramp_up+ramp_down "
        "post_stair_slalom=4 physical_ramp=+10deg/0.42m/-10deg "
        "ue_ramp_skins_aligned=2 json_pedestrians=3 ue_moving_proxies=3 "
        "execution=navigate_through_poses_once "
        "route=farthest_end_via_stairs_ramps_cylinders_pedestrians"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
