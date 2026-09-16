#!/usr/bin/env python3
"""Run deterministic Go2W motion smoke tests on real MuJoCo map files.

The script does not synthesize illustrations.  Each image is a raw RGB frame
returned by ``mujoco.Renderer`` after time integration of the loaded MJCF.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import mujoco
import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCENE_DIR = ROOT_DIR / "matrix/src/robot_mujoco/zsibot_robots/go2w"
DEFAULT_OUTPUT = ROOT_DIR / "artifacts/go2w_multimap_sim_20260824"


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    scene_file: str
    spawn_xyz: tuple[float, float, float]
    spawn_yaw_rad: float
    settle_s: float
    drive_s: float
    wheel_speed_rad_s: float
    wheel_torque_limit_nm: float
    camera_lookat: tuple[float, float, float]
    camera_distance: float
    camera_azimuth: float
    camera_elevation: float
    expected_axis: str


SCENARIOS = (
    Scenario(
        "stairs",
        "scene_terrain_stairs_rl.xml",
        (0.18, 0.0, 0.65),
        0.0,
        2.0,
        10.0,
        7.0,
        4.0,
        (1.35, 0.0, 0.25),
        3.4,
        135.0,
        -24.0,
        "x",
    ),
    Scenario(
        "slope",
        "scene_terrain_ramp_rl.xml",
        (0.0, 0.35, 0.65),
        math.pi / 2.0,
        2.0,
        14.0,
        3.0,
        8.0,
        (0.0, 1.75, 0.32),
        2.6,
        0.0,
        -24.0,
        "y",
    ),
    Scenario(
        "yard_obstacles",
        "scene_terrain_yard.xml",
        (0.90, 16.45, 0.65),
        math.pi / 2.0,
        2.0,
        8.0,
        4.0,
        4.0,
        (0.05, 18.05, 0.30),
        4.4,
        150.0,
        -25.0,
        "y",
    ),
    Scenario(
        "crowd_corridor",
        "scene_terrain_crowd.xml",
        (3.00, 0.0, 0.65),
        0.0,
        2.0,
        9.0,
        5.0,
        4.0,
        (5.00, 0.0, 0.30),
        4.8,
        137.0,
        -27.0,
        "x",
    ),
)


LEG_TARGETS = {
    f"{leg}_{joint}_joint": value
    for leg in ("FL", "FR", "RL", "RR")
    for joint, value in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.6))
}


def write_ppm(path: Path, rgb: np.ndarray) -> None:
    height, width, channels = rgb.shape
    if channels != 3 or rgb.dtype != np.uint8:
        raise ValueError("renderer output must be uint8 RGB")
    with path.open("wb") as stream:
        stream.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        stream.write(rgb.tobytes())


def quat_from_yaw(yaw_rad: float) -> np.ndarray:
    return np.asarray(
        [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)],
        dtype=float,
    )


def roll_pitch_yaw(quat_wxyz: np.ndarray) -> tuple[float, float, float]:
    matrix = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(matrix, quat_wxyz)
    matrix = matrix.reshape(3, 3)
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return roll, pitch, yaw


def set_initial_state(model: mujoco.MjModel, data: mujoco.MjData, scenario: Scenario) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.qpos[0:3] = scenario.spawn_xyz
    data.qpos[3:7] = quat_from_yaw(scenario.spawn_yaw_rad)
    for joint_name, target in LEG_TARGETS.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = target
    mujoco.mj_forward(model, data)


def apply_controller(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    wheel_speed_rad_s: float,
    wheel_torque_limit_nm: float,
    simulation_time: float,
) -> None:
    for actuator_id in range(model.nu):
        actuator_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
        )
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        qpos_address = int(model.jnt_qposadr[joint_id])
        dof_address = int(model.jnt_dofadr[joint_id])
        if actuator_name and actuator_name.endswith("wheel"):
            command = 0.9 * (wheel_speed_rad_s - data.qvel[dof_address])
            data.ctrl[actuator_id] = float(
                np.clip(command, -wheel_torque_limit_nm, wheel_torque_limit_nm)
            )
        elif joint_name in LEG_TARGETS:
            command = (
                50.0 * (LEG_TARGETS[joint_name] - data.qpos[qpos_address])
                - 1.5 * data.qvel[dof_address]
            )
            low, high = model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = float(np.clip(command, low, high))
        elif actuator_name == "dynamic_crossing_position":
            data.ctrl[actuator_id] = 0.75 * math.sin(1.2 * simulation_time)
        else:
            data.ctrl[actuator_id] = 0.0


def render_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scenario: Scenario,
    output: Path,
) -> None:
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = scenario.camera_lookat
    camera.distance = scenario.camera_distance
    camera.azimuth = scenario.camera_azimuth
    camera.elevation = scenario.camera_elevation
    with mujoco.Renderer(model, height=900, width=1600) as renderer:
        renderer.update_scene(data, camera=camera)
        write_ppm(output, renderer.render())


def run_scenario(scenario: Scenario, output_dir: Path) -> dict[str, object]:
    scene = (SCENE_DIR / scenario.scene_file).resolve()
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    set_initial_state(model, data, scenario)
    start_position = data.qpos[0:3].copy()
    minimum_height = float(data.qpos[2])
    maximum_tilt = 0.0
    maximum_contacts = 0
    finite = True
    total_steps = int(round((scenario.settle_s + scenario.drive_s) / model.opt.timestep))
    settle_steps = int(round(scenario.settle_s / model.opt.timestep))

    render_frame(
        model,
        data,
        scenario,
        output_dir / f"{scenario.scenario_id}_start.ppm",
    )
    for step in range(total_steps):
        target_wheel_speed = 0.0 if step < settle_steps else scenario.wheel_speed_rad_s
        apply_controller(
            model,
            data,
            target_wheel_speed,
            scenario.wheel_torque_limit_nm,
            data.time,
        )
        mujoco.mj_step(model, data)
        roll, pitch, _ = roll_pitch_yaw(data.qpos[3:7])
        minimum_height = min(minimum_height, float(data.qpos[2]))
        maximum_tilt = max(maximum_tilt, abs(roll), abs(pitch))
        maximum_contacts = max(maximum_contacts, int(data.ncon))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            finite = False
            break

    render_frame(
        model,
        data,
        scenario,
        output_dir / f"{scenario.scenario_id}_final.ppm",
    )
    final_position = data.qpos[0:3].copy()
    roll, pitch, yaw = roll_pitch_yaw(data.qpos[3:7])
    axis_index = 0 if scenario.expected_axis == "x" else 1
    signed_progress = float(final_position[axis_index] - start_position[axis_index])
    planar_displacement = float(np.linalg.norm(final_position[:2] - start_position[:2]))
    stable = bool(
        finite
        and final_position[2] > 0.22
        and abs(roll) < 0.75
        and abs(pitch) < 0.75
    )
    motion_observed = abs(signed_progress) >= 0.20
    return {
        "scenario_id": scenario.scenario_id,
        "scene_file": scenario.scene_file,
        "simulated_time_s": round(float(data.time), 4),
        "steps_completed": step + 1,
        "start_position_m": [round(float(value), 4) for value in start_position],
        "final_position_m": [round(float(value), 4) for value in final_position],
        "signed_progress_m": round(signed_progress, 4),
        "planar_displacement_m": round(planar_displacement, 4),
        "final_rpy_rad": [round(value, 4) for value in (roll, pitch, yaw)],
        "minimum_base_height_m": round(minimum_height, 4),
        "maximum_tilt_rad": round(maximum_tilt, 4),
        "maximum_contact_count": maximum_contacts,
        "finite_state": finite,
        "stable_at_end": stable,
        "motion_observed": motion_observed,
        "smoke_result": "PASS" if stable and motion_observed else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for scenario in SCENARIOS:
        print(f"[RUN] {scenario.scenario_id}: {scenario.scene_file}", flush=True)
        result = run_scenario(scenario, args.output_dir)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    report = {
        "schema_version": 1,
        "controller": "joint PD standing + wheel velocity feedback",
        "render_source": "raw mujoco.Renderer RGB frames",
        "scenarios": results,
    }
    report_path = args.output_dir / "multimap_sim_results.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"[OK] report: {report_path}")
    return 0 if all(item["smoke_result"] == "PASS" for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
