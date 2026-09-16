#!/usr/bin/env python3
"""Run a physical Nezha2 indoor multi-level traversal demo.

The scene is a single MuJoCo indoor model containing a living-room entry,
stairs/landing, and an inclined hallway.  Four reset-based segments are used
so every terrain transition begins from a measured safe pose:

1. LiDAR-only local avoidance through the entry obstacle;
2. stair ascent;
3. stair descent from the same landing;
4. uphill ramp traversal.

Resets are explicit in ``summary.json``; this script never presents the four
segments as a fabricated continuous route.  The policy remains the original
``policy_zyl_0115_2.pt``.  Only the entry segment supplies a point-cloud A*
planner; the terrain segments receive their declared velocity reference.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

import cv2
import mujoco
import numpy as np
import torch

import nezha2_closed_loop_navigation_evidence as core


SCENE = core.SCENE_DIR / "scene_terrain_home_multilevel.xml"
ENTRY_SCENE = core.SCENE_DIR / "scene_terrain_home_entry_avoidance.xml"
POLICY = core.MODEL_DIR / "policy_zyl_0115_2.pt"
FRAME_RATE = 10
CAPTURE_PERIOD_S = 1.0 / FRAME_RATE
RENDER_OPTION = mujoco.MjvOption()
# Camera frames retain only the same groups used by the physical LiDAR
# contract (terrain/robot/obstacles).  Group-3 wall dressing is deliberately
# excluded so an exterior free camera never hides the robot behind a wall.
RENDER_OPTION.geomgroup[:] = (1, 1, 1, 0, 0, 0)


@dataclass(frozen=True)
class Phase:
    name: str
    start_xyz: tuple[float, float, float]
    goal_y: float
    command_vx: float
    timeout_s: float
    local_avoidance: bool = False


PHASES = (
    Phase("entry_obstacle_avoidance", (0.0, 0.00, 0.47), 5.35, 0.0, 36.0, True),
    Phase("stairs_up", (0.0, 0.00, 0.47), 4.45, 0.62, 22.0),
    Phase("stairs_down", (0.0, 4.42, 0.985), 0.12, -0.35, 24.0),
    Phase("uphill_ramp", (0.0, 5.05, 0.985), 9.55, 0.62, 20.0),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=15)
    parser.add_argument("--phase", choices=("all", *(phase.name for phase in PHASES)), default="all")
    return parser.parse_args()


def phase_camera(phase: Phase) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    lookat = {
        "entry_obstacle_avoidance": (0.0, 2.8, 0.45),
        "stairs_up": (0.0, 2.4, 0.55),
        "stairs_down": (0.0, 2.4, 0.55),
        "uphill_ramp": (0.0, 7.5, 1.10),
    }[phase.name]
    camera.lookat[:] = lookat
    camera.distance = 6.2
    camera.azimuth = -118.0
    camera.elevation = -28.0
    return camera


def controller_ids(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, list[int], int]:
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in core.JOINTS]
    qadr = np.asarray([model.jnt_qposadr[joint] for joint in joints])
    vadr = np.asarray([model.jnt_dofadr[joint] for joint in joints])
    actuators = [int(np.where(model.actuator_trnid[:, 0] == joint)[0][0]) for joint in joints]
    gyro = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
    return qadr, vadr, actuators, int(model.sensor_adr[gyro])


def policy_torque(
    policy: torch.jit.ScriptModule,
    data: mujoco.MjData,
    qadr: np.ndarray,
    vadr: np.ndarray,
    gyro_adr: int,
    history: np.ndarray,
    action: np.ndarray,
    command: np.ndarray,
    update_policy: bool,
) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(data.qpos[qadr], dtype=np.float32)
    dq = np.asarray(data.qvel[vadr], dtype=np.float32)
    relative_q = q - core.DEFAULT
    relative_q[~core.LEG_MASK] = 0.0
    observation = np.concatenate(
        (
            np.asarray(data.sensordata[gyro_adr : gyro_adr + 3], dtype=np.float32) * 0.25,
            core.gravity_from_quaternion(np.asarray(data.qpos[3:7], dtype=np.float32)) * 0.5,
            command * core.COMMAND_SCALE,
            relative_q,
            dq * 0.05,
            action,
        )
    ).astype(np.float32)
    if data.time < 2.0:
        phase = np.clip(data.time / 2.0, 0.0, 1.0)
        phase = phase * phase * (3.0 - 2.0 * phase)
        target = q + phase * (core.DEFAULT - q)
        wheel_velocity = np.zeros(16, dtype=np.float32)
    else:
        if update_policy:
            action = policy(
                torch.from_numpy(observation[None]), torch.from_numpy(history.reshape(1, -1))
            ).cpu().numpy()[0].astype(np.float32)
            action = np.clip(action, -100.0, 100.0)
            history[:-1] = history[1:]
            history[-1] = observation
        target = core.DEFAULT.copy()
        target[core.LEG_MASK] += action[core.LEG_MASK] * core.ACTION_SCALE[core.LEG_MASK]
        wheel_velocity = np.zeros(16, dtype=np.float32)
        wheel_velocity[~core.LEG_MASK] = action[~core.LEG_MASK] * core.ACTION_SCALE[~core.LEG_MASK]
    torque = np.clip(
        core.KP * (target - q) + core.KD * (wheel_velocity - dq),
        -core.TORQUE_LIMIT,
        core.TORQUE_LIMIT,
    )
    return torque, action


def render_frame(renderer: mujoco.Renderer, data: mujoco.MjData, camera: mujoco.MjvCamera) -> np.ndarray:
    renderer.update_scene(data, camera=camera, scene_option=RENDER_OPTION)
    return cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)


def phase_complete(phase: Phase, data: mujoco.MjData) -> bool:
    return data.qpos[1] >= phase.goal_y if phase.command_vx >= 0 else data.qpos[1] <= phase.goal_y


def obstacle_contact(model: mujoco.MjModel, data: mujoco.MjData) -> bool:
    obstacle_ids = {
        index
        for index in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or "").startswith("obstacle_entry")
    }
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.geom1 in obstacle_ids or contact.geom2 in obstacle_ids:
            other = contact.geom2 if contact.geom1 in obstacle_ids else contact.geom1
            if other >= 0 and model.geom_bodyid[other] != 0:
                return True
    return False


def capture_avoidance_evidence(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    phase: Phase,
    phase_dir: Path,
    plan: core.LocalPlan,
) -> dict[str, object]:
    camera = phase_camera(phase)
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    from PIL import Image

    with mujoco.Renderer(model, height=900, width=1600) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=RENDER_OPTION)
        Image.fromarray(renderer.render()).save(phase_dir / "simulation.png")
    scan = core.lidar_scan(model, data, core.DIAGNOSTIC_AZIMUTH_COUNT, core.DIAGNOSTIC_ELEVATION_DEG)
    base_position, base_rotation = core.base_frame(model, data)
    points_base = core.points_in_base_frame(scan.points_world, base_position, base_rotation)
    # Reuse the established full-return plotting code; it renders every valid
    # diagnostic return and obtains extents from the data itself.
    core.plot_full_point_cloud(points_base, "obstacles", float(data.time), phase_dir / "pointcloud_full.png")
    core.plot_planner(plan, float(data.time), phase_dir / "local_plan.png")
    core.serialize_capture(
        phase_dir / "capture.json", "home_entry_obstacle_avoidance", data, scan, points_base,
        "first LiDAR local-plan update with lateral heading", plan,
    )
    return {
        "time_s": float(data.time),
        "diagnostic_raw_rays": int(len(scan.distance_m)),
        "diagnostic_valid_returns": int(np.count_nonzero(scan.valid)),
        "planner_path_found": bool(plan.path_found),
        "heading_offset_rad": float(plan.heading_offset_rad),
    }


def run_phase(policy: torch.jit.ScriptModule, phase: Phase, output_dir: Path) -> dict[str, object]:
    scene = ENTRY_SCENE if phase.local_avoidance else SCENE
    model = mujoco.MjModel.from_xml_path(str(scene))
    # The stock MJCF offscreen buffer is 640 px wide.  Allocate before the
    # renderer is created so the recorded physics frames retain wheel detail.
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 540
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    data.qpos[:3] = phase.start_xyz
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    qadr, vadr, actuators, gyro_adr = controller_ids(model)
    period = max(1, round(0.02 / model.opt.timestep))
    history = np.zeros((5, 57), dtype=np.float32)
    action = np.zeros(16, dtype=np.float32)
    command = np.zeros(3, dtype=np.float32)
    next_replan = 2.0
    previous_target: np.ndarray | None = None
    preferred_sign = 0
    last_plan: core.LocalPlan | None = None
    planner_updates: list[dict[str, object]] = []
    trajectory: list[list[float]] = []
    evidence: dict[str, object] | None = None
    collision = False
    phase_dir = output_dir / phase.name
    phase_dir.mkdir(parents=True, exist_ok=True)
    video = cv2.VideoWriter(
        str(phase_dir / "mujoco_physics.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), FRAME_RATE, (960, 540)
    )
    if not video.isOpened():
        raise RuntimeError(f"cannot create video for {phase.name}")
    camera = phase_camera(phase)
    next_frame = 0.0
    try:
        with mujoco.Renderer(model, height=540, width=960) as renderer:
            for step in range(round(phase.timeout_s / model.opt.timestep)):
                if data.time >= next_frame:
                    video.write(render_frame(renderer, data, camera))
                    next_frame += CAPTURE_PERIOD_S
                if data.time < 2.0:
                    command[:] = 0.0
                elif phase.local_avoidance:
                    # A velocity reference is held between 5 Hz LiDAR plans.
                    # Clearing it on non-replan physics ticks would make the
                    # robot receive only 2 ms command impulses, not local
                    # obstacle avoidance.
                    if data.time + 1e-9 >= next_replan:
                        scan = core.lidar_scan(model, data, core.PLANNER_AZIMUTH_COUNT, core.PLANNER_ELEVATION_DEG)
                        last_plan = core.planner_from_scan(
                            model, data, scan, np.array([0.0, phase.goal_y]), preferred_sign, previous_target
                        )
                        command = last_plan.command.copy()
                        previous_target = last_plan.target_world_xy.copy()
                        if preferred_sign == 0 and abs(last_plan.heading_offset_rad) >= 0.08:
                            preferred_sign = 1 if last_plan.heading_offset_rad > 0 else -1
                        planner_updates.append(
                            {
                                "time_s": float(data.time),
                                "path_found": bool(last_plan.path_found),
                                "command_vx_vy_wz": [float(value) for value in command],
                                "heading_offset_rad": float(last_plan.heading_offset_rad),
                                "obstacle_return_count": int(len(last_plan.obstacle_points_body)),
                            }
                        )
                        if evidence is None and last_plan.path_found and abs(last_plan.heading_offset_rad) >= 0.08:
                            evidence = capture_avoidance_evidence(model, data, phase, phase_dir, last_plan)
                        next_replan += core.PLANNER_PERIOD_S
                else:
                    command[:] = (phase.command_vx, 0.0, 0.0)
                torque, action = policy_torque(
                    policy, data, qadr, vadr, gyro_adr, history, action, command, step % period == 0
                )
                data.ctrl[:] = 0.0
                for index, actuator in enumerate(actuators):
                    data.ctrl[actuator] = float(torque[index])
                mujoco.mj_step(model, data)
                if step % max(1, round(0.05 / model.opt.timestep)) == 0:
                    trajectory.append([float(data.time), *map(float, data.qpos[:3])])
                collision = collision or obstacle_contact(model, data)
                if data.qpos[2] < 0.14 or not np.isfinite(data.qpos).all() or phase_complete(phase, data):
                    break
    finally:
        video.release()
    if phase.local_avoidance and evidence is None and last_plan is not None:
        evidence = capture_avoidance_evidence(model, data, phase, phase_dir, last_plan)
    success = bool(phase_complete(phase, data) and data.qpos[2] > 0.25 and not collision)
    np.save(phase_dir / "trajectory_txyz.npy", np.asarray(trajectory, dtype=np.float64))
    (phase_dir / "planner_updates.json").write_text(json.dumps(planner_updates, indent=2) + "\n", encoding="utf-8")
    result = {
        "phase": asdict(phase),
        "scene": str(scene),
        "success": success,
        "collision_with_entry_obstacle": collision if phase.local_avoidance else None,
        "time_s": float(data.time),
        "final_xyz": [float(value) for value in data.qpos[:3]],
        "trajectory_samples": len(trajectory),
        "planner_updates": len(planner_updates),
        "evidence": evidence,
        "policy": "policy_zyl_0115_2.pt (unmodified)",
    }
    (phase_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))
    return result


def main() -> int:
    args = parse_args()
    if not SCENE.is_file() or not ENTRY_SCENE.is_file() or not POLICY.is_file():
        raise SystemExit("indoor scene or validated Nezha2 policy is missing")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    torch.set_grad_enabled(False)
    policy = torch.jit.load(str(POLICY), map_location="cpu").eval()
    phases = PHASES if args.phase == "all" else tuple(phase for phase in PHASES if phase.name == args.phase)
    results = [run_phase(policy, phase, args.output_dir) for phase in phases]
    summary = {
        "scenes": {"multi_level": str(SCENE), "entry_avoidance": str(ENTRY_SCENE)},
        "scene_type": "coordinated physical indoor scene package (living-room entry plus multi-level hall)",
        "execution": "four reset-based physical traversals; the entry and terrain rooms are separate physical areas, and resets are explicit rather than a continuous-route claim",
        "local_avoidance": "entry phase: MuJoCo LiDAR -> inflated occupancy -> A* -> velocity reference",
        "terrain": "upstairs, downstairs and 10 degree uphill ramp use the unmodified Nezha2 terrain policy",
        "all_success": all(bool(result["success"]) for result in results),
        "results": results,
    }
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return 0 if summary["all_success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
