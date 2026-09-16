#!/usr/bin/env python3
"""Run the unmodified Nezha2 RL-SAR TorchScript policy in MuJoCo.

This benchmark intentionally loads a ``policy_zyl_0115_*.pt`` file directly.
It does not use the deterministic VAE export or a hand-authored gait.  The
observation, history depth, action scaling, and PD gains are read from the
original rl_sar_nezha2_0115 configuration contract.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np
import torch
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/rl_sar/models/nezha/robot_lab"
SCENE = ROOT / "matrix/src/robot_mujoco/zsibot_robots/nezha2/scene_terrain_yard.xml"
JOINTS = [
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "RL", "RR")
    for joint in ("hip", "thigh", "calf", "foot")
]
DEFAULT = np.array(
    [0.0, 0.925, -1.85, 0.0, 0.0, 0.925, -1.85, 0.0,
     0.0, -0.925, 1.85, 0.0, 0.0, -0.925, 1.85, 0.0],
    dtype=np.float32,
)
KP = np.array([150.0, 150.0, 150.0, 0.0] * 4, dtype=np.float32)
KD = np.array([4.0, 4.0, 4.0, 1.2] * 4, dtype=np.float32)
ACTION_SCALE = np.array([0.25, 0.25, 0.25, 10.0] * 4, dtype=np.float32)
TORQUE_LIMIT = np.array([150.0, 150.0, 150.0, 10.0] * 4, dtype=np.float32)
COMMAND_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
LEG_MASK = np.arange(16) % 4 != 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", default="policy_zyl_0115_1.pt")
    parser.add_argument("--seed", type=int, default=15)
    parser.add_argument("--vx", type=float, default=0.65)
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--stand-seconds", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def gravity_from_quaternion(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = map(float, quat)
    return np.array(
        [-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(w * w - x * x - y * y + z * z)],
        dtype=np.float32,
    )


def make_camera(data: mujoco.MjData) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    # Track the physical trunk, rather than a fixed map point.  This keeps the
    # recorded final frame visible after the robot has completed the stairs.
    camera.lookat[:] = [float(data.qpos[0]), float(data.qpos[1]), max(0.45, float(data.qpos[2]))]
    camera.distance = 4.8
    camera.azimuth = -118.0
    camera.elevation = -18.0
    return camera


def render(model: mujoco.MjModel, data: mujoco.MjData, destination: Path) -> None:
    with mujoco.Renderer(model, height=900, width=1600) as renderer:
        renderer.update_scene(data, camera=make_camera(data))
        Image.fromarray(renderer.render()).save(destination)


def euler_from_quaternion(quat: np.ndarray) -> tuple[float, float]:
    w, x, y, z = map(float, quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    return roll, pitch


def main() -> int:
    args = parse_args()
    if args.policy.endswith("_deterministic.pt"):
        raise ValueError("A deterministic export is not valid for this raw-policy benchmark")
    policy_path = MODEL_DIR / args.policy
    if not policy_path.is_file():
        raise FileNotFoundError(policy_path)
    if not SCENE.is_file():
        raise FileNotFoundError(SCENE)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed)
    torch.set_grad_enabled(False)
    policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINTS]
    qadr = np.array([model.jnt_qposadr[joint] for joint in joint_ids])
    vadr = np.array([model.jnt_dofadr[joint] for joint in joint_ids])
    actuator_ids = [int(np.where(model.actuator_trnid[:, 0] == joint)[0][0]) for joint in joint_ids]
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
    gyro_adr = int(model.sensor_adr[gyro_id])

    render(model, data, args.output_dir / "start.png")
    history = np.zeros((5, 57), dtype=np.float32)
    previous_action = np.zeros(16, dtype=np.float32)
    command = np.array([args.vx, 0.0, 0.0], dtype=np.float32)
    policy_period = max(1, round(0.02 / model.opt.timestep))
    report_period = max(1, round(1.0 / model.opt.timestep))
    total_steps = round(args.duration / model.opt.timestep)
    trajectory: list[dict[str, float]] = []
    final_reason = "timeout"

    for step in range(total_steps):
        q = np.asarray(data.qpos[qadr], dtype=np.float32)
        dq = np.asarray(data.qvel[vadr], dtype=np.float32)
        gravity = gravity_from_quaternion(np.asarray(data.qpos[3:7], dtype=np.float32))
        gyro = np.asarray(data.sensordata[gyro_adr : gyro_adr + 3], dtype=np.float32)
        relative_q = q - DEFAULT
        relative_q[~LEG_MASK] = 0.0
        observation = np.concatenate(
            [gyro * 0.25, gravity * 0.5, command * COMMAND_SCALE, relative_q, dq * 0.05, previous_action]
        ).astype(np.float32)

        if data.time < args.stand_seconds:
            phase = np.clip(data.time / args.stand_seconds, 0.0, 1.0)
            phase = phase * phase * (3.0 - 2.0 * phase)
            target = q + phase * (DEFAULT - q)
            wheel_velocity = np.zeros(16, dtype=np.float32)
        else:
            if step % policy_period == 0:
                with torch.no_grad():
                    previous_action = policy(
                        torch.from_numpy(observation[None]),
                        torch.from_numpy(history.reshape(1, -1)),
                    ).cpu().numpy()[0].astype(np.float32)
                previous_action = np.clip(previous_action, -100.0, 100.0)
                history[:-1] = history[1:]
                history[-1] = observation
            target = DEFAULT.copy()
            target[LEG_MASK] += previous_action[LEG_MASK] * ACTION_SCALE[LEG_MASK]
            wheel_velocity = np.zeros(16, dtype=np.float32)
            wheel_velocity[~LEG_MASK] = previous_action[~LEG_MASK] * ACTION_SCALE[~LEG_MASK]

        torque = np.clip(KP * (target - q) + KD * (wheel_velocity - dq), -TORQUE_LIMIT, TORQUE_LIMIT)
        data.ctrl[:] = 0.0
        for index, actuator_id in enumerate(actuator_ids):
            data.ctrl[actuator_id] = float(torque[index])
        mujoco.mj_step(model, data)

        if step % report_period == 0:
            roll, pitch = euler_from_quaternion(np.asarray(data.qpos[3:7]))
            state = {
                "t": round(float(data.time), 3), "x": round(float(data.qpos[0]), 4),
                "y": round(float(data.qpos[1]), 4), "z": round(float(data.qpos[2]), 4),
                "roll": round(roll, 4), "pitch": round(pitch, 4),
            }
            trajectory.append(state)
            print("state", " ".join(f"{key}={value}" for key, value in state.items()))
        if not np.isfinite(data.qpos).all() or data.qpos[2] < 0.15:
            final_reason = "safety"
            break
        if data.qpos[1] > 4.7 and data.qpos[2] > 0.8:
            final_reason = "stairs_success"
            break

    render(model, data, args.output_dir / "final.png")
    roll, pitch = euler_from_quaternion(np.asarray(data.qpos[3:7]))
    result = {
        "benchmark": "raw_nezha2_rl_sar_policy",
        "source_policy": str(policy_path),
        "source_scene": str(SCENE),
        "seed": args.seed,
        "command_vx_mps": args.vx,
        "final_reason": final_reason,
        "final_state": {
            "t": float(data.time), "x": float(data.qpos[0]), "y": float(data.qpos[1]),
            "z": float(data.qpos[2]), "roll": roll, "pitch": pitch,
        },
        "trajectory_1hz": trajectory,
    }
    (args.output_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print("RESULT", json.dumps(result["final_state"]), f"reason={final_reason}")
    return 0 if final_reason == "stairs_success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
