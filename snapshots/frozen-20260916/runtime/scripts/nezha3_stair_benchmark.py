#!/usr/bin/env python3
"""Headless Nezha3 policy/MuJoCo stair-contract benchmark."""

import argparse
from pathlib import Path

import mujoco
import numpy as np
import torch


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vx", type=float, default=0.65)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument(
        "--yardworld",
        action="store_true",
        help="Use the official YardWorld stairs and start facing world +Y.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    scene_name = "scene_terrain_yard.xml" if args.yardworld else "scene_terrain_custom.xml"
    xml = root / "matrix/src/robot_mujoco/zsibot_robots/custom" / scene_name
    policy_path = root / "third_party/nezha3_rl_sar/policy/policy_stair.pt"
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    if args.yardworld:
        half_sqrt_two = np.sqrt(0.5)
        data.qpos[3:7] = [half_sqrt_two, 0.0, 0.0, half_sqrt_two]
        mujoco.mj_forward(model, data)
    policy = torch.jit.load(str(policy_path), map_location="cpu").eval()

    joint_names = [
        f"{leg}_{joint}_joint"
        for leg in ("FL", "FR", "RL", "RR")
        for joint in ("hip", "thigh", "calf", "foot")
    ]
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in joint_names
    ]
    qadr = np.array([model.jnt_qposadr[joint_id] for joint_id in joint_ids])
    vadr = np.array([model.jnt_dofadr[joint_id] for joint_id in joint_ids])
    actuator_ids = [
        int(np.where(model.actuator_trnid[:, 0] == joint_id)[0][0])
        for joint_id in joint_ids
    ]

    default = np.array(
        [0.1, 0.925, -1.85, 0, -0.1, 0.925, -1.85, 0,
         0.1, 0.925, -1.85, 0, -0.1, 0.925, -1.85, 0],
        dtype=np.float32,
    )
    kp = np.array([150, 220, 220, 0] * 4, dtype=np.float32)
    kd = np.array([4, 4, 4, 1.2] * 4, dtype=np.float32)
    action_scale = np.array([0.25, 0.25, 0.25, 10] * 4, dtype=np.float32)
    effort = np.array([150, 150, 150, 30] * 4, dtype=np.float32)
    command = np.array([args.vx, 0, 0], dtype=np.float32)
    command_scale = np.array([3.5, 1, 0.7], dtype=np.float32)
    leg_mask = np.arange(16) % 4 != 3
    history = np.zeros((5, 57), dtype=np.float32)
    last_action = np.zeros(16, dtype=np.float32)
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
    gyro_adr = model.sensor_adr[gyro_id]
    policy_period = max(1, round(0.02 / model.opt.timestep))
    report_period = max(1, round(args.report_period / model.opt.timestep))

    for step in range(round(args.duration / model.opt.timestep)):
        q = np.asarray(data.qpos[qadr], dtype=np.float32)
        dq = np.asarray(data.qvel[vadr], dtype=np.float32)
        w, x, y, z = map(float, data.qpos[3:7])
        gravity = np.array(
            [-2 * (x*z - w*y), -2 * (y*z + w*x), -(w*w - x*x - y*y + z*z)],
            dtype=np.float32,
        )
        gyro = np.asarray(data.sensordata[gyro_adr : gyro_adr + 3], dtype=np.float32)
        relative_q = q - default
        relative_q[~leg_mask] = 0
        observation = np.concatenate(
            [gyro * 0.25, gravity * 0.5, command * command_scale,
             relative_q, dq * 0.05, last_action]
        ).astype(np.float32)

        if step % policy_period == 0:
            with torch.no_grad():
                output = policy(
                    torch.from_numpy(observation[None]),
                    torch.from_numpy(history.reshape(1, -1)),
                )
            last_action = np.clip(output.numpy()[0], -100, 100).astype(np.float32)
            history[:-1] = history[1:]
            history[-1] = observation

        target = default.copy()
        target[leg_mask] += last_action[leg_mask] * action_scale[leg_mask]
        target_velocity = np.zeros(16, dtype=np.float32)
        target_velocity[~leg_mask] = last_action[~leg_mask] * action_scale[~leg_mask]
        torque = np.clip(kp * (target - q) + kd * (target_velocity - dq), -effort, effort)
        data.ctrl[:] = 0
        for policy_index, actuator_id in enumerate(actuator_ids):
            data.ctrl[actuator_id] = torque[policy_index]
        mujoco.mj_step(model, data)

        if step % report_period == 0:
            print(f"state t={data.time:.2f} x={data.qpos[0]:.3f} y={data.qpos[1]:.3f} z={data.qpos[2]:.3f}")
        if not np.isfinite(data.qpos).all() or data.qpos[2] < 0.12:
            print(f"STAIR_FAIL reason=safety t={data.time:.2f} z={data.qpos[2]:.3f}")
            return 1
        progress = data.qpos[1] if args.yardworld else data.qpos[0]
        success_height = 0.8 if args.yardworld else 0.75
        if progress > 4.7 and data.qpos[2] > success_height:
            print(
                f"STAIR_SUCCESS t={data.time:.2f} x={data.qpos[0]:.3f} "
                f"y={data.qpos[1]:.3f} z={data.qpos[2]:.3f}"
            )
            return 0

    print(
        f"STAIR_FAIL reason=timeout x={data.qpos[0]:.3f} "
        f"y={data.qpos[1]:.3f} z={data.qpos[2]:.3f}"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
