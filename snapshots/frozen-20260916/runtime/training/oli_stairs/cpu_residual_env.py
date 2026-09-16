"""Exact CPU MuJoCo environment for residual OLI stair-policy training."""

from __future__ import annotations

from dataclasses import replace
import os
import json

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault(
    "XLA_FLAGS",
    "--xla_cpu_multi_thread_eigen=false intra_op_parallelism_threads=1",
)

from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import jax.numpy as jp
import mujoco
import numpy as np
import onnx
from onnx import numpy_helper

from training.oli_stairs.constants import (
    ACTION_SIZE,
    AUXILIARY_JOINT_NAMES,
    JOINT_NAMES,
    OBSERVATION_FRAME_SIZE,
    OBSERVATION_HISTORY,
    OBSERVATION_SIZE,
    StairSpec,
    load_control_parameters,
)
from training.oli_stairs.generate_mjcf import (
    PHYSICAL_ACTUATOR_JOINT_NAMES,
    PHYSICAL_TORQUE_LIMITS,
)
from training.oli_stairs.parallel_kinematics import (
    left_ankle_pr_to_ab,
    pr_torque_to_ab,
    right_ankle_pr_to_ab,
    waist_pr_to_ab,
)
from training.oli_stairs.cpu_residual_policy import NumpyResidualActor


class NumpyActor:
    """Evaluate the deployed OLI MLP without an ONNX runtime dependency."""

    def __init__(self, path: Path) -> None:
        model = onnx.load(str(path.expanduser().resolve()))
        tensors = {
            item.name: numpy_helper.to_array(item) for item in model.graph.initializer
        }
        self.layers = tuple(
            (tensors[f"actor.{index}.weight"], tensors[f"actor.{index}.bias"])
            for index in (0, 2, 4, 6)
        )
        bounded = any(node.name == "action_scale" for node in model.graph.node)
        self.output_scale = (
            float(np.asarray(tensors["clip_scale"]).reshape(())) if bounded else None
        )

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float32)[None]
        for weight, bias in self.layers[:-1]:
            value = value @ weight.T + bias
            value = np.where(value > 0.0, value, np.expm1(value))
        value = value[0].astype(np.float64)[None]
        weight, bias = self.layers[-1]
        value = value @ weight.T + bias
        if self.output_scale is not None:
            value = self.output_scale * value / (1.0 + np.abs(value))
        return value[0].astype(np.float64)


def _quaternion_from_two_vectors(
    source: np.ndarray, target: np.ndarray
) -> np.ndarray:
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    dot = float(np.dot(source, target))
    if dot < -0.999999:
        axis = np.cross(source, np.asarray([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1.0e-8:
            axis = np.cross(source, np.asarray([0.0, 1.0, 0.0]))
        axis /= np.linalg.norm(axis)
        return np.asarray([0.0, axis[0], axis[1], axis[2]])
    quaternion = np.asarray([1.0 + dot, *np.cross(source, target)])
    return quaternion / np.linalg.norm(quaternion)


class OliCpuResidualEnv(gym.Env[np.ndarray, np.ndarray]):
    """Train a bounded residual around a fixed, deployment-compatible actor."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        model_path: Path,
        policy_path: Path,
        spec: StairSpec,
        *,
        residual_scale: float = 0.50,
        episode_length: int = 800,
        speed: float = 0.30,
        lateral_speed: float = 0.05,
        yaw_rate: float = 0.0,
        yaw_rate_start_step: int = 0,
        history_lateral_speed: float = 0.08,
        history_reset_x: float = 0.99,
        lateral_hold_kp: float = 0.0,
        lateral_hold_limit: float = 0.05,
        heading_hold_kp: float = 0.0,
        heading_hold_limit: float = 0.15,
        heading_hold_start_x: float = -np.inf,
        heading_hold_start_step: int = 0,
        residual_start_x: float = -np.inf,
        fixed_residual_policy: Path | None = None,
        reset_trace_path: Path | tuple[Path, ...] | None = None,
        reset_trace_start_step: int = 0,
        reset_trace_end_step: int = 2**31 - 1,
        reset_trace_start_x: float = -np.inf,
        reset_trace_end_x: float = np.inf,
        reset_trace_probability: float = 0.0,
        reset_x_noise: float = 0.0,
        reset_y_noise: float = 0.0,
        reset_yaw_noise: float = 0.0,
        control_profile: str = "walk",
        stand_gain_blend: float = 0.0,
        policy_action_scale: float = 1.0,
        target_step: int | None = None,
        target_shaping_scale: float = 0.0,
        balance_assist_scale: float = 0.0,
        balance_assist_decay_start_step: int = 2**31 - 1,
        balance_assist_decay_end_step: int = 2**31 - 1,
        torque_limit_scale: float = 1.0,
        swing_collision_mask_leg: str = "none",
        swing_collision_mask_start_step: int = 2**31 - 1,
        swing_collision_mask_end_step: int = 2**31 - 1,
        balance_assist_target_x_offset: float = 0.0,
        balance_assist_target_z_offset: float = 0.0,
    ) -> None:
        super().__init__()
        self.spec = spec
        self.residual_scale = float(residual_scale)
        self.episode_length = int(episode_length)
        self.speed = float(speed)
        self.lateral_speed = float(lateral_speed)
        self.yaw_rate = float(yaw_rate)
        self.yaw_rate_start_step = int(yaw_rate_start_step)
        self.history_lateral_speed = float(history_lateral_speed)
        self.history_reset_x = float(history_reset_x)
        self.lateral_hold_kp = float(lateral_hold_kp)
        self.lateral_hold_limit = float(lateral_hold_limit)
        self.heading_hold_kp = float(heading_hold_kp)
        self.heading_hold_limit = float(heading_hold_limit)
        self.heading_hold_start_x = float(heading_hold_start_x)
        self.heading_hold_start_step = int(heading_hold_start_step)
        self.residual_start_x = float(residual_start_x)
        self.fixed_residual_actor = (
            NumpyResidualActor(fixed_residual_policy)
            if fixed_residual_policy is not None
            else None
        )
        self.reset_trace_probability = float(reset_trace_probability)
        self.reset_x_noise = float(reset_x_noise)
        self.reset_y_noise = float(reset_y_noise)
        self.reset_yaw_noise = float(reset_yaw_noise)
        self.policy_action_scale = float(policy_action_scale)
        self.target_step = (
            int(target_step) if target_step is not None else self.spec.step_count
        )
        if not 1 <= self.target_step <= self.spec.step_count:
            raise ValueError("target_step must be within the staircase")
        self.target_shaping_scale = float(target_shaping_scale)
        self.balance_assist_scale = float(balance_assist_scale)
        self.balance_assist_decay_start_step = int(
            balance_assist_decay_start_step
        )
        self.balance_assist_decay_end_step = int(balance_assist_decay_end_step)
        self.balance_assist_target_base = np.zeros(3, dtype=np.float64)
        self.balance_assist_target_yaw = 0.0
        self.torque_limit_scale = float(torque_limit_scale)
        self.swing_collision_mask_leg = swing_collision_mask_leg
        self.swing_collision_mask_start_step = int(
            swing_collision_mask_start_step
        )
        self.swing_collision_mask_end_step = int(swing_collision_mask_end_step)
        self.balance_assist_target_x_offset = float(
            balance_assist_target_x_offset
        )
        self.balance_assist_target_z_offset = float(
            balance_assist_target_z_offset
        )

        self.model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
        if self.torque_limit_scale != 1.0:
            self.model.actuator_ctrlrange[:] *= self.torque_limit_scale
        self.control_dt = 0.01
        self.data = mujoco.MjData(self.model)
        self.last_control_contact_counts: dict[tuple[int, int], int] = {}
        self.actor = NumpyActor(policy_path)
        self.control = load_control_parameters(profile=control_profile)
        if not 0.0 <= stand_gain_blend <= 1.0:
            raise ValueError("stand_gain_blend must be in [0, 1]")
        if stand_gain_blend > 0.0:
            stand_control = load_control_parameters(profile="stand")
            self.control = replace(
                self.control,
                kp=(1.0 - stand_gain_blend) * self.control.kp
                + stand_gain_blend * stand_control.kp,
                kd=(1.0 - stand_gain_blend) * self.control.kd
                + stand_gain_blend * stand_control.kd,
            )

        self.qpos_ids = np.asarray(
            [self.model.joint(name).qposadr[0] for name in JOINT_NAMES],
            dtype=np.int32,
        )
        self.qvel_ids = np.asarray(
            [self.model.joint(name).dofadr[0] for name in JOINT_NAMES],
            dtype=np.int32,
        )
        self.physical_qpos_ids = np.asarray(
            [
                self.model.joint(name).qposadr[0]
                for name in PHYSICAL_ACTUATOR_JOINT_NAMES
            ],
            dtype=np.int32,
        )
        self.foot_site_ids = np.asarray(
            [
                self.model.site("left_foot_training_site").id,
                self.model.site("right_foot_training_site").id,
            ],
            dtype=np.int32,
        )
        self.foot_geom_ids = {
            "left": np.asarray(
                [
                    self.model.geom(name).id
                    for name in (
                        "left_foot",
                        "contact_foot_heel_L",
                        "contact_foot_center_L",
                        "contact_foot_tip_L",
                    )
                ],
                dtype=np.int32,
            ),
            "right": np.asarray(
                [
                    self.model.geom(name).id
                    for name in (
                        "right_foot",
                        "contact_foot_heel_R",
                        "contact_foot_center_R",
                        "contact_foot_tip_R",
                    )
                ],
                dtype=np.int32,
            ),
        }
        self.original_geom_contype = self.model.geom_contype.copy()
        self.original_geom_conaffinity = self.model.geom_conaffinity.copy()
        self.gyro_sensor = self._sensor_slice("Body_Gyro")
        self.soft_lowers, self.soft_uppers = self._soft_joint_limits(0.98)
        self.motor_limits = self.control.torque_limit.copy()
        self.motor_limits[np.asarray((4, 5, 10, 11, 13, 14))] = np.asarray(
            PHYSICAL_TORQUE_LIMITS
        )
        self.init_qpos = self._make_initial_qpos()
        self.reset_snapshots: tuple[dict, ...] = ()
        if reset_trace_path is not None:
            trace_paths = (
                reset_trace_path
                if isinstance(reset_trace_path, tuple)
                else (reset_trace_path,)
            )
            records = []
            for trace_path in trace_paths:
                records.extend(
                    json.loads(
                        trace_path.expanduser().resolve().read_text(encoding="utf-8")
                    )
                )
            self.reset_snapshots = tuple(
                record
                for record in records
                if reset_trace_start_step <= int(record["step"]) <= reset_trace_end_step
                and "state_qpos" in record
                and reset_trace_start_x
                <= float(record["state_qpos"][0])
                <= reset_trace_end_x
            )
            if not self.reset_snapshots:
                raise ValueError(
                    f"No reset snapshots in steps {reset_trace_start_step}.."
                    f"{reset_trace_end_step}: {trace_paths}"
                )

        self.observation_space = spaces.Box(
            -self.control.observation_clip,
            self.control.observation_clip,
            shape=(OBSERVATION_SIZE,),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(
            -1.0, 1.0, shape=(ACTION_SIZE,), dtype=np.float32
        )
        initial_yaw_rate = self.yaw_rate if self.yaw_rate_start_step <= 0 else 0.0
        self.command = np.asarray(
            [self.speed, self.lateral_speed, initial_yaw_rate], dtype=np.float64
        )
        self.observation = np.zeros(OBSERVATION_SIZE, dtype=np.float32)
        self.last_action = np.zeros(ACTION_SIZE, dtype=np.float64)
        self.previous_residual = np.zeros(ACTION_SIZE, dtype=np.float64)
        self.steps = 0
        self.max_support_height = 0.0
        self.previous_support_height = 0.0
        self.previous_x = 0.0
        self.previous_z = 0.0
        self.history_reset_done = False
        self.top_hold_steps = 0
        self.top_support_run = 0
        self.longest_top_support = 0
        self.longest_success_hold = 0
        self.previous_target_potential = 0.0

    @property
    def target_height(self) -> float:
        return self.target_step * self.spec.step_height

    @property
    def target_riser_x(self) -> float:
        return self.spec.start_x + (self.target_step - 1) * self.spec.step_depth

    def _sensor_slice(self, name: str) -> np.ndarray:
        sensor = self.model.sensor(name)
        start = self.model.sensor_adr[sensor.id]
        size = self.model.sensor_dim[sensor.id]
        return np.arange(start, start + size, dtype=np.int32)

    def _soft_joint_limits(self, scale: float) -> tuple[np.ndarray, np.ndarray]:
        lower = np.asarray(
            [self.model.joint(name).range[0] for name in JOINT_NAMES],
            dtype=np.float32,
        )
        upper = np.asarray(
            [self.model.joint(name).range[1] for name in JOINT_NAMES],
            dtype=np.float32,
        )
        center = (lower + upper) * 0.5
        radius = (upper - lower) * 0.5
        return center - radius * scale, center + radius * scale

    def _close_parallel_constraints(self, qpos: np.ndarray) -> np.ndarray:
        data = mujoco.MjData(self.model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self.model, data)
        for equality_index in range(self.model.neq):
            body1 = int(self.model.eq_obj1id[equality_index])
            body2 = int(self.model.eq_obj2id[equality_index])
            parent = int(self.model.body_parentid[body2])
            body1_rotation = data.xmat[body1].reshape(3, 3)
            parent_rotation = data.xmat[parent].reshape(3, 3)
            target_world = (
                data.xpos[body1]
                + body1_rotation @ self.model.eq_data[equality_index, :3]
            )
            desired_parent = parent_rotation.T @ (target_world - data.xpos[body2])
            source = self.model.eq_data[equality_index, 3:6]
            quaternion = _quaternion_from_two_vectors(source, desired_parent)
            joint_index = int(self.model.body_jntadr[body2])
            address = int(self.model.jnt_qposadr[joint_index])
            qpos[address : address + 4] = quaternion
        return qpos

    def _make_initial_qpos(self) -> np.ndarray:
        qpos = np.array(self.model.qpos0, dtype=np.float64, copy=True)
        qpos[2] = 1.0
        qpos[3] = 1.0
        qpos[self.qpos_ids] = self.control.default_angle
        for indices, mapping in (
            ((4, 5), left_ankle_pr_to_ab),
            ((10, 11), right_ankle_pr_to_ab),
            ((13, 14), waist_pr_to_ab),
        ):
            pair = np.asarray(indices)
            qpos[self.physical_qpos_ids[pair]] = np.asarray(
                mapping(jp.asarray(self.control.default_angle[pair]))
            )
        qpos = self._close_parallel_constraints(qpos)
        data = mujoco.MjData(self.model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self.model, data)
        sole_height = min(data.site_xpos[index, 2] for index in self.foot_site_ids)
        qpos[2] += 0.004 - sole_height
        return np.asarray(qpos, dtype=np.float32)

    @staticmethod
    def _projected_gravity(quaternion: np.ndarray) -> np.ndarray:
        w, x, y, z = quaternion
        rotation = np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        return rotation.T @ np.asarray([0.0, 0.0, -1.0])

    def _frame(self, last_action: np.ndarray) -> np.ndarray:
        return np.clip(
            np.concatenate(
                [
                    self.data.sensordata[self.gyro_sensor]
                    * self.control.gyro_scale,
                    self._projected_gravity(self.data.qpos[3:7]),
                    self.command,
                    (self.data.qpos[self.qpos_ids] - self.control.default_angle)
                    * self.control.dof_pos_scale,
                    self.data.qvel[self.qvel_ids] * self.control.dof_vel_scale,
                    last_action,
                ]
            ),
            -self.control.observation_clip,
            self.control.observation_clip,
        ).astype(np.float32)

    def _terrain_height(self, x: float) -> float:
        height = 0.0
        ramp_length = self.spec.riser_ramp_length
        for step in range(1, self.spec.step_count + 1):
            riser_x = self.spec.start_x + (step - 1) * self.spec.step_depth
            if ramp_length > 0.0 and x < riser_x:
                ramp_start = riser_x - ramp_length
                if x <= ramp_start:
                    return height
                phase = (x - ramp_start) / ramp_length
                return height + phase * self.spec.step_height
            if x < riser_x:
                return height
            height = step * self.spec.step_height
        return height

    def _support_height(self) -> float:
        supported = [0.0]
        for foot in self.data.site_xpos[self.foot_site_ids]:
            height = self._terrain_height(float(foot[0]))
            if foot[2] - height < 0.035:
                supported.append(height)
        return max(supported)

    def _set_balance_assist_target(self) -> None:
        self.balance_assist_target_base = self.data.qpos[:3].copy()
        self.balance_assist_target_base[0] += self.balance_assist_target_x_offset
        self.balance_assist_target_base[2] += self.balance_assist_target_z_offset
        quaternion = self.data.qpos[3:7]
        self.balance_assist_target_yaw = np.arctan2(
            2.0
            * (
                quaternion[0] * quaternion[3]
                + quaternion[1] * quaternion[2]
            ),
            1.0 - 2.0 * (quaternion[2] ** 2 + quaternion[3] ** 2),
        )

    def _target_potential(self) -> float:
        exposed_tread_depth = max(
            self.spec.step_depth - self.spec.riser_ramp_length,
            0.01,
        )
        target_x = self.target_riser_x + 0.5 * exposed_tread_depth
        feet = self.data.site_xpos[self.foot_site_ids]
        foot_scores = -np.sqrt(
            np.square(feet[:, 0] - target_x)
            + 2.25 * np.square(feet[:, 2] - self.target_height)
        )
        base_target = self.target_height + 0.78
        return float(np.max(foot_scores) - 0.30 * abs(self.data.qpos[2] - base_target))

    def _apply_control(self, action: np.ndarray) -> None:
        self.model.geom_contype[:] = self.original_geom_contype
        self.model.geom_conaffinity[:] = self.original_geom_conaffinity
        if (
            self.swing_collision_mask_leg in self.foot_geom_ids
            and self.swing_collision_mask_start_step
            <= self.steps
            < self.swing_collision_mask_end_step
        ):
            masked = self.foot_geom_ids[self.swing_collision_mask_leg]
            self.model.geom_contype[masked] = 0
            self.model.geom_conaffinity[masked] = 0
        target = np.clip(
            self.control.default_angle + action * self.control.action_scale,
            self.soft_lowers,
            self.soft_uppers,
        )
        position = self.data.qpos[self.qpos_ids]
        velocity = self.data.qvel[self.qvel_ids]
        torque = np.clip(
            self.control.kp * (target - position) - self.control.kd * velocity,
            -self.torque_limit_scale * self.control.torque_limit,
            self.torque_limit_scale * self.control.torque_limit,
        ).astype(np.float64)
        for indices, mapping in (
            ((4, 5), left_ankle_pr_to_ab),
            ((10, 11), right_ankle_pr_to_ab),
            ((13, 14), waist_pr_to_ab),
        ):
            pair = np.asarray(indices)
            torque[pair] = np.asarray(
                pr_torque_to_ab(
                    mapping,
                    jp.asarray(position[pair]),
                    jp.asarray(torque[pair]),
                )
            )
        self.data.ctrl[:] = np.clip(
            torque,
            -self.torque_limit_scale * self.motor_limits,
            self.torque_limit_scale * self.motor_limits,
        )
        contact_counts: dict[tuple[int, int], int] = {}
        for _ in range(round(self.control_dt / self.model.opt.timestep)):
            self.data.qfrc_applied[:] = 0.0
            if self.balance_assist_scale > 0.0:
                if self.steps <= self.balance_assist_decay_start_step:
                    decay = 1.0
                elif self.steps >= self.balance_assist_decay_end_step:
                    decay = 0.0
                else:
                    decay = 1.0 - (
                        self.steps - self.balance_assist_decay_start_step
                    ) / max(
                        self.balance_assist_decay_end_step
                        - self.balance_assist_decay_start_step,
                        1,
                    )
                scale = self.balance_assist_scale * decay
                position_error = self.balance_assist_target_base - self.data.qpos[:3]
                force = np.asarray((2200.0, 3200.0, 5200.0)) * position_error
                force -= np.asarray((350.0, 500.0, 700.0)) * self.data.qvel[:3]
                quaternion = self.data.qpos[3:7]
                w, x, y, z = quaternion
                roll = np.arctan2(
                    2.0 * (w * x + y * z),
                    1.0 - 2.0 * (x * x + y * y),
                )
                pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
                yaw = np.arctan2(
                    2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z),
                )
                yaw_error = np.arctan2(
                    np.sin(self.balance_assist_target_yaw - yaw),
                    np.cos(self.balance_assist_target_yaw - yaw),
                )
                torque_assist = -np.asarray((1600.0, 1600.0, 500.0)) * np.asarray(
                    (roll, pitch, -yaw_error)
                )
                torque_assist -= np.asarray((180.0, 180.0, 80.0)) * self.data.qvel[3:6]
                self.data.qfrc_applied[:3] = scale * np.clip(
                    force, -12000.0, 12000.0
                )
                self.data.qfrc_applied[3:6] = scale * np.clip(
                    torque_assist, -4000.0, 4000.0
                )
            mujoco.mj_step(self.model, self.data)
            substep_pairs = {
                tuple(sorted((int(contact.geom1), int(contact.geom2))))
                for contact in self.data.contact[: self.data.ncon]
            }
            for pair in substep_pairs:
                contact_counts[pair] = contact_counts.get(pair, 0) + 1
        self.last_control_contact_counts = contact_counts

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        options = options or {}
        use_snapshot = bool(self.reset_snapshots) and (
            bool(options.get("use_snapshot", False))
            or self.np_random.random() < self.reset_trace_probability
        )
        if use_snapshot:
            snapshot_index = options.get("snapshot_index")
            if snapshot_index is None:
                snapshot_index = int(self.np_random.integers(len(self.reset_snapshots)))
            snapshot = self.reset_snapshots[int(snapshot_index)]
            self.data = mujoco.MjData(self.model)
            self.data.qpos[:] = np.asarray(snapshot["state_qpos"], dtype=np.float64)
            self.data.qvel[:] = np.asarray(snapshot["state_qvel"], dtype=np.float64)
            self.data.time = float(snapshot["state_time"])
            mujoco.mj_forward(self.model, self.data)
            self.data.qacc_warmstart[:] = np.asarray(
                snapshot["state_qacc_warmstart"], dtype=np.float64
            )
            self._set_balance_assist_target()
            initial_yaw_rate = (
                self.yaw_rate if self.yaw_rate_start_step <= 0 else 0.0
            )
            self.command[:] = [self.speed, self.lateral_speed, initial_yaw_rate]
            self.observation = np.asarray(snapshot["observation"], dtype=np.float32)
            self.last_action = self.observation[71:102].astype(np.float64, copy=True)
            self.previous_residual = np.zeros(ACTION_SIZE, dtype=np.float64)
            self.last_control_contact_counts = {}
            self.steps = 0
            support_height = self._support_height()
            self.max_support_height = support_height
            self.previous_support_height = support_height
            self.previous_x = float(self.data.qpos[0])
            self.previous_z = float(self.data.qpos[2])
            self.history_reset_done = self.data.qpos[0] >= self.history_reset_x
            self.top_hold_steps = 0
            self.top_support_run = 0
            self.longest_top_support = 0
            self.longest_success_hold = 0
            self.previous_target_potential = self._target_potential()
            return self.observation.copy(), self._info(False)

        x_offset = float(options.get("x_offset", self.np_random.uniform(
            -self.reset_x_noise, self.reset_x_noise
        )))
        y_offset = float(options.get("y_offset", self.np_random.uniform(
            -self.reset_y_noise, self.reset_y_noise
        )))
        yaw_offset = float(options.get("yaw_offset", self.np_random.uniform(
            -self.reset_yaw_noise, self.reset_yaw_noise
        )))
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = self.init_qpos
        self.data.qpos[0] = self.spec.start_x - 0.72 + x_offset
        self.data.qpos[1] = y_offset
        self.data.qpos[3:7] = [
            np.cos(yaw_offset * 0.5),
            0.0,
            0.0,
            np.sin(yaw_offset * 0.5),
        ]
        mujoco.mj_forward(self.model, self.data)
        self._set_balance_assist_target()

        initial_yaw_rate = self.yaw_rate if self.yaw_rate_start_step <= 0 else 0.0
        self.command[:] = [self.speed, self.history_lateral_speed, initial_yaw_rate]
        self.last_action = np.zeros(ACTION_SIZE, dtype=np.float64)
        history_frame = self._frame(self.last_action)
        self.observation = np.tile(history_frame, OBSERVATION_HISTORY)
        self.command[1] = self.lateral_speed
        self.previous_residual = np.zeros(ACTION_SIZE, dtype=np.float64)
        self.last_control_contact_counts = {}
        self.steps = 0
        self.max_support_height = 0.0
        self.previous_support_height = 0.0
        self.previous_x = float(self.data.qpos[0])
        self.previous_z = float(self.data.qpos[2])
        self.history_reset_done = False
        self.top_hold_steps = 0
        self.top_support_run = 0
        self.longest_top_support = 0
        self.longest_success_hold = 0
        self.previous_target_potential = self._target_potential()
        return self.observation.copy(), self._info(False)

    def _info(self, success: bool) -> dict:
        return {
            "is_success": success,
            "support_height": self._support_height(),
            "max_support_height": self.max_support_height,
            "base_x": float(self.data.qpos[0]),
            "base_z": float(self.data.qpos[2]),
            "longest_top_support": self.longest_top_support,
            "success_hold_steps": self.top_hold_steps,
            "longest_success_hold": self.longest_success_hold,
        }

    def step(
        self, residual_action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict]:
        yaw = np.arctan2(
            2.0
            * (
                self.data.qpos[3] * self.data.qpos[6]
                + self.data.qpos[4] * self.data.qpos[5]
            ),
            1.0
            - 2.0
            * (self.data.qpos[5] ** 2 + self.data.qpos[6] ** 2),
        )
        self.command[0] = self.speed
        self.command[1] = np.clip(
            self.lateral_speed - self.lateral_hold_kp * self.data.qpos[1],
            -self.lateral_hold_limit,
            self.lateral_hold_limit,
        )
        heading_feedback = (
            np.clip(
                -self.heading_hold_kp * yaw,
                -self.heading_hold_limit,
                self.heading_hold_limit,
            )
            if self.steps >= self.heading_hold_start_step
            and self.data.qpos[0] >= self.heading_hold_start_x
            else 0.0
        )
        commanded_yaw_rate = (
            self.yaw_rate if self.steps >= self.yaw_rate_start_step else 0.0
        )
        self.command[2] = np.clip(
            commanded_yaw_rate + heading_feedback,
            -self.heading_hold_limit,
            self.heading_hold_limit,
        )
        feedback_residual = (
            np.clip(np.asarray(residual_action, dtype=np.float64), -1.0, 1.0)
            * self.residual_scale
        )
        residual = (
            feedback_residual
            if self.data.qpos[0] >= self.residual_start_x
            else np.zeros(ACTION_SIZE, dtype=np.float64)
        )
        if (
            self.fixed_residual_actor is not None
            and self.data.qpos[0] >= self.fixed_residual_actor.residual_start_x
        ):
            residual = residual + (
                self.fixed_residual_actor.residual_scale
                * self.fixed_residual_actor(self.observation)
            )
        base_action = self.policy_action_scale * self.actor(self.observation)
        action = np.clip(base_action + residual, -100.0, 100.0)
        self._apply_control(action)
        self.steps += 1

        frame = self._frame(action)
        self.observation = np.roll(self.observation, OBSERVATION_FRAME_SIZE)
        self.observation[:OBSERVATION_FRAME_SIZE] = frame
        if not self.history_reset_done and self.data.qpos[0] >= self.history_reset_x:
            self.last_action = np.zeros(ACTION_SIZE, dtype=np.float64)
            self.observation = np.tile(self._frame(self.last_action), OBSERVATION_HISTORY)
            self.history_reset_done = True
        else:
            self.last_action = action

        support_height = self._support_height()
        target_potential = self._target_potential()
        target_potential_gain = target_potential - self.previous_target_potential
        support_gain = max(0.0, support_height - self.max_support_height)
        self.max_support_height = max(self.max_support_height, support_height)
        dx = float(self.data.qpos[0] - self.previous_x)
        dz = float(self.data.qpos[2] - self.previous_z)
        body_up_z = 1.0 - 2.0 * (
            self.data.qpos[4] ** 2 + self.data.qpos[5] ** 2
        )
        top_supported = support_height >= self.target_height - 1.0e-6
        high_enough = self.data.qpos[2] > self.target_height + 0.72
        far_enough = self.data.qpos[0] > self.target_riser_x + 0.20
        self.top_support_run = (
            self.top_support_run + 1 if top_supported and high_enough else 0
        )
        self.longest_top_support = max(
            self.longest_top_support, self.top_support_run
        )
        self.top_hold_steps = (
            self.top_hold_steps + 1
            if top_supported and high_enough and far_enough
            else 0
        )
        self.longest_success_hold = max(
            self.longest_success_hold, self.top_hold_steps
        )
        success = self.top_hold_steps >= 20
        fallen = self.data.qpos[2] < support_height + 0.62 or body_up_z < 0.45
        invalid = not np.all(np.isfinite(self.data.qpos))
        terminated = bool(success or fallen or invalid)
        truncated = self.steps >= self.episode_length

        residual_rate = np.mean(np.square(residual - self.previous_residual))
        local_curriculum = self.target_step < self.spec.step_count
        target_foot_x = float(np.max(self.data.site_xpos[self.foot_site_ids, 0]))
        target_approach = np.clip(
            (target_foot_x - self.target_riser_x) / self.spec.step_depth,
            -1.0,
            1.0,
        )
        target_base_height = np.clip(
            (self.data.qpos[2] - (self.target_height + 0.62)) / 0.10,
            -1.0,
            1.0,
        )
        reward = (
            8.0 * np.clip(dx / 0.01, -0.5, 0.5)
            + (300.0 if local_curriculum else 80.0)
            * support_gain
            / max(self.spec.step_height, 1.0e-6)
            + 4.0 * np.clip(dz / 0.01, -0.5, 0.5)
            + 0.5 * max(body_up_z, 0.0)
            + 2.0 * support_height / max(self.spec.step_height, 1.0e-6)
            + ((24.0 if local_curriculum else 4.0) if top_supported else 0.0)
            + (6.0 * target_approach if local_curriculum else 0.0)
            + (4.0 * target_base_height if local_curriculum else 0.0)
            + (6.0 if local_curriculum and high_enough else 0.0)
            - 2.0 * abs(float(self.data.qpos[1]))
            - 0.02 * float(np.mean(np.square(residual)))
            - 0.01 * float(residual_rate)
            + self.target_shaping_scale * target_potential_gain
        )
        if success:
            reward += 2000.0 if local_curriculum else 1000.0
        elif fallen or invalid:
            reward -= 1000.0 if local_curriculum else 200.0

        self.previous_x = float(self.data.qpos[0])
        self.previous_z = float(self.data.qpos[2])
        self.previous_support_height = support_height
        self.previous_residual = residual
        self.previous_target_potential = target_potential
        return (
            self.observation.copy(),
            float(reward),
            terminated,
            truncated,
            self._info(success),
        )
