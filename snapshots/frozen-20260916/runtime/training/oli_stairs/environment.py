"""OLI HU_D04_01 blind stair-climbing environment for MJX."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import jax
import jax.numpy as jp
from ml_collections import config_dict
import mujoco
from mujoco import mjx
from mujoco.mjx._src import math
import numpy as np
import onnx
from onnx import numpy_helper

from mujoco_playground._src import mjx_env
from training.oli_stairs.constants import (
    ACTION_SIZE,
    ARTIFACT_ROOT,
    AUXILIARY_JOINT_NAMES,
    JOINT_NAMES,
    OBSERVATION_FRAME_SIZE,
    OBSERVATION_HISTORY,
    StairSpec,
    load_control_parameters,
)
from training.oli_stairs.generate_mjcf import (
    PHYSICAL_ACTUATOR_JOINT_NAMES,
    PHYSICAL_TORQUE_LIMITS,
    generate_full_training_model,
    generate_training_model,
)
from training.oli_stairs.parallel_kinematics import (
    left_ankle_pr_to_ab,
    pr_torque_to_ab,
    right_ankle_pr_to_ab,
    waist_pr_to_ab,
)


def default_config() -> config_dict.ConfigDict:
    return config_dict.create(
        ctrl_dt=0.01,
        sim_dt=0.002,
        episode_length=1800,
        action_repeat=1,
        action_clip=8.0,
        action_gain=1.0,
        leg_action_gain=-1.0,
        action_filter_alpha=0.10,
        leg_action_filter_alpha=-1.0,
        sagittal_action_filter_alpha=-1.0,
        teacher_policy="",
        teacher_blend=1.0,
        control_profile="walk",
        parallel_control_mode="motor_pd",
        full_parallel_model=True,
        soft_joint_limit=0.98,
        command_speed=[0.25, 0.40],
        command_lateral=[-0.02, 0.02],
        command_yaw=[-0.04, 0.04],
        history_lateral_speed=None,
        heading_hold_kp=0.0,
        heading_hold_max_yaw_rate=0.30,
        heading_hold_target=0.0,
        heading_action_kp=0.0,
        heading_action_limit=0.5,
        stand_probability=0.05,
        stair_approach_distance=0.72,
        progress_window_steps=100000,
        min_window_progress=0.12,
        reset_joint_noise=0.025,
        reset_velocity_noise=0.10,
        reset_x_noise=0.06,
        reset_y_noise=0.08,
        reset_yaw_noise=0.05,
        observation_noise=1.0,
        balance_assist_min=0.0,
        balance_assist_max=0.0,
        balance_assist_upright_kp=900.0,
        balance_assist_upright_kd=120.0,
        balance_assist_heading_kp=300.0,
        balance_assist_heading_kd=60.0,
        balance_assist_lateral_kp=600.0,
        balance_assist_lateral_kd=120.0,
        balance_assist_height_kp=1400.0,
        balance_assist_height_kd=220.0,
        balance_assist_height_max_weight_fraction=0.90,
        noise_scales=config_dict.create(
            gyro=0.04,
            gravity=0.015,
            joint_pos=0.01,
            joint_vel=0.10,
        ),
        reward_config=config_dict.create(
            tracking_sigma=0.002,
            base_height_sigma=0.06,
            upright_sigma=0.10,
            swing_clearance=0.18,
            scales=config_dict.create(
                tracking_lin_vel=10.0,
                tracking_yaw=0.3,
                forward_progress=20.0,
                backward_velocity=-20.0,
                heading_alignment=1.0,
                upward_progress=2.0,
                upright=5.0,
                base_height=5.0,
                alive=1.0,
                stair_foot_progress=20.0,
                stair_foot_target=0.0,
                stair_height=20.0,
                stair_success=500.0,
                lateral_drift=-0.5,
                vertical_velocity=-0.50,
                angular_velocity=-0.05,
                feet_slip=-0.05,
                feet_clearance=-0.3,
                action_rate=-0.01,
                joint_velocity=-0.001,
                torques=-0.01,
                pose=-0.10,
                teacher_deviation=-0.05,
                termination=-500.0,
            ),
        ),
        impl="jax",
        naconmax=256,
        njmax=512,
    )


class OliStairEnv(mjx_env.MjxEnv):
    """Train a terrain-blind policy with the official OLI 510-D interface."""

    def __init__(
        self,
        spec: StairSpec,
        xml_path: Optional[Path] = None,
        config: Optional[config_dict.ConfigDict] = None,
        config_overrides: Optional[dict[str, Any]] = None,
    ) -> None:
        config = config or default_config()
        super().__init__(config, config_overrides)
        self.spec = spec
        self._full_parallel_model = bool(self._config.full_parallel_model)
        if self._config.parallel_control_mode not in (
            "motor_pd",
            "projected_torque",
        ):
            raise ValueError(
                "parallel_control_mode must be 'motor_pd' or 'projected_torque'"
            )

        if xml_path is None:
            stage_name = f"h{int(round(spec.step_height * 100)):02d}"
            model_kind = "full" if self._full_parallel_model else "reduced"
            xml_path = ARTIFACT_ROOT / "models" / f"oli_{model_kind}_{stage_name}.xml"
        self._xml_path = Path(xml_path)
        if self._full_parallel_model:
            generate_full_training_model(self._xml_path, spec)
        else:
            generate_training_model(self._xml_path, spec)

        self._mj_model = mujoco.MjModel.from_xml_path(str(self._xml_path))
        self._mj_model.opt.timestep = self.sim_dt
        self._mjx_model = mjx.put_model(self._mj_model, impl=self._config.impl)

        control = load_control_parameters(profile=str(self._config.control_profile))
        self._action_scale = jp.asarray(control.action_scale)
        self._action_gain = jp.full(
            (ACTION_SIZE,), self._config.action_gain, dtype=jp.float32
        )
        if self._config.leg_action_gain >= 0.0:
            self._action_gain = self._action_gain.at[:12].set(
                self._config.leg_action_gain
            )
        self._torque_limit = jp.asarray(control.torque_limit)
        self._kp = jp.asarray(control.kp)
        self._kd = jp.asarray(control.kd)
        self._default_pose = jp.asarray(control.default_angle)
        self._action_filter_alpha = jp.full(
            (ACTION_SIZE,), self._config.action_filter_alpha, dtype=jp.float32
        )
        if self._config.leg_action_filter_alpha >= 0.0:
            self._action_filter_alpha = self._action_filter_alpha.at[:12].set(
                self._config.leg_action_filter_alpha
            )
        if self._config.sagittal_action_filter_alpha >= 0.0:
            sagittal = jp.asarray((0, 3, 4, 6, 9, 10), dtype=jp.int32)
            self._action_filter_alpha = self._action_filter_alpha.at[sagittal].set(
                self._config.sagittal_action_filter_alpha
            )
        self._gyro_scale = control.gyro_scale
        self._dof_pos_scale = control.dof_pos_scale
        self._dof_vel_scale = control.dof_vel_scale
        self._observation_clip = control.observation_clip
        actuator_limits = control.torque_limit.copy()
        if self._full_parallel_model:
            for index, limit in zip(
                (4, 5, 10, 11, 13, 14), PHYSICAL_TORQUE_LIMITS
            ):
                actuator_limits[index] = limit
        self._actuator_torque_limit = jp.asarray(actuator_limits)

        self._qpos_ids = np.asarray(
            [self._mj_model.joint(name).qposadr[0] for name in JOINT_NAMES],
            dtype=np.int32,
        )
        self._qvel_ids = np.asarray(
            [self._mj_model.joint(name).dofadr[0] for name in JOINT_NAMES],
            dtype=np.int32,
        )
        self._lowers = jp.asarray(
            [self._mj_model.joint(name).range[0] for name in JOINT_NAMES]
        )
        self._uppers = jp.asarray(
            [self._mj_model.joint(name).range[1] for name in JOINT_NAMES]
        )
        center = (self._lowers + self._uppers) * 0.5
        radius = (self._uppers - self._lowers) * 0.5
        self._soft_lowers = center - radius * self._config.soft_joint_limit
        self._soft_uppers = center + radius * self._config.soft_joint_limit
        self._physical_qpos_ids = None
        self._physical_qvel_ids = None
        self._reset_joint_noise_mask = jp.ones(ACTION_SIZE, dtype=jp.float32)
        self._parallel_qvel_ids = np.empty(0, dtype=np.int32)
        if self._full_parallel_model:
            self._physical_qpos_ids = np.asarray(
                [
                    self._mj_model.joint(name).qposadr[0]
                    for name in PHYSICAL_ACTUATOR_JOINT_NAMES
                ],
                dtype=np.int32,
            )
            self._physical_qvel_ids = np.asarray(
                [
                    self._mj_model.joint(name).dofadr[0]
                    for name in PHYSICAL_ACTUATOR_JOINT_NAMES
                ],
                dtype=np.int32,
            )
            coupled_indices = np.asarray((4, 5, 10, 11, 13, 14), dtype=np.int32)
            self._reset_joint_noise_mask = self._reset_joint_noise_mask.at[
                coupled_indices
            ].set(0.0)
            parallel_qvel_ids = set(self._qvel_ids[coupled_indices].tolist())
            dof_width = {
                int(mujoco.mjtJoint.mjJNT_FREE): 6,
                int(mujoco.mjtJoint.mjJNT_BALL): 3,
                int(mujoco.mjtJoint.mjJNT_SLIDE): 1,
                int(mujoco.mjtJoint.mjJNT_HINGE): 1,
            }
            for name in AUXILIARY_JOINT_NAMES:
                joint_id = self._mj_model.joint(name).id
                start = int(self._mj_model.jnt_dofadr[joint_id])
                width = dof_width[int(self._mj_model.jnt_type[joint_id])]
                parallel_qvel_ids.update(range(start, start + width))
            self._parallel_qvel_ids = np.asarray(
                sorted(parallel_qvel_ids), dtype=np.int32
            )

        self._base_body_id = self._mj_model.body("base_link").id
        self._foot_site_ids = np.asarray(
            [
                self._mj_model.site("left_foot_training_site").id,
                self._mj_model.site("right_foot_training_site").id,
            ],
            dtype=np.int32,
        )
        self._gyro_sensor = self._sensor_slice("Body_Gyro")
        self._local_velocity_sensor = self._sensor_slice("Base_LocalLinVel")
        self._global_velocity_sensor = self._sensor_slice("Base_GlobalLinVel")
        self._foot_velocity_sensors = np.asarray(
            [
                self._sensor_slice("LeftFoot_GlobalLinVel"),
                self._sensor_slice("RightFoot_GlobalLinVel"),
            ],
            dtype=np.int32,
        )

        self._init_q = self._make_initial_qpos()
        self._standing_base_height = float(self._init_q[2])
        self._robot_weight = float(
            np.sum(self._mj_model.body_mass) * abs(self._mj_model.opt.gravity[2])
        )
        self._pose_weights = jp.asarray(
            [
                0.4,
                1.0,
                1.0,
                0.3,
                0.3,
                0.5,
                0.4,
                1.0,
                1.0,
                0.3,
                0.3,
                0.5,
                1.0,
                1.0,
                1.0,
                2.0,
                2.0,
                1.5,
                1.5,
                1.5,
                1.0,
                2.0,
                2.0,
                2.0,
                1.5,
                1.5,
                1.5,
                1.0,
                2.0,
                2.0,
                2.0,
            ],
            dtype=jp.float32,
        )
        self._teacher_params, self._teacher_output_scale = (
            self._load_teacher_policy(str(self._config.teacher_policy))
        )
        if not 0.0 <= self._config.teacher_blend <= 1.0:
            raise ValueError("teacher_blend must be between 0 and 1")
        if self._config.action_gain <= 0.0:
            raise ValueError("action_gain must be positive")
        if not (
            self._config.leg_action_gain == -1.0
            or self._config.leg_action_gain > 0.0
        ):
            raise ValueError("leg_action_gain must be -1 or positive")
        if not 0.0 <= self._config.action_filter_alpha <= 1.0:
            raise ValueError("action_filter_alpha must be between 0 and 1")
        if not (
            self._config.leg_action_filter_alpha == -1.0
            or 0.0 <= self._config.leg_action_filter_alpha <= 1.0
        ):
            raise ValueError(
                "leg_action_filter_alpha must be -1 or between 0 and 1"
            )
        if not (
            self._config.sagittal_action_filter_alpha == -1.0
            or 0.0 <= self._config.sagittal_action_filter_alpha <= 1.0
        ):
            raise ValueError(
                "sagittal_action_filter_alpha must be -1 or between 0 and 1"
            )

        frame_noise = np.concatenate(
            [
                np.full(3, self._config.noise_scales.gyro),
                np.full(3, self._config.noise_scales.gravity),
                np.zeros(3),
                np.full(ACTION_SIZE, self._config.noise_scales.joint_pos),
                np.full(ACTION_SIZE, self._config.noise_scales.joint_vel),
                np.zeros(ACTION_SIZE),
            ]
        )
        self._frame_noise_scale = jp.asarray(frame_noise, dtype=jp.float32)
        if self._frame_noise_scale.shape != (OBSERVATION_FRAME_SIZE,):
            raise ValueError("OLI observation noise vector has the wrong size")
        if not (
            0.0 <= self._config.balance_assist_min
            <= self._config.balance_assist_max
            <= 1.0
        ):
            raise ValueError(
                "balance_assist_min/max must satisfy 0 <= min <= max <= 1"
            )

    def _sensor_slice(self, name: str) -> np.ndarray:
        sensor = self._mj_model.sensor(name)
        start = self._mj_model.sensor_adr[sensor.id]
        size = self._mj_model.sensor_dim[sensor.id]
        return np.arange(start, start + size, dtype=np.int32)

    @staticmethod
    def _load_teacher_policy(path: str):
        if not path:
            return None, None
        model = onnx.load(str(Path(path).expanduser().resolve()))
        tensors = {
            tensor.name: jp.asarray(numpy_helper.to_array(tensor))
            for tensor in model.graph.initializer
        }
        layers = tuple(
            (
                tensors[f"actor.{index}.weight"],
                tensors[f"actor.{index}.bias"],
            )
            for index in (0, 2, 4, 6)
        )
        bounded_output = any(node.name == "action_scale" for node in model.graph.node)
        output_scale = (
            float(np.asarray(tensors["clip_scale"]).reshape(()))
            if bounded_output
            else None
        )
        return layers, output_scale

    def _teacher_action(self, observation: jax.Array) -> jax.Array:
        value = observation
        for weight, bias in self._teacher_params[:-1]:
            value = jp.dot(value, weight.T) + bias
            value = jp.where(value > 0.0, value, jp.expm1(value))
        weight, bias = self._teacher_params[-1]
        raw = jp.dot(value, weight.T) + bias
        if self._teacher_output_scale is None:
            return raw
        return self._teacher_output_scale * raw / (1.0 + jp.abs(raw))

    def _make_initial_qpos(self) -> jp.ndarray:
        qpos = np.array(self._mj_model.qpos0, dtype=np.float64, copy=True)
        qpos[2] = 1.0
        qpos[3] = 1.0
        qpos[self._qpos_ids] = np.asarray(self._default_pose)
        if self._full_parallel_model:
            for indices, mapping in (
                ((4, 5), left_ankle_pr_to_ab),
                ((10, 11), right_ankle_pr_to_ab),
                ((13, 14), waist_pr_to_ab),
            ):
                pair = np.asarray(self._default_pose)[list(indices)]
                motor_pair = np.asarray(mapping(jp.asarray(pair)))
                qpos[self._physical_qpos_ids[list(indices)]] = motor_pair
            qpos = self._close_parallel_constraints(qpos)

        data = mujoco.MjData(self._mj_model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self._mj_model, data)
        sole_height = min(data.site_xpos[site_id, 2] for site_id in self._foot_site_ids)
        qpos[2] += 0.004 - sole_height
        data.qpos[:] = qpos
        mujoco.mj_forward(self._mj_model, data)
        return jp.asarray(qpos, dtype=jp.float32)

    def _close_parallel_constraints(self, qpos: np.ndarray) -> np.ndarray:
        """Orient the six passive rods so every connect constraint starts closed."""
        data = mujoco.MjData(self._mj_model)
        data.qpos[:] = qpos
        mujoco.mj_forward(self._mj_model, data)

        for equality_index in range(self._mj_model.neq):
            body1 = int(self._mj_model.eq_obj1id[equality_index])
            body2 = int(self._mj_model.eq_obj2id[equality_index])
            parent = int(self._mj_model.body_parentid[body2])
            body1_rotation = data.xmat[body1].reshape(3, 3)
            parent_rotation = data.xmat[parent].reshape(3, 3)
            target_world = (
                data.xpos[body1]
                + body1_rotation @ self._mj_model.eq_data[equality_index, :3]
            )
            desired_parent = parent_rotation.T @ (target_world - data.xpos[body2])
            source = self._mj_model.eq_data[equality_index, 3:6]
            quaternion = self._quaternion_from_two_vectors(source, desired_parent)

            joint_index = int(self._mj_model.body_jntadr[body2])
            qpos_address = int(self._mj_model.jnt_qposadr[joint_index])
            qpos[qpos_address : qpos_address + 4] = quaternion

        return qpos

    @staticmethod
    def _quaternion_from_two_vectors(
        source: np.ndarray, target: np.ndarray
    ) -> np.ndarray:
        source = source / np.linalg.norm(source)
        target = target / np.linalg.norm(target)
        dot = float(np.dot(source, target))
        if dot < -0.999999:
            axis = np.cross(source, np.asarray([1.0, 0.0, 0.0]))
            if np.linalg.norm(axis) < 1e-8:
                axis = np.cross(source, np.asarray([0.0, 1.0, 0.0]))
            axis /= np.linalg.norm(axis)
            return np.asarray([0.0, axis[0], axis[1], axis[2]])
        quaternion = np.asarray([1.0 + dot, *np.cross(source, target)])
        return quaternion / np.linalg.norm(quaternion)

    def _sample_command(self, rng: jax.Array) -> jp.ndarray:
        speed_rng, lateral_rng, yaw_rng, stand_rng = jax.random.split(rng, 4)
        command = jp.asarray(
            [
                jax.random.uniform(
                    speed_rng,
                    minval=self._config.command_speed[0],
                    maxval=self._config.command_speed[1],
                ),
                jax.random.uniform(
                    lateral_rng,
                    minval=self._config.command_lateral[0],
                    maxval=self._config.command_lateral[1],
                ),
                jax.random.uniform(
                    yaw_rng,
                    minval=self._config.command_yaw[0],
                    maxval=self._config.command_yaw[1],
                ),
            ],
            dtype=jp.float32,
        )
        should_stand = (
            jax.random.uniform(stand_rng) < self._config.stand_probability
        )
        return jp.where(should_stand, jp.zeros_like(command), command)

    def _apply_heading_hold(
        self, data: mjx.Data, nominal_command: jax.Array
    ) -> jax.Array:
        if self._config.heading_hold_kp <= 0.0:
            return nominal_command
        quat = data.qpos[3:7]
        yaw = jp.arctan2(
            2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
            1.0 - 2.0 * (jp.square(quat[2]) + jp.square(quat[3])),
        )
        error = jp.arctan2(
            jp.sin(yaw - self._config.heading_hold_target),
            jp.cos(yaw - self._config.heading_hold_target),
        )
        yaw_rate = jp.clip(
            nominal_command[2] - self._config.heading_hold_kp * error,
            -self._config.heading_hold_max_yaw_rate,
            self._config.heading_hold_max_yaw_rate,
        )
        return nominal_command.at[2].set(yaw_rate)

    def reset(self, rng: jax.Array) -> mjx_env.State:
        (
            rng,
            position_rng,
            yaw_rng,
            joint_rng,
            velocity_rng,
            command_rng,
            assist_rng,
            noise_rng,
        ) = jax.random.split(rng, 8)

        qpos = self._init_q
        start_x = (
            self.spec.start_x - self._config.stair_approach_distance
            if self.spec.step_height > 0
            else -0.2
        )
        xy_noise = jax.random.uniform(position_rng, (2,), minval=-1.0, maxval=1.0)
        qpos = qpos.at[0].set(start_x + self._config.reset_x_noise * xy_noise[0])
        qpos = qpos.at[1].set(self._config.reset_y_noise * xy_noise[1])
        yaw = jax.random.uniform(
            yaw_rng,
            (),
            minval=-self._config.reset_yaw_noise,
            maxval=self._config.reset_yaw_noise,
        )
        qpos = qpos.at[3:7].set(
            jp.asarray([jp.cos(yaw * 0.5), 0.0, 0.0, jp.sin(yaw * 0.5)])
        )
        joint_noise = jax.random.uniform(
            joint_rng, (ACTION_SIZE,), minval=-1.0, maxval=1.0
        )
        qpos = qpos.at[self._qpos_ids].add(
            joint_noise
            * self._reset_joint_noise_mask
            * self._config.reset_joint_noise
        )

        qvel = jp.zeros(self.mjx_model.nv)
        qvel_noise = jax.random.uniform(
            velocity_rng, (self.mjx_model.nv,), minval=-1.0, maxval=1.0
        )
        if self._full_parallel_model:
            qvel_noise = qvel_noise.at[self._parallel_qvel_ids].set(0.0)
        qvel = qvel + qvel_noise * self._config.reset_velocity_noise

        data = mjx_env.make_data(
            self.mj_model,
            qpos=qpos,
            qvel=qvel,
            impl=self.mjx_model.impl.value,
            naconmax=self._config.naconmax,
            njmax=self._config.njmax,
        )
        data = mjx.forward(self.mjx_model, data)

        nominal_command = self._sample_command(command_rng)
        info = {
            "rng": rng,
            "last_action": jp.zeros(ACTION_SIZE),
            "command": self._apply_heading_hold(data, nominal_command),
            "nominal_command": nominal_command,
            "step": jp.zeros((), dtype=jp.int32),
            "previous_x": data.qpos[0],
            "previous_z": data.qpos[2],
            "progress_anchor_x": data.qpos[0],
            "success": jp.zeros((), dtype=jp.float32),
            "balance_assist": jax.random.uniform(
                assist_rng,
                (),
                minval=self._config.balance_assist_min,
                maxval=self._config.balance_assist_max,
            ),
        }
        observation_info = info
        if self._config.history_lateral_speed is not None:
            observation_info = {
                **info,
                "command": info["command"].at[1].set(
                    self._config.history_lateral_speed
                ),
            }
        frame = self._get_frame_observation(data, observation_info, noise_rng)
        observation = jp.tile(frame, OBSERVATION_HISTORY)

        metrics = {
            f"reward/{name}": jp.zeros(())
            for name in self._config.reward_config.scales.keys()
        }
        metrics.update(
            {
                "stair/base_x": data.qpos[0],
                "stair/base_z": data.qpos[2],
                "stair/success": jp.zeros(()),
                "training/balance_assist": info["balance_assist"],
            }
        )
        return mjx_env.State(
            data=data,
            obs=observation,
            reward=jp.zeros(()),
            done=jp.zeros(()),
            metrics=metrics,
            info=info,
        )

    def step(self, state: mjx_env.State, action: jax.Array) -> mjx_env.State:
        rng, noise_rng = jax.random.split(state.info["rng"])
        action = jp.clip(action, -self._config.action_clip, self._config.action_clip)
        teacher_deviation = jp.zeros(())
        if self._teacher_params is not None:
            teacher_action = self._teacher_action(state.obs)
            teacher_deviation = jp.mean(jp.square(action - teacher_action))
            action = teacher_action + self._config.teacher_blend * (
                action - teacher_action
            )
        action = action * self._action_gain
        if self._config.heading_action_kp != 0.0:
            quat = state.data.qpos[3:7]
            yaw = jp.arctan2(
                2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                1.0 - 2.0 * (jp.square(quat[2]) + jp.square(quat[3])),
            )
            error = jp.arctan2(
                jp.sin(yaw - self._config.heading_hold_target),
                jp.cos(yaw - self._config.heading_hold_target),
            )
            correction = jp.clip(
                -self._config.heading_action_kp * error,
                -self._config.heading_action_limit,
                self._config.heading_action_limit,
            )
            action = action.at[jp.asarray((2, 8))].add(correction)
        action = self._action_filter_alpha * action + (
            1.0 - self._action_filter_alpha
        ) * state.info["last_action"]
        target = self._default_pose + action * self._action_scale
        target = jp.clip(target, self._soft_lowers, self._soft_uppers)
        control = (
            self._pr_pd_control(state.data, target)
            if self._full_parallel_model
            else target
        )
        assisted_data = self._apply_balance_assist(
            state.data, state.info["balance_assist"]
        )
        data = mjx_env.step(
            self.mjx_model,
            assisted_data,
            control,
            self.n_substeps,
        )

        success = self._success(data)
        next_step = state.info["step"] + 1
        progress_window_complete = (
            next_step % self._config.progress_window_steps == 0
        )
        done = self._termination(
            data,
            state.info["step"],
            state.info["progress_anchor_x"],
            progress_window_complete,
        )
        done = done | success
        rewards = self._get_reward(
            data,
            action,
            state.info,
            done,
            success,
            teacher_deviation,
        )
        scaled_rewards = {
            name: value * self._config.reward_config.scales[name]
            for name, value in rewards.items()
        }
        reward = jp.clip(sum(scaled_rewards.values()) * self.dt, -5.0, 5.0)

        state.info["command"] = self._apply_heading_hold(
            data, state.info["nominal_command"]
        )
        frame = self._get_frame_observation(data, state.info, noise_rng)
        observation = jp.roll(state.obs, OBSERVATION_FRAME_SIZE)
        observation = observation.at[:OBSERVATION_FRAME_SIZE].set(frame)

        state.info["rng"] = rng
        state.info["last_action"] = action
        state.info["step"] += 1
        state.info["previous_x"] = data.qpos[0]
        state.info["previous_z"] = data.qpos[2]
        state.info["progress_anchor_x"] = jp.where(
            progress_window_complete,
            data.qpos[0],
            state.info["progress_anchor_x"],
        )
        state.info["success"] = jp.float32(success)

        for name, value in scaled_rewards.items():
            state.metrics[f"reward/{name}"] = value
        state.metrics["stair/base_x"] = data.qpos[0]
        state.metrics["stair/base_z"] = data.qpos[2]
        state.metrics["stair/success"] = jp.float32(success)
        state.metrics["training/balance_assist"] = state.info["balance_assist"]

        return state.replace(
            data=data,
            obs=observation,
            reward=reward,
            done=jp.float32(done),
        )

    def _pr_pd_control(self, data: mjx.Data, target: jax.Array) -> jax.Array:
        position = data.qpos[self._qpos_ids]
        velocity = data.qvel[self._qvel_ids]
        control = self._kp * (target - position) - self._kd * velocity
        control = jp.clip(control, -self._torque_limit, self._torque_limit)
        physical_position = data.qpos[self._physical_qpos_ids]
        physical_velocity = data.qvel[self._physical_qvel_ids]
        for indices, mapping in (
            ((4, 5), left_ankle_pr_to_ab),
            ((10, 11), right_ankle_pr_to_ab),
            ((13, 14), waist_pr_to_ab),
        ):
            pair = jp.asarray(indices)
            if self._config.parallel_control_mode == "projected_torque":
                ab_torque = pr_torque_to_ab(
                    mapping,
                    position[pair],
                    control[pair],
                )
            else:
                target_ab = mapping(target[pair])
                ab_torque = (
                    self._kp[pair] * (target_ab - physical_position[pair])
                    - self._kd[pair] * physical_velocity[pair]
                )
            limits = self._actuator_torque_limit[pair]
            control = control.at[pair].set(jp.clip(ab_torque, -limits, limits))
        return control

    def _apply_balance_assist(
        self, data: mjx.Data, assist: jax.Array
    ) -> mjx.Data:
        """Apply a training-only stabilizing wrench to the floating base."""
        base_quat = data.qpos[3:7]
        body_up = math.rotate(jp.asarray([0.0, 0.0, 1.0]), base_quat)
        upright_axis = jp.cross(body_up, jp.asarray([0.0, 0.0, 1.0]))

        local_angular_velocity = data.sensordata[self._gyro_sensor]
        world_angular_velocity = math.rotate(local_angular_velocity, base_quat)
        local_damping = jp.asarray(
            [
                -self._config.balance_assist_upright_kd
                * local_angular_velocity[0],
                -self._config.balance_assist_upright_kd
                * local_angular_velocity[1],
                0.0,
            ]
        )
        torque = assist * (
            self._config.balance_assist_upright_kp * upright_axis
            + math.rotate(local_damping, base_quat)
        )
        base_forward = math.rotate(jp.asarray([1.0, 0.0, 0.0]), base_quat)
        heading_error = jp.arctan2(base_forward[1], base_forward[0])
        heading_torque = (
            -self._config.balance_assist_heading_kp * heading_error
            - self._config.balance_assist_heading_kd * world_angular_velocity[2]
        )
        torque = torque.at[2].add(assist * heading_torque)
        lateral_force = -assist * (
            self._config.balance_assist_lateral_kp * data.qpos[1]
            + self._config.balance_assist_lateral_kd * data.qvel[1]
        )
        target_height = self._support_height(data) + self._standing_base_height
        height_force = assist * jp.clip(
            self._config.balance_assist_height_kp
            * (target_height - data.qpos[2])
            - self._config.balance_assist_height_kd * data.qvel[2],
            0.0,
            self._robot_weight
            * self._config.balance_assist_height_max_weight_fraction,
        )

        wrench = jp.zeros_like(data.xfrc_applied)
        wrench = wrench.at[self._base_body_id, 1].set(lateral_force)
        wrench = wrench.at[self._base_body_id, 2].set(height_force)
        wrench = wrench.at[self._base_body_id, 3:6].set(torque)
        return data.replace(xfrc_applied=wrench)

    def _get_frame_observation(
        self,
        data: mjx.Data,
        info: dict[str, Any],
        rng: jax.Array,
    ) -> jp.ndarray:
        frame = jp.concatenate(
            [
                data.sensordata[self._gyro_sensor] * self._gyro_scale,
                self._projected_gravity(data),
                info["command"],
                (data.qpos[self._qpos_ids] - self._default_pose)
                * self._dof_pos_scale,
                data.qvel[self._qvel_ids] * self._dof_vel_scale,
                info["last_action"],
            ]
        )
        noise = jax.random.uniform(rng, frame.shape, minval=-1.0, maxval=1.0)
        frame = frame + (
            noise * self._frame_noise_scale * self._config.observation_noise
        )
        return jp.clip(frame, -self._observation_clip, self._observation_clip)

    def _projected_gravity(self, data: mjx.Data) -> jp.ndarray:
        return math.rotate(
            jp.asarray([0.0, 0.0, -1.0]), math.quat_inv(data.qpos[3:7])
        )

    def _terrain_height(self, x: jax.Array) -> jax.Array:
        if self.spec.step_height <= 0.0:
            return jp.zeros_like(x)
        if self.spec.riser_ramp_length > 0.0:
            height = jp.zeros_like(x)
            for step in range(1, self.spec.step_count + 1):
                riser_x = self.spec.start_x + (step - 1) * self.spec.step_depth
                ramp_start = riser_x - self.spec.riser_ramp_length
                phase = jp.clip(
                    (x - ramp_start) / self.spec.riser_ramp_length,
                    0.0,
                    1.0,
                )
                ramp_height = (
                    (step - 1) * self.spec.step_height
                    + phase * self.spec.step_height
                )
                height = jp.where(x >= ramp_start, ramp_height, height)
            return height
        step_index = jp.floor((x - self.spec.start_x) / self.spec.step_depth) + 1
        step_index = jp.clip(step_index, 0, self.spec.step_count)
        return step_index * self.spec.step_height

    def _foot_state(self, data: mjx.Data) -> tuple[jax.Array, jax.Array, jax.Array]:
        position = data.site_xpos[self._foot_site_ids]
        velocity = data.sensordata[self._foot_velocity_sensors]
        clearance = position[:, 2] - self._terrain_height(position[:, 0])
        return position, velocity, clearance

    def _support_height(self, data: mjx.Data) -> jax.Array:
        feet = data.site_xpos[self._foot_site_ids]
        terrain_heights = self._terrain_height(feet[:, 0])
        clearance = feet[:, 2] - terrain_heights
        supported_heights = jp.where(clearance < 0.035, terrain_heights, 0.0)
        return jp.max(supported_heights)

    def _success(self, data: mjx.Data) -> jax.Array:
        if self.spec.step_height <= 0.0:
            return jp.asarray(False)
        reached_landing = data.qpos[0] > self.spec.end_x + 0.35
        high_enough = data.qpos[2] > self.spec.top_height + 0.72
        return reached_landing & high_enough

    def _termination(
        self,
        data: mjx.Data,
        step: jax.Array,
        progress_anchor_x: jax.Array,
        progress_window_complete: jax.Array,
    ) -> jax.Array:
        projected_gravity = self._projected_gravity(data)
        base_forward = math.rotate(
            jp.asarray([1.0, 0.0, 0.0]), data.qpos[3:7]
        )
        support_height = self._support_height(data)
        fallen = data.qpos[2] < support_height + 0.62
        tipped = projected_gravity[2] > -0.45
        left_course = jp.abs(data.qpos[1]) > self.spec.half_width - 0.25
        wrong_heading = base_forward[0] < 0.707
        stalled = progress_window_complete & (
            data.qpos[0] - progress_anchor_x < self._config.min_window_progress
        )
        invalid = ~jp.all(jp.isfinite(data.qpos)) | ~jp.all(jp.isfinite(data.qvel))
        return (
            ((fallen | tipped) & (step > 20))
            | (wrong_heading & (step > 100))
            | stalled
            | left_course
            | invalid
        )

    def _get_reward(
        self,
        data: mjx.Data,
        action: jax.Array,
        info: dict[str, Any],
        done: jax.Array,
        success: jax.Array,
        teacher_deviation: jax.Array,
    ) -> dict[str, jax.Array]:
        local_velocity = data.sensordata[self._local_velocity_sensor]
        world_velocity = data.sensordata[self._global_velocity_sensor]
        gyro = data.sensordata[self._gyro_sensor]
        gravity = self._projected_gravity(data)
        base_forward = math.rotate(
            jp.asarray([1.0, 0.0, 0.0]), data.qpos[3:7]
        )
        command = info["command"]
        foot_position, foot_velocity, clearance = self._foot_state(data)

        desired_world_velocity = math.rotate(
            jp.asarray([command[0], command[1], 0.0]), data.qpos[3:7]
        )
        velocity_error = jp.sum(
            jp.square(desired_world_velocity[:2] - world_velocity[:2])
        )
        tracking = jp.exp(
            -velocity_error / self._config.reward_config.tracking_sigma
        )
        yaw_tracking = jp.exp(
            -jp.square(command[2] - gyro[2])
            / self._config.reward_config.tracking_sigma
        )
        upright = jp.exp(
            -jp.sum(jp.square(gravity[:2]))
            / self._config.reward_config.upright_sigma
        )

        support_height = self._support_height(data)
        if self.spec.step_height > 0.0:
            approach_start = (
                self.spec.start_x - self._config.stair_approach_distance
            )
            foot_progress = jp.clip(
                (jp.min(foot_position[:, 0]) - approach_start)
                / (self.spec.end_x + 0.10 - approach_start),
                0.0,
                1.0,
            )
        else:
            foot_progress = jp.asarray(0.0)
        stair_height = (
            support_height / self.spec.top_height
            if self.spec.top_height > 0.0
            else jp.asarray(0.0)
        )
        if self.spec.step_height > 0.0:
            target_step = jp.floor(
                (foot_position[:, 0] - self.spec.start_x)
                / self.spec.step_depth
            ) + 1
            target_step = jp.clip(target_step, 1, self.spec.step_count)
            exposed_tread_depth = max(
                self.spec.step_depth - self.spec.riser_ramp_length,
                0.01,
            )
            target_x = (
                self.spec.start_x
                + (target_step - 1) * self.spec.step_depth
                + 0.5 * exposed_tread_depth
            )
            target_z = target_step * self.spec.step_height + 0.003
            desired_y = data.qpos[1] + jp.asarray([0.13, -0.13])
            target_error = (
                jp.square((foot_position[:, 0] - target_x) / 0.18)
                + jp.square((foot_position[:, 1] - desired_y) / 0.20)
                + jp.square((foot_position[:, 2] - target_z) / 0.12)
            )
            stair_foot_target = jp.max(jp.exp(-target_error))
        else:
            stair_foot_target = jp.asarray(0.0)
        target_height = support_height + self._standing_base_height
        base_height = jp.exp(
            -jp.square(data.qpos[2] - target_height)
            / self._config.reward_config.base_height_sigma
        )

        contact = clearance < 0.025
        foot_speed_xy_sq = jp.sum(jp.square(foot_velocity[:, :2]), axis=-1)
        foot_speed_xy = jp.sqrt(foot_speed_xy_sq + 1e-6)
        feet_slip = jp.sum(foot_speed_xy_sq * contact)
        clearance_error = jp.square(
            clearance - self._config.reward_config.swing_clearance
        )
        feet_clearance = jp.sum(
            clearance_error * (~contact) * jp.clip(foot_speed_xy, 0.0, 1.0)
        )

        joint_position = data.qpos[self._qpos_ids]
        joint_velocity = data.qvel[self._qvel_ids]
        torque_ratio = data.actuator_force / self._actuator_torque_limit
        pose_cost = jp.mean(
            self._pose_weights * jp.square(joint_position - self._default_pose)
        )
        forward_speed = (data.qpos[0] - info["previous_x"]) / self.dt
        moving = command[0] > 0.01

        return {
            "tracking_lin_vel": tracking,
            "tracking_yaw": yaw_tracking,
            "forward_progress": jp.where(
                moving,
                jp.clip(forward_speed, -2.0, command[0] * 1.25),
                0.0,
            ),
            "backward_velocity": jp.where(
                moving,
                jp.square(jp.minimum(forward_speed, 0.0)),
                0.0,
            ),
            "heading_alignment": 0.5 * (base_forward[0] + 1.0),
            "upward_progress": jp.clip(
                (data.qpos[2] - info["previous_z"]) / self.dt, -0.5, 1.0
            ),
            "upright": upright,
            "base_height": base_height,
            "alive": jp.asarray(1.0),
            "stair_foot_progress": foot_progress,
            "stair_foot_target": stair_foot_target,
            "stair_height": stair_height,
            "stair_success": jp.float32(success),
            "lateral_drift": jp.square(data.qpos[1])
            + jp.square(local_velocity[1]),
            "vertical_velocity": jp.square(local_velocity[2]),
            "angular_velocity": jp.sum(jp.square(gyro[:2])),
            "feet_slip": feet_slip,
            "feet_clearance": feet_clearance,
            "action_rate": jp.mean(jp.square(action - info["last_action"])),
            "joint_velocity": jp.mean(jp.square(joint_velocity)),
            "torques": jp.mean(jp.square(torque_ratio)),
            "pose": pose_cost,
            "teacher_deviation": teacher_deviation,
            "termination": jp.float32(done & ~success),
        }

    @property
    def xml_path(self) -> str:
        return str(self._xml_path)

    @property
    def action_size(self) -> int:
        return ACTION_SIZE

    @property
    def mj_model(self) -> mujoco.MjModel:
        return self._mj_model

    @property
    def mjx_model(self) -> mjx.Model:
        return self._mjx_model
