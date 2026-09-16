#!/usr/bin/env python3
"""Expose the MATRiX G1 full-body runtime to ROS 2 and RoamerX/Nav2."""

from __future__ import annotations

import json
import math
import os
import socket
import struct
import time
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


BUILTIN_POSES = {"home", "wave", "reach", "grasp", "pickup", "carry", "release", "auto"}
PAYLOAD_ACTIONS = {"attach", "grasp", "pickup", "release", "drop"}
LEG_JOINTS = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
}


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, float(value)))


def roll_pitch_yaw(quaternion_wxyz: list[float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion_wxyz
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class G1MobileManipRosBridge(Node):
    def __init__(self) -> None:
        super().__init__("g1_mobile_manip_ros_bridge")
        self.host = os.getenv("G1_MOBILE_MANIP_HOST", "127.0.0.1")
        self.port = int(os.getenv("G1_MOBILE_MANIP_PORT", "15151"))
        self.cmd_vel_topic = os.getenv("G1_CMD_VEL_TOPIC", "/cmd_vel")
        self.max_vx = float(os.getenv("G1_NAV_MAX_VX", "0.55"))
        self.max_vy = float(os.getenv("G1_NAV_MAX_VY", "0.25"))
        self.max_wz = float(os.getenv("G1_NAV_MAX_WZ", "0.70"))
        self.command_timeout = float(os.getenv("G1_CMD_VEL_TIMEOUT", "0.50"))
        self.status_rate = max(1.0, float(os.getenv("G1_ROS_STATUS_RATE", "20.0")))
        self.upper_speed = float(os.getenv("G1_UPPER_BODY_SPEED", "1.2"))
        self.lidar_translation = (
            float(os.getenv("G1_LIDAR_X", "0.10")),
            float(os.getenv("G1_LIDAR_Y", "0.0")),
            float(os.getenv("G1_LIDAR_Z", "0.35")),
        )

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.setblocking(False)
        self.status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.status_socket.setblocking(False)

        reliable_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/odom/mujoco_odom", 20
        )
        self.joint_publisher = self.create_publisher(
            JointState, "/joint_states", 20
        )
        self.lidar_publisher = self.create_publisher(
            PointCloud2, "/livox/lidar", reliable_sensor_qos
        )
        self.imu_publisher = self.create_publisher(
            Imu, "/imu/data", reliable_sensor_qos
        )
        self.runtime_status_publisher = self.create_publisher(
            String, "/g1/runtime_status", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 20)
        self.create_subscription(String, "/g1/upper_pose", self.upper_pose_callback, 10)
        self.create_subscription(
            JointState, "/g1/joint_targets", self.joint_targets_callback, 10
        )
        self.create_subscription(
            String, "/g1/payload_action", self.payload_action_callback, 10
        )
        self.create_timer(1.0 / self.status_rate, self.poll_runtime)

        self.last_cmd_vel_time = time.monotonic()
        self.command_active = False
        self.fall_latched = False
        self.runtime_seen = False
        self.upper_pose = "home"
        self.known_joint_names: set[str] = set()
        self.last_command_log = 0.0
        self.last_state_log = 0.0
        self.last_status_request = 0.0
        self.publish_static_transforms()
        self.get_logger().info(
            f"Listening on {self.cmd_vel_topic} and forwarding to udp://{self.host}:{self.port}"
        )
        self.get_logger().info(
            "Publishing /odom/mujoco_odom, TF, /joint_states, /livox/lidar and /imu/data"
        )

    def destroy_node(self) -> bool:
        try:
            self.send_command({"command": "stop"})
        except OSError:
            pass
        self.command_socket.close()
        self.status_socket.close()
        return super().destroy_node()

    def publish_static_transforms(self) -> None:
        now = self.get_clock().now().to_msg()
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0

        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = now
        base_to_lidar.header.frame_id = "base_link"
        base_to_lidar.child_frame_id = "lidar"
        base_to_lidar.transform.translation.x = self.lidar_translation[0]
        base_to_lidar.transform.translation.y = self.lidar_translation[1]
        base_to_lidar.transform.translation.z = self.lidar_translation[2]
        base_to_lidar.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([map_to_odom, base_to_lidar])

    def send_command(self, command: dict[str, Any]) -> None:
        payload = json.dumps(command, separators=(",", ":")).encode("utf-8")
        self.command_socket.sendto(payload, (self.host, self.port))

    def drain_command_replies(self) -> None:
        while True:
            try:
                payload, _ = self.command_socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not reply.get("ok", False):
                self.get_logger().error(f"Runtime rejected command: {reply.get('error', reply)}")

    def cmd_vel_callback(self, message: Twist) -> None:
        self.last_cmd_vel_time = time.monotonic()
        if self.fall_latched:
            command = {"command": "stop"}
            values = (0.0, 0.0, 0.0)
        else:
            values = (
                clamp(message.linear.x, self.max_vx),
                clamp(message.linear.y, self.max_vy),
                clamp(message.angular.z, self.max_wz),
            )
            if max(abs(value) for value in values) <= 1e-4:
                command = {"command": "stop"}
                self.command_active = False
            else:
                command = {
                    "command": "move",
                    "vx": values[0],
                    "vy": values[1],
                    "yaw": values[2],
                }
                self.command_active = True
        self.send_command(command)
        now = time.monotonic()
        if now - self.last_command_log >= 1.0:
            self.get_logger().info(
                "cmd_vel vx=%.3f vy=%.3f wz=%.3f%s"
                % (*values, " FALL-STOP" if self.fall_latched else "")
            )
            self.last_command_log = now

    def upper_pose_callback(self, message: String) -> None:
        pose = message.data.strip().lower()
        if pose not in BUILTIN_POSES:
            self.get_logger().error(
                f"Unknown upper pose '{pose}'; expected one of {sorted(BUILTIN_POSES)}"
            )
            return
        self.upper_pose = pose
        self.send_command({"command": "upper", "pose": pose})
        self.get_logger().info(f"Upper-body pose: {pose}")

    def joint_targets_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position) or not message.name:
            self.get_logger().error(
                "/g1/joint_targets requires equally sized, non-empty name and position arrays"
            )
            return
        targets: dict[str, float] = {}
        for name, position in zip(message.name, message.position):
            if name in LEG_JOINTS:
                self.get_logger().error(f"Leg joint '{name}' is owned by DreamWaQ")
                return
            if self.known_joint_names and name not in self.known_joint_names:
                self.get_logger().error(f"Unknown G1 joint '{name}'")
                return
            if not math.isfinite(position):
                self.get_logger().error(f"Joint target for '{name}' is not finite")
                return
            targets[name] = float(position)
        self.send_command(
            {
                "command": "upper",
                "pose": self.upper_pose,
                "speed": self.upper_speed,
                "joints": targets,
            }
        )
        self.get_logger().info(f"Applied {len(targets)} upper-body joint targets")

    def payload_action_callback(self, message: String) -> None:
        action = message.data.strip().lower()
        if action not in PAYLOAD_ACTIONS:
            self.get_logger().error(
                f"Unknown payload action '{action}'; expected one of {sorted(PAYLOAD_ACTIONS)}"
            )
            return
        self.send_command({"command": "payload", "action": action})
        self.get_logger().info(f"Payload action: {action}")

    def poll_runtime(self) -> None:
        self.drain_command_replies()
        now = time.monotonic()
        if self.command_active and now - self.last_cmd_vel_time > self.command_timeout:
            self.send_command({"command": "stop"})
            self.command_active = False
            self.get_logger().warning(
                f"No cmd_vel for {self.command_timeout:.2f}s; sent stop"
            )

        latest = None
        while True:
            try:
                payload, _ = self.status_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if reply.get("ok") and isinstance(reply.get("status"), dict):
                latest = reply["status"]
        if latest is not None:
            self.publish_state(latest)

        request = b'{"command":"status"}'
        try:
            self.status_socket.sendto(request, (self.host, self.port))
            self.last_status_request = now
        except OSError as error:
            self.get_logger().error(f"Could not query G1 runtime: {error}")

    def publish_state(self, state: dict[str, Any]) -> None:
        required = (
            "position", "quaternion_wxyz", "linear_velocity", "angular_velocity",
            "joint_names", "joint_positions", "joint_velocities", "lidar_points",
        )
        missing = [name for name in required if name not in state]
        if missing:
            self.get_logger().error(f"Runtime status is missing fields: {missing}")
            return

        now = self.get_clock().now().to_msg()
        position = state["position"]
        quaternion = state["quaternion_wxyz"]
        linear_world = state["linear_velocity"]
        angular = state["angular_velocity"]
        roll, pitch, yaw = roll_pitch_yaw(quaternion)
        upright = position[2] > 0.50 and abs(roll) < 0.75 and abs(pitch) < 0.75
        if not upright and not self.fall_latched:
            self.fall_latched = True
            self.command_active = False
            self.send_command({"command": "stop"})
            self.get_logger().error(
                "FALL PROTECTION: stopped navigation (z=%.2f roll=%.2f pitch=%.2f)"
                % (position[2], roll, pitch)
            )
        elif upright and self.fall_latched:
            self.fall_latched = False
            self.get_logger().info("G1 is upright again; cmd_vel forwarding restored")

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.w = float(quaternion[0])
        odom.pose.pose.orientation.x = float(quaternion[1])
        odom.pose.pose.orientation.y = float(quaternion[2])
        odom.pose.pose.orientation.z = float(quaternion[3])
        odom.pose.covariance[0] = 1e-4
        odom.pose.covariance[7] = 1e-4
        odom.pose.covariance[14] = 1e-3
        odom.pose.covariance[21] = 1e-3
        odom.pose.covariance[28] = 1e-3
        odom.pose.covariance[35] = 1e-4
        odom.twist.twist.linear.x = (
            math.cos(yaw) * linear_world[0] + math.sin(yaw) * linear_world[1]
        )
        odom.twist.twist.linear.y = (
            -math.sin(yaw) * linear_world[0] + math.cos(yaw) * linear_world[1]
        )
        odom.twist.twist.linear.z = float(linear_world[2])
        odom.twist.twist.angular.x = float(angular[0])
        odom.twist.twist.angular.y = float(angular[1])
        odom.twist.twist.angular.z = float(angular[2])
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = now
        joints.name = [str(name) for name in state["joint_names"]]
        joints.position = [float(value) for value in state["joint_positions"]]
        joints.velocity = [float(value) for value in state["joint_velocities"]]
        self.known_joint_names = set(joints.name)
        self.joint_publisher.publish(joints)

        cloud = PointCloud2()
        cloud.header.stamp = now
        cloud.header.frame_id = "lidar"
        cloud.height = 1
        cloud.width = len(state["lidar_points"])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b"".join(
            struct.pack("<fff", float(point[0]), float(point[1]), float(point[2]))
            for point in state["lidar_points"]
        )
        self.lidar_publisher.publish(cloud)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "base_link"
        imu.orientation = odom.pose.pose.orientation
        imu.angular_velocity.x = float(angular[0])
        imu.angular_velocity.y = float(angular[1])
        imu.angular_velocity.z = float(angular[2])
        acceleration = state.get("imu_linear_acceleration", [0.0, 0.0, 0.0])
        imu.linear_acceleration.x = float(acceleration[0])
        imu.linear_acceleration.y = float(acceleration[1])
        imu.linear_acceleration.z = float(acceleration[2])
        self.imu_publisher.publish(imu)

        summary = {
            key: state.get(key)
            for key in ("running", "sim_time", "position", "velocity_command", "control", "mission", "payload", "metrics")
        }
        self.runtime_status_publisher.publish(
            String(data=json.dumps(summary, ensure_ascii=False, separators=(",", ":")))
        )
        self.runtime_seen = True
        monotonic_now = time.monotonic()
        if monotonic_now - self.last_state_log >= 5.0:
            self.get_logger().info(
                "G1 state x=%.2f y=%.2f z=%.2f joints=%d lidar_points=%d"
                % (position[0], position[1], position[2], len(joints.name), cloud.width)
            )
            self.last_state_log = monotonic_now


def main() -> None:
    rclpy.init()
    node = G1MobileManipRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
