#!/usr/bin/env python3
"""Bridge ROS navigation commands and OLI MuJoCo state over loopback UDP."""

import json
import math
import os
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OliCmdVelBridge(Node):
    def __init__(self):
        super().__init__("oli_cmd_vel_bridge")
        self.host = os.getenv("OLI_CMD_VEL_HOST", "127.0.0.1")
        self.port = int(os.getenv("OLI_CMD_VEL_PORT", "15001"))
        self.topic = os.getenv("OLI_CMD_VEL_TOPIC", "/cmd_vel")
        self.arm_file = os.getenv(
            "OLI_NAV_ARM_FILE",
            "/home/unitree/matrix_go2w_lcm_demo/.run/oli_navigation.armed",
        )
        self.max_vx = float(os.getenv("OLI_NAV_MAX_VX", "0.08"))
        self.min_forward_vx = float(
            os.getenv("OLI_NAV_MIN_FORWARD_VX", "0.08")
        )
        self.max_reverse_vx = float(os.getenv("OLI_NAV_MAX_REVERSE_VX", "0.05"))
        self.linear_deadband = float(
            os.getenv("OLI_NAV_LINEAR_DEADBAND", "0.015")
        )
        self.max_vy = float(os.getenv("OLI_NAV_MAX_VY", "0.0"))
        self.max_wz = float(os.getenv("OLI_NAV_MAX_WZ", "0.30"))
        self.min_turn_wz = float(os.getenv("OLI_NAV_MIN_TURN_WZ", "0.24"))
        self.turn_deadband = float(os.getenv("OLI_NAV_TURN_DEADBAND", "0.04"))
        self.linear_accel = float(os.getenv("OLI_NAV_LINEAR_ACCEL", "0.10"))
        self.linear_decel = float(os.getenv("OLI_NAV_LINEAR_DECEL", "0.20"))
        self.angular_accel = float(os.getenv("OLI_NAV_ANGULAR_ACCEL", "0.30"))
        self.command_timeout = float(os.getenv("OLI_NAV_COMMAND_TIMEOUT", "0.35"))
        self.arm_warmup = float(os.getenv("OLI_NAV_ARM_WARMUP", "2.0"))
        self.output_rate = float(os.getenv("OLI_NAV_OUTPUT_RATE", "50.0"))
        self.base_yaw_offset = math.radians(
            float(os.getenv("OLI_NAV_BASE_YAW_OFFSET_DEG", "0.0"))
        )
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_socket.bind((
            os.getenv("OLI_ROS_STATE_HOST", "127.0.0.1"),
            int(os.getenv("OLI_ROS_STATE_PORT", "15002")),
        ))
        self.state_socket.setblocking(False)
        self.last_log_time = 0.0
        self.last_state_log_time = 0.0
        self.fall_latched = False
        self.target_command = (0.0, 0.0, 0.0)
        self.output_command = (0.0, 0.0, 0.0)
        self.last_input_time = 0.0
        self.last_output_time = time.monotonic()
        self.was_armed = False
        self.armed_at = 0.0
        self.subscription = self.create_subscription(
            Twist, self.topic, self.command_callback, 10
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/odom/mujoco_odom", 20
        )
        self.joint_publisher = self.create_publisher(
            JointState, "/joint_states", 20
        )
        self.lidar_publisher = self.create_publisher(
            PointCloud2, "/livox/lidar", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.create_timer(0.01, self.poll_state)
        self.create_timer(1.0 / self.output_rate, self.publish_command)
        self.publish_static_transforms()
        self.get_logger().info(
            f"Forwarding {self.topic} to udp://{self.host}:{self.port}"
        )
        self.get_logger().info(
            "Publishing /odom/mujoco_odom, /joint_states and /livox/lidar"
        )

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
        base_to_lidar.transform.translation.z = 0.45
        base_to_lidar.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([map_to_odom, base_to_lidar])

    def command_callback(self, message: Twist) -> None:
        vx = max(
            -self.max_reverse_vx,
            min(self.max_vx, float(message.linear.x)),
        )
        if abs(vx) < self.linear_deadband:
            vx = 0.0
        elif 0.0 < vx < self.min_forward_vx:
            vx = self.min_forward_vx
        elif -self.max_reverse_vx < vx < 0.0:
            vx = -self.max_reverse_vx
        vy = max(-self.max_vy, min(self.max_vy, float(message.linear.y)))
        wz = max(-self.max_wz, min(self.max_wz, float(message.angular.z)))
        if abs(wz) < self.turn_deadband:
            wz = 0.0
        elif abs(wz) < self.min_turn_wz:
            # The vendor policy ignores small yaw commands, so promote real
            # heading corrections above its measured angular dead zone.
            wz = math.copysign(self.min_turn_wz, wz)

        # Humanoid balance degrades when a large turn and full forward command
        # arrive together. Preserve steering while reducing stride length.
        if self.max_wz > 0.0:
            turn_ratio = min(1.0, abs(wz) / self.max_wz)
            forward_limit = self.max_vx * (1.0 - 0.65 * turn_ratio)
            vx = min(vx, max(0.04, forward_limit))

        self.target_command = (vx, vy, wz)
        self.last_input_time = time.monotonic()

    @staticmethod
    def _slew(
        current: float,
        target: float,
        accel: float,
        decel: float,
        dt: float,
    ) -> float:
        rate = decel if abs(target) < abs(current) else accel
        delta = max(-rate * dt, min(rate * dt, target - current))
        return current + delta

    def publish_command(self) -> None:
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_output_time))
        self.last_output_time = now
        armed = os.path.exists(self.arm_file)

        if armed and not self.was_armed:
            self.armed_at = now
            self.output_command = (0.0, 0.0, 0.0)
        self.was_armed = armed

        input_fresh = now - self.last_input_time <= self.command_timeout
        warmed_up = armed and now - self.armed_at >= self.arm_warmup
        target = (
            self.target_command
            if input_fresh and warmed_up
            else (0.0, 0.0, 0.0)
        )
        if not armed:
            target = (0.0, 0.0, 0.0)

        current_vx, current_vy, current_wz = self.output_command
        next_command = (
            self._slew(
                current_vx, target[0], self.linear_accel, self.linear_decel, dt
            ),
            self._slew(
                current_vy, target[1], self.linear_accel, self.linear_decel, dt
            ),
            self._slew(
                current_wz,
                target[2],
                self.angular_accel,
                self.angular_accel,
                dt,
            ),
        )
        if not armed:
            next_command = (0.0, 0.0, 0.0)
        self.output_command = next_command
        self.socket.sendto(
            struct.pack("<fff", *next_command), (self.host, self.port)
        )

        now = time.monotonic()
        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                "%s cmd_vel vx=%.3f vy=%.3f wz=%.3f"
                % ("ARMED" if armed else "DISARMED", *next_command)
            )
            self.last_log_time = now

    def poll_state(self) -> None:
        latest = None
        while True:
            try:
                payload, _ = self.state_socket.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                latest = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        if latest is not None:
            self.publish_state(latest)

    def publish_state(self, state) -> None:
        now = self.get_clock().now().to_msg()
        position = state["position"]
        qw, qx, qy, qz = state["quaternion_wxyz"]
        linear_world = state["linear_velocity"]
        angular = state["angular_velocity"]

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )

        offset_w = math.cos(self.base_yaw_offset / 2.0)
        offset_z = math.sin(self.base_yaw_offset / 2.0)
        nav_qw = qw * offset_w - qz * offset_z
        nav_qx = qx * offset_w + qy * offset_z
        nav_qy = -qx * offset_z + qy * offset_w
        nav_qz = qw * offset_z + qz * offset_w
        nav_yaw = yaw + self.base_yaw_offset

        odom.pose.pose.orientation.w = nav_qw
        odom.pose.pose.orientation.x = nav_qx
        odom.pose.pose.orientation.y = nav_qy
        odom.pose.pose.orientation.z = nav_qz

        # A navigation goal can keep publishing after a humanoid has lost its
        # footing.  Latch the bridge closed before forwarding another command;
        # the simulator can then be recovered without the old goal moving it.
        upright = position[2] > 0.62 and abs(roll) < 0.75 and abs(pitch) < 0.75
        if os.path.exists(self.arm_file) and not upright:
            try:
                os.remove(self.arm_file)
            except FileNotFoundError:
                pass
            self.socket.sendto(
                struct.pack("<fff", 0.0, 0.0, 0.0), (self.host, self.port)
            )
            self.target_command = (0.0, 0.0, 0.0)
            self.output_command = (0.0, 0.0, 0.0)
            if not self.fall_latched:
                self.get_logger().error(
                    "FALL PROTECTION: navigation DISARMED "
                    "(z=%.2f roll=%.2f pitch=%.2f)"
                    % (position[2], roll, pitch)
                )
                self.fall_latched = True
        elif upright and not os.path.exists(self.arm_file):
            self.fall_latched = False

        odom.twist.twist.linear.x = (
            math.cos(nav_yaw) * linear_world[0]
            + math.sin(nav_yaw) * linear_world[1]
        )
        odom.twist.twist.linear.y = (
            -math.sin(nav_yaw) * linear_world[0]
            + math.cos(nav_yaw) * linear_world[1]
        )
        odom.twist.twist.linear.z = linear_world[2]
        odom.twist.twist.angular.x = angular[0]
        odom.twist.twist.angular.y = angular[1]
        odom.twist.twist.angular.z = angular[2]
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = now
        joints.name = state["joint_names"]
        joints.position = state["joint_positions"]
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
            struct.pack("<fff", *point) for point in state["lidar_points"]
        )
        self.lidar_publisher.publish(cloud)

        monotonic_now = time.monotonic()
        if monotonic_now - self.last_state_log_time >= 5.0:
            self.get_logger().info(
                "OLI state x=%.2f y=%.2f z=%.2f lidar_points=%d"
                % (position[0], position[1], position[2], cloud.width)
            )
            self.last_state_log_time = monotonic_now


def main() -> None:
    rclpy.init()
    node = OliCmdVelBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
