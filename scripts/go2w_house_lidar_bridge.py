#!/usr/bin/env python3
"""Normalize HouseWorld LiDAR and optionally add nearby MJCF geometry.

The default path is intentionally a relay only: ``/livox/lidar_raw`` is
republished as ``/livox/lidar`` with the canonical ``lidar`` frame. Model
geometry is an opt-in fallback for simulator assets that are not visible to
the Mid360 sensor.
"""

from __future__ import annotations

import copy
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    ROOT
    / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_house.xml"
)
INPUT_TOPIC = "/livox/lidar_raw"
OUTPUT_TOPIC = "/livox/lidar"
OUTPUT_FRAME = "lidar"
FUSION_ENV = "GO2W_HOUSE_MODEL_FUSION"


def env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off", ""}:
        return False
    raise ValueError(f"{name} must be one of 0/1, false/true, no/yes, off/on")


def model_fusion_enabled() -> bool:
    return env_flag(FUSION_ENV, default=False)


def _numbers(value: str | None, expected: int, default: tuple[float, ...]) -> np.ndarray:
    if value is None:
        parsed = default
    else:
        parsed = tuple(float(item) for item in value.split())
    if len(parsed) < expected:
        raise ValueError(f"expected at least {expected} values, got {parsed}")
    return np.asarray(parsed[:expected], dtype=np.float32)


def _normalize_quaternion_wxyz(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-8:
        raise ValueError("zero-length MJCF quaternion")
    return quaternion / norm


def rotate_wxyz(quaternion: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate row vectors by an MJCF quaternion ordered as w, x, y, z."""

    w, x, y, z = _normalize_quaternion_wxyz(quaternion)
    rotation = np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float32,
    )
    return vectors @ rotation.T


def rotate_xyzw(quaternion: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    """Rotate row vectors by a ROS quaternion ordered as x, y, z, w."""

    return rotate_wxyz(
        np.asarray(
            (quaternion[3], quaternion[0], quaternion[1], quaternion[2]),
            dtype=np.float32,
        ),
        vectors,
    )


def _axis_samples(half_extent: float, spacing: float) -> np.ndarray:
    count = max(2, int(math.ceil((2.0 * half_extent) / spacing)) + 1)
    return np.linspace(-half_extent, half_extent, count, dtype=np.float32)


def _box_surface_points(size: np.ndarray, spacing: float) -> np.ndarray:
    x_values = _axis_samples(float(size[0]), spacing)
    y_values = _axis_samples(float(size[1]), spacing)
    z_values = _axis_samples(float(size[2]), spacing)
    points: list[tuple[float, float, float]] = []

    # Vertical faces are sufficient for 2-D obstacle marking and avoid filling
    # large pieces of furniture with redundant interior/top samples.
    for z_value in z_values:
        for x_value in x_values:
            points.append((float(x_value), -float(size[1]), float(z_value)))
            points.append((float(x_value), float(size[1]), float(z_value)))
        for y_value in y_values[1:-1]:
            points.append((-float(size[0]), float(y_value), float(z_value)))
            points.append((float(size[0]), float(y_value), float(z_value)))
    return np.asarray(points, dtype=np.float32)


def _cylinder_surface_points(size: np.ndarray, spacing: float) -> np.ndarray:
    radius = float(size[0])
    half_height = float(size[1])
    angle_count = max(12, int(math.ceil((2.0 * math.pi * radius) / spacing)))
    angles = np.linspace(0.0, 2.0 * math.pi, angle_count, endpoint=False)
    z_values = _axis_samples(half_height, spacing)
    return np.asarray(
        [
            (radius * math.cos(angle), radius * math.sin(angle), float(z_value))
            for z_value in z_values
            for angle in angles
        ],
        dtype=np.float32,
    )


def load_house_model_points(
    scene: Path = DEFAULT_SCENE,
    *,
    spacing: float = 0.12,
) -> np.ndarray:
    """Sample HouseWorld box/cylinder surfaces in the MJCF world frame."""

    if spacing < 0.04 or spacing > 0.50:
        raise ValueError("model point spacing must be between 0.04 m and 0.50 m")
    root = ET.parse(scene).getroot()
    point_groups: list[np.ndarray] = []
    supported_geometries = 0
    for geometry in root.findall(".//worldbody//geom"):
        geometry_type = geometry.get("type", "sphere")
        if geometry_type not in {"box", "cylinder"}:
            continue
        size_count = 3 if geometry_type == "box" else 2
        size = _numbers(geometry.get("size"), size_count, (0.0,) * size_count)
        if np.any(size <= 0.0):
            continue
        local_points = (
            _box_surface_points(size, spacing)
            if geometry_type == "box"
            else _cylinder_surface_points(size, spacing)
        )
        quaternion = _numbers(
            geometry.get("quat"), 4, (1.0, 0.0, 0.0, 0.0)
        )
        position = _numbers(geometry.get("pos"), 3, (0.0, 0.0, 0.0))
        point_groups.append(rotate_wxyz(quaternion, local_points) + position)
        supported_geometries += 1

    if supported_geometries == 0 or not point_groups:
        raise RuntimeError(f"no HouseWorld box/cylinder geometry found in {scene}")
    return np.concatenate(point_groups, axis=0).astype(np.float32, copy=False)


def filter_nearby_points(points_in_lidar: np.ndarray, radius: float) -> np.ndarray:
    if radius <= 0.0:
        raise ValueError("model fusion radius must be positive")
    if not len(points_in_lidar):
        return points_in_lidar
    planar_distance_squared = np.square(points_in_lidar[:, :2]).sum(axis=1)
    return points_in_lidar[planar_distance_squared <= radius * radius]


def _build_node_class():
    # ROS imports stay inside the runtime factory so offline contract tests can
    # import and exercise the model helpers without sourcing a ROS environment.
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    class HouseLidarBridge(Node):
        def __init__(self) -> None:
            super().__init__("go2w_house_lidar_bridge")
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.fusion_enabled = model_fusion_enabled()
            self.publisher_conflict = False
            self.publisher = self.create_publisher(PointCloud2, OUTPUT_TOPIC, sensor_qos)
            self.subscription = self.create_subscription(
                PointCloud2, INPUT_TOPIC, self.on_cloud, sensor_qos
            )
            self.guard_timer = self.create_timer(1.0, self.guard_single_publisher)
            self.published = 0

            self.world_points = np.empty((0, 3), dtype=np.float32)
            self.fusion_radius = 0.0
            self.tf_buffer = None
            self.tf_listener = None
            if self.fusion_enabled:
                scene = Path(os.environ.get("GO2W_HOUSE_MODEL_SCENE", str(DEFAULT_SCENE)))
                spacing = float(
                    os.environ.get("GO2W_HOUSE_MODEL_POINT_SPACING_M", "0.12")
                )
                self.fusion_radius = float(
                    os.environ.get("GO2W_HOUSE_MODEL_FUSION_RADIUS_M", "6.0")
                )
                if self.fusion_radius < 0.5 or self.fusion_radius > 12.0:
                    raise ValueError(
                        "GO2W_HOUSE_MODEL_FUSION_RADIUS_M must be in [0.5, 12.0]"
                    )
                self.world_points = load_house_model_points(scene, spacing=spacing)
                self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
                self.tf_listener = TransformListener(self.tf_buffer, self)
                self.get_logger().warning(
                    "House model fusion explicitly enabled: "
                    f"points={len(self.world_points)} radius={self.fusion_radius:.1f}m"
                )
            else:
                self.get_logger().info(
                    f"House LiDAR relay active: {INPUT_TOPIC} -> {OUTPUT_TOPIC} "
                    f"frame={OUTPUT_FRAME}; model fusion disabled"
                )

        def guard_single_publisher(self) -> None:
            publisher_count = self.count_publishers(OUTPUT_TOPIC)
            if publisher_count <= 1 or self.publisher_conflict:
                return
            self.publisher_conflict = True
            self.get_logger().fatal(
                f"refusing duplicate {OUTPUT_TOPIC} publishers: discovered "
                f"{publisher_count}; stop the YardWorld cloud bridge"
            )
            self.destroy_subscription(self.subscription)
            self.destroy_publisher(self.publisher)
            self.publisher = None
            rclpy.shutdown()

        @staticmethod
        def _raw_xyz(message: PointCloud2) -> np.ndarray:
            raw = point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
            if isinstance(raw, np.ndarray) and raw.dtype.names:
                return np.column_stack((raw["x"], raw["y"], raw["z"])).astype(
                    np.float32, copy=False
                )
            return np.asarray(list(raw), dtype=np.float32).reshape((-1, 3))

        def on_cloud(self, message: PointCloud2) -> None:
            if self.publisher_conflict or self.publisher is None:
                return
            if not self.fusion_enabled:
                normalized = copy.copy(message)
                normalized.header = copy.copy(message.header)
                normalized.header.frame_id = OUTPUT_FRAME
                self.publisher.publish(normalized)
                self.published += 1
                return

            try:
                transform = self.tf_buffer.lookup_transform(
                    OUTPUT_FRAME, "odom", Time()
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f"waiting for odom -> {OUTPUT_FRAME} TF: {exc}",
                    throttle_duration_sec=2.0,
                )
                return

            rotation = transform.transform.rotation
            translation = transform.transform.translation
            quaternion = np.asarray(
                (rotation.x, rotation.y, rotation.z, rotation.w), dtype=np.float32
            )
            offset = np.asarray(
                (translation.x, translation.y, translation.z), dtype=np.float32
            )
            modeled_in_lidar = rotate_xyzw(quaternion, self.world_points) + offset
            nearby_modeled = filter_nearby_points(
                modeled_in_lidar, self.fusion_radius
            ).astype(np.float32, copy=False)
            lidar_points = self._raw_xyz(message)
            fused = np.concatenate((lidar_points, nearby_modeled), axis=0)
            header = copy.copy(message.header)
            header.frame_id = OUTPUT_FRAME
            self.publisher.publish(point_cloud2.create_cloud_xyz32(header, fused))
            self.published += 1
            if self.published == 1:
                self.get_logger().info(
                    f"publishing fused House LiDAR: raw={len(lidar_points)} "
                    f"nearby_model={len(nearby_modeled)}"
                )

    return HouseLidarBridge, rclpy


def main() -> None:
    HouseLidarBridge, rclpy = _build_node_class()
    rclpy.init()
    node = HouseLidarBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
