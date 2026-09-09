#!/usr/bin/python3
"""Fuse the Go2-W world model into MATRiX's PointCloud2 stream.

The MATRiX UE sensor publishes the physical ray-cast cloud on
``/livox/lidar_raw``.  This node adds the cylinder geometry from the active
MuJoCo scene and, when explicitly requested, the moving ``human1`` actors
described by a MATRiX ``scene.json`` file.  The latter is intentionally an
opt-in overlay so the previously validated YardWorld route remains unchanged
unless an extended scene is selected.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / "scene/scene_terrain_yard_extended_nav.xml"
DEFAULT_JSON = ROOT / "scene/scene_go2w_extended_nav.json"
TERRAIN_CENTER_LABELS = {
    "Cylinder37": "extended_stair_up_center_cylinder",
    "Cylinder38": "extended_stair_down_center_cylinder",
    "Cylinder39": "extended_ramp_up_center_cylinder",
    "Cylinder41": "extended_ramp_down_center_cylinder",
}


def resolve_path(value: str | None, default: Path) -> Path:
    """Resolve an environment path without changing the old default contract."""
    if not value:
        return default
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate
    return (ROOT / candidate).resolve()


def xyz_from_json(value: object, *, scale: float = 0.01) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        return None
    try:
        result = tuple(float(value[axis]) * scale for axis in ("x", "y", "z"))
    except (KeyError, TypeError, ValueError):
        return None
    if len(result) != 3 or not all(math.isfinite(item) for item in result):
        return None
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class DynamicActor:
    name: str
    avoid: bool
    position: tuple[float, float, float]
    waypoints: tuple[tuple[float, float, float], ...]
    speed_mps: float

    @property
    def path(self) -> tuple[tuple[float, float, float], ...]:
        return (self.position, *self.waypoints) if self.waypoints else (self.position,)

    def at(self, elapsed_s: float) -> tuple[float, float, float]:
        """Return a ping-pong position along the JSON polyline."""
        path = self.path
        if len(path) == 1 or self.speed_mps <= 1e-5:
            return path[0]
        lengths = [math.dist(path[index], path[index + 1]) for index in range(len(path) - 1)]
        total = sum(lengths)
        if total <= 1e-5:
            return path[0]
        # The UE scene loader walks the trajectory and returns to its start.
        # A triangular (ping-pong) phase avoids teleporting at a loop seam and
        # keeps generated LiDAR returns deterministic.
        period = 2.0 * total / self.speed_mps
        phase = (elapsed_s % period) * self.speed_mps
        distance = phase if phase <= total else 2.0 * total - phase
        for index, segment_length in enumerate(lengths):
            if distance <= segment_length or index == len(lengths) - 1:
                ratio = 0.0 if segment_length <= 1e-5 else distance / segment_length
                start = path[index]
                end = path[index + 1]
                return tuple(
                    start[axis] + ratio * (end[axis] - start[axis]) for axis in range(3)
                )  # type: ignore[return-value]
            distance -= segment_length
        return path[-1]


@dataclass(frozen=True)
class StaticCylinder:
    name: str
    position: tuple[float, float, float]
    radius: float
    half_height: float


def parse_dynamic_actors(path: Path) -> tuple[DynamicActor, ...]:
    if not path.is_file():
        raise FileNotFoundError(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("scene JSON root must be an object")
    actors: list[DynamicActor] = []
    for key, element in payload.items():
        if not isinstance(element, dict) or element.get("type") != "dynamic":
            continue
        if element.get("model") != "human1":
            continue
        position = xyz_from_json(element.get("position"))
        if position is None:
            continue
        raw_trajectory = element.get("trajectory", {})
        trajectory_items: list[tuple[str, tuple[float, float, float]]] = []
        if isinstance(raw_trajectory, dict):
            for name, raw_point in raw_trajectory.items():
                point = xyz_from_json(raw_point)
                if point is not None:
                    trajectory_items.append((str(name), point))
        trajectory_items.sort(key=lambda item: item[0])
        try:
            speed = float(element.get("velocity", 0.25))
        except (TypeError, ValueError):
            speed = 0.25
        if not math.isfinite(speed):
            speed = 0.25
        actors.append(
            DynamicActor(
                name=str(element.get("name", key)),
                avoid=bool(element.get("avoid", True)),
                position=position,
                waypoints=tuple(point for _, point in trajectory_items),
                speed_mps=max(0.0, speed),
            )
        )
    return tuple(actors)


def rotate(quat: np.ndarray, vectors: np.ndarray) -> np.ndarray:
    xyz = quat[:3]
    w = quat[3]
    cross = np.cross(np.broadcast_to(xyz, vectors.shape), vectors)
    return vectors + 2.0 * (w * cross + np.cross(np.broadcast_to(xyz, vectors.shape), cross))


def parse_static_cylinders(scene: Path) -> tuple[StaticCylinder, ...]:
    root = ET.parse(scene).getroot()
    cylinders: list[StaticCylinder] = []
    for index, geom in enumerate(root.findall(".//worldbody/geom")):
        if geom.get("type") != "cylinder":
            continue
        # UE-only visual twins are intentionally non-colliding and must not
        # be injected into the navigation cloud a second time.
        if geom.get("contype") == "0" and geom.get("conaffinity") == "0":
            continue
        try:
            pos = tuple(float(value) for value in geom.get("pos", "0 0 0").split())
            size = tuple(float(value) for value in geom.get("size", "0 0").split())
        except ValueError:
            continue
        if len(pos) != 3 or len(size) < 2 or size[0] <= 0 or size[1] <= 0:
            continue
        cylinders.append(
            StaticCylinder(
                name=str(geom.get("name") or f"cylinder_{index}"),
                position=pos,
                radius=size[0],
                half_height=size[1],
            )
        )
    if not cylinders:
        raise RuntimeError(f"no world-model cylinders found in {scene}")
    return tuple(cylinders)


def cylinder_points(cylinders: tuple[StaticCylinder, ...]) -> np.ndarray:
    points: list[tuple[float, float, float]] = []
    for cylinder in cylinders:
        pos = cylinder.position
        radius = cylinder.radius
        half_height = cylinder.half_height
        # Dense side-wall returns keep slim bollards observable at the local
        # costmap's 8 cm voxel resolution while remaining inexpensive.
        for z in np.linspace(
            pos[2] - half_height + 0.03,
            pos[2] + half_height - 0.03,
            max(3, int(math.ceil(half_height * 10))),
        ):
            for angle in np.linspace(0.0, 2.0 * math.pi, 48, endpoint=False):
                points.append(
                    (pos[0] + radius * math.cos(angle), pos[1] + radius * math.sin(angle), z)
                )
    return np.asarray(points, dtype=np.float32)


def human_points(actor_positions: tuple[tuple[str, tuple[float, float, float]], ...]) -> np.ndarray:
    """Make a compact LiDAR silhouette for each JSON human position."""
    points: list[tuple[float, float, float]] = []
    for _, (x, y, _) in actor_positions:
        # Torso and head radii are deliberately conservative: the local
        # planner sees a person before the UE mesh reaches the robot footprint.
        for z in np.linspace(0.08, 1.38, 8):
            radius = 0.25 if z < 1.15 else 0.18
            for angle in np.linspace(0.0, 2.0 * math.pi, 20, endpoint=False):
                points.append((x + radius * math.cos(angle), y + radius * math.sin(angle), float(z)))
        for z in np.linspace(1.42, 1.68, 3):
            for angle in np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False):
                points.append((x + 0.16 * math.cos(angle), y + 0.16 * math.sin(angle), float(z)))
    return np.asarray(points, dtype=np.float32) if points else np.empty((0, 3), dtype=np.float32)


class WorldObstacleCloud(Node):
    def __init__(self) -> None:
        super().__init__("go2w_world_obstacle_cloud")
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scene = resolve_path(os.environ.get("GO2W_WORLD_SCENE"), DEFAULT_SCENE)
        self.scene_json: Path | None = None
        self.actors: tuple[DynamicActor, ...] = ()
        requested_json = os.environ.get("GO2W_SCENE_JSON", "").strip()
        if requested_json:
            self.scene_json = resolve_path(requested_json, DEFAULT_JSON)
            try:
                self.actors = parse_dynamic_actors(self.scene_json)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self.get_logger().error(f"cannot load dynamic scene JSON {self.scene_json}: {exc}")
                raise
        self.static_cylinders = parse_static_cylinders(self.scene)
        self.world_points = cylinder_points(self.static_cylinders)
        self.started_at = time.monotonic()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar", sensor_qos)
        self.subscription = self.create_subscription(
            PointCloud2, "/livox/lidar_raw", self.on_cloud, sensor_qos
        )
        self.marker_publisher = self.create_publisher(MarkerArray, "/go2w/dynamic_obstacles/markers", 10)
        self.status_publisher = self.create_publisher(String, "/go2w/world_model/status", 10)
        self.published = 0
        self.last_status_at = 0.0
        self.get_logger().info(
            "world-model fusion armed: "
            f"scene={self.scene.name} static_cylinder_points={len(self.world_points)} "
            f"dynamic_actors={len(self.actors)} "
            f"json={self.scene_json.name if self.scene_json else 'disabled'}"
        )

    def actor_positions(self, elapsed_s: float) -> tuple[tuple[str, tuple[float, float, float]], ...]:
        return tuple((actor.name, actor.at(elapsed_s)) for actor in self.actors)

    def publish_markers(self, positions: tuple[tuple[str, tuple[float, float, float]], ...]) -> None:
        markers: list[Marker] = []
        marker_id = 0
        for cylinder in self.static_cylinders:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "go2w_static_cylinders"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = cylinder.position[0]
            marker.pose.position.y = cylinder.position[1]
            marker.pose.position.z = cylinder.position[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * cylinder.radius
            marker.scale.y = 2.0 * cylinder.radius
            marker.scale.z = 2.0 * cylinder.half_height
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.04
            marker.color.a = 0.78 if cylinder.name in TERRAIN_CENTER_LABELS else 0.32
            marker.lifetime.sec = 1
            markers.append(marker)
            marker_id += 1
        for name, (x, y, _) in positions:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "go2w_json_pedestrians"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.84
            marker.scale.x = 0.50
            marker.scale.y = 0.50
            marker.scale.z = 1.68
            marker.color.r = 0.75
            marker.color.g = 0.10
            marker.color.b = 0.85
            marker.color.a = 0.70
            marker.lifetime.sec = 1
            markers.append(marker)
            marker_id += 1
        self.marker_publisher.publish(MarkerArray(markers=markers))

    def publish_status(self, positions: tuple[tuple[str, tuple[float, float, float]], ...], modeled: int) -> None:
        now = time.monotonic()
        if now - self.last_status_at < 1.0:
            return
        self.last_status_at = now
        payload = {
            "scene": str(self.scene),
            "scene_json": str(self.scene_json) if self.scene_json else None,
            "dynamic_actor_count": len(positions),
            "dynamic_actors": [
                {"name": name, "x": round(float(position[0]), 3), "y": round(float(position[1]), 3)}
                for name, position in positions
            ],
            "static_cylinder_count": len(self.static_cylinders),
            "terrain_center_cylinders": [
                {
                    "name": TERRAIN_CENTER_LABELS[cylinder.name],
                    "xml_name": cylinder.name,
                    "x": round(cylinder.position[0], 3),
                    "y": round(cylinder.position[1], 3),
                    "z": round(cylinder.position[2], 3),
                    "radius": round(cylinder.radius, 3),
                }
                for cylinder in self.static_cylinders
                if cylinder.name in TERRAIN_CENTER_LABELS
            ],
            "static_modeled_points": int(len(self.world_points)),
            "dynamic_modeled_points": int(modeled),
            "published_clouds": self.published,
        }
        self.status_publisher.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def on_cloud(self, message: PointCloud2) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("lidar", "odom", Time())
        except TransformException as exc:
            self.get_logger().warning(f"waiting for odom -> lidar TF: {exc}", throttle_duration_sec=2.0)
            return

        raw = point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        if isinstance(raw, np.ndarray) and raw.dtype.names:
            lidar_points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float32, copy=False)
        else:
            lidar_points = np.asarray(list(raw), dtype=np.float32)

        elapsed = time.monotonic() - self.started_at
        positions = self.actor_positions(elapsed)
        dynamic_world = human_points(positions)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        quat = np.asarray((rotation.x, rotation.y, rotation.z, rotation.w), dtype=np.float32)
        offset = np.asarray((translation.x, translation.y, translation.z), dtype=np.float32)
        modeled_world = np.concatenate((self.world_points, dynamic_world), axis=0)
        modeled = rotate(quat, modeled_world) + offset
        fused = np.concatenate((lidar_points, modeled.astype(np.float32, copy=False)), axis=0)
        header = message.header
        header.frame_id = "lidar"
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, fused))
        self.publish_markers(positions)
        self.publish_status(positions, len(dynamic_world))
        self.published += 1
        if self.published == 1:
            self.get_logger().info(
                f"publishing fused /livox/lidar: raw={len(lidar_points)} "
                f"static={len(self.world_points)} dynamic={len(dynamic_world)}"
            )


def main() -> None:
    rclpy.init()
    node = WorldObstacleCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
