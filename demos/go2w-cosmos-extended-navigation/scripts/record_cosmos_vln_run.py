#!/usr/bin/python3
"""Record a Cosmos VLN mission and copy delayed visualization keyframes."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import time
from typing import Any
import xml.etree.ElementTree as ET

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ROOT_DIR = Path(__file__).resolve().parents[1]
TERMINAL_STATES = {"succeeded", "failed", "blocked", "canceled"}
TERRAIN_CENTER_LABELS = {
    "Cylinder37": "extended_stair_up_center_cylinder",
    "Cylinder38": "extended_stair_down_center_cylinder",
    "Cylinder39": "extended_ramp_up_center_cylinder",
    "Cylinder41": "extended_ramp_down_center_cylinder",
}


def safe_name(value: object) -> str:
    rendered = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value).strip())
    return rendered.strip("_.") or "event"


def parse_json(data: str) -> dict[str, Any]:
    try:
        value = json.loads(data)
    except json.JSONDecodeError:
        return {"raw": data}
    return value if isinstance(value, dict) else {"value": value}


def quaternion_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def load_terrain_center_obstacles() -> dict[str, dict[str, Any]]:
    scene_value = os.environ.get("GO2W_WORLD_SCENE", "").strip()
    if not scene_value:
        return {}
    scene = Path(scene_value).expanduser()
    if not scene.is_absolute():
        scene = ROOT_DIR / scene
    try:
        root = ET.parse(scene).getroot()
    except (OSError, ET.ParseError):
        return {}
    obstacles: dict[str, dict[str, Any]] = {}
    for geom in root.findall(".//worldbody/geom"):
        name = str(geom.get("name") or "")
        if name not in TERRAIN_CENTER_LABELS or geom.get("type") != "cylinder":
            continue
        try:
            x, y, z = (float(value) for value in geom.get("pos", "").split())
            radius, half_height = (
                float(value) for value in geom.get("size", "").split()[:2]
            )
        except (TypeError, ValueError):
            continue
        obstacles[TERRAIN_CENTER_LABELS[name]] = {
            "xml_name": name,
            "x": x,
            "y": y,
            "z": z,
            "radius_m": radius,
            "half_height_m": half_height,
        }
    return obstacles


class RunRecorder(Node):
    def __init__(self, artifact_dir: Path, timeout_sec: float) -> None:
        super().__init__("cosmos_vln_run_recorder")
        self.artifact_dir = artifact_dir
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.timeout_sec = timeout_sec
        self.started_monotonic = time.monotonic()
        self.started_wall = time.time()
        self.events_path = self.artifact_dir / "events.jsonl"
        self.events_file = self.events_path.open("a", encoding="utf-8", buffering=1)
        self.odom_path = self.artifact_dir / "odom_physics.csv"
        self.odom_file = self.odom_path.open("w", encoding="utf-8", buffering=1)
        self.odom_file.write(
            "wall_time,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,vx_mps,vy_mps,wz_rps\n"
        )
        self.latest_visualization = (
            ROOT_DIR / ".run/cosmos_vln/latest_visualization.jpg"
        )
        self.pending_snapshots: list[tuple[float, str]] = []
        self.snapshots: list[str] = []
        self.stage_results: list[dict[str, Any]] = []
        self.latest_route_command: dict[str, Any] = {}
        self.latest_planner_status: dict[str, Any] = {}
        self.latest_pose: dict[str, float] = {}
        self.latest_world_model: dict[str, Any] = {}
        self.dynamic_status_samples = 0
        self.min_dynamic_center_distance_m = math.inf
        self.nearest_dynamic_actor = ""
        self.terrain_center_obstacles = load_terrain_center_obstacles()
        self.terrain_avoidance_metrics: dict[str, dict[str, Any]] = {
            name: {
                **obstacle,
                "min_robot_center_distance_m": math.inf,
                "closest_robot_pose": None,
                "passed": False,
            }
            for name, obstacle in self.terrain_center_obstacles.items()
        }
        self.previous_xy: tuple[float, float] | None = None
        self.travel_distance_m = 0.0
        self.terminal_payload: dict[str, Any] | None = None
        self.continuous_action_requests = 0
        self.staged_action_requests = 0
        self.ramp_approach_z: float | None = None
        self.ramp_peak_z = -math.inf
        self.ramp_exit_z: float | None = None
        self.ramp_up_max_abs_pitch = 0.0
        self.ramp_down_max_abs_pitch = 0.0
        self.finish_after = math.inf
        self.done = False
        self.exit_code = 1

        self.create_subscription(
            String, "/cosmos_vln/mission_status", self.on_mission_status, 20
        )
        self.create_subscription(
            String, "/cosmos_vln/route_command", self.on_route_command, 10
        )
        self.create_subscription(String, "/cosmos_vln/status", self.on_status, 10)
        self.create_subscription(
            String, "/go2w/world_model/status", self.on_world_model_status, 10
        )
        self.create_subscription(
            Odometry, "/odom/mujoco_odom", self.on_odom, 20
        )
        self.create_timer(0.1, self.on_timer)
        self.write_event("recorder", {"state": "ready"})

    def write_event(self, topic: str, payload: dict[str, Any]) -> None:
        event = {"received_at": time.time(), "topic": topic, "payload": payload}
        self.events_file.write(json.dumps(event, ensure_ascii=False) + "\n")

    def schedule_snapshot(self, label: str, delay_sec: float = 0.75) -> None:
        self.pending_snapshots.append(
            (time.monotonic() + max(0.0, delay_sec), safe_name(label))
        )

    def copy_snapshot(self, label: str) -> None:
        if not self.latest_visualization.is_file():
            self.write_event(
                "recorder",
                {"state": "snapshot_missing", "label": label},
            )
            return
        destination = self.artifact_dir / f"{label}.jpg"
        shutil.copy2(self.latest_visualization, destination)
        if destination.name not in self.snapshots:
            self.snapshots.append(destination.name)
        self.write_event(
            "recorder",
            {"state": "snapshot_saved", "path": destination.name},
        )

    def on_route_command(self, message: String) -> None:
        payload = parse_json(message.data)
        self.latest_route_command = payload
        self.write_event("route_command", payload)

    def on_status(self, message: String) -> None:
        payload = parse_json(message.data)
        self.latest_planner_status = payload
        self.write_event("cosmos_status", payload)

    def on_world_model_status(self, message: String) -> None:
        payload = parse_json(message.data)
        self.latest_world_model = payload
        self.dynamic_status_samples += 1
        self.write_event("world_model_status", payload)

    def on_mission_status(self, message: String) -> None:
        payload = parse_json(message.data)
        self.write_event("mission_status", payload)
        state = str(payload.get("state", ""))
        if state == "started":
            self.schedule_snapshot("visual_00_start")
        elif state == "continuous_goal_requested":
            self.continuous_action_requests += 1
        elif state == "goal_requested":
            self.staged_action_requests += 1
        elif state == "stage_completed":
            stage_index = int(payload.get("stage_index", len(self.stage_results)))
            stage_id = safe_name(payload.get("stage_id", f"stage_{stage_index + 1}"))
            self.stage_results.append(payload)
            self.schedule_snapshot(f"visual_{stage_index + 1:02d}_{stage_id}")
        if state in TERMINAL_STATES and self.terminal_payload is None:
            self.terminal_payload = payload
            self.exit_code = 0 if state == "succeeded" else 1
            self.schedule_snapshot("visual_final", delay_sec=0.75)
            self.finish_after = time.monotonic() + 1.5

    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        current = (float(position.x), float(position.y))
        if self.previous_xy is not None:
            delta = math.hypot(
                current[0] - self.previous_xy[0], current[1] - self.previous_xy[1]
            )
            if delta <= 0.5:
                self.travel_distance_m += delta
        self.previous_xy = current
        self.latest_pose = {
            "x": current[0],
            "y": current[1],
            "z": float(position.z),
            "roll_rad": roll,
            "pitch_rad": pitch,
            "yaw_rad": yaw,
        }
        self.odom_file.write(
            f"{time.time():.6f},{current[0]:.6f},{current[1]:.6f},"
            f"{float(position.z):.6f},{roll:.6f},{pitch:.6f},{yaw:.6f},"
            f"{float(linear.x):.6f},{float(linear.y):.6f},{float(angular.z):.6f}\n"
        )
        if 12.10 <= current[1] <= 12.85:
            if self.ramp_approach_z is None:
                self.ramp_approach_z = float(position.z)
            else:
                self.ramp_approach_z = min(self.ramp_approach_z, float(position.z))
        if 12.75 <= current[1] <= 15.55:
            self.ramp_peak_z = max(self.ramp_peak_z, float(position.z))
            self.ramp_up_max_abs_pitch = max(self.ramp_up_max_abs_pitch, abs(pitch))
        if 15.45 <= current[1] <= 18.30:
            self.ramp_peak_z = max(self.ramp_peak_z, float(position.z))
            self.ramp_down_max_abs_pitch = max(
                self.ramp_down_max_abs_pitch, abs(pitch)
            )
        if 17.80 <= current[1] <= 18.85:
            self.ramp_exit_z = float(position.z)
        for metric in self.terrain_avoidance_metrics.values():
            distance = math.hypot(current[0] - metric["x"], current[1] - metric["y"])
            if distance < metric["min_robot_center_distance_m"]:
                metric["min_robot_center_distance_m"] = distance
                metric["closest_robot_pose"] = dict(self.latest_pose)
            if current[1] >= metric["y"] + 0.40:
                metric["passed"] = True
        actors = self.latest_world_model.get("dynamic_actors", [])
        if isinstance(actors, list):
            for actor in actors:
                if not isinstance(actor, dict):
                    continue
                try:
                    distance = math.hypot(
                        current[0] - float(actor["x"]),
                        current[1] - float(actor["y"]),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                if distance < self.min_dynamic_center_distance_m:
                    self.min_dynamic_center_distance_m = distance
                    self.nearest_dynamic_actor = str(actor.get("name", ""))

    def on_timer(self) -> None:
        now = time.monotonic()
        ready = [item for item in self.pending_snapshots if item[0] <= now]
        self.pending_snapshots = [
            item for item in self.pending_snapshots if item[0] > now
        ]
        for _, label in ready:
            self.copy_snapshot(label)

        if self.terminal_payload is None and now - self.started_monotonic > self.timeout_sec:
            self.terminal_payload = {
                "state": "timeout",
                "reason": "recorder_timeout",
                "time": time.time(),
            }
            self.schedule_snapshot("visual_final", delay_sec=0.0)
            self.finish_after = now + 0.5
            self.write_event("recorder", self.terminal_payload)
        if self.terminal_payload is not None and now >= self.finish_after:
            self.finalize()
            self.done = True

    def finalize(self) -> None:
        terminal = self.terminal_payload or {"state": "unknown"}
        nav_mode_file = ROOT_DIR / ".run/go2w/nav_mode"
        try:
            navigation_mode = nav_mode_file.read_text().strip()
        except OSError:
            navigation_mode = "unknown"
        route_id = terminal.get("route_id") or self.latest_route_command.get("route_id")
        peak_z = self.ramp_peak_z if math.isfinite(self.ramp_peak_z) else None
        elevation_gain = (
            peak_z - self.ramp_approach_z
            if peak_z is not None and self.ramp_approach_z is not None
            else None
        )
        elevation_drop = (
            peak_z - self.ramp_exit_z
            if peak_z is not None and self.ramp_exit_z is not None
            else None
        )
        required_gain = float(os.environ.get("COSMOS_VLN_RAMP_MIN_ELEVATION_GAIN", "0.30"))
        required_pitch = float(os.environ.get("COSMOS_VLN_RAMP_MIN_PITCH", "0.07"))
        crest_min_z = float(os.environ.get("COSMOS_VLN_RAMP_CREST_MIN_Z", "0.72"))
        exit_max_z = float(os.environ.get("COSMOS_VLN_RAMP_EXIT_MAX_Z", "0.58"))
        physical_ramp_metrics = {
            "passed": bool(
                elevation_gain is not None
                and elevation_drop is not None
                and elevation_gain >= required_gain
                and elevation_drop >= required_gain * 0.75
                and peak_z is not None
                and peak_z >= crest_min_z
                and self.ramp_up_max_abs_pitch >= required_pitch * 0.50
                and self.ramp_down_max_abs_pitch >= required_pitch
                and self.ramp_exit_z is not None
                and self.ramp_exit_z <= exit_max_z
            ),
            "approach_z_m": self.ramp_approach_z,
            "crest_max_z_m": peak_z,
            "exit_z_m": self.ramp_exit_z,
            "elevation_gain_m": elevation_gain,
            "elevation_drop_m": elevation_drop,
            "uphill_max_abs_pitch_rad": self.ramp_up_max_abs_pitch,
            "downhill_max_abs_pitch_rad": self.ramp_down_max_abs_pitch,
            "minimum_elevation_gain_m": required_gain,
            "minimum_pitch_rad": required_pitch,
            "trace": self.odom_path.name,
        }
        terrain_metrics: dict[str, dict[str, Any]] = {}
        for name, metric in self.terrain_avoidance_metrics.items():
            item = dict(metric)
            distance = item.get("min_robot_center_distance_m")
            if isinstance(distance, float) and math.isfinite(distance):
                item["min_robot_center_distance_m"] = distance
                item["min_robot_center_to_obstacle_surface_m"] = max(
                    0.0, distance - float(item["radius_m"])
                )
            else:
                item["min_robot_center_distance_m"] = None
                item["min_robot_center_to_obstacle_surface_m"] = None
            terrain_metrics[name] = item
        result = {
            "result": terminal.get("state", "unknown"),
            "reason": terminal.get("reason", ""),
            "mission_id": terminal.get("mission_id", ""),
            "mission": terminal.get("mission", ""),
            "route_id": route_id,
            "confidence": self.latest_route_command.get("confidence"),
            "cosmos_job_id": self.latest_route_command.get("job_id"),
            "started_at": self.started_wall,
            "finished_at": time.time(),
            "duration_sec": time.monotonic() - self.started_monotonic,
            "stage_count": terminal.get("stage_count"),
            "stage_results": self.stage_results,
            "future_prediction": self.latest_route_command.get(
                "future_prediction", terminal.get("future_prediction", {})
            ),
            "final_pose": self.latest_pose,
            "recorded_travel_distance_m": self.travel_distance_m,
            "world_model": self.latest_world_model,
            "dynamic_status_samples": self.dynamic_status_samples,
            "min_dynamic_center_distance_m": (
                self.min_dynamic_center_distance_m
                if math.isfinite(self.min_dynamic_center_distance_m)
                else None
            ),
            "nearest_dynamic_actor": self.nearest_dynamic_actor or None,
            "terrain_center_obstacle_count": len(terrain_metrics),
            "terrain_avoidance_metrics": terrain_metrics,
            "physical_ramp_metrics": physical_ramp_metrics,
            "continuous_navigation": {
                "execution_mode": terminal.get("execution_mode"),
                "navigate_through_poses_goal_count": self.continuous_action_requests,
                "navigate_to_pose_goal_count": self.staged_action_requests,
                "intermediate_goal_restarts": terminal.get("intermediate_goal_restarts"),
                "passed": (
                    terminal.get("execution_mode") == "navigate_through_poses"
                    and self.continuous_action_requests == 1
                    and self.staged_action_requests == 0
                    and terminal.get("intermediate_goal_restarts") == 0
                ),
            },
            "final_navigation_mode": navigation_mode,
            "snapshots": self.snapshots,
            "terminal_status": terminal,
        }
        temporary = self.artifact_dir / "result.json.tmp"
        temporary.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.artifact_dir / "result.json")
        self.write_event("recorder", {"state": "finalized", "result": result["result"]})
        self.events_file.flush()

    def close(self) -> None:
        if not self.events_file.closed:
            self.events_file.close()
        if not self.odom_file.closed:
            self.odom_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=1200.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RunRecorder(args.artifact_dir, max(30.0, args.timeout_sec))
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.terminal_payload = {"state": "interrupted", "reason": "keyboard_interrupt"}
        node.finalize()
        node.exit_code = 1
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
