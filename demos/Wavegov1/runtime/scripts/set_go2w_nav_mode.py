#!/usr/bin/python3
"""Atomically switch Go2-W Nav2/controller parameters in three batch requests."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
from typing import Any

from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node


def number(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return default


def vector(name: str, default: list[float]) -> list[float]:
    value = os.environ.get(name)
    if not value:
        return default
    try:
        parsed = json.loads(value)
        if isinstance(parsed, list) and len(parsed) == len(default):
            return [float(item) for item in parsed]
    except (TypeError, ValueError, json.JSONDecodeError):
        pass
    return default


def profile(mode: str) -> dict[str, dict[str, Any]]:
    if mode == "avoid":
        height = (0.10, 1.20)
        controller = {
            "FollowPath.vx_max": number("AVOID_NAV_VX_MAX", 0.45),
            "FollowPath.vx_min": number("AVOID_NAV_VX_MIN", -0.15),
            "FollowPath.wz_max": number("AVOID_NAV_WZ_MAX", 0.60),
            "FollowPath.vx_std": number("AVOID_NAV_VX_STD", 0.18),
            "FollowPath.wz_std": number("AVOID_NAV_WZ_STD", 0.45),
        }
        optimizer = {
            "max_velocity": vector("AVOID_NAV_MAX_VELOCITY", [0.45, 0.0, 0.60]),
            "min_velocity": vector("AVOID_NAV_MIN_VELOCITY", [-0.15, 0.0, -0.60]),
            "max_accel": vector("AVOID_NAV_MAX_ACCEL", [0.70, 0.0, 0.80]),
            "max_decel": vector("AVOID_NAV_MAX_DECEL", [-0.90, 0.0, -0.80]),
        }
    elif mode == "up":
        height = (0.65, 1.20)
        controller = {
            "FollowPath.vx_max": number("STAIR_NAV_VX_MAX", 0.80),
            "FollowPath.vx_min": number("STAIR_NAV_VX_MIN", 0.55),
            "FollowPath.wz_max": number("STAIR_NAV_WZ_MAX", 0.35),
            "FollowPath.vx_std": number("STAIR_NAV_VX_STD", 0.20),
            "FollowPath.wz_std": number("STAIR_NAV_WZ_STD", 0.40),
        }
        optimizer = {
            "max_velocity": vector("STAIR_NAV_MAX_VELOCITY", [0.80, 0.0, 0.35]),
            "min_velocity": vector("STAIR_NAV_MIN_VELOCITY", [-0.20, 0.0, -0.35]),
            "max_accel": vector("STAIR_NAV_MAX_ACCEL", [0.80, 0.0, 0.80]),
            "max_decel": vector("STAIR_NAV_MAX_DECEL", [-0.80, 0.0, -0.80]),
        }
    elif mode == "down":
        height = (0.65, 1.20)
        controller = {
            "FollowPath.vx_max": number("STAIR_DOWN_VX_MAX", 0.35),
            "FollowPath.vx_min": number("STAIR_DOWN_VX_MIN", -0.15),
            "FollowPath.wz_max": number("STAIR_DOWN_WZ_MAX", 0.50),
            "FollowPath.vx_std": number("STAIR_DOWN_VX_STD", 0.16),
            "FollowPath.wz_std": number("STAIR_DOWN_WZ_STD", 0.40),
        }
        optimizer = {
            "max_velocity": vector("STAIR_DOWN_MAX_VELOCITY", [0.35, 0.0, 0.50]),
            "min_velocity": vector("STAIR_DOWN_MIN_VELOCITY", [-0.15, 0.0, -0.50]),
            "max_accel": vector("STAIR_DOWN_MAX_ACCEL", [0.60, 0.0, 0.60]),
            "max_decel": vector("STAIR_DOWN_MAX_DECEL", [-0.60, 0.0, -0.60]),
        }
    elif mode == "slope":
        height = (0.55, 1.40)
        controller = {
            "FollowPath.vx_max": number("SLOPE_NAV_VX_MAX", 0.35),
            "FollowPath.vx_min": number("SLOPE_NAV_VX_MIN", 0.0),
            "FollowPath.wz_max": number("SLOPE_NAV_WZ_MAX", 0.55),
            "FollowPath.vx_std": number("SLOPE_NAV_VX_STD", 0.16),
            "FollowPath.wz_std": number("SLOPE_NAV_WZ_STD", 0.40),
        }
        optimizer = {
            "max_velocity": vector("SLOPE_NAV_MAX_VELOCITY", [0.35, 0.0, 0.55]),
            "min_velocity": vector("SLOPE_NAV_MIN_VELOCITY", [0.0, 0.0, -0.55]),
            "max_accel": vector("SLOPE_NAV_MAX_ACCEL", [0.65, 0.0, 0.75]),
            "max_decel": vector("SLOPE_NAV_MAX_DECEL", [-0.75, 0.0, -0.75]),
        }
    elif mode == "flat":
        height = (0.10, 1.20)
        controller = {
            "FollowPath.vx_max": 0.95,
            "FollowPath.vx_min": -0.45,
            "FollowPath.wz_max": 1.00,
            "FollowPath.vx_std": 0.20,
            "FollowPath.wz_std": 0.40,
        }
        optimizer = {
            "max_velocity": [0.95, 0.0, 1.00],
            "min_velocity": [-0.45, 0.0, -1.00],
            "max_accel": [1.20, 0.0, 1.20],
            "max_decel": [-1.00, 0.0, -1.20],
        }
    else:
        raise ValueError(f"unsupported mode: {mode}")
    if os.environ.get("GO2W_MJ_STATE_RELAY") == "1":
        controller["progress_checker.movement_time_allowance"] = 40.0
    local = {
        "obstacle_layer.enabled": True,
        "obstacle_layer.min_obstacle_height": height[0],
        "obstacle_layer.max_obstacle_height": height[1],
        "obstacle_layer.pointcloud.min_obstacle_height": height[0],
        "obstacle_layer.pointcloud.max_obstacle_height": height[1],
    }
    return {
        "/local_costmap/local_costmap": local,
        "/controller_server": controller,
        "/velocity_optimizer": optimizer,
    }


class ModeSetter(Node):
    def __init__(self) -> None:
        super().__init__("go2w_navigation_mode_setter")

    def wait(self, future: Any, timeout: float = 5.0) -> Any:
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("parameter service timed out")
        return future.result()

    @staticmethod
    def parameter(name: str, value: Any) -> ParameterMessage:
        if isinstance(value, bool):
            encoded = ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=value
            )
        elif isinstance(value, list):
            encoded = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(item) for item in value],
            )
        else:
            encoded = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
            )
        return ParameterMessage(name=name, value=encoded)

    def apply(self, node_name: str, values: dict[str, Any]) -> None:
        client = self.create_client(SetParameters, f"{node_name}/set_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"parameter service unavailable: {node_name}")
        request = SetParameters.Request()
        request.parameters = [self.parameter(name, value) for name, value in values.items()]
        response = self.wait(client.call_async(request))
        failures = [item.reason for item in response.results if not item.successful]
        if failures:
            raise RuntimeError(f"{node_name} rejected parameters: {failures}")

    def clear_local_costmap(self) -> None:
        client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("local costmap clear service unavailable")
        self.wait(client.call_async(ClearEntireCostmap.Request()))


def main() -> int:
    if len(sys.argv) != 2 or sys.argv[1] not in {"avoid", "up", "down", "slope", "flat"}:
        print(f"Usage: {sys.argv[0]} {{avoid|up|down|slope|flat}}", file=sys.stderr)
        return 2
    mode = sys.argv[1]
    rclpy.init()
    node = ModeSetter()
    try:
        values = profile(mode)
        for node_name, parameters in values.items():
            node.apply(node_name, parameters)
        node.clear_local_costmap()
        mode_path = os.environ.get("GO2W_NAV_MODE_FILE") or os.environ.get(
            "STAIR_NAV_MODE_FILE"
        )
        if mode_path:
            destination = Path(mode_path)
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_suffix(destination.suffix + ".tmp")
            temporary.write_text(mode + "\n", encoding="utf-8")
            temporary.replace(destination)
        print(json.dumps({"mode": mode, "status": "ok", "batch_requests": 3}))
        return 0
    except Exception as exc:
        print(f"[ERROR] failed to set Go2-W navigation mode: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
