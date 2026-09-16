#!/usr/bin/env python3
"""Generate conservative Navigo parameters for OLI flat-ground walking."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "genisom_roamerx_open/src/navigation/src/robot_navigo/params"
    / "navigo_params.yaml"
)
OUTPUT = ROOT / ".run/oli_navigo_params.yaml"


def main() -> None:
    data = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))

    behavior = data["behavior_server"]["ros__parameters"]
    behavior.update(
        {
            "max_rotational_vel": 0.30,
            "min_rotational_vel": 0.24,
            "rotational_acc_lim": 0.30,
        }
    )

    for costmap_name in ("local_costmap", "global_costmap"):
        costmap = data[costmap_name][costmap_name]["ros__parameters"]
        costmap["footprint"] = (
            "[[0.32,0.26],[0.32,-0.26],[-0.24,-0.26],[-0.24,0.26]]"
        )

    global_costmap = data["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]
    global_costmap["inflation_layer"]["inflation_radius"] = 0.35

    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    local["inflation_layer"].update(
        {"cost_scaling_factor": 3.0, "inflation_radius": 0.45}
    )
    local["obstacle_layer"].update(
        {"enabled": True, "min_obstacle_height": 0.08}
    )
    local["obstacle_layer"]["pointcloud"]["min_obstacle_height"] = 0.08

    planner = data["planner_server"]["ros__parameters"]
    planner["expected_planner_frequency"] = 10.0
    planner["GridBased"]["tolerance"] = 0.25

    controller = data["controller_server"]["ros__parameters"]
    controller["controller_frequency"] = 20.0
    controller["progress_checker"].update(
        {"required_movement_radius": 0.05, "movement_time_allowance": 30.0}
    )
    controller["general_goal_checker"].update(
        {"xy_goal_tolerance": 0.18, "yaw_goal_tolerance": 0.25}
    )
    follow = controller["FollowPath"]
    follow.update(
        {
            "time_steps": 48,
            "model_dt": 0.05,
            "batch_size": 1200,
            "vx_std": 0.04,
            "vy_std": 0.0,
            "wz_std": 0.16,
            "vx_max": 0.08,
            "vx_min": -0.04,
            "vy_max": 0.0,
            "wz_max": 0.30,
            "ax_max": 0.10,
            "ay_max": 0.0,
            "ax_min": -0.20,
            "az_max": 0.30,
            "prune_distance": 1.0,
        }
    )
    follow["PreferForwardCritic"]["threshold_to_consider"] = 0.30

    velocity = data["velocity_optimizer"]["ros__parameters"]
    velocity.update(
        {
            "smoothing_frequency": 30.0,
            "max_velocity": [0.08, 0.0, 0.30],
            "min_velocity": [-0.04, 0.0, -0.30],
            "deadband_velocity": [0.015, 0.0, 0.04],
            "velocity_timeout": 0.35,
            "max_accel": [0.10, 0.0, 0.30],
            "max_decel": [-0.20, 0.0, -0.40],
        }
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )
    print(OUTPUT)


if __name__ == "__main__":
    main()
