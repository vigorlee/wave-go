#!/usr/bin/env python3
"""Reproducible Nezha2 traversal and point-cloud evidence.

This runner deliberately separates the two control layers:

* ``policy_zyl_0115_2.pt`` is the unmodified 16-actuator TorchScript
  locomotion policy.  It receives only proprioception and a velocity command;
  it is not presented as a perception-to-navigation policy.
* In the obstacle map, a deterministic point-cloud local planner produces the
  velocity command.  At 5 Hz it uses a MuJoCo ``mj_multiRay`` scan emitted
  from the model's ``Lidar`` body, removes only ground-height returns, inflates
  occupied cells by the robot safety radius, and computes an A* path to the
  declared goal.  No obstacle coordinates or prewritten waypoints are used by
  the planner.

The figure point clouds are event-triggered and state-synchronous: the
simulation frame, raw point cloud, sensor pose, planner state, and metadata
are all written from the exact same ``mjData`` state.  Every valid diagnostic
scan return is retained in ``lidar_capture.npz``; the visualizer derives its
axes from the full scan and does not crop or randomly subsample it.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import matplotlib as mpl

mpl.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import torch
from PIL import Image


mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "font.size": 7,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
    }
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/rl_sar/models/nezha/robot_lab"
SCENE_DIR = ROOT / "matrix/src/robot_mujoco/zsibot_robots/nezha2"
SCENES = {
    "stairs": SCENE_DIR / "scene_terrain_stairs_clean.xml",
    "slope": SCENE_DIR / "scene_terrain_slope.xml",
    "obstacles": SCENE_DIR / "scene_terrain_obstacles.xml",
}
JOINTS = [
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "RL", "RR")
    for joint in ("hip", "thigh", "calf", "foot")
]
DEFAULT = np.array(
    [0.0, 0.925, -1.85, 0.0, 0.0, 0.925, -1.85, 0.0,
     0.0, -0.925, 1.85, 0.0, 0.0, -0.925, 1.85, 0.0],
    dtype=np.float32,
)
KP = np.array([150.0, 150.0, 150.0, 0.0] * 4, dtype=np.float32)
KD = np.array([4.0, 4.0, 4.0, 1.2] * 4, dtype=np.float32)
ACTION_SCALE = np.array([0.25, 0.25, 0.25, 10.0] * 4, dtype=np.float32)
TORQUE_LIMIT = np.array([150.0, 150.0, 150.0, 10.0] * 4, dtype=np.float32)
COMMAND_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
LEG_MASK = np.arange(16) % 4 != 3

LIDAR_MIN_RANGE_M = 0.05
LIDAR_MAX_RANGE_M = 12.0
PLANNER_AZIMUTH_COUNT = 360
PLANNER_ELEVATION_DEG = np.array([-16.0, -8.0, 0.0, 8.0, 16.0], dtype=np.float64)
DIAGNOSTIC_AZIMUTH_COUNT = 720
DIAGNOSTIC_ELEVATION_DEG = np.linspace(-26.0, 18.0, 36, dtype=np.float64)
PLANNER_PERIOD_S = 0.20
GRID_RESOLUTION_M = 0.10
GRID_X_MIN_M = -1.20
GRID_X_MAX_M = 7.00
GRID_Y_MIN_M = -3.20
GRID_Y_MAX_M = 3.20
ROBOT_SAFETY_RADIUS_M = 0.46
OCCUPANCY_HEIGHT_MIN_M = -0.22
OCCUPANCY_HEIGHT_MAX_M = 1.20
OBSTACLE_GOAL_WORLD = np.array([0.0, 5.35], dtype=np.float64)
TERRAIN_CAPTURE_MIDLINE_Y = {"stairs": 2.55, "slope": 2.70}


@dataclass
class LidarScan:
    """A ray-cast scan and the coordinate frame needed to reproduce it."""

    origin_world: np.ndarray
    rotation_world_from_lidar: np.ndarray
    directions_lidar: np.ndarray
    directions_world: np.ndarray
    distance_m: np.ndarray
    geom_id: np.ndarray
    valid: np.ndarray
    points_world: np.ndarray


@dataclass
class LocalPlan:
    """Point-cloud-derived grid plan in the instantaneous robot base frame."""

    command: np.ndarray
    path_body: np.ndarray
    target_body: np.ndarray
    target_world_xy: np.ndarray
    used_previous_target: bool
    goal_body: np.ndarray
    obstacle_points_body: np.ndarray
    occupied: np.ndarray
    inflated: np.ndarray
    x_centres: np.ndarray
    y_centres: np.ndarray
    path_found: bool
    min_clearance_m: float
    heading_offset_rad: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=("all", *SCENES), default="all")
    parser.add_argument("--policy", default="policy_zyl_0115_2.pt")
    parser.add_argument("--seed", type=int, default=15)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def gravity_from_quaternion(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = map(float, quat)
    return np.array(
        [-2.0 * (x * z - w * y), -2.0 * (y * z + w * x), -(w * w - x * x - y * y + z * z)],
        dtype=np.float32,
    )


def yaw_from_quaternion(quat: np.ndarray) -> float:
    w, x, y, z = map(float, quat)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def make_camera(scenario: str) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 2.55, 0.42]
    camera.distance = 6.4 if scenario != "obstacles" else 6.8
    camera.azimuth = -112.0 if scenario != "obstacles" else -92.0
    camera.elevation = -26.0 if scenario != "obstacles" else -42.0
    return camera


def render_simulation(model: mujoco.MjModel, data: mujoco.MjData, scenario: str, destination: Path) -> None:
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    with mujoco.Renderer(model, height=900, width=1600) as renderer:
        renderer.update_scene(data, camera=make_camera(scenario))
        Image.fromarray(renderer.render()).save(destination)


def ray_directions_lidar(azimuth_count: int, elevation_deg: np.ndarray) -> np.ndarray:
    azimuth = np.linspace(-math.pi, math.pi, azimuth_count, endpoint=False, dtype=np.float64)
    elevation = np.deg2rad(np.asarray(elevation_deg, dtype=np.float64))
    aa, ee = np.meshgrid(azimuth, elevation)
    horizontal = np.cos(ee)
    return np.column_stack(
        (
            horizontal.ravel() * np.cos(aa).ravel(),
            horizontal.ravel() * np.sin(aa).ravel(),
            np.sin(ee).ravel(),
        )
    ).astype(np.float64)


def lidar_scan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    azimuth_count: int,
    elevation_deg: np.ndarray,
) -> LidarScan:
    """Run an all-return MuJoCo LiDAR emulation from the physical Lidar body."""

    lidar_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Lidar")
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "BASE_LINK")
    origin_world = np.asarray(data.xpos[lidar_body], dtype=np.float64).copy()
    rotation_world_from_lidar = np.asarray(data.xmat[lidar_body], dtype=np.float64).reshape(3, 3).copy()
    directions_lidar = ray_directions_lidar(azimuth_count, elevation_deg)
    directions_world = directions_lidar @ rotation_world_from_lidar.T
    ray_count = len(directions_world)
    geom_id = np.full(ray_count, -1, dtype=np.int32)
    distance_m = np.full(ray_count, -1.0, dtype=np.float64)
    # Static terrain and obstacles are group 0; robot collision and visual geoms
    # are groups 1/2, so they are excluded before ray intersection rather than
    # removed retrospectively from an already self-occluded scan.
    environment_geom_group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    mujoco.mj_multiRay(
        model,
        data,
        origin_world,
        directions_world.ravel(),
        environment_geom_group,
        1,
        base_body,
        geom_id,
        distance_m,
        ray_count,
        LIDAR_MAX_RANGE_M,
    )
    valid = (distance_m > LIDAR_MIN_RANGE_M) & (distance_m < LIDAR_MAX_RANGE_M) & (geom_id >= 0)
    points_world = origin_world[None, :] + directions_world[valid] * distance_m[valid, None]
    return LidarScan(
        origin_world=origin_world,
        rotation_world_from_lidar=rotation_world_from_lidar,
        directions_lidar=directions_lidar,
        directions_world=directions_world,
        distance_m=distance_m,
        geom_id=geom_id,
        valid=valid,
        points_world=points_world,
    )


def base_frame(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "BASE_LINK")
    position = np.asarray(data.xpos[base_body], dtype=np.float64).copy()
    rotation_world_from_base = np.asarray(data.xmat[base_body], dtype=np.float64).reshape(3, 3).copy()
    return position, rotation_world_from_base


def points_in_base_frame(points_world: np.ndarray, base_position: np.ndarray, rotation_world_from_base: np.ndarray) -> np.ndarray:
    return (points_world - base_position[None, :]) @ rotation_world_from_base


def grid_indices(points_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ix = np.floor((points_xy[:, 0] - GRID_X_MIN_M) / GRID_RESOLUTION_M).astype(np.int32)
    iy = np.floor((points_xy[:, 1] - GRID_Y_MIN_M) / GRID_RESOLUTION_M).astype(np.int32)
    nx = int(round((GRID_X_MAX_M - GRID_X_MIN_M) / GRID_RESOLUTION_M))
    ny = int(round((GRID_Y_MAX_M - GRID_Y_MIN_M) / GRID_RESOLUTION_M))
    valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    return ix, iy, valid


def inflate_occupancy(occupied: np.ndarray) -> np.ndarray:
    """Inflate occupied cells by a fixed, documented robot safety radius."""

    radius_cells = int(math.ceil(ROBOT_SAFETY_RADIUS_M / GRID_RESOLUTION_M))
    offsets = [
        (dy, dx)
        for dy in range(-radius_cells, radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1)
        if math.hypot(dx, dy) * GRID_RESOLUTION_M <= ROBOT_SAFETY_RADIUS_M
    ]
    inflated = occupied.copy()
    ys, xs = np.nonzero(occupied)
    for y, x in zip(ys.tolist(), xs.tolist()):
        for dy, dx in offsets:
            yy, xx = y + dy, x + dx
            if 0 <= yy < inflated.shape[0] and 0 <= xx < inflated.shape[1]:
                inflated[yy, xx] = True
    return inflated


def cell_from_xy(point: np.ndarray) -> tuple[int, int]:
    x, y = map(float, point[:2])
    ix = int(math.floor((x - GRID_X_MIN_M) / GRID_RESOLUTION_M))
    iy = int(math.floor((y - GRID_Y_MIN_M) / GRID_RESOLUTION_M))
    return iy, ix


def xy_from_cell(cell: tuple[int, int]) -> np.ndarray:
    iy, ix = cell
    return np.array(
        [
            GRID_X_MIN_M + (ix + 0.5) * GRID_RESOLUTION_M,
            GRID_Y_MIN_M + (iy + 0.5) * GRID_RESOLUTION_M,
        ],
        dtype=np.float64,
    )


def nearest_free_cell(occupied: np.ndarray, target: tuple[int, int]) -> Optional[tuple[int, int]]:
    """Find the nearest free grid cell when a goal falls inside the safety halo."""

    ny, nx = occupied.shape
    ty = min(max(target[0], 0), ny - 1)
    tx = min(max(target[1], 0), nx - 1)
    if not occupied[ty, tx]:
        return ty, tx
    max_radius = max(nx, ny)
    for radius in range(1, max_radius):
        for yy in range(max(0, ty - radius), min(ny, ty + radius + 1)):
            for xx in range(max(0, tx - radius), min(nx, tx + radius + 1)):
                if max(abs(yy - ty), abs(xx - tx)) == radius and not occupied[yy, xx]:
                    return yy, xx
    return None


def astar_path(
    occupied: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    preferred_lateral_sign: int,
) -> Optional[list[tuple[int, int]]]:
    """Eight-connected A* on the point-cloud occupancy grid."""

    ny, nx = occupied.shape
    if occupied[start] or occupied[goal]:
        return None
    neighbours = [
        (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0), (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
    ]
    open_heap: list[tuple[float, float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, 0.0, start))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start: 0.0}

    while open_heap:
        _, g_current, current = heapq.heappop(open_heap)
        if g_current != g_score.get(current):
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))
        for dy, dx, cost in neighbours:
            yy, xx = current[0] + dy, current[1] + dx
            if not (0 <= yy < ny and 0 <= xx < nx) or occupied[yy, xx]:
                continue
            neighbour = yy, xx
            tie_break = 0.04 if current == start and preferred_lateral_sign and dy * preferred_lateral_sign < 0 else 0.0
            tentative = g_current + cost + tie_break
            if tentative >= g_score.get(neighbour, float("inf")):
                continue
            came_from[neighbour] = current
            g_score[neighbour] = tentative
            heuristic = math.hypot(goal[0] - yy, goal[1] - xx)
            heapq.heappush(open_heap, (tentative + heuristic, tentative, neighbour))
    return None


def segment_is_free(inflated: np.ndarray, target_body: np.ndarray) -> bool:
    """Check whether a look-ahead command can safely shortcut the A* polyline."""

    samples = max(2, int(math.ceil(np.linalg.norm(target_body) / (0.5 * GRID_RESOLUTION_M))))
    for point in np.linspace(np.zeros(2), target_body, samples):
        iy, ix = cell_from_xy(point)
        if not (0 <= iy < inflated.shape[0] and 0 <= ix < inflated.shape[1]) or inflated[iy, ix]:
            return False
    return True


def planner_from_scan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scan: LidarScan,
    goal_world_xy: np.ndarray,
    preferred_lateral_sign: int,
    previous_target_world_xy: Optional[np.ndarray],
) -> LocalPlan:
    """Build a grid from LiDAR returns and turn its A* path into ``[vx, vy, wz]``."""

    base_position, rotation_world_from_base = base_frame(model, data)
    points_body = points_in_base_frame(scan.points_world, base_position, rotation_world_from_base)
    obstacle_mask = (
        (points_body[:, 2] >= OCCUPANCY_HEIGHT_MIN_M)
        & (points_body[:, 2] <= OCCUPANCY_HEIGHT_MAX_M)
    )
    obstacle_points_body = points_body[obstacle_mask]
    nx = int(round((GRID_X_MAX_M - GRID_X_MIN_M) / GRID_RESOLUTION_M))
    ny = int(round((GRID_Y_MAX_M - GRID_Y_MIN_M) / GRID_RESOLUTION_M))
    occupied = np.zeros((ny, nx), dtype=bool)
    if len(obstacle_points_body):
        ix, iy, valid = grid_indices(obstacle_points_body[:, :2])
        occupied[iy[valid], ix[valid]] = True
    inflated = inflate_occupancy(occupied)

    goal_world_3d = np.array([goal_world_xy[0], goal_world_xy[1], base_position[2]], dtype=np.float64)
    goal_body = points_in_base_frame(goal_world_3d[None, :], base_position, rotation_world_from_base)[0, :2]
    start_cell = nearest_free_cell(inflated, cell_from_xy(np.zeros(2, dtype=np.float64)))
    goal_cell = nearest_free_cell(inflated, cell_from_xy(goal_body))
    cells = (
        astar_path(inflated, start_cell, goal_cell, preferred_lateral_sign)
        if start_cell is not None and goal_cell is not None
        else None
    )
    path_body = np.asarray([xy_from_cell(cell) for cell in cells], dtype=np.float64) if cells else np.empty((0, 2), dtype=np.float64)
    path_found = len(path_body) > 1

    if path_found:
        target_body = path_body[1]
        for candidate in path_body[1:]:
            if float(np.linalg.norm(candidate)) <= 1.00 and segment_is_free(inflated, candidate):
                target_body = candidate
            elif float(np.linalg.norm(candidate)) > 1.00:
                break
    else:
        # A planner failure is exposed as a zero forward command, not hidden by a fallback route.
        target_body = np.zeros(2, dtype=np.float64)

    used_previous_target = False
    if previous_target_world_xy is not None:
        previous_world_3d = np.array([previous_target_world_xy[0], previous_target_world_xy[1], base_position[2]], dtype=np.float64)
        previous_body = points_in_base_frame(previous_world_3d[None, :], base_position, rotation_world_from_base)[0, :2]
        if (
            path_found
            and previous_body[0] > 0.12
            and float(np.linalg.norm(previous_body)) <= 1.40
            and segment_is_free(inflated, previous_body)
        ):
            # This short-lived, map-derived target supplies temporal continuity;
            # it is discarded as soon as it is reached, leaves the local map, or is blocked.
            target_body = previous_body
            used_previous_target = True

    distance = float(np.linalg.norm(target_body))
    heading_offset_rad = math.atan2(float(target_body[1]), max(float(target_body[0]), 1e-8)) if distance else 0.0
    speed = min(0.48, 0.62 * distance) if path_found else 0.0
    command = np.array(
        [
            np.clip(speed * math.cos(heading_offset_rad), 0.0, 0.52),
            np.clip(speed * math.sin(heading_offset_rad), -0.30, 0.30),
            np.clip(0.65 * heading_offset_rad, -0.32, 0.32),
        ],
        dtype=np.float32,
    )
    if len(obstacle_points_body):
        min_clearance_m = float(np.min(np.linalg.norm(obstacle_points_body[:, :2], axis=1)))
    else:
        min_clearance_m = float("inf")
    x_centres = GRID_X_MIN_M + (np.arange(nx) + 0.5) * GRID_RESOLUTION_M
    y_centres = GRID_Y_MIN_M + (np.arange(ny) + 0.5) * GRID_RESOLUTION_M
    target_world = base_position + rotation_world_from_base @ np.array([target_body[0], target_body[1], 0.0])
    return LocalPlan(
        command=command,
        path_body=path_body,
        target_body=target_body,
        target_world_xy=target_world[:2],
        used_previous_target=used_previous_target,
        goal_body=goal_body,
        obstacle_points_body=obstacle_points_body,
        occupied=occupied,
        inflated=inflated,
        x_centres=x_centres,
        y_centres=y_centres,
        path_found=path_found,
        min_clearance_m=min_clearance_m,
        heading_offset_rad=heading_offset_rad,
    )


def plot_full_point_cloud(
    points_body: np.ndarray,
    scenario: str,
    capture_time_s: float,
    destination: Path,
) -> None:
    """Plot every retained diagnostic return in the instantaneous base frame."""

    fig = plt.figure(figsize=(7.2, 5.0), dpi=600, facecolor="#07131f")
    ax = fig.add_subplot(111, projection="3d", facecolor="#07131f")
    ranges = np.linalg.norm(points_body, axis=1)
    ax.scatter(
        points_body[:, 0],
        points_body[:, 1],
        points_body[:, 2],
        c=ranges,
        cmap="viridis",
        vmin=float(np.min(ranges)),
        vmax=float(np.max(ranges)),
        s=0.8,
        alpha=0.82,
        linewidths=0,
        rasterized=True,
    )
    ax.scatter([0.0], [0.0], [0.0], marker="^", color="#ffdd44", s=70, label="Nezha2 base")
    limits = np.vstack((np.min(points_body, axis=0), np.max(points_body, axis=0)))
    span = np.maximum(limits[1] - limits[0], 0.4)
    padding = 0.04 * span + 0.05
    ax.set_xlim(float(limits[0, 0] - padding[0]), float(limits[1, 0] + padding[0]))
    ax.set_ylim(float(limits[0, 1] - padding[1]), float(limits[1, 1] + padding[1]))
    ax.set_zlim(float(limits[0, 2] - padding[2]), float(limits[1, 2] + padding[2]))
    ax.set_box_aspect(tuple(span.tolist()))
    ax.set_xlabel("forward / m", color="white", labelpad=8, fontsize=9)
    ax.set_ylabel("left / m", color="white", labelpad=8, fontsize=9)
    ax.set_zlabel("height / m", color="white", labelpad=8, fontsize=9)
    ax.tick_params(colors="#d7e3ef", labelsize=7)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((0.03, 0.08, 0.12, 1.0))
        axis._axinfo["grid"]["color"] = (0.35, 0.48, 0.58, 0.28)
    ax.view_init(elev=24, azim=-61)
    label = {"stairs": "Stairs", "slope": "Slope", "obstacles": "Complex obstacles"}[scenario]
    ax.set_title(
        f"{label}: all valid MuJoCo ray-cast returns ({len(points_body):,}; t={capture_time_s:.3f} s)",
        color="white",
        pad=10,
        fontsize=10,
    )
    fig.tight_layout(pad=0.8)
    fig.savefig(destination, dpi=600, facecolor=fig.get_facecolor())
    fig.savefig(destination.with_suffix(".svg"), bbox_inches="tight", facecolor=fig.get_facecolor())
    fig.savefig(destination.with_suffix(".pdf"), bbox_inches="tight", facecolor=fig.get_facecolor())
    fig.savefig(destination.with_suffix(".tiff"), dpi=600, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_planner(plan: LocalPlan, capture_time_s: float, destination: Path) -> None:
    """Show the actual point-cloud costmap and A* path, not a hand-drawn route."""

    fig, ax = plt.subplots(figsize=(7.2, 5.6), dpi=600, facecolor="white")
    extent = [GRID_X_MIN_M, GRID_X_MAX_M, GRID_Y_MIN_M, GRID_Y_MAX_M]
    ax.imshow(plan.inflated.astype(np.uint8), extent=extent, origin="lower", cmap="Greys", alpha=0.45, interpolation="nearest")
    if len(plan.obstacle_points_body):
        ax.scatter(
            plan.obstacle_points_body[:, 0],
            plan.obstacle_points_body[:, 1],
            s=2.0,
            color="#343a40",
            alpha=0.55,
            linewidths=0,
            label="Obstacle-height LiDAR returns",
            rasterized=True,
        )
    if plan.path_found:
        ax.plot(plan.path_body[:, 0], plan.path_body[:, 1], color="#e07a1f", linewidth=2.0, label="A* path")
        ax.scatter(plan.target_body[0], plan.target_body[1], color="#d94801", marker="x", s=58, linewidths=2.0, label="Command target")
    ax.scatter(0.0, 0.0, color="#1c6ca1", marker="^", s=62, label="Nezha2 base")
    ax.scatter(plan.goal_body[0], plan.goal_body[1], color="#2a9d8f", marker="*", s=100, label="Goal")
    ax.add_patch(plt.Circle((0.0, 0.0), ROBOT_SAFETY_RADIUS_M, fill=False, color="#1c6ca1", linestyle="--", linewidth=1.0))
    ax.set_xlim(GRID_X_MIN_M, GRID_X_MAX_M)
    ax.set_ylim(GRID_Y_MIN_M, GRID_Y_MAX_M)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("forward / m", fontsize=9)
    ax.set_ylabel("left / m", fontsize=9)
    ax.set_title(
        f"Point-cloud local plan at t={capture_time_s:.3f} s | {GRID_RESOLUTION_M:.2f} m grid, {ROBOT_SAFETY_RADIUS_M:.2f} m inflation",
        fontsize=9,
    )
    ax.legend(loc="upper right", fontsize=7, frameon=True, framealpha=0.92)
    ax.grid(alpha=0.22, linewidth=0.5)
    fig.tight_layout(pad=0.8)
    fig.savefig(destination, dpi=600)
    fig.savefig(destination.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(destination.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(destination.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(fig)


def make_evidence_figure(
    simulation_path: Path,
    cloud_path: Path,
    planner_path: Optional[Path],
    scenario: str,
    destination: Path,
) -> None:
    """Assemble actual-state visual evidence without manipulating the source panels."""

    paths = [simulation_path, cloud_path] + ([planner_path] if planner_path is not None else [])
    labels = {"stairs": "Stairs", "slope": "Slope", "obstacles": "Complex obstacles"}
    titles = [
        f"(a) {labels[scenario]} MuJoCo state",
        "(b) Full state-synchronous ray-cast point cloud",
    ]
    if planner_path is not None:
        titles.append("(c) Point-cloud-derived local plan")
    fig, axes = plt.subplots(1, len(paths), figsize=(15.0 if len(paths) == 3 else 10.6, 4.0), dpi=600)
    if len(paths) == 1:
        axes = [axes]
    for axis, source, title in zip(axes, paths, titles):
        axis.imshow(Image.open(source).convert("RGB"))
        axis.set_title(title, fontsize=8.5, color="#174e77", pad=6)
        axis.set_axis_off()
    fig.subplots_adjust(left=0.005, right=0.995, top=0.89, bottom=0.02, wspace=0.012)
    fig.savefig(destination, dpi=600)
    fig.savefig(destination.with_suffix(".svg"), bbox_inches="tight")
    fig.savefig(destination.with_suffix(".pdf"), bbox_inches="tight")
    fig.savefig(destination.with_suffix(".tiff"), dpi=600, bbox_inches="tight")
    plt.close(fig)


def serialize_capture(
    destination: Path,
    scenario: str,
    data: mujoco.MjData,
    scan: LidarScan,
    points_body: np.ndarray,
    trigger: str,
    plan: Optional[LocalPlan],
) -> None:
    """Persist raw scan arrays and exact state metadata for auditability."""

    payload: dict[str, Any] = {
        "scenario": scenario,
        "capture_trigger": trigger,
        "time_s": float(data.time),
        "qpos": [float(value) for value in data.qpos],
        "qvel": [float(value) for value in data.qvel],
        "lidar_origin_world_m": [float(value) for value in scan.origin_world],
        "lidar_rotation_world_from_lidar": scan.rotation_world_from_lidar.tolist(),
        "diagnostic_scan": {
            "azimuth_beams": DIAGNOSTIC_AZIMUTH_COUNT,
            "elevation_beams": len(DIAGNOSTIC_ELEVATION_DEG),
            "elevation_deg": [float(value) for value in DIAGNOSTIC_ELEVATION_DEG],
            "raw_ray_count": int(len(scan.distance_m)),
            "valid_return_count": int(np.count_nonzero(scan.valid)),
            "range_filter_m": [LIDAR_MIN_RANGE_M, LIDAR_MAX_RANGE_M],
            "visualization": "all valid returns; no random downsampling or spatial crop",
        },
    }
    if plan is not None:
        payload["point_cloud_local_planner"] = {
            "replan_period_s": PLANNER_PERIOD_S,
            "grid_resolution_m": GRID_RESOLUTION_M,
            "robot_safety_radius_m": ROBOT_SAFETY_RADIUS_M,
            "occupancy_height_filter_base_frame_m": [OCCUPANCY_HEIGHT_MIN_M, OCCUPANCY_HEIGHT_MAX_M],
            "path_found": bool(plan.path_found),
            "command_vx_vy_wz": [float(value) for value in plan.command],
            "goal_body_xy_m": [float(value) for value in plan.goal_body],
            "target_body_xy_m": [float(value) for value in plan.target_body],
            "heading_offset_rad": float(plan.heading_offset_rad),
            "occupied_point_count": int(len(plan.obstacle_points_body)),
            "min_obstacle_point_distance_m": None if math.isinf(plan.min_clearance_m) else float(plan.min_clearance_m),
        }
    destination.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(
        destination.with_name("lidar_capture.npz"),
        points_world=scan.points_world,
        points_base=points_body,
        directions_lidar=scan.directions_lidar,
        directions_world=scan.directions_world,
        distance_m=scan.distance_m,
        geom_id=scan.geom_id,
        valid=scan.valid,
        lidar_origin_world=scan.origin_world,
        lidar_rotation_world_from_lidar=scan.rotation_world_from_lidar,
    )


def capture_evidence(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scenario: str,
    output_dir: Path,
    trigger: str,
    plan: Optional[LocalPlan],
) -> dict[str, Any]:
    """Capture panels and raw data from one, single, exact MuJoCo state."""

    render_simulation(model, data, scenario, output_dir / "simulation.png")
    scan = lidar_scan(model, data, DIAGNOSTIC_AZIMUTH_COUNT, DIAGNOSTIC_ELEVATION_DEG)
    base_position, rotation_world_from_base = base_frame(model, data)
    points_body = points_in_base_frame(scan.points_world, base_position, rotation_world_from_base)
    plot_full_point_cloud(points_body, scenario, float(data.time), output_dir / "pointcloud_full.png")
    planner_path: Optional[Path] = None
    if plan is not None:
        planner_path = output_dir / "planner.png"
        plot_planner(plan, float(data.time), planner_path)
        # Keep the perception evidence and the planner evidence as separate,
        # readable panels for the Word document; the three-panel composite is
        # retained as an overview artifact.
        make_evidence_figure(
            output_dir / "simulation.png",
            output_dir / "pointcloud_full.png",
            None,
            scenario,
            output_dir / "perception_evidence.png",
        )
    make_evidence_figure(
        output_dir / "simulation.png",
        output_dir / "pointcloud_full.png",
        planner_path,
        scenario,
        output_dir / "evidence.png",
    )
    serialize_capture(output_dir / "capture.json", scenario, data, scan, points_body, trigger, plan)
    return {
        "time_s": float(data.time),
        "trigger": trigger,
        "valid_returns": int(np.count_nonzero(scan.valid)),
        "raw_rays": int(len(scan.distance_m)),
    }


def run_scenario(scenario: str, policy: torch.jit.ScriptModule, seed: int, output_root: Path) -> dict[str, object]:
    torch.manual_seed(seed)
    output_dir = output_root / scenario
    output_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENES[scenario]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINTS]
    qadr = np.array([model.jnt_qposadr[joint] for joint in joint_ids])
    vadr = np.array([model.jnt_dofadr[joint] for joint in joint_ids])
    actuator_ids = [int(np.where(model.actuator_trnid[:, 0] == joint)[0][0]) for joint in joint_ids]
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
    gyro_adr = int(model.sensor_adr[gyro_id])
    policy_period = max(1, round(0.02 / model.opt.timestep))
    total_steps = round((18.0 if scenario != "obstacles" else 32.0) / model.opt.timestep)
    history = np.zeros((5, 57), dtype=np.float32)
    previous_action = np.zeros(16, dtype=np.float32)
    trajectory: list[tuple[float, float]] = []
    capture: Optional[dict[str, Any]] = None
    collision = False
    last_plan: Optional[LocalPlan] = None
    planner_replans: list[dict[str, Any]] = []
    next_replan_time = 2.0
    # A tie-break is locked after the first non-straight plan to suppress
    # left/right oscillation in geometrically symmetric local maps.  It contains
    # no obstacle coordinates or hand-authored route, and remains only a small
    # start-cell cost preference rather than a hard constraint.
    preferred_lateral_sign = 0
    previous_target_world_xy: Optional[np.ndarray] = None
    command = np.zeros(3, dtype=np.float32)

    obstacle_geom_ids = {
        geom_id
        for geom_id in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "").startswith("obstacle_")
    }

    for step in range(total_steps):
        q = np.asarray(data.qpos[qadr], dtype=np.float32)
        dq = np.asarray(data.qvel[vadr], dtype=np.float32)
        if data.time < 2.0:
            command[:] = 0.0
        elif scenario == "obstacles":
            if data.time + 1e-9 >= next_replan_time:
                planner_scan = lidar_scan(model, data, PLANNER_AZIMUTH_COUNT, PLANNER_ELEVATION_DEG)
                last_plan = planner_from_scan(
                    model,
                    data,
                    planner_scan,
                    OBSTACLE_GOAL_WORLD,
                    preferred_lateral_sign,
                    previous_target_world_xy,
                )
                command = last_plan.command.copy()
                previous_target_world_xy = last_plan.target_world_xy.copy()
                if preferred_lateral_sign == 0 and abs(last_plan.heading_offset_rad) >= 0.08:
                    preferred_lateral_sign = 1 if last_plan.heading_offset_rad > 0 else -1
                planner_replans.append(
                    {
                        "time_s": float(data.time),
                        "planner_ray_count": int(len(planner_scan.distance_m)),
                        "planner_valid_returns": int(np.count_nonzero(planner_scan.valid)),
                        "path_found": bool(last_plan.path_found),
                        "command_vx_vy_wz": [float(value) for value in command],
                        "heading_offset_rad": float(last_plan.heading_offset_rad),
                        "preferred_lateral_sign": int(preferred_lateral_sign),
                        "used_previous_target": bool(last_plan.used_previous_target),
                    }
                )
                next_replan_time += PLANNER_PERIOD_S
        else:
            # These two maps validate traversal under a declared straight corridor; no perception input is supplied to the locomotion policy.
            command = np.array([0.62, 0.0, 0.0], dtype=np.float32)

        # Freeze evidence before the control step.  For the obstacle case this
        # makes the rendered camera frame, diagnostic 720x36 point cloud, and
        # the 5 Hz planner output refer to exactly the same mjData state.
        # Terrain captures use the first state at or past their declared map
        # midline, likewise before its next control step.
        if capture is None:
            if scenario in TERRAIN_CAPTURE_MIDLINE_Y and data.qpos[1] >= TERRAIN_CAPTURE_MIDLINE_Y[scenario]:
                capture = capture_evidence(
                    model,
                    data,
                    scenario,
                    output_dir,
                    f"map-defined terrain midline y={TERRAIN_CAPTURE_MIDLINE_Y[scenario]:.2f} m",
                    None,
                )
            elif scenario == "obstacles" and last_plan is not None and abs(last_plan.heading_offset_rad) >= 0.12:
                capture = capture_evidence(
                    model,
                    data,
                    scenario,
                    output_dir,
                    "first point-cloud planner update with |heading offset| >= 0.12 rad",
                    last_plan,
                )

        relative_q = q - DEFAULT
        relative_q[~LEG_MASK] = 0.0
        observation = np.concatenate(
            [
                np.asarray(data.sensordata[gyro_adr : gyro_adr + 3], dtype=np.float32) * 0.25,
                gravity_from_quaternion(np.asarray(data.qpos[3:7], dtype=np.float32)) * 0.5,
                command * COMMAND_SCALE,
                relative_q,
                dq * 0.05,
                previous_action,
            ]
        ).astype(np.float32)

        if data.time < 2.0:
            phase = np.clip(data.time / 2.0, 0.0, 1.0)
            phase = phase * phase * (3.0 - 2.0 * phase)
            target = q + phase * (DEFAULT - q)
            wheel_velocity = np.zeros(16, dtype=np.float32)
        else:
            if step % policy_period == 0:
                previous_action = policy(
                    torch.from_numpy(observation[None]), torch.from_numpy(history.reshape(1, -1))
                ).cpu().numpy()[0].astype(np.float32)
                previous_action = np.clip(previous_action, -100.0, 100.0)
                history[:-1] = history[1:]
                history[-1] = observation
            target = DEFAULT.copy()
            target[LEG_MASK] += previous_action[LEG_MASK] * ACTION_SCALE[LEG_MASK]
            wheel_velocity = np.zeros(16, dtype=np.float32)
            wheel_velocity[~LEG_MASK] = previous_action[~LEG_MASK] * ACTION_SCALE[~LEG_MASK]

        torque = np.clip(KP * (target - q) + KD * (wheel_velocity - dq), -TORQUE_LIMIT, TORQUE_LIMIT)
        data.ctrl[:] = 0.0
        for index, actuator_id in enumerate(actuator_ids):
            data.ctrl[actuator_id] = float(torque[index])
        mujoco.mj_step(model, data)

        if step % max(1, round(0.05 / model.opt.timestep)) == 0:
            trajectory.append((float(data.qpos[0]), float(data.qpos[1])))
        if scenario == "obstacles":
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                if contact.geom1 in obstacle_geom_ids or contact.geom2 in obstacle_geom_ids:
                    other = contact.geom2 if contact.geom1 in obstacle_geom_ids else contact.geom1
                    if other >= 0 and model.geom_bodyid[other] != 0:
                        collision = True
                        break

        if data.qpos[2] < 0.14 or not np.isfinite(data.qpos).all():
            break
        goal_y = {"stairs": 4.45, "slope": 4.55, "obstacles": 5.35}[scenario]
        if data.qpos[1] >= goal_y:
            break

    if capture is None:
        fallback_plan = last_plan if scenario == "obstacles" else None
        capture = capture_evidence(model, data, scenario, output_dir, "terminal state fallback", fallback_plan)
    np.save(output_dir / "trajectory_world_xy.npy", np.asarray(trajectory, dtype=np.float64))
    (output_dir / "planner_replans.json").write_text(
        json.dumps(planner_replans, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    goal_y = {"stairs": 4.45, "slope": 4.55, "obstacles": 5.35}[scenario]
    success = bool(data.qpos[1] >= goal_y and data.qpos[2] >= 0.25 and not collision)
    result: dict[str, object] = {
        "scenario": scenario,
        "scene": str(SCENES[scenario]),
        "policy": "policy_zyl_0115_2.pt (unmodified)",
        "success": success,
        # Stair and slope traversals necessarily make terrain contact; only the
        # obstacle scene reports a non-terrain collision measurement.
        "collision": collision if scenario == "obstacles" else None,
        "collision_definition": (
            "robot contact with a geom named obstacle_*; terrain contacts excluded"
            if scenario == "obstacles"
            else "not evaluated: terrain contact is an expected part of traversal"
        ),
        "time_s": float(data.time),
        "final_xyz": [float(value) for value in data.qpos[:3]],
        "trajectory_samples": len(trajectory),
        "capture": capture,
        "obstacle_strategy": (
            "point-cloud local occupancy + safety inflation + A* velocity command to locomotion policy"
            if scenario == "obstacles"
            else "declared straight velocity reference to locomotion policy"
        ),
    }
    (output_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))
    return result


def main() -> int:
    args = parse_args()
    policy_path = MODEL_DIR / args.policy
    if args.policy.endswith("_deterministic.pt"):
        raise ValueError("This evidence run requires policy_zyl_0115_2.pt, not a deterministic replacement.")
    torch.set_grad_enabled(False)
    policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
    scenarios = tuple(SCENES) if args.scenario == "all" else (args.scenario,)
    results = [run_scenario(name, policy, args.seed, args.output_dir) for name in scenarios]
    (args.output_dir / "summary.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0 if all(bool(result["success"]) for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
