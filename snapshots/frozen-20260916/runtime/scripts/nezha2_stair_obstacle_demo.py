#!/usr/bin/env python3
"""Terrain-aware, point-cloud-only obstacle avoidance while Nezha2 climbs stairs.

This demo keeps the pretrained ``policy_zyl_0115_2.pt`` untouched.  It accepts
only proprioception and a smoothed ``[vx, vy, wz]`` reference.  A separate
planner performs the navigation work: it uses MuJoCo ray-cast returns, removes
only the known traversable stair treads/risers, inflates remaining obstacle
returns by the measured Nezha2 collision footprint, and replans an A* route to
the top platform.  The planner never receives an obstacle coordinate or a
hand-authored detour.

The moving cylinder is a physical MuJoCo mocap obstacle.  At every replan its
position is observable only through the current LiDAR scan; its pose is logged
after planning solely for audit and collision checking.
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

import nezha2_closed_loop_navigation_evidence as core


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


SCENE = core.SCENE_DIR / "scene_terrain_stairs_obstacles.xml"
POLICY_NAME = "policy_zyl_0115_2.pt"
GOAL_WORLD_XY = np.array([0.0, 5.02], dtype=np.float64)

GRID_RESOLUTION_M = 0.05
GRID_X_MIN_M, GRID_X_MAX_M = -1.50, 1.50
GRID_Y_MIN_M, GRID_Y_MAX_M = -0.35, 5.30
STAIR_HALF_WIDTH_M = 1.50
STAIR_FIRST_EDGE_Y_M = 0.75
STAIR_TREAD_DEPTH_M = 0.28
STAIR_RISE_M = 0.05
STAIR_COUNT = 10
PLANNER_PERIOD_S = 0.10
PLANNER_AZIMUTH_COUNT = 360
PLANNER_ELEVATION_DEG = np.array([-16.0, -8.0, 0.0, 8.0, 16.0], dtype=np.float64)
DIAGNOSTIC_AZIMUTH_COUNT = 720
DIAGNOSTIC_ELEVATION_DEG = np.linspace(-26.0, 18.0, 36, dtype=np.float64)
OCCUPANCY_HEIGHT_MAX_ABOVE_TERRAIN_M = 1.20
DYNAMIC_INITIAL_X_M = 1.05
DYNAMIC_FINAL_X_M = 0.45


@dataclass
class Footprint:
    half_forward_m: float
    half_lateral_m: float
    margin_m: float


@dataclass
class StairPlan:
    desired_command: np.ndarray
    path_world_xy: np.ndarray
    path_height_m: np.ndarray
    target_world_xy: np.ndarray
    goal_world_xy: np.ndarray
    obstacle_points_world: np.ndarray
    terrain_mask: np.ndarray
    occupied: np.ndarray
    inflated: np.ndarray
    x_centres: np.ndarray
    y_centres: np.ndarray
    path_found: bool
    heading_offset_rad: float
    clearance_to_obstacle_m: Optional[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=15)
    return parser.parse_args()


def stair_height(y: np.ndarray | float) -> np.ndarray:
    """Semantic traversability layer extracted from the known stair geometry."""

    values = np.asarray(y, dtype=np.float64)
    step_index = np.floor((values - STAIR_FIRST_EDGE_Y_M) / STAIR_TREAD_DEPTH_M).astype(np.int32) + 1
    step_index = np.clip(step_index, 0, STAIR_COUNT)
    step_index = np.where(values < STAIR_FIRST_EDGE_Y_M, 0, step_index)
    return step_index.astype(np.float64) * STAIR_RISE_M


def global_semantic_route() -> tuple[np.ndarray, np.ndarray]:
    """Task-level centreline route from the semantic stair map, before LiDAR avoidance."""

    y_nodes = np.unique(
        np.concatenate(
            (
                np.array([0.0, STAIR_FIRST_EDGE_Y_M], dtype=np.float64),
                STAIR_FIRST_EDGE_Y_M + np.arange(1, STAIR_COUNT + 1, dtype=np.float64) * STAIR_TREAD_DEPTH_M,
                np.array([GOAL_WORLD_XY[1]], dtype=np.float64),
            )
        )
    )
    route_xy = np.column_stack((np.zeros_like(y_nodes), y_nodes))
    return route_xy, stair_height(y_nodes)


def is_stair_riser(points_world: np.ndarray) -> np.ndarray:
    """Identify vertical stair faces so they remain traversable terrain, not obstacles."""

    y = points_world[:, 1]
    z = points_world[:, 2]
    result = np.zeros(len(points_world), dtype=bool)
    for index in range(STAIR_COUNT):
        edge = STAIR_FIRST_EDGE_Y_M + index * STAIR_TREAD_DEPTH_M
        lower = index * STAIR_RISE_M
        upper = (index + 1) * STAIR_RISE_M
        result |= (np.abs(y - edge) <= 0.065) & (z >= lower - 0.05) & (z <= upper + 0.07)
    return result


def terrain_and_obstacle_masks(points_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Classify scan returns with terrain height semantics; no obstacle pose is used."""

    ground = stair_height(points_world[:, 1])
    in_corridor = (
        (np.abs(points_world[:, 0]) <= STAIR_HALF_WIDTH_M + 0.06)
        & (points_world[:, 1] >= GRID_Y_MIN_M - 0.10)
        & (points_world[:, 1] <= GRID_Y_MAX_M + 0.10)
    )
    tread = np.abs(points_world[:, 2] - ground) <= 0.075
    terrain = in_corridor & (tread | is_stair_riser(points_world))
    height_above_terrain = points_world[:, 2] - ground
    obstacle = (
        in_corridor
        & ~terrain
        & (height_above_terrain >= 0.10)
        & (height_above_terrain <= OCCUPANCY_HEIGHT_MAX_ABOVE_TERRAIN_M)
    )
    return terrain, obstacle


def grid_shape() -> tuple[int, int]:
    nx = int(round((GRID_X_MAX_M - GRID_X_MIN_M) / GRID_RESOLUTION_M))
    ny = int(round((GRID_Y_MAX_M - GRID_Y_MIN_M) / GRID_RESOLUTION_M))
    return ny, nx


def cell_from_world(point_xy: np.ndarray) -> tuple[int, int]:
    return (
        int(math.floor((float(point_xy[1]) - GRID_Y_MIN_M) / GRID_RESOLUTION_M)),
        int(math.floor((float(point_xy[0]) - GRID_X_MIN_M) / GRID_RESOLUTION_M)),
    )


def world_from_cell(cell: tuple[int, int]) -> np.ndarray:
    iy, ix = cell
    return np.array(
        [GRID_X_MIN_M + (ix + 0.5) * GRID_RESOLUTION_M, GRID_Y_MIN_M + (iy + 0.5) * GRID_RESOLUTION_M],
        dtype=np.float64,
    )


def nearest_free(grid: np.ndarray, target: tuple[int, int]) -> Optional[tuple[int, int]]:
    ny, nx = grid.shape
    ty, tx = min(max(target[0], 0), ny - 1), min(max(target[1], 0), nx - 1)
    if not grid[ty, tx]:
        return ty, tx
    for radius in range(1, max(nx, ny)):
        for yy in range(max(0, ty - radius), min(ny, ty + radius + 1)):
            for xx in range(max(0, tx - radius), min(nx, tx + radius + 1)):
                if max(abs(yy - ty), abs(xx - tx)) == radius and not grid[yy, xx]:
                    return yy, xx
    return None


def astar(grid: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> Optional[list[tuple[int, int]]]:
    if grid[start] or grid[goal]:
        return None
    neighbours = [
        (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0), (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
    ]
    open_heap: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    score = {start: 0.0}
    while open_heap:
        _, current_cost, current = heapq.heappop(open_heap)
        if current_cost != score.get(current):
            continue
        if current == goal:
            result = [current]
            while current in came_from:
                current = came_from[current]
                result.append(current)
            return list(reversed(result))
        for dy, dx, edge_cost in neighbours:
            yy, xx = current[0] + dy, current[1] + dx
            if not (0 <= yy < grid.shape[0] and 0 <= xx < grid.shape[1]) or grid[yy, xx]:
                continue
            candidate = (yy, xx)
            proposed = current_cost + edge_cost
            if proposed >= score.get(candidate, float("inf")):
                continue
            score[candidate] = proposed
            came_from[candidate] = current
            heuristic = math.hypot(goal[0] - yy, goal[1] - xx)
            heapq.heappush(open_heap, (proposed + heuristic, proposed, candidate))
    return None


def collision_footprint(model: mujoco.MjModel, data: mujoco.MjData) -> Footprint:
    """Compute a conservative rectangle from Nezha2 collision geoms at the initial pose."""

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "BASE_LINK")
    base_position = np.asarray(data.xpos[base_id], dtype=np.float64)
    rotation = np.asarray(data.xmat[base_id], dtype=np.float64).reshape(3, 3)
    extent = np.zeros(2, dtype=np.float64)
    for geom_id in np.nonzero(model.geom_group == 1)[0]:
        centre = (np.asarray(data.geom_xpos[geom_id]) - base_position) @ rotation
        if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX:
            size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
            geom_rotation = np.asarray(data.geom_xmat[geom_id], dtype=np.float64).reshape(3, 3)
            for sx in (-size[0], size[0]):
                for sy in (-size[1], size[1]):
                    for sz in (-size[2], size[2]):
                        corner = (np.asarray(data.geom_xpos[geom_id]) + geom_rotation @ np.array([sx, sy, sz]) - base_position) @ rotation
                        extent = np.maximum(extent, np.abs(corner[:2]))
        else:
            extent = np.maximum(extent, np.abs(centre[:2]) + float(model.geom_rbound[geom_id]))
    return Footprint(half_forward_m=float(extent[0] + 0.08), half_lateral_m=float(extent[1] + 0.08), margin_m=0.08)


def make_grid(obstacle_points_world: np.ndarray, footprint: Footprint) -> tuple[np.ndarray, np.ndarray]:
    ny, nx = grid_shape()
    obstacle_grid = np.zeros((ny, nx), dtype=bool)
    x_centres = GRID_X_MIN_M + (np.arange(nx) + 0.5) * GRID_RESOLUTION_M
    # Outside the physically traversable stair width is always blocked for the robot centre.
    boundary = np.zeros((ny, nx), dtype=bool)
    boundary[:, np.abs(x_centres) > STAIR_HALF_WIDTH_M - footprint.half_lateral_m] = True
    if len(obstacle_points_world):
        ix = np.floor((obstacle_points_world[:, 0] - GRID_X_MIN_M) / GRID_RESOLUTION_M).astype(np.int32)
        iy = np.floor((obstacle_points_world[:, 1] - GRID_Y_MIN_M) / GRID_RESOLUTION_M).astype(np.int32)
        valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        obstacle_grid[iy[valid], ix[valid]] = True
    # The corridor edge is already expressed at the robot-centre boundary.
    # It must not be footprint-inflated a second time.
    inflated = boundary.copy()
    lateral_cells = int(math.ceil(footprint.half_lateral_m / GRID_RESOLUTION_M))
    forward_cells = int(math.ceil(footprint.half_forward_m / GRID_RESOLUTION_M))
    offsets = [
        (dy, dx)
        for dy in range(-forward_cells, forward_cells + 1)
        for dx in range(-lateral_cells, lateral_cells + 1)
        if (abs(dx) * GRID_RESOLUTION_M <= footprint.half_lateral_m)
        and (abs(dy) * GRID_RESOLUTION_M <= footprint.half_forward_m)
    ]
    obstacle_cells = np.argwhere(obstacle_grid)
    for iy, ix in obstacle_cells:
        for dy, dx in offsets:
            yy, xx = int(iy + dy), int(ix + dx)
            if 0 <= yy < ny and 0 <= xx < nx:
                inflated[yy, xx] = True
    return boundary | obstacle_grid, inflated


def segment_free(grid: np.ndarray, start_xy: np.ndarray, end_xy: np.ndarray) -> bool:
    distance = float(np.linalg.norm(end_xy - start_xy))
    samples = max(2, int(math.ceil(distance / (0.5 * GRID_RESOLUTION_M))))
    for point in np.linspace(start_xy, end_xy, samples):
        iy, ix = cell_from_world(point)
        if not (0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1]) or grid[iy, ix]:
            return False
    return True


def plan_from_scan(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    scan: core.LidarScan,
    footprint: Footprint,
) -> StairPlan:
    terrain_mask, obstacle_mask = terrain_and_obstacle_masks(scan.points_world)
    obstacle_points = scan.points_world[obstacle_mask]
    occupied, inflated = make_grid(obstacle_points, footprint)
    base_position, rotation = core.base_frame(model, data)
    start_xy = base_position[:2]
    start_cell = nearest_free(inflated, cell_from_world(start_xy))
    goal_cell = nearest_free(inflated, cell_from_world(GOAL_WORLD_XY))
    cells = astar(inflated, start_cell, goal_cell) if start_cell and goal_cell else None
    path = np.asarray([world_from_cell(cell) for cell in cells], dtype=np.float64) if cells else np.empty((0, 2), dtype=np.float64)
    path_found = len(path) > 1
    if path_found:
        target = path[1]
        for candidate in path[1:]:
            if float(np.linalg.norm(candidate - start_xy)) <= 0.72 and segment_free(inflated, start_xy, candidate):
                target = candidate
            elif float(np.linalg.norm(candidate - start_xy)) > 0.72:
                break
        target_body = core.points_in_base_frame(
            np.array([[target[0], target[1], base_position[2]]], dtype=np.float64), base_position, rotation
        )[0, :2]
        distance = float(np.linalg.norm(target_body))
        heading = math.atan2(float(target_body[1]), max(float(target_body[0]), 1e-8))
        # Same forward-command regime validated by the existing Nezha2 stair runner.
        speed = min(0.62, 0.85 * distance)
        command = np.array(
            [
                np.clip(speed * math.cos(heading), 0.0, 0.62),
                np.clip(speed * math.sin(heading), -0.32, 0.32),
                np.clip(0.72 * heading, -0.45, 0.45),
            ],
            dtype=np.float32,
        )
    else:
        target = start_xy.copy()
        heading = 0.0
        command = np.zeros(3, dtype=np.float32)
    x_centres = GRID_X_MIN_M + (np.arange(inflated.shape[1]) + 0.5) * GRID_RESOLUTION_M
    y_centres = GRID_Y_MIN_M + (np.arange(inflated.shape[0]) + 0.5) * GRID_RESOLUTION_M
    clearance = None
    if len(obstacle_points):
        clearance = float(np.min(np.linalg.norm(obstacle_points[:, :2] - start_xy[None, :], axis=1)))
    return StairPlan(
        desired_command=command,
        path_world_xy=path,
        path_height_m=stair_height(path[:, 1]) if len(path) else np.empty(0, dtype=np.float64),
        target_world_xy=target,
        goal_world_xy=GOAL_WORLD_XY.copy(),
        obstacle_points_world=obstacle_points,
        terrain_mask=terrain_mask,
        occupied=occupied,
        inflated=inflated,
        x_centres=x_centres,
        y_centres=y_centres,
        path_found=path_found,
        heading_offset_rad=float(heading),
        clearance_to_obstacle_m=clearance,
    )


def dynamic_obstacle_position(time_s: float) -> np.ndarray:
    """A deterministic physical obstacle motion; this pose is not planner input."""

    if time_s <= 4.5:
        x = DYNAMIC_INITIAL_X_M
    elif time_s < 7.0:
        x = DYNAMIC_INITIAL_X_M + (DYNAMIC_FINAL_X_M - DYNAMIC_INITIAL_X_M) * (time_s - 4.5) / 2.5
    else:
        x = DYNAMIC_FINAL_X_M
    return np.array([x, 2.85, 0.700], dtype=np.float64)


def set_dynamic_obstacle(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "dynamic_stair_obstacle")
    mocap_id = int(model.body_mocapid[body_id])
    position = dynamic_obstacle_position(float(data.time))
    data.mocap_pos[mocap_id] = position
    data.mocap_quat[mocap_id] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    mujoco.mj_forward(model, data)
    return position


def slew_command(current: np.ndarray, desired: np.ndarray, dt: float) -> np.ndarray:
    limits = np.array([0.55, 0.55, 1.20], dtype=np.float32)
    return current + np.clip(desired - current, -limits * dt, limits * dt)


def make_camera() -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    # Low-side oblique view keeps every tread, the robot, and all three
    # physical obstacles visible in the audit frame.
    camera.lookat[:] = [0.0, 2.65, 0.35]
    camera.distance = 6.4
    camera.azimuth = 45.0
    camera.elevation = -15.0
    return camera


def render_state(model: mujoco.MjModel, data: mujoco.MjData, destination: Path) -> None:
    model.vis.global_.offwidth, model.vis.global_.offheight = 1600, 900
    with mujoco.Renderer(model, height=900, width=1600) as renderer:
        renderer.update_scene(data, camera=make_camera())
        Image.fromarray(renderer.render()).save(destination)


def save_figure(fig: plt.Figure, destination: Path, *, facecolor: Optional[str] = None) -> None:
    options: dict[str, Any] = {"bbox_inches": "tight"}
    if facecolor is not None:
        options["facecolor"] = facecolor
    fig.savefig(destination, dpi=600, **options)
    fig.savefig(destination.with_suffix(".svg"), **options)
    fig.savefig(destination.with_suffix(".pdf"), **options)
    fig.savefig(destination.with_suffix(".tiff"), dpi=600, **options)


def plot_full_point_cloud(
    points_base: np.ndarray,
    time_s: float,
    plan: StairPlan,
    base_position: np.ndarray,
    rotation: np.ndarray,
    destination: Path,
) -> None:
    fig = plt.figure(figsize=(7.2, 5.0), dpi=600, facecolor="#07131f")
    ax = fig.add_subplot(111, projection="3d", facecolor="#07131f")
    ranges = np.linalg.norm(points_base, axis=1)
    ax.scatter(points_base[:, 0], points_base[:, 1], points_base[:, 2], c=ranges, cmap="viridis", s=0.8, alpha=0.82, linewidths=0, rasterized=True)
    global_xy, global_height = global_semantic_route()
    global_world = np.column_stack((global_xy, global_height))
    global_base = core.points_in_base_frame(global_world, base_position, rotation)
    ax.plot(
        global_base[:, 0], global_base[:, 1], global_base[:, 2],
        color="#51c4c5", linewidth=1.5, linestyle="--", label="Global semantic route",
    )
    if plan.path_found:
        local_world = np.column_stack((plan.path_world_xy, plan.path_height_m))
        local_base = core.points_in_base_frame(local_world, base_position, rotation)
        ax.plot(
            local_base[:, 0], local_base[:, 1], local_base[:, 2],
            color="#ff9d35", linewidth=2.2, label="LiDAR-constrained A* replan",
        )
    ax.scatter([0], [0], [0], marker="^", color="#ffdd44", s=70, label="Nezha2 base")
    bounds = np.vstack((np.min(points_base, axis=0), np.max(points_base, axis=0)))
    span = np.maximum(bounds[1] - bounds[0], 0.4)
    padding = 0.04 * span + 0.05
    ax.set_xlim(float(bounds[0, 0] - padding[0]), float(bounds[1, 0] + padding[0]))
    ax.set_ylim(float(bounds[0, 1] - padding[1]), float(bounds[1, 1] + padding[1]))
    ax.set_zlim(float(bounds[0, 2] - padding[2]), float(bounds[1, 2] + padding[2]))
    ax.set_box_aspect(tuple(span.tolist()))
    ax.set_xlabel("forward / m", color="white", labelpad=8, fontsize=9)
    ax.set_ylabel("left / m", color="white", labelpad=8, fontsize=9)
    ax.set_zlabel("height / m", color="white", labelpad=8, fontsize=9)
    ax.tick_params(colors="#d7e3ef", labelsize=7)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((0.03, 0.08, 0.12, 1.0))
        axis._axinfo["grid"]["color"] = (0.35, 0.48, 0.58, 0.28)
    ax.view_init(elev=24, azim=-61)
    ax.set_title(f"All valid MuJoCo ray-cast returns ({len(points_base):,}; t={time_s:.3f} s)", color="white", pad=10, fontsize=10)
    legend = ax.legend(loc="upper left", fontsize=7)
    for label in legend.get_texts():
        label.set_color("white")
    fig.tight_layout(pad=0.8)
    save_figure(fig, destination, facecolor="#07131f")
    plt.close(fig)


def plot_plan(plan: StairPlan, capture_time_s: float, footprint: Footprint, destination: Path) -> None:
    fig, (ax_xy, ax_profile) = plt.subplots(1, 2, figsize=(11.4, 4.9), dpi=600, gridspec_kw={"width_ratios": [1.15, 0.85]})
    extent = [GRID_X_MIN_M, GRID_X_MAX_M, GRID_Y_MIN_M, GRID_Y_MAX_M]
    terrain = np.tile(stair_height(plan.y_centres)[:, None], (1, len(plan.x_centres)))
    terrain_image = ax_xy.imshow(terrain, extent=extent, origin="lower", cmap="Blues", alpha=0.60, interpolation="nearest")
    ax_xy.imshow(plan.inflated.astype(np.uint8), extent=extent, origin="lower", cmap="Greys", alpha=0.38, interpolation="nearest")
    if len(plan.obstacle_points_world):
        ax_xy.scatter(plan.obstacle_points_world[:, 0], plan.obstacle_points_world[:, 1], s=3, c="#343a40", alpha=0.65, linewidths=0, label="LiDAR obstacle returns", rasterized=True)
    global_xy, global_height = global_semantic_route()
    ax_xy.plot(global_xy[:, 0], global_xy[:, 1], color="#2a9d8f", linewidth=1.5, linestyle="--", label="Global semantic route")
    if plan.path_found:
        ax_xy.plot(plan.path_world_xy[:, 0], plan.path_world_xy[:, 1], color="#e07a1f", linewidth=2.0, label="Local LiDAR-constrained A* route")
        ax_xy.scatter(*plan.target_world_xy, marker="x", s=62, linewidths=2.2, color="#d94801", label="Velocity target")
    ax_xy.scatter(0, 0, s=0, alpha=0)  # Stabilize autoscaling before the semantic axes are fixed.
    ax_xy.scatter(plan.goal_world_xy[0], plan.goal_world_xy[1], marker="*", s=105, color="#2a9d8f", label="Goal")
    ax_xy.set(xlim=(GRID_X_MIN_M, GRID_X_MAX_M), ylim=(GRID_Y_MIN_M, GRID_Y_MAX_M), xlabel="world lateral x / m", ylabel="world ascent y / m")
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_title("LiDAR obstacles + traversable stair-height layer", fontsize=9)
    ax_xy.legend(loc="upper left", fontsize=7, frameon=True, framealpha=0.92)
    ax_xy.grid(alpha=0.20, linewidth=0.5)
    colorbar = fig.colorbar(terrain_image, ax=ax_xy, fraction=0.045, pad=0.03)
    colorbar.set_label("traversable terrain height / m", fontsize=7)
    colorbar.ax.tick_params(labelsize=7)
    profile_y = np.linspace(GRID_Y_MIN_M, GRID_Y_MAX_M, 500)
    profile_z = stair_height(profile_y)
    ax_profile.step(profile_y, profile_z, where="post", color="#215a8d", linewidth=1.8, label="Stair terrain reference")
    ax_profile.step(global_xy[:, 1], global_height, where="post", color="#2a9d8f", linewidth=1.5, linestyle="--", label="Global height reference")
    if plan.path_found:
        ax_profile.plot(plan.path_world_xy[:, 1], plan.path_height_m, color="#e07a1f", linewidth=2.0, label="Local A* height reference")
    if len(plan.obstacle_points_world):
        ax_profile.scatter(plan.obstacle_points_world[:, 1], plan.obstacle_points_world[:, 2], s=3, c="#343a40", alpha=0.50, linewidths=0, label="Obstacle returns", rasterized=True)
    ax_profile.set(xlabel="world ascent y / m", ylabel="height z / m", xlim=(GRID_Y_MIN_M, GRID_Y_MAX_M), ylim=(-0.05, 1.05))
    ax_profile.set_title("Height-constrained route reference", fontsize=9)
    ax_profile.grid(alpha=0.22, linewidth=0.5)
    ax_profile.legend(loc="upper left", fontsize=7, frameon=True, framealpha=0.92)
    fig.suptitle(
        f"Point-cloud stair plan at t={capture_time_s:.3f} s | {GRID_RESOLUTION_M:.2f} m grid | footprint {2 * footprint.half_forward_m:.2f} x {2 * footprint.half_lateral_m:.2f} m",
        fontsize=10,
        y=0.99,
    )
    fig.tight_layout(pad=0.8)
    save_figure(fig, destination)
    plt.close(fig)


def plot_executed_trajectory(
    trajectory_world_xy: np.ndarray,
    plan: StairPlan,
    capture_base_world_xy: np.ndarray,
    destination: Path,
) -> None:
    """Post-run audit: actual qpos trajectory versus the same captured A* plan."""

    fig, ax = plt.subplots(figsize=(5.6, 7.0), dpi=600)
    extent = [GRID_X_MIN_M, GRID_X_MAX_M, GRID_Y_MIN_M, GRID_Y_MAX_M]
    terrain = np.tile(stair_height(plan.y_centres)[:, None], (1, len(plan.x_centres)))
    image = ax.imshow(terrain, extent=extent, origin="lower", cmap="Blues", alpha=0.58, interpolation="nearest")
    ax.imshow(plan.inflated.astype(np.uint8), extent=extent, origin="lower", cmap="Greys", alpha=0.32, interpolation="nearest")
    if len(plan.obstacle_points_world):
        ax.scatter(
            plan.obstacle_points_world[:, 0], plan.obstacle_points_world[:, 1],
            s=2.0, c="#3e464e", alpha=0.50, linewidths=0, label="LiDAR obstacle returns at capture", rasterized=True,
        )
    global_xy, _ = global_semantic_route()
    ax.plot(global_xy[:, 0], global_xy[:, 1], color="#2a9d8f", linewidth=1.5, linestyle="--", label="Global semantic route")
    if len(trajectory_world_xy):
        ax.plot(
            trajectory_world_xy[:, 0], trajectory_world_xy[:, 1],
            color="#7b2cbf", linewidth=1.8, label="Executed qpos trajectory (20 Hz)",
        )
    if plan.path_found:
        ax.plot(plan.path_world_xy[:, 0], plan.path_world_xy[:, 1], color="#e07a1f", linewidth=1.9, label="Captured local A* route")
    ax.scatter(capture_base_world_xy[0], capture_base_world_xy[1], marker="o", s=45, color="#106ba3", edgecolors="white", linewidths=0.65, label="Robot at capture", zorder=5)
    ax.scatter(plan.goal_world_xy[0], plan.goal_world_xy[1], marker="*", s=120, color="#2a9d8f", label="Goal", zorder=5)
    ax.set(
        xlim=(GRID_X_MIN_M, GRID_X_MAX_M), ylim=(GRID_Y_MIN_M, GRID_Y_MAX_M),
        xlabel="world lateral x / m", ylabel="world ascent y / m",
        title="Executed Nezha2 ascent and captured point-cloud plan",
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.20, linewidth=0.5)
    ax.legend(loc="upper left", fontsize=7, frameon=True, framealpha=0.92)
    colorbar = fig.colorbar(image, ax=ax, fraction=0.046, pad=0.03)
    colorbar.set_label("traversable terrain height / m", fontsize=7)
    colorbar.ax.tick_params(labelsize=7)
    fig.tight_layout(pad=0.8)
    save_figure(fig, destination)
    plt.close(fig)


def make_demo_figure(simulation: Path, point_cloud: Path, planner: Path, destination: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15.0, 4.0), dpi=600)
    titles = ["(a) Same-state MuJoCo frame", "(b) Full raw ray-cast point cloud", "(c) Terrain-aware local plan"]
    for axis, image_path, title in zip(axes, (simulation, point_cloud, planner), titles):
        axis.imshow(Image.open(image_path).convert("RGB"))
        axis.set_title(title, fontsize=8.5, color="#174e77", pad=6)
        axis.set_axis_off()
    fig.subplots_adjust(left=0.004, right=0.996, top=0.89, bottom=0.02, wspace=0.012)
    save_figure(fig, destination)
    plt.close(fig)


def capture(model: mujoco.MjModel, data: mujoco.MjData, plan: StairPlan, footprint: Footprint, output_dir: Path, trigger: str) -> dict[str, Any]:
    render_state(model, data, output_dir / "simulation.png")
    diagnostic = core.lidar_scan(model, data, DIAGNOSTIC_AZIMUTH_COUNT, DIAGNOSTIC_ELEVATION_DEG)
    base_position, rotation = core.base_frame(model, data)
    points_base = core.points_in_base_frame(diagnostic.points_world, base_position, rotation)
    terrain_mask, obstacle_mask = terrain_and_obstacle_masks(diagnostic.points_world)
    plot_full_point_cloud(points_base, float(data.time), plan, base_position, rotation, output_dir / "pointcloud_full.png")
    plot_plan(plan, float(data.time), footprint, output_dir / "planner.png")
    make_demo_figure(output_dir / "simulation.png", output_dir / "pointcloud_full.png", output_dir / "planner.png", output_dir / "demo.png")
    payload = {
        "capture_trigger": trigger,
        "time_s": float(data.time),
        "qpos": [float(value) for value in data.qpos],
        "qvel": [float(value) for value in data.qvel],
        "lidar_origin_world_m": [float(value) for value in diagnostic.origin_world],
        "diagnostic_scan": {
            "azimuth_beams": DIAGNOSTIC_AZIMUTH_COUNT,
            "elevation_beams": len(DIAGNOSTIC_ELEVATION_DEG),
            "raw_ray_count": int(len(diagnostic.distance_m)),
            "valid_return_count": int(np.count_nonzero(diagnostic.valid)),
            "visualization": "all valid returns; no random downsampling or spatial crop",
        },
        "planner": {
            "replan_period_s": PLANNER_PERIOD_S,
            "grid_resolution_m": GRID_RESOLUTION_M,
            "traversability": "stair tread/riser height semantics; obstacle coordinates are not planner input",
            "collision_footprint_m": {"full_forward": 2 * footprint.half_forward_m, "full_lateral": 2 * footprint.half_lateral_m, "margin": footprint.margin_m},
            "path_found": bool(plan.path_found),
            "target_world_xy_m": [float(value) for value in plan.target_world_xy],
            "desired_command_vx_vy_wz": [float(value) for value in plan.desired_command],
            "obstacle_return_count": int(len(plan.obstacle_points_world)),
        },
    }
    (output_dir / "capture.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    np.savez_compressed(
        output_dir / "lidar_capture.npz",
        points_world=diagnostic.points_world,
        points_base=points_base,
        directions_lidar=diagnostic.directions_lidar,
        directions_world=diagnostic.directions_world,
        distance_m=diagnostic.distance_m,
        geom_id=diagnostic.geom_id,
        valid=diagnostic.valid,
        terrain_return_mask=terrain_mask,
        obstacle_return_mask=obstacle_mask,
        lidar_origin_world=diagnostic.origin_world,
        lidar_rotation_world_from_lidar=diagnostic.rotation_world_from_lidar,
        planner_path_world_xy=plan.path_world_xy,
        planner_path_height_m=plan.path_height_m,
        global_semantic_route_world_xy=global_semantic_route()[0],
        global_semantic_route_height_m=global_semantic_route()[1],
    )
    return {
        "time_s": float(data.time),
        "trigger": trigger,
        "raw_rays": int(len(diagnostic.distance_m)),
        "valid_returns": int(np.count_nonzero(diagnostic.valid)),
        "base_world_xy": [float(base_position[0]), float(base_position[1])],
    }


def run(seed: int, output_dir: Path) -> dict[str, Any]:
    torch.manual_seed(seed)
    output_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    set_dynamic_obstacle(model, data)
    footprint = collision_footprint(model, data)
    policy = torch.jit.load(str(core.MODEL_DIR / POLICY_NAME), map_location="cpu").eval()
    torch.set_grad_enabled(False)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in core.JOINTS]
    qadr = np.array([model.jnt_qposadr[joint] for joint in joint_ids])
    vadr = np.array([model.jnt_dofadr[joint] for joint in joint_ids])
    actuator_ids = [int(np.where(model.actuator_trnid[:, 0] == joint)[0][0]) for joint in joint_ids]
    gyro_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")
    gyro_adr = int(model.sensor_adr[gyro_id])
    obstacle_geom_ids = {gid for gid in range(model.ngeom) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "").startswith("obstacle_")}
    policy_period = max(1, round(0.02 / model.opt.timestep))
    history = np.zeros((5, 57), dtype=np.float32)
    previous_action = np.zeros(16, dtype=np.float32)
    command = np.zeros(3, dtype=np.float32)
    desired_command = np.zeros(3, dtype=np.float32)
    planner_replans: list[dict[str, Any]] = []
    trajectory: list[tuple[float, float]] = []
    next_replan_time = 2.0
    last_plan: Optional[StairPlan] = None
    captured_plan: Optional[StairPlan] = None
    evidence: Optional[dict[str, Any]] = None
    collision = False
    total_steps = round(30.0 / model.opt.timestep)
    for step in range(total_steps):
        dynamic_position = set_dynamic_obstacle(model, data)
        q = np.asarray(data.qpos[qadr], dtype=np.float32)
        dq = np.asarray(data.qvel[vadr], dtype=np.float32)
        if data.time < 2.0:
            desired_command[:] = 0.0
        elif data.time + 1e-9 >= next_replan_time:
            planner_scan = core.lidar_scan(model, data, PLANNER_AZIMUTH_COUNT, PLANNER_ELEVATION_DEG)
            last_plan = plan_from_scan(model, data, planner_scan, footprint)
            desired_command = last_plan.desired_command.copy()
            planner_replans.append(
                {
                    "time_s": float(data.time),
                    "planner_ray_count": int(len(planner_scan.distance_m)),
                    "planner_valid_returns": int(np.count_nonzero(planner_scan.valid)),
                    "path_found": bool(last_plan.path_found),
                    "desired_command_vx_vy_wz": [float(value) for value in desired_command],
                    "target_world_xy_m": [float(value) for value in last_plan.target_world_xy],
                    "dynamic_obstacle_world_xyz_m": [float(value) for value in dynamic_position],
                    "obstacle_return_count": int(len(last_plan.obstacle_points_world)),
                }
            )
            next_replan_time += PLANNER_PERIOD_S
        command = slew_command(command, desired_command, float(model.opt.timestep))
        obstacle_has_moved = abs(float(dynamic_position[0]) - DYNAMIC_INITIAL_X_M) >= 0.10
        obstacle_turning_response = float(np.linalg.norm(last_plan.desired_command[1:])) >= 0.12 if last_plan is not None else False
        if evidence is None and last_plan is not None and obstacle_has_moved and obstacle_turning_response:
            evidence = capture(
                model,
                data,
                last_plan,
                footprint,
                output_dir,
                "first 10 Hz replan with a >=0.12 lateral/yaw correction after the moving obstacle displaced by >=0.10 m",
            )
            captured_plan = last_plan
        relative_q = q - core.DEFAULT
        relative_q[~core.LEG_MASK] = 0.0
        observation = np.concatenate(
            [
                np.asarray(data.sensordata[gyro_adr : gyro_adr + 3], dtype=np.float32) * 0.25,
                core.gravity_from_quaternion(np.asarray(data.qpos[3:7], dtype=np.float32)) * 0.5,
                command * core.COMMAND_SCALE,
                relative_q,
                dq * 0.05,
                previous_action,
            ]
        ).astype(np.float32)
        if data.time < 2.0:
            phase = np.clip(data.time / 2.0, 0.0, 1.0)
            phase = phase * phase * (3.0 - 2.0 * phase)
            target = q + phase * (core.DEFAULT - q)
            wheel_velocity = np.zeros(16, dtype=np.float32)
        else:
            if step % policy_period == 0:
                previous_action = policy(torch.from_numpy(observation[None]), torch.from_numpy(history.reshape(1, -1))).cpu().numpy()[0].astype(np.float32)
                previous_action = np.clip(previous_action, -100.0, 100.0)
                history[:-1] = history[1:]
                history[-1] = observation
            target = core.DEFAULT.copy()
            target[core.LEG_MASK] += previous_action[core.LEG_MASK] * core.ACTION_SCALE[core.LEG_MASK]
            wheel_velocity = np.zeros(16, dtype=np.float32)
            wheel_velocity[~core.LEG_MASK] = previous_action[~core.LEG_MASK] * core.ACTION_SCALE[~core.LEG_MASK]
        torque = np.clip(core.KP * (target - q) + core.KD * (wheel_velocity - dq), -core.TORQUE_LIMIT, core.TORQUE_LIMIT)
        data.ctrl[:] = 0.0
        for index, actuator_id in enumerate(actuator_ids):
            data.ctrl[actuator_id] = float(torque[index])
        mujoco.mj_step(model, data)
        if step % max(1, round(0.05 / model.opt.timestep)) == 0:
            trajectory.append((float(data.qpos[0]), float(data.qpos[1])))
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.geom1 in obstacle_geom_ids or contact.geom2 in obstacle_geom_ids:
                other = contact.geom2 if contact.geom1 in obstacle_geom_ids else contact.geom1
                if other >= 0 and model.geom_bodyid[other] != 0:
                    collision = True
                    break
        if collision or data.qpos[2] < 0.14 or not np.isfinite(data.qpos).all() or data.qpos[1] >= GOAL_WORLD_XY[1]:
            break
    if evidence is None:
        if last_plan is None:
            fallback_scan = core.lidar_scan(model, data, PLANNER_AZIMUTH_COUNT, PLANNER_ELEVATION_DEG)
            last_plan = plan_from_scan(model, data, fallback_scan, footprint)
        evidence = capture(model, data, last_plan, footprint, output_dir, "terminal-state fallback")
        captured_plan = last_plan
    np.save(output_dir / "trajectory_world_xy.npy", np.asarray(trajectory, dtype=np.float64))
    if captured_plan is not None:
        plot_executed_trajectory(
            np.asarray(trajectory, dtype=np.float64),
            captured_plan,
            np.asarray(evidence["base_world_xy"], dtype=np.float64),
            output_dir / "executed_trajectory.png",
        )
    (output_dir / "planner_replans.json").write_text(json.dumps(planner_replans, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    result: dict[str, Any] = {
        "scene": str(SCENE),
        "policy": f"{POLICY_NAME} (unmodified)",
        "seed": seed,
        "success": bool(data.qpos[1] >= GOAL_WORLD_XY[1] and data.qpos[2] >= 0.25 and not collision),
        "nonterrain_obstacle_collision": collision,
        "collision_definition": "robot contact with a geom named obstacle_*; terrain contacts are expected and excluded",
        "time_s": float(data.time),
        "final_xyz": [float(value) for value in data.qpos[:3]],
        "planner_replans": len(planner_replans),
        "planner_rate_hz": 1.0 / PLANNER_PERIOD_S,
        "capture": evidence,
        "collision_footprint_m": {"full_forward": 2 * footprint.half_forward_m, "full_lateral": 2 * footprint.half_lateral_m, "margin": footprint.margin_m},
        "strategy": "terrain-height traversability + LiDAR obstacle layer + footprint inflation + A* route + rate-limited velocity reference",
    }
    (output_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False))
    return result


def main() -> int:
    args = parse_args()
    result = run(args.seed, args.output_dir)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
