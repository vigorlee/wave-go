#!/usr/bin/python3
"""Build a fail-closed static map for the Go2-W HouseWorld scene."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import struct
from typing import Iterable, Sequence
import xml.etree.ElementTree as ET
import zlib


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    ROOT_DIR
    / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_house.xml"
)
DEFAULT_ROUTES = ROOT_DIR / "config/cosmos_vln_house_routes.json"
DEFAULT_OUTPUT_DIR = ROOT_DIR / ".run/go2w_house/map"

MAP_RESOLUTION = 0.05
PROJECTION_MIN_Z = 0.10
PROJECTION_MAX_Z = 1.20
MAP_CONNECTIVITY_CLEARANCE = 0.10
PREVIEW_SAFETY_CLEARANCE = 0.40
SPAWN = (1.10, -7.20)

# This follows the architectural shell instead of the rectangular XML extent.
# Everything outside the polygon is occupied. Rooms disconnected from SPAWN are
# also occupied, which prevents planners from leaking through tiny model seams.
HOUSE_BOUNDS_POLYGON: tuple[tuple[float, float], ...] = (
    (-4.98, -9.51),
    (2.32, -9.51),
    (2.32, -8.76),
    (14.65, -8.76),
    (14.65, 8.29),
    (9.09, 8.29),
    (9.09, 7.19),
    (4.15, 7.19),
    (4.15, 10.03),
    (-2.74, 10.03),
    (-2.74, 2.11),
    (-4.98, 2.11),
)

RECOMMENDED_ROUTE: tuple[tuple[str, float, float], ...] = (
    ("spawn", 1.10, -7.20),
    ("west_room", -3.70, -0.30),
    ("north_lounge", -0.50, 7.50),
    ("north_room", 1.00, 8.50),
    ("upper_center", 7.50, 2.50),
    ("corridor_west", 8.20, 1.20),
    ("corridor_east", 9.70, 1.20),
    ("east_door_inside", 10.25, 3.45),
    ("kitchen", 12.00, 5.00),
    ("far_northeast_safe", 10.35, 7.45),
)


@dataclass(frozen=True)
class ProjectedObstacle:
    name: str
    kind: str
    center_x: float
    center_y: float
    half_x_or_radius: float
    half_y: float
    yaw_rad: float
    min_z: float
    max_z: float

    def signed_clearance(self, x: float, y: float) -> float:
        """Return positive outside clearance and negative penetration depth."""
        dx = x - self.center_x
        dy = y - self.center_y
        if self.kind == "cylinder":
            return math.hypot(dx, dy) - self.half_x_or_radius

        cosine = math.cos(self.yaw_rad)
        sine = math.sin(self.yaw_rad)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        outside_x = max(abs(local_x) - self.half_x_or_radius, 0.0)
        outside_y = max(abs(local_y) - self.half_y, 0.0)
        if outside_x or outside_y:
            return math.hypot(outside_x, outside_y)
        return -min(
            self.half_x_or_radius - abs(local_x),
            self.half_y - abs(local_y),
        )


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def cell_center(self, column: int, row: int) -> tuple[float, float]:
        return (
            self.origin_x + (column + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            math.floor((x - self.origin_x) / self.resolution),
            math.floor((y - self.origin_y) / self.resolution),
        )


@dataclass(frozen=True)
class HouseMap:
    spec: GridSpec
    obstacles: tuple[ProjectedObstacle, ...]
    raw_free: tuple[bool, ...]
    spawn_reachable: tuple[bool, ...]

    def index(self, column: int, row: int) -> int:
        return row * self.spec.width + column

    def in_grid(self, column: int, row: int) -> bool:
        return 0 <= column < self.spec.width and 0 <= row < self.spec.height

    def is_navigable_cell(self, column: int, row: int) -> bool:
        return self.in_grid(column, row) and self.spawn_reachable[
            self.index(column, row)
        ]

    def is_navigable_point(self, x: float, y: float) -> bool:
        return self.is_navigable_cell(*self.spec.world_to_cell(x, y))


def quaternion_yaw(values: Sequence[float]) -> float:
    if len(values) != 4:
        raise ValueError(f"expected a four-value quaternion, got {values!r}")
    w, x, y, z = values
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_projected_obstacles(
    scene_path: Path = DEFAULT_SCENE,
    min_z: float = PROJECTION_MIN_Z,
    max_z: float = PROJECTION_MAX_Z,
) -> tuple[ProjectedObstacle, ...]:
    if min_z > max_z:
        raise ValueError("projection min_z must not exceed max_z")
    root = ET.parse(scene_path).getroot()
    obstacles: list[ProjectedObstacle] = []
    for geom in root.findall(".//worldbody/geom"):
        kind = geom.get("type", "sphere")
        if kind not in {"box", "cylinder"}:
            continue
        position = tuple(float(value) for value in geom.get("pos", "0 0 0").split())
        size = tuple(float(value) for value in geom.get("size", "").split())
        if len(position) != 3 or len(size) < 2:
            raise ValueError(f"invalid geometry dimensions for {geom.get('name')!r}")
        geom_min_z = position[2] - size[-1]
        geom_max_z = position[2] + size[-1]
        if geom_max_z < min_z or geom_min_z > max_z:
            continue
        if kind == "box" and len(size) != 3:
            raise ValueError(f"box {geom.get('name')!r} must have three size values")
        yaw_rad = 0.0
        half_y = 0.0
        if kind == "box":
            quaternion = tuple(
                float(value) for value in geom.get("quat", "1 0 0 0").split()
            )
            yaw_rad = quaternion_yaw(quaternion)
            half_y = size[1]
        obstacles.append(
            ProjectedObstacle(
                name=geom.get("name", f"unnamed_{len(obstacles)}"),
                kind=kind,
                center_x=position[0],
                center_y=position[1],
                half_x_or_radius=size[0],
                half_y=half_y,
                yaw_rad=yaw_rad,
                min_z=geom_min_z,
                max_z=geom_max_z,
            )
        )
    if not obstacles:
        raise RuntimeError(f"no projected box/cylinder obstacles found in {scene_path}")
    return tuple(obstacles)


def point_in_polygon(
    x: float,
    y: float,
    polygon: Sequence[tuple[float, float]] = HOUSE_BOUNDS_POLYGON,
) -> bool:
    inside = False
    for start, end in zip(polygon, (*polygon[1:], polygon[0])):
        x1, y1 = start
        x2, y2 = end
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
    return inside


def point_segment_distance(
    x: float,
    y: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    x1, y1 = start
    x2, y2 = end
    vx = x2 - x1
    vy = y2 - y1
    scale = ((x - x1) * vx + (y - y1) * vy) / (vx * vx + vy * vy)
    scale = max(0.0, min(1.0, scale))
    return math.hypot(x - (x1 + scale * vx), y - (y1 + scale * vy))


def boundary_clearance(
    x: float,
    y: float,
    polygon: Sequence[tuple[float, float]] = HOUSE_BOUNDS_POLYGON,
) -> float:
    if not point_in_polygon(x, y, polygon):
        return -1.0
    return min(
        point_segment_distance(x, y, start, end)
        for start, end in zip(polygon, (*polygon[1:], polygon[0]))
    )


def point_clearance(
    x: float,
    y: float,
    obstacles: Sequence[ProjectedObstacle],
    polygon: Sequence[tuple[float, float]] = HOUSE_BOUNDS_POLYGON,
) -> float:
    clearance = boundary_clearance(x, y, polygon)
    if clearance < 0.0:
        return clearance
    return min(clearance, *(item.signed_clearance(x, y) for item in obstacles))


def default_grid_spec(resolution: float = MAP_RESOLUTION) -> GridSpec:
    min_x = min(point[0] for point in HOUSE_BOUNDS_POLYGON)
    max_x = max(point[0] for point in HOUSE_BOUNDS_POLYGON)
    min_y = min(point[1] for point in HOUSE_BOUNDS_POLYGON)
    max_y = max(point[1] for point in HOUSE_BOUNDS_POLYGON)
    return GridSpec(
        resolution=resolution,
        origin_x=min_x,
        origin_y=min_y,
        width=math.ceil((max_x - min_x) / resolution),
        height=math.ceil((max_y - min_y) / resolution),
    )


def flood_spawn_component(
    spec: GridSpec, component_free: Sequence[bool]
) -> tuple[bool, ...]:
    spawn_cell = spec.world_to_cell(*SPAWN)
    column, row = spawn_cell
    if not (0 <= column < spec.width and 0 <= row < spec.height):
        raise RuntimeError(f"spawn {SPAWN} is outside the map grid")
    spawn_index = row * spec.width + column
    if not component_free[spawn_index]:
        raise RuntimeError(f"spawn {SPAWN} is occupied in the projected HouseWorld map")

    reachable = [False] * len(component_free)
    reachable[spawn_index] = True
    pending: deque[tuple[int, int]] = deque([spawn_cell])
    while pending:
        current_column, current_row = pending.popleft()
        for delta_column, delta_row in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_column = current_column + delta_column
            next_row = current_row + delta_row
            if not (0 <= next_column < spec.width and 0 <= next_row < spec.height):
                continue
            next_index = next_row * spec.width + next_column
            if component_free[next_index] and not reachable[next_index]:
                reachable[next_index] = True
                pending.append((next_column, next_row))
    return tuple(reachable)


def build_house_map(
    scene_path: Path = DEFAULT_SCENE,
    resolution: float = MAP_RESOLUTION,
) -> HouseMap:
    obstacles = load_projected_obstacles(scene_path)
    spec = default_grid_spec(resolution)
    raw_free: list[bool] = []
    component_free: list[bool] = []
    for row in range(spec.height):
        for column in range(spec.width):
            x, y = spec.cell_center(column, row)
            clearance = point_clearance(x, y, obstacles)
            raw_free.append(clearance > 0.0)
            component_free.append(clearance >= MAP_CONNECTIVITY_CLEARANCE)
    spawn_reachable = flood_spawn_component(spec, component_free)
    return HouseMap(
        spec=spec,
        obstacles=obstacles,
        raw_free=tuple(raw_free),
        spawn_reachable=spawn_reachable,
    )


def route_points(routes_file: Path | None) -> tuple[tuple[str, float, float], ...]:
    if routes_file is None or not routes_file.is_file():
        return RECOMMENDED_ROUTE
    payload = json.loads(routes_file.read_text(encoding="utf-8"))
    routes = payload.get("routes")
    if not isinstance(routes, list) or not routes:
        return RECOMMENDED_ROUTE
    longest = max(
        routes,
        key=lambda route: len(route.get("stages", [])) if isinstance(route, dict) else 0,
    )
    stages = longest.get("stages") if isinstance(longest, dict) else None
    if not isinstance(stages, list) or not stages:
        return RECOMMENDED_ROUTE
    points: list[tuple[str, float, float]] = [("spawn", *SPAWN)]
    for index, stage in enumerate(stages):
        if not isinstance(stage, dict) or not isinstance(stage.get("goal"), dict):
            return RECOMMENDED_ROUTE
        goal = stage["goal"]
        try:
            points.append(
                (
                    str(stage.get("stage_id", f"stage_{index + 1}")),
                    float(goal["x"]),
                    float(goal["y"]),
                )
            )
        except (KeyError, TypeError, ValueError):
            return RECOMMENDED_ROUTE
    return tuple(points)


def write_pgm(path: Path, house_map: HouseMap) -> None:
    spec = house_map.spec
    pixels = bytearray()
    for row in reversed(range(spec.height)):
        for column in range(spec.width):
            pixels.append(254 if house_map.is_navigable_cell(column, row) else 0)
    path.write_bytes(f"P5\n{spec.width} {spec.height}\n255\n".encode("ascii") + pixels)


def write_yaml(path: Path, pgm_name: str, spec: GridSpec) -> None:
    path.write_text(
        "\n".join(
            (
                f"image: {pgm_name}",
                f"resolution: {spec.resolution:.6f}",
                f"origin: [{spec.origin_x:.6f}, {spec.origin_y:.6f}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "mode: trinary",
                "",
            )
        ),
        encoding="ascii",
    )


def set_pixel(
    pixels: bytearray,
    width: int,
    height: int,
    column: int,
    row: int,
    color: tuple[int, int, int],
) -> None:
    if 0 <= column < width and 0 <= row < height:
        offset = (row * width + column) * 3
        pixels[offset : offset + 3] = bytes(color)


def draw_line(
    pixels: bytearray,
    width: int,
    height: int,
    start: tuple[int, int],
    end: tuple[int, int],
    color: tuple[int, int, int],
    thickness: int = 2,
) -> None:
    x1, y1 = start
    x2, y2 = end
    dx = abs(x2 - x1)
    step_x = 1 if x1 < x2 else -1
    dy = -abs(y2 - y1)
    step_y = 1 if y1 < y2 else -1
    error = dx + dy
    while True:
        for offset_y in range(-thickness, thickness + 1):
            for offset_x in range(-thickness, thickness + 1):
                if offset_x * offset_x + offset_y * offset_y <= thickness * thickness:
                    set_pixel(
                        pixels,
                        width,
                        height,
                        x1 + offset_x,
                        y1 + offset_y,
                        color,
                    )
        if x1 == x2 and y1 == y2:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x1 += step_x
        if doubled <= dx:
            error += dx
            y1 += step_y


def draw_circle(
    pixels: bytearray,
    width: int,
    height: int,
    center: tuple[int, int],
    radius: int,
    color: tuple[int, int, int],
) -> None:
    center_x, center_y = center
    for offset_y in range(-radius, radius + 1):
        for offset_x in range(-radius, radius + 1):
            if offset_x * offset_x + offset_y * offset_y <= radius * radius:
                set_pixel(
                    pixels,
                    width,
                    height,
                    center_x + offset_x,
                    center_y + offset_y,
                    color,
                )


def preview_navigation_mask(
    house_map: HouseMap,
    safety_clearance: float = PREVIEW_SAFETY_CLEARANCE,
) -> tuple[bool, ...]:
    mask: list[bool] = []
    spec = house_map.spec
    for row in range(spec.height):
        for column in range(spec.width):
            index = house_map.index(column, row)
            if not house_map.spawn_reachable[index]:
                mask.append(False)
                continue
            x, y = spec.cell_center(column, row)
            mask.append(point_clearance(x, y, house_map.obstacles) >= safety_clearance)
    return tuple(mask)


def astar_preview_path(
    house_map: HouseMap,
    navigation_mask: Sequence[bool],
    start: tuple[float, float],
    goal: tuple[float, float],
) -> list[tuple[int, int]]:
    spec = house_map.spec
    start_cell = spec.world_to_cell(*start)
    goal_cell = spec.world_to_cell(*goal)

    def free(cell: tuple[int, int]) -> bool:
        column, row = cell
        return house_map.in_grid(column, row) and navigation_mask[
            house_map.index(column, row)
        ]

    if not free(start_cell) or not free(goal_cell):
        raise ValueError(f"preview route endpoint is unsafe: {start} -> {goal}")
    costs = {start_cell: 0.0}
    previous: dict[tuple[int, int], tuple[int, int]] = {}
    pending: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_cell)]
    moves = (
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    )
    while pending:
        _, cost, current = heapq.heappop(pending)
        if cost != costs.get(current):
            continue
        if current == goal_cell:
            break
        for delta_column, delta_row, multiplier in moves:
            neighbor = current[0] + delta_column, current[1] + delta_row
            if not free(neighbor):
                continue
            if delta_column and delta_row:
                if not free((current[0] + delta_column, current[1])):
                    continue
                if not free((current[0], current[1] + delta_row)):
                    continue
            next_cost = cost + multiplier * spec.resolution
            if next_cost >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = next_cost
            previous[neighbor] = current
            heuristic = math.hypot(
                neighbor[0] - goal_cell[0], neighbor[1] - goal_cell[1]
            ) * spec.resolution
            heapq.heappush(
                pending, (next_cost + heuristic, next_cost, neighbor)
            )
    if goal_cell not in costs:
        raise ValueError(f"preview route is disconnected: {start} -> {goal}")
    path = [goal_cell]
    while path[-1] != start_cell:
        path.append(previous[path[-1]])
    path.reverse()
    return path


def preview_route_path(
    house_map: HouseMap,
    route: Sequence[tuple[str, float, float]],
) -> list[tuple[int, int]]:
    navigation_mask = preview_navigation_mask(house_map)
    path: list[tuple[int, int]] = []
    for start, goal in zip(route, route[1:]):
        segment = astar_preview_path(
            house_map,
            navigation_mask,
            (start[1], start[2]),
            (goal[1], goal[2]),
        )
        path.extend(segment if not path else segment[1:])
    return path


def png_chunk(kind: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + kind
        + data
        + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    )


def write_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    rows = b"".join(
        b"\x00" + pixels[row * width * 3 : (row + 1) * width * 3]
        for row in range(height)
    )
    payload = (
        b"\x89PNG\r\n\x1a\n"
        + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + png_chunk(b"IDAT", zlib.compress(rows, level=9))
        + png_chunk(b"IEND", b"")
    )
    path.write_bytes(payload)


def write_preview(
    path: Path,
    house_map: HouseMap,
    route: Sequence[tuple[str, float, float]],
) -> None:
    spec = house_map.spec
    pixels = bytearray(spec.width * spec.height * 3)
    for display_row, map_row in enumerate(reversed(range(spec.height))):
        for column in range(spec.width):
            index = house_map.index(column, map_row)
            if house_map.spawn_reachable[index]:
                color = (242, 242, 238)
            elif house_map.raw_free[index]:
                color = (100, 100, 100)
            else:
                color = (22, 22, 22)
            offset = (display_row * spec.width + column) * 3
            pixels[offset : offset + 3] = bytes(color)

    def preview_cell(x: float, y: float) -> tuple[int, int]:
        column, map_row = spec.world_to_cell(x, y)
        return column, spec.height - 1 - map_row

    route_path = preview_route_path(house_map, route)
    display_path = [
        (column, spec.height - 1 - map_row) for column, map_row in route_path
    ]
    for start, end in zip(display_path, display_path[1:]):
        draw_line(pixels, spec.width, spec.height, start, end, (32, 113, 205), 2)
    cells = [preview_cell(x, y) for _, x, y in route]
    for index, cell in enumerate(cells):
        draw_circle(
            pixels,
            spec.width,
            spec.height,
            cell,
            4 if index in {0, len(cells) - 1} else 3,
            (30, 160, 70) if index == 0 else (220, 55, 45),
        )
    write_png(path, spec.width, spec.height, pixels)


def write_outputs(
    output_dir: Path,
    house_map: HouseMap,
    route: Sequence[tuple[str, float, float]],
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = output_dir / "house_map.pgm"
    yaml_path = output_dir / "house_map.yaml"
    preview_path = output_dir / "house_map_preview.png"
    write_pgm(pgm_path, house_map)
    write_yaml(yaml_path, pgm_path.name, house_map.spec)
    write_preview(preview_path, house_map, route)
    return {
        "scene": str(DEFAULT_SCENE),
        "projection_z_m": [PROJECTION_MIN_Z, PROJECTION_MAX_Z],
        "resolution_m": house_map.spec.resolution,
        "map_connectivity_clearance_m": MAP_CONNECTIVITY_CLEARANCE,
        "width": house_map.spec.width,
        "height": house_map.spec.height,
        "obstacle_count": len(house_map.obstacles),
        "box_count": sum(item.kind == "box" for item in house_map.obstacles),
        "cylinder_count": sum(item.kind == "cylinder" for item in house_map.obstacles),
        "raw_free_cells": sum(house_map.raw_free),
        "spawn_reachable_cells": sum(house_map.spawn_reachable),
        "sealed_free_cells": sum(
            raw and not reachable
            for raw, reachable in zip(house_map.raw_free, house_map.spawn_reachable)
        ),
        "route_waypoint_count": len(route),
        "outputs": {
            "pgm": str(pgm_path),
            "yaml": str(yaml_path),
            "preview": str(preview_path),
        },
    }


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--routes-file", type=Path, default=DEFAULT_ROUTES)
    parser.add_argument("--resolution", type=float, default=MAP_RESOLUTION)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    if not 0.01 <= args.resolution <= 0.20:
        raise ValueError("map resolution must be between 0.01 and 0.20 metres")
    scene = args.scene.resolve()
    house_map = build_house_map(scene, args.resolution)
    route = route_points(args.routes_file.resolve())
    report = write_outputs(args.output_dir.resolve(), house_map, route)
    report["scene"] = str(scene)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
