#!/usr/bin/python3
"""Offline safety validation for approved Go2-W HouseWorld routes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Sequence

from go2w_house_map import (
    DEFAULT_SCENE,
    HOUSE_BOUNDS_POLYGON,
    MAP_RESOLUTION,
    SPAWN,
    GridSpec,
    HouseMap,
    build_house_map,
    point_clearance,
    point_in_polygon,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ROUTES_FILE = ROOT_DIR / "config/cosmos_vln_house_routes.json"
DEFAULT_MAP_DIR = ROOT_DIR / ".run/go2w_house/map"
DEFAULT_OUTPUT = ROOT_DIR / ".run/go2w_house/strategy_report.json"
DEFAULT_SAFETY_CLEARANCE = 0.40
ARRIVAL_HEADING_LOOKBACK_M = 0.50
MAX_ARRIVAL_YAW_ERROR_RAD = 0.35


@dataclass(frozen=True)
class HouseWaypoint:
    waypoint_id: str
    frame_id: str
    x: float
    y: float
    yaw_rad: float


@dataclass(frozen=True)
class HouseRoute:
    route_id: str
    description: str
    stages: tuple[HouseWaypoint, ...]


@dataclass(frozen=True)
class ForbiddenGoal:
    goal_id: str
    x: float
    y: float
    reason: str


@dataclass(frozen=True)
class HouseCatalog:
    scene_id: int
    scene_name: str
    source_xml: Path
    bounds_polygon: tuple[tuple[float, float], ...]
    spawn: HouseWaypoint
    forbidden_goals: tuple[ForbiddenGoal, ...]
    routes: tuple[HouseRoute, ...]


@dataclass(frozen=True)
class AStarResult:
    path_length_m: float
    cell_count: int
    min_center_clearance_m: float
    arrival_yaw_rad: float
    arrival_heading_lookback_m: float


def angle_error_rad(actual: float, expected: float) -> float:
    return abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected)))


def finite_float(value: Any, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{label} must be finite")
    return parsed


def parse_waypoint(payload: Any, waypoint_id: str) -> HouseWaypoint:
    if not isinstance(payload, dict):
        raise ValueError(f"{waypoint_id} goal must be an object")
    frame_id = payload.get("frame_id")
    if frame_id != "map":
        raise ValueError(f"{waypoint_id} frame_id must be 'map'")
    return HouseWaypoint(
        waypoint_id=waypoint_id,
        frame_id=frame_id,
        x=finite_float(payload.get("x"), f"{waypoint_id}.x"),
        y=finite_float(payload.get("y"), f"{waypoint_id}.y"),
        yaw_rad=finite_float(payload.get("yaw_rad"), f"{waypoint_id}.yaw_rad"),
    )


def resolve_source_xml(routes_file: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("scene.source_xml must be a non-empty path")
    path = Path(value)
    if not path.is_absolute():
        root_candidate = ROOT_DIR / path
        config_candidate = routes_file.parent / path
        path = root_candidate if root_candidate.exists() else config_candidate
    return path.resolve()


def load_house_catalog(routes_file: Path = DEFAULT_ROUTES_FILE) -> HouseCatalog:
    routes_file = routes_file.resolve()
    payload = json.loads(routes_file.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("version") != 1:
        raise ValueError("house route catalog version must be 1")

    scene = payload.get("scene")
    if not isinstance(scene, dict):
        raise ValueError("scene must be an object")
    if scene.get("id") != 6 or scene.get("name") != "HouseWorld":
        raise ValueError("scene must identify HouseWorld with id 6")
    raw_polygon = scene.get("bounds_polygon")
    if not isinstance(raw_polygon, list) or len(raw_polygon) < 3:
        raise ValueError("scene.bounds_polygon must contain at least three points")
    polygon: list[tuple[float, float]] = []
    for index, point in enumerate(raw_polygon):
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError(f"scene.bounds_polygon[{index}] must be [x, y]")
        polygon.append(
            (
                finite_float(point[0], f"bounds_polygon[{index}].x"),
                finite_float(point[1], f"bounds_polygon[{index}].y"),
            )
        )
    expected = HOUSE_BOUNDS_POLYGON
    if len(polygon) != len(expected) or any(
        abs(actual_x - expected_x) > 1e-6 or abs(actual_y - expected_y) > 1e-6
        for (actual_x, actual_y), (expected_x, expected_y) in zip(polygon, expected)
    ):
        raise ValueError("scene.bounds_polygon does not match the approved HouseWorld shell")

    spawn = parse_waypoint(payload.get("spawn"), "spawn")
    if math.hypot(spawn.x - SPAWN[0], spawn.y - SPAWN[1]) > 1e-6:
        raise ValueError(f"spawn must remain at the approved pose {SPAWN}")

    forbidden_payload = payload.get("forbidden_goals")
    if not isinstance(forbidden_payload, list) or not forbidden_payload:
        raise ValueError("forbidden_goals must be a non-empty list")
    forbidden: list[ForbiddenGoal] = []
    forbidden_ids: set[str] = set()
    for index, item in enumerate(forbidden_payload):
        if not isinstance(item, dict):
            raise ValueError(f"forbidden_goals[{index}] must be an object")
        goal_id = item.get("id")
        if not isinstance(goal_id, str) or not goal_id:
            raise ValueError(f"forbidden_goals[{index}].id must be non-empty")
        if goal_id in forbidden_ids:
            raise ValueError(f"duplicate forbidden goal id {goal_id!r}")
        forbidden_ids.add(goal_id)
        reason = item.get("reason")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"forbidden goal {goal_id!r} requires a reason")
        forbidden.append(
            ForbiddenGoal(
                goal_id=goal_id,
                x=finite_float(item.get("x"), f"forbidden {goal_id}.x"),
                y=finite_float(item.get("y"), f"forbidden {goal_id}.y"),
                reason=reason.strip(),
            )
        )

    routes_payload = payload.get("routes")
    if not isinstance(routes_payload, list) or not routes_payload:
        raise ValueError("routes must be a non-empty list")
    routes: list[HouseRoute] = []
    route_ids: set[str] = set()
    for route_index, route_payload in enumerate(routes_payload):
        if not isinstance(route_payload, dict):
            raise ValueError(f"routes[{route_index}] must be an object")
        route_id = route_payload.get("route_id")
        if not isinstance(route_id, str) or not route_id:
            raise ValueError(f"routes[{route_index}].route_id must be non-empty")
        if route_id in route_ids:
            raise ValueError(f"duplicate route_id {route_id!r}")
        route_ids.add(route_id)
        stages_payload = route_payload.get("stages")
        if not isinstance(stages_payload, list) or not stages_payload:
            raise ValueError(f"route {route_id!r} requires non-empty stages")
        stages: list[HouseWaypoint] = []
        stage_ids: set[str] = set()
        for stage_index, stage_payload in enumerate(stages_payload):
            if not isinstance(stage_payload, dict):
                raise ValueError(f"route {route_id!r} stage {stage_index} must be an object")
            stage_id = stage_payload.get("stage_id")
            if not isinstance(stage_id, str) or not stage_id:
                raise ValueError(f"route {route_id!r} stage_id must be non-empty")
            if stage_id in stage_ids:
                raise ValueError(f"route {route_id!r} has duplicate stage_id {stage_id!r}")
            stage_ids.add(stage_id)
            if stage_payload.get("profile") != "generic":
                raise ValueError(f"house stage {stage_id!r} profile must be 'generic'")
            if stage_payload.get("navigation_mode") != "avoid":
                raise ValueError(
                    f"house stage {stage_id!r} navigation_mode must be 'avoid'"
                )
            stages.append(parse_waypoint(stage_payload.get("goal"), stage_id))
        routes.append(
            HouseRoute(
                route_id=route_id,
                description=str(route_payload.get("description", "")),
                stages=tuple(stages),
            )
        )

    source_xml = resolve_source_xml(routes_file, scene.get("source_xml"))
    if source_xml != DEFAULT_SCENE.resolve():
        raise ValueError(
            f"scene.source_xml must resolve to {DEFAULT_SCENE.resolve()}, got {source_xml}"
        )
    return HouseCatalog(
        scene_id=6,
        scene_name="HouseWorld",
        source_xml=source_xml,
        bounds_polygon=tuple(polygon),
        spawn=spawn,
        forbidden_goals=tuple(forbidden),
        routes=tuple(routes),
    )


def read_pgm(path: Path) -> tuple[int, int, int, bytes]:
    data = path.read_bytes()
    position = 0

    def token() -> bytes:
        nonlocal position
        while position < len(data):
            if data[position : position + 1] == b"#":
                position = data.find(b"\n", position)
                if position < 0:
                    raise ValueError(f"unterminated comment in {path}")
            elif data[position : position + 1].isspace():
                position += 1
            else:
                break
        start = position
        while position < len(data) and not data[position : position + 1].isspace():
            position += 1
        if start == position:
            raise ValueError(f"truncated PGM header in {path}")
        return data[start:position]

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    while position < len(data) and data[position : position + 1].isspace():
        position += 1
    if magic != b"P5" or max_value != 255:
        raise ValueError(f"{path} must be an 8-bit binary PGM")
    return width, height, max_value, data[position:]


def validate_map_artifacts(map_dir: Path, house_map: HouseMap) -> dict[str, Any]:
    spec = house_map.spec
    pgm = map_dir / "house_map.pgm"
    yaml = map_dir / "house_map.yaml"
    preview = map_dir / "house_map_preview.png"
    missing = [str(path) for path in (pgm, yaml, preview) if not path.is_file()]
    if missing:
        raise ValueError(f"missing generated HouseWorld map artifacts: {missing}")
    width, height, _, payload = read_pgm(pgm)
    if (width, height, len(payload)) != (
        spec.width,
        spec.height,
        spec.width * spec.height,
    ):
        raise ValueError(
            "house_map.pgm dimensions do not match the approved geometry grid: "
            f"got {(width, height, len(payload))}"
        )
    expected = bytes(
        254 if house_map.is_navigable_cell(column, row) else 0
        for row in reversed(range(spec.height))
        for column in range(spec.width)
    )
    if payload != expected:
        raise ValueError("house_map.pgm occupancy does not match approved HouseWorld geometry")
    yaml_text = yaml.read_text(encoding="ascii")
    resolution_match = re.search(r"^resolution:\s*([0-9.]+)\s*$", yaml_text, re.M)
    origin_match = re.search(
        r"^origin:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,",
        yaml_text,
        re.M,
    )
    if resolution_match is None or origin_match is None:
        raise ValueError("house_map.yaml is missing resolution or origin")
    resolution = float(resolution_match.group(1))
    origin = (float(origin_match.group(1)), float(origin_match.group(2)))
    if abs(resolution - spec.resolution) > 1e-9 or any(
        abs(actual - expected) > 1e-6
        for actual, expected in zip(origin, (spec.origin_x, spec.origin_y))
    ):
        raise ValueError("house_map.yaml resolution/origin do not match the geometry grid")
    if preview.read_bytes()[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("house_map_preview.png is not a PNG file")
    return {
        "validated": True,
        "pgm": str(pgm),
        "yaml": str(yaml),
        "preview": str(preview),
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": list(origin),
    }


class HouseRoutePlanner:
    def __init__(self, house_map: HouseMap, safety_clearance_m: float) -> None:
        if safety_clearance_m < 0.30:
            raise ValueError("safety clearance below 0.30 m is not approved for Go2-W")
        self.house_map = house_map
        self.safety_clearance_m = safety_clearance_m
        clearances: list[float] = []
        navigation_free: list[bool] = []
        spec = house_map.spec
        for row in range(spec.height):
            for column in range(spec.width):
                index = house_map.index(column, row)
                if not house_map.spawn_reachable[index]:
                    clearances.append(-1.0)
                    navigation_free.append(False)
                    continue
                x, y = spec.cell_center(column, row)
                clearance = point_clearance(x, y, house_map.obstacles)
                clearances.append(clearance)
                navigation_free.append(clearance >= safety_clearance_m)
        self.clearances = tuple(clearances)
        self.navigation_free = tuple(navigation_free)

    @property
    def spec(self) -> GridSpec:
        return self.house_map.spec

    def point_report(self, waypoint: HouseWaypoint) -> dict[str, Any]:
        column, row = self.spec.world_to_cell(waypoint.x, waypoint.y)
        inside_grid = self.house_map.in_grid(column, row)
        inside_boundary = point_in_polygon(waypoint.x, waypoint.y)
        raw_reachable = inside_grid and self.house_map.is_navigable_cell(column, row)
        clearance = point_clearance(
            waypoint.x, waypoint.y, self.house_map.obstacles
        )
        safe = (
            inside_boundary
            and raw_reachable
            and clearance >= self.safety_clearance_m
            and self.navigation_free[self.house_map.index(column, row)]
        )
        return {
            "waypoint_id": waypoint.waypoint_id,
            "x": waypoint.x,
            "y": waypoint.y,
            "yaw_rad": waypoint.yaw_rad,
            "inside_scene_boundary": inside_boundary,
            "spawn_component_reachable": raw_reachable,
            "center_clearance_m": clearance,
            "required_clearance_m": self.safety_clearance_m,
            "validated": safe,
        }

    def _free(self, cell: tuple[int, int]) -> bool:
        column, row = cell
        return self.house_map.in_grid(column, row) and self.navigation_free[
            self.house_map.index(column, row)
        ]

    def _arrival_heading(
        self,
        path: Sequence[tuple[int, int]],
        start: HouseWaypoint,
        goal: HouseWaypoint,
    ) -> tuple[float, float]:
        if len(path) < 2:
            delta_x = goal.x - start.x
            delta_y = goal.y - start.y
            if math.hypot(delta_x, delta_y) <= 1.0e-9:
                return start.yaw_rad, 0.0
            return math.atan2(delta_y, delta_x), math.hypot(delta_x, delta_y)

        points = [self.spec.cell_center(column, row) for column, row in path]
        goal_x, goal_y = points[-1]
        remaining = ARRIVAL_HEADING_LOOKBACK_M
        sampled_x, sampled_y = points[0]
        sampled_distance = 0.0
        for index in range(len(points) - 1, 0, -1):
            current_x, current_y = points[index]
            previous_x, previous_y = points[index - 1]
            segment_length = math.hypot(
                current_x - previous_x, current_y - previous_y
            )
            if remaining <= segment_length:
                fraction = remaining / segment_length
                sampled_x = current_x + fraction * (previous_x - current_x)
                sampled_y = current_y + fraction * (previous_y - current_y)
                sampled_distance += remaining
                break
            remaining -= segment_length
            sampled_distance += segment_length
        return (
            math.atan2(goal_y - sampled_y, goal_x - sampled_x),
            sampled_distance,
        )

    def astar(self, start: HouseWaypoint, goal: HouseWaypoint) -> AStarResult | None:
        start_cell = self.spec.world_to_cell(start.x, start.y)
        goal_cell = self.spec.world_to_cell(goal.x, goal.y)
        if not self._free(start_cell) or not self._free(goal_cell):
            return None
        costs = {start_cell: 0.0}
        previous: dict[tuple[int, int], tuple[int, int]] = {}
        queue: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start_cell)]
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
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost != costs.get(current):
                continue
            if current == goal_cell:
                break
            for delta_column, delta_row, multiplier in moves:
                neighbor = (
                    current[0] + delta_column,
                    current[1] + delta_row,
                )
                if not self._free(neighbor):
                    continue
                if delta_column and delta_row:
                    if not self._free((current[0] + delta_column, current[1])):
                        continue
                    if not self._free((current[0], current[1] + delta_row)):
                        continue
                next_cost = cost + multiplier * self.spec.resolution
                if next_cost >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = next_cost
                previous[neighbor] = current
                heuristic = math.hypot(
                    neighbor[0] - goal_cell[0], neighbor[1] - goal_cell[1]
                ) * self.spec.resolution
                heapq.heappush(queue, (next_cost + heuristic, next_cost, neighbor))
        if goal_cell not in costs:
            return None
        reverse_path = [goal_cell]
        while reverse_path[-1] != start_cell:
            reverse_path.append(previous[reverse_path[-1]])
        path = list(reversed(reverse_path))
        min_clearance = min(
            self.clearances[self.house_map.index(column, row)]
            for column, row in path
        )
        arrival_yaw, arrival_lookback = self._arrival_heading(path, start, goal)
        return AStarResult(
            path_length_m=costs[goal_cell],
            cell_count=len(path),
            min_center_clearance_m=min_clearance,
            arrival_yaw_rad=arrival_yaw,
            arrival_heading_lookback_m=arrival_lookback,
        )


def validate_catalog(
    catalog: HouseCatalog,
    house_map: HouseMap,
    safety_clearance_m: float = DEFAULT_SAFETY_CLEARANCE,
    map_dir: Path | None = None,
) -> dict[str, Any]:
    planner = HouseRoutePlanner(house_map, safety_clearance_m)
    spawn_report = planner.point_report(catalog.spawn)
    route_reports: list[dict[str, Any]] = []
    for route in catalog.routes:
        previous = catalog.spawn
        waypoints: list[dict[str, Any]] = []
        segments: list[dict[str, Any]] = []
        route_valid = spawn_report["validated"]
        for stage in route.stages:
            waypoint_report = planner.point_report(stage)
            waypoints.append(waypoint_report)
            path = planner.astar(previous, stage)
            segment_report: dict[str, Any] = {
                "from": previous.waypoint_id,
                "to": stage.waypoint_id,
                "validated": path is not None,
            }
            if path is not None:
                arrival_yaw_error = angle_error_rad(
                    stage.yaw_rad, path.arrival_yaw_rad
                )
                arrival_yaw_validated = (
                    arrival_yaw_error <= MAX_ARRIVAL_YAW_ERROR_RAD
                )
                segment_report.update(
                    {
                        "path_length_m": path.path_length_m,
                        "path_cell_count": path.cell_count,
                        "min_center_clearance_m": path.min_center_clearance_m,
                        "arrival_yaw_rad": path.arrival_yaw_rad,
                        "configured_yaw_rad": stage.yaw_rad,
                        "arrival_yaw_error_rad": arrival_yaw_error,
                        "arrival_heading_lookback_m": (
                            path.arrival_heading_lookback_m
                        ),
                        "max_arrival_yaw_error_rad": MAX_ARRIVAL_YAW_ERROR_RAD,
                        "arrival_yaw_validated": arrival_yaw_validated,
                        "validated": arrival_yaw_validated,
                    }
                )
            segments.append(segment_report)
            route_valid = (
                route_valid
                and waypoint_report["validated"]
                and segment_report["validated"]
            )
            previous = stage
        route_reports.append(
            {
                "route_id": route.route_id,
                "stage_count": len(route.stages),
                "waypoints": waypoints,
                "segments": segments,
                "total_path_length_m": sum(
                    float(segment.get("path_length_m", 0.0)) for segment in segments
                ),
                "validated": route_valid,
            }
        )

    forbidden_reports: list[dict[str, Any]] = []
    for goal in catalog.forbidden_goals:
        waypoint = HouseWaypoint(goal.goal_id, "map", goal.x, goal.y, 0.0)
        report = planner.point_report(waypoint)
        report.update(
            {
                "reason": goal.reason,
                "disabled": not report["validated"],
            }
        )
        forbidden_reports.append(report)

    artifacts: dict[str, Any] | None = None
    if map_dir is not None:
        artifacts = validate_map_artifacts(map_dir.resolve(), house_map)
    validated = (
        spawn_report["validated"]
        and all(route["validated"] for route in route_reports)
        and all(goal["disabled"] for goal in forbidden_reports)
        and (artifacts is None or artifacts["validated"])
    )
    return {
        "validated": validated,
        "drives_robot": False,
        "scene": {
            "id": catalog.scene_id,
            "name": catalog.scene_name,
            "source_xml": str(catalog.source_xml),
            "bounds_polygon": [list(point) for point in catalog.bounds_polygon],
        },
        "map": {
            "resolution_m": house_map.spec.resolution,
            "width": house_map.spec.width,
            "height": house_map.spec.height,
            "projected_obstacle_count": len(house_map.obstacles),
            "box_count": sum(item.kind == "box" for item in house_map.obstacles),
            "cylinder_count": sum(
                item.kind == "cylinder" for item in house_map.obstacles
            ),
            "spawn_reachable_cells": sum(house_map.spawn_reachable),
            "navigation_free_cells": sum(planner.navigation_free),
            "safety_clearance_m": safety_clearance_m,
        },
        "map_artifacts": artifacts,
        "spawn": spawn_report,
        "routes": route_reports,
        "forbidden_goals": forbidden_reports,
    }


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--routes-file", type=Path, default=DEFAULT_ROUTES_FILE)
    parser.add_argument("--map-dir", type=Path, default=DEFAULT_MAP_DIR)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--safety-clearance",
        type=float,
        default=DEFAULT_SAFETY_CLEARANCE,
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    report: dict[str, Any]
    try:
        catalog = load_house_catalog(args.routes_file)
        house_map = build_house_map(catalog.source_xml, MAP_RESOLUTION)
        report = validate_catalog(
            catalog,
            house_map,
            safety_clearance_m=args.safety_clearance,
            map_dir=args.map_dir,
        )
    except Exception as exc:
        report = {
            "validated": False,
            "drives_robot": False,
            "error": str(exc),
        }
    rendered = json.dumps(report, indent=2, sort_keys=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    if not report.get("validated"):
        print("[HOUSE_STRATEGY_FAIL] offline route validation failed", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
