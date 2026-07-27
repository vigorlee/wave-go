#!/usr/bin/python3
"""Pure protocol helpers for NWM-Cosmos3Edge high-level route commands."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable


ACTION_NAVIGATE = "navigate_to_route"
ACTION_HOLD = "hold_position"
ACTION_COMPLETE = "mission_complete"
ALLOWED_ACTIONS = {ACTION_NAVIGATE, ACTION_HOLD, ACTION_COMPLETE}
COMMAND_FIELDS = {
    "action",
    "route_id",
    "future_prediction",
    "confidence",
    "reason",
}
FUTURE_FIELDS = {"expected_observation", "hazards", "progress"}
SUPPORTED_PROFILES = {"generic", "stair_up", "stair_down"}
SUPPORTED_NAVIGATION_MODES = {"avoid", "flat", "up", "down"}
PROFILE_NAVIGATION_MODES = {
    "generic": {"avoid", "flat"},
    "stair_up": {"up"},
    "stair_down": {"down"},
}


@dataclass(frozen=True)
class RouteStage:
    stage_id: str
    profile: str
    navigation_mode: str
    frame_id: str
    x: float
    y: float
    yaw_rad: float


@dataclass(frozen=True)
class RouteDefinition:
    route_id: str
    description: str
    task_examples: tuple[str, ...]
    stages: tuple[RouteStage, ...]

    @property
    def profile(self) -> str:
        return self.stages[0].profile

    @property
    def navigation_mode(self) -> str:
        return self.stages[0].navigation_mode

    @property
    def frame_id(self) -> str:
        return self.stages[0].frame_id

    @property
    def x(self) -> float:
        return self.stages[0].x

    @property
    def y(self) -> float:
        return self.stages[0].y

    @property
    def yaw_rad(self) -> float:
        return self.stages[0].yaw_rad

    def prompt_entry(self) -> dict[str, Any]:
        return {
            "route_id": self.route_id,
            "description": self.description,
            "task_examples": list(self.task_examples),
        }


def finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field} must be numeric")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field} must be finite")
    return parsed


def validate_required_route(
    command: dict[str, Any], required_route_id: str
) -> None:
    """Reject a wrong navigation route before a task-specific demo can move."""
    if required_route_id and (
        command.get("action") != ACTION_NAVIGATE
        or command.get("route_id") != required_route_id
    ):
        raise ValueError(
            f"command {command.get('action')!r} / {command.get('route_id')!r} "
            f"does not match required navigation route {required_route_id!r}"
        )


def parse_route_stage(
    entry: dict[str, Any], field: str, default_stage_id: str | None = None
) -> RouteStage:
    stage_id = entry.get("stage_id", default_stage_id)
    if not isinstance(stage_id, str) or not re.fullmatch(r"[a-z0-9_]+", stage_id):
        raise ValueError(f"{field}.stage_id is invalid")

    profile = entry.get("profile")
    if profile not in SUPPORTED_PROFILES:
        raise ValueError(f"{field}.profile is unsupported")
    navigation_mode = entry.get("navigation_mode")
    if navigation_mode not in SUPPORTED_NAVIGATION_MODES:
        raise ValueError(f"{field}.navigation_mode is unsupported")
    if navigation_mode not in PROFILE_NAVIGATION_MODES[profile]:
        raise ValueError(
            f"{field}.navigation_mode is invalid for profile {profile}"
        )

    goal = entry.get("goal")
    if not isinstance(goal, dict):
        raise ValueError(f"{field}.goal must be an object")
    frame_id = goal.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError(f"{field}.goal.frame_id is required")

    return RouteStage(
        stage_id=stage_id,
        profile=profile,
        navigation_mode=navigation_mode,
        frame_id=frame_id.strip(),
        x=finite_number(goal.get("x"), f"{field}.goal.x"),
        y=finite_number(goal.get("y"), f"{field}.goal.y"),
        yaw_rad=finite_number(goal.get("yaw_rad"), f"{field}.goal.yaw_rad"),
    )


def load_route_catalog(path: Path) -> dict[str, RouteDefinition]:
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load route catalog {path}: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("version") != 1:
        raise ValueError("route catalog version must be 1")
    entries = payload.get("routes")
    if not isinstance(entries, list) or not entries:
        raise ValueError("route catalog must contain a non-empty routes list")

    routes: dict[str, RouteDefinition] = {}
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError(f"routes[{index}] must be an object")
        route_id = entry.get("route_id")
        if not isinstance(route_id, str) or not re.fullmatch(r"[a-z0-9_]+", route_id):
            raise ValueError(f"routes[{index}].route_id is invalid")
        if route_id in routes:
            raise ValueError(f"duplicate route_id: {route_id}")
        description = entry.get("description")
        if not isinstance(description, str) or not description.strip():
            raise ValueError(f"routes[{index}].description is required")
        examples = entry.get("task_examples", [])
        if not isinstance(examples, list) or not all(
            isinstance(value, str) and value.strip() for value in examples
        ):
            raise ValueError(f"routes[{index}].task_examples must be strings")

        stage_entries = entry.get("stages")
        if stage_entries is None:
            stages = (parse_route_stage(entry, f"routes[{index}]", "goal"),)
        else:
            if any(field in entry for field in ("profile", "navigation_mode", "goal")):
                raise ValueError(
                    f"routes[{index}] cannot mix stages with a top-level goal"
                )
            if not isinstance(stage_entries, list) or not stage_entries:
                raise ValueError(f"routes[{index}].stages must be a non-empty list")
            parsed_stages: list[RouteStage] = []
            stage_ids: set[str] = set()
            for stage_index, stage_entry in enumerate(stage_entries):
                if not isinstance(stage_entry, dict):
                    raise ValueError(
                        f"routes[{index}].stages[{stage_index}] must be an object"
                    )
                stage = parse_route_stage(
                    stage_entry, f"routes[{index}].stages[{stage_index}]"
                )
                if stage.stage_id in stage_ids:
                    raise ValueError(
                        f"duplicate stage_id in route {route_id}: {stage.stage_id}"
                    )
                stage_ids.add(stage.stage_id)
                parsed_stages.append(stage)
            stages = tuple(parsed_stages)

        routes[route_id] = RouteDefinition(
            route_id=route_id,
            description=description.strip(),
            task_examples=tuple(value.strip() for value in examples),
            stages=stages,
        )
    return routes


def prompt_route_catalog(routes: Iterable[RouteDefinition]) -> str:
    entries = [route.prompt_entry() for route in routes]
    return json.dumps(entries, ensure_ascii=False, indent=2)


def route_selection_guidance(routes: Iterable[RouteDefinition]) -> str:
    """Build scene-neutral guidance for choosing one catalog-owned route."""
    entries = tuple(routes)
    if not entries:
        raise ValueError("route selection guidance requires at least one route")
    guidance = (
        "The selected route must satisfy the entire task, not merely its first "
        "phase. When the task asks for the longest route, a far endpoint, multiple "
        "areas, obstacles, or stairs, choose the approved route whose description "
        "covers all requested phases instead of a shorter partial route."
    )
    if any(len(route.stages) > 1 for route in entries):
        guidance += (
            " Multi-stage route execution is owned by the navigation supervisor; "
            "select only its route_id and do not reproduce its stages."
        )
    return guidance


def extract_route_command_json(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    candidates: list[dict[str, Any]] = []
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, _ = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and "action" in value:
            candidates.append(value)
    if not candidates:
        raise ValueError("reasoner output did not contain a route command JSON object")
    return candidates[-1]


def validate_route_command(
    response: dict[str, Any], allowed_route_ids: Iterable[str]
) -> dict[str, Any]:
    if not isinstance(response, dict):
        raise ValueError("route command must be an object")
    extra_fields = set(response) - COMMAND_FIELDS
    if extra_fields:
        fields = ", ".join(sorted(extra_fields))
        raise ValueError(f"route command contains forbidden fields: {fields}")

    action = response.get("action")
    if action not in ALLOWED_ACTIONS:
        raise ValueError(f"unsupported route command action: {action}")
    route_id = response.get("route_id")
    allowed = set(allowed_route_ids)
    if action == ACTION_NAVIGATE:
        if not isinstance(route_id, str) or route_id not in allowed:
            raise ValueError(f"route_id is not approved: {route_id}")
    elif route_id is not None:
        raise ValueError(f"route_id must be null for action {action}")

    confidence = finite_number(response.get("confidence"), "confidence")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be in [0, 1]")
    reason = response.get("reason", "")
    if not isinstance(reason, str):
        raise ValueError("reason must be a string")

    future = response.get("future_prediction")
    if not isinstance(future, dict):
        raise ValueError("future_prediction must be an object")
    extra_future_fields = set(future) - FUTURE_FIELDS
    if extra_future_fields:
        fields = ", ".join(sorted(extra_future_fields))
        raise ValueError(f"future_prediction contains unsupported fields: {fields}")
    expected = future.get("expected_observation", "")
    progress = future.get("progress", "")
    hazards = future.get("hazards", [])
    if not isinstance(expected, str) or not isinstance(progress, str):
        raise ValueError("future prediction descriptions must be strings")
    if not isinstance(hazards, list) or not all(
        isinstance(hazard, str) for hazard in hazards
    ):
        raise ValueError("future_prediction.hazards must be a list of strings")

    return {
        "action": action,
        "route_id": route_id,
        "future_prediction": {
            "expected_observation": expected,
            "hazards": hazards,
            "progress": progress,
        },
        "confidence": confidence,
        "reason": reason,
    }
