#!/usr/bin/python3

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from cosmos_vln_protocol import (  # noqa: E402
    ACTION_COMPLETE,
    ACTION_HOLD,
    ACTION_NAVIGATE,
    extract_route_command_json,
    load_route_catalog,
    prompt_route_catalog,
    validate_required_route,
    validate_route_command,
)


class CosmosVlnProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.routes = load_route_catalog(ROOT_DIR / "config/cosmos_vln_routes.json")

    def test_catalog_contains_validated_upper_landing_route(self) -> None:
        route = self.routes["upper_landing"]
        self.assertEqual(route.profile, "stair_up")
        self.assertAlmostEqual(route.x, 0.2)
        self.assertAlmostEqual(route.y, 4.8)
        self.assertEqual(len(route.stages), 1)
        self.assertEqual(route.stages[0].stage_id, "goal")

    def test_catalog_contains_validated_three_stage_far_route(self) -> None:
        route = self.routes["far_end_via_stairs"]
        self.assertEqual(
            [stage.stage_id for stage in route.stages],
            [
                "climb_to_upper_landing",
                "descend_far_staircase",
                "navigate_to_far_end",
            ],
        )
        self.assertEqual(
            [stage.navigation_mode for stage in route.stages],
            ["up", "down", "avoid"],
        )
        self.assertAlmostEqual(route.stages[0].x, 0.2)
        self.assertAlmostEqual(route.stages[0].y, 4.8)
        self.assertAlmostEqual(route.stages[1].x, 0.2)
        self.assertAlmostEqual(route.stages[1].y, 10.2)
        self.assertAlmostEqual(route.stages[2].x, 0.4)
        self.assertAlmostEqual(route.stages[2].y, 14.4)

    def test_catalog_contains_validated_farthest_obstacle_route(self) -> None:
        route = self.routes["farthest_end_via_stairs_and_obstacles"]
        expected = [
            ("climb_to_upper_landing", "stair_up", "up", 0.2, 4.8, math.pi / 2),
            (
                "descend_far_staircase",
                "stair_down",
                "down",
                0.2,
                10.2,
                math.pi / 2,
            ),
            ("navigate_to_far_end", "generic", "avoid", 0.4, 14.4, math.pi / 2),
            (
                "approach_remote_obstacles",
                "generic",
                "avoid",
                1.2,
                17.2,
                math.pi / 2,
            ),
            (
                "clear_remote_obstacles",
                "generic",
                "avoid",
                1.2,
                19.5,
                math.pi / 2,
            ),
            (
                "traverse_north_corridor",
                "generic",
                "avoid",
                0.4,
                28.8,
                math.pi / 2,
            ),
            ("round_north_corner", "generic", "avoid", 2.5, 32.8, 0.0),
            ("cross_north_gallery", "generic", "avoid", 17.5, 32.5, 0.0),
            (
                "enter_east_corridor",
                "generic",
                "avoid",
                18.0,
                31.0,
                -math.pi / 2,
            ),
            (
                "traverse_east_corridor",
                "generic",
                "avoid",
                18.0,
                26.5,
                -math.pi / 2,
            ),
            (
                "navigate_to_farthest_endpoint",
                "generic",
                "avoid",
                18.0,
                22.8,
                -math.pi / 2,
            ),
        ]
        self.assertEqual(len(route.stages), len(expected))
        for stage, (stage_id, profile, mode, x, y, yaw_rad) in zip(
            route.stages, expected
        ):
            with self.subTest(stage_id=stage_id):
                self.assertEqual(stage.stage_id, stage_id)
                self.assertEqual(stage.profile, profile)
                self.assertEqual(stage.navigation_mode, mode)
                self.assertEqual(stage.frame_id, "map")
                self.assertAlmostEqual(stage.x, x)
                self.assertAlmostEqual(stage.y, y)
                self.assertAlmostEqual(stage.yaw_rad, yaw_rad)

        shorter_route = self.routes["far_end_via_stairs"]
        self.assertEqual(route.stages[:3], shorter_route.stages)

    def test_prompt_catalog_does_not_expose_stage_controls(self) -> None:
        prompt_entries = json.loads(prompt_route_catalog(self.routes.values()))
        self.assertTrue(prompt_entries)
        self.assertIn(
            "farthest_end_via_stairs_and_obstacles",
            {entry["route_id"] for entry in prompt_entries},
        )
        for entry in prompt_entries:
            self.assertEqual(
                set(entry), {"route_id", "description", "task_examples"}
            )

    def test_extended_route_contains_mixed_terrain_and_pedestrian_stages(self) -> None:
        route = self.routes["farthest_end_via_stairs_ramps_cylinders_pedestrians"]
        self.assertEqual(len(route.stages), 18)
        stage_ids = [stage.stage_id for stage in route.stages]
        for required in (
            "slalom_pass_first_pair",
            "ramp_up_entry",
            "ramp_up_crest",
            "ramp_down_entry",
            "ramp_down_exit",
            "cross_dynamic_pedestrian_zone",
        ):
            self.assertIn(required, stage_ids)
        self.assertEqual(route.stages[-1].x, 18.0)
        self.assertEqual(route.stages[-1].y, 22.8)
        self.assertTrue(all(stage.navigation_mode in {"avoid", "up", "down", "slope"} for stage in route.stages))
        self.assertEqual(route.stages[8].profile, "slope_down")
        self.assertTrue(
            all(stage.navigation_mode == "slope" for stage in route.stages[5:9])
        )
        self.assertGreaterEqual(route.stages[8].y, 17.95)
        self.assertAlmostEqual(route.stages[10].y, 20.8)

    def test_duplicate_stage_ids_fail_closed(self) -> None:
        payload = {
            "version": 1,
            "routes": [
                {
                    "route_id": "invalid_route",
                    "description": "Invalid duplicate stage test route.",
                    "task_examples": [],
                    "stages": [
                        {
                            "stage_id": "duplicate",
                            "profile": "generic",
                            "navigation_mode": "avoid",
                            "goal": {
                                "frame_id": "map",
                                "x": 0.0,
                                "y": 0.0,
                                "yaw_rad": 0.0,
                            },
                        },
                        {
                            "stage_id": "duplicate",
                            "profile": "generic",
                            "navigation_mode": "avoid",
                            "goal": {
                                "frame_id": "map",
                                "x": 1.0,
                                "y": 1.0,
                                "yaw_rad": 0.0,
                            },
                        },
                    ],
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            catalog = Path(directory) / "routes.json"
            catalog.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "duplicate stage_id"):
                load_route_catalog(catalog)

    def test_stair_profile_mode_mismatch_fails_closed(self) -> None:
        payload = {
            "version": 1,
            "routes": [
                {
                    "route_id": "invalid_stair_mode",
                    "description": "Invalid stair mode test route.",
                    "task_examples": [],
                    "profile": "stair_down",
                    "navigation_mode": "up",
                    "goal": {
                        "frame_id": "map",
                        "x": 0.0,
                        "y": 1.0,
                        "yaw_rad": 0.0,
                    },
                }
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            catalog = Path(directory) / "routes.json"
            catalog.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, "invalid for profile"):
                load_route_catalog(catalog)

    def test_extract_and_validate_navigation_command(self) -> None:
        raw = """analysis\n{
          "action": "navigate_to_route",
          "route_id": "upper_landing",
          "future_prediction": {
            "expected_observation": "upper landing",
            "hazards": ["stair edges"],
            "progress": "complete the crossing"
          },
          "confidence": 0.82,
          "reason": "The task matches the approved stair route."
        }"""
        command = validate_route_command(
            extract_route_command_json(raw), self.routes.keys()
        )
        self.assertEqual(command["action"], ACTION_NAVIGATE)
        self.assertEqual(command["route_id"], "upper_landing")

    def test_unknown_route_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "not approved"):
            validate_route_command(
                {
                    "action": ACTION_NAVIGATE,
                    "route_id": "invented_route",
                    "future_prediction": {},
                    "confidence": 0.8,
                    "reason": "",
                },
                self.routes.keys(),
            )

    def test_task_specific_required_route_fails_closed_before_navigation(self) -> None:
        wrong_route = {
            "action": ACTION_NAVIGATE,
            "route_id": "upper_landing",
        }
        with self.assertRaisesRegex(
            ValueError, "does not match required navigation route"
        ):
            validate_required_route(wrong_route, "far_end_via_stairs")

        correct_route = {
            "action": ACTION_NAVIGATE,
            "route_id": "far_end_via_stairs",
        }
        validate_required_route(correct_route, "far_end_via_stairs")

        hold = {"action": ACTION_HOLD, "route_id": None}
        with self.assertRaisesRegex(
            ValueError, "does not match required navigation route"
        ):
            validate_required_route(hold, "far_end_via_stairs")

        complete = {"action": ACTION_COMPLETE, "route_id": None}
        with self.assertRaisesRegex(
            ValueError, "does not match required navigation route"
        ):
            validate_required_route(complete, "far_end_via_stairs")

        validate_required_route(wrong_route, "")

    def test_farthest_task_rejects_shorter_allowlisted_route(self) -> None:
        required_route = "farthest_end_via_stairs_and_obstacles"
        shorter_route = {
            "action": ACTION_NAVIGATE,
            "route_id": "far_end_via_stairs",
        }
        with self.assertRaisesRegex(
            ValueError, "does not match required navigation route"
        ):
            validate_required_route(shorter_route, required_route)

        command = validate_route_command(
            {
                "action": ACTION_NAVIGATE,
                "route_id": required_route,
                "future_prediction": {},
                "confidence": 0.9,
                "reason": "The task requests the longest approved route.",
            },
            self.routes.keys(),
        )
        validate_required_route(command, required_route)

    def test_motion_fields_are_forbidden(self) -> None:
        for field, value in (
            ("forward_m", 0.5),
            ("cmd_vel", {"x": 0.5}),
            ("map_target", {"x": 1.0, "y": 2.0}),
            ("waypoints", [[1.0, 2.0]]),
            ("trajectory", [[0.0, 0.0], [1.0, 1.0]]),
        ):
            with self.subTest(field=field), self.assertRaisesRegex(
                ValueError, f"forbidden fields: {field}"
            ):
                validate_route_command(
                    {
                        "action": ACTION_NAVIGATE,
                        "route_id": "upper_landing",
                        "future_prediction": {},
                        "confidence": 0.8,
                        "reason": "",
                        field: value,
                    },
                    self.routes.keys(),
                )

    def test_bridge_has_no_navigation_or_velocity_client(self) -> None:
        source = (ROOT_DIR / "scripts/cosmos_vln_bridge.py").read_text()
        self.assertNotIn("NavigateToPose", source)
        self.assertNotIn("ActionClient", source)
        self.assertNotIn('"/cmd_vel"', source)
        self.assertNotIn('"/navigate_to_pose"', source)

    def test_hold_position_requires_no_route(self) -> None:
        command = validate_route_command(
            {
                "action": ACTION_HOLD,
                "route_id": None,
                "future_prediction": {
                    "expected_observation": "unchanged scene",
                    "hazards": ["blocked route"],
                    "progress": "paused",
                },
                "confidence": 0.7,
                "reason": "The route is blocked.",
            },
            self.routes.keys(),
        )
        self.assertEqual(command["action"], ACTION_HOLD)


if __name__ == "__main__":
    unittest.main()
