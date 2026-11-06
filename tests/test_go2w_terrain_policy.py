#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from go2w_terrain_policy import (  # noqa: E402
    TerrainObservation,
    TerrainPolicySelector,
    load_policy_config,
)


class TerrainPolicySelectorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_policy_config()
        self.selector = TerrainPolicySelector(self.config)

    def command(self, **changes):
        observation = {
            "age_s": 0.1,
            "localization_confidence": 0.95,
            "slope_deg": 0.0,
            "stair_probability": 0.0,
            "stair_direction": "none",
            "corridor_clearance_m": 2.0,
            "obstacle_complexity": 0.1,
            "dynamic_time_to_collision_s": 10.0,
        }
        observation.update(changes)
        return self.selector.command(TerrainObservation(**observation), 0.60, 1.20)

    def test_flat_ground_uses_rolling_and_clamps_command(self) -> None:
        command = self.command()
        self.assertEqual(command.profile, "rolling")
        self.assertAlmostEqual(command.forward_speed_mps, 0.45)
        self.assertAlmostEqual(command.yaw_rate_rps, 0.80)

    def test_uphill_and_downhill_have_distinct_profiles(self) -> None:
        self.assertEqual(self.command(slope_deg=12.0).profile, "slope_up")
        self.assertEqual(self.command(slope_deg=-12.0).profile, "slope_down")

    def test_stair_evidence_has_priority_over_slope(self) -> None:
        command = self.command(
            slope_deg=14.0,
            stair_probability=0.85,
            stair_direction="up",
        )
        self.assertEqual(command.profile, "stair_up")
        self.assertEqual(command.locomotion_mode, "step")

    def test_complex_and_narrow_obstacles_select_safe_profiles(self) -> None:
        self.assertEqual(
            self.command(obstacle_complexity=0.9).profile, "complex_obstacle"
        )
        self.assertEqual(
            self.command(corridor_clearance_m=0.62).profile, "narrow_passage"
        )

    def test_dynamic_collision_prediction_stops_and_requests_replan(self) -> None:
        command = self.command(dynamic_time_to_collision_s=0.8)
        self.assertEqual(command.profile, "dynamic_wait")
        self.assertEqual(command.forward_speed_mps, 0.0)
        self.assertEqual(command.yaw_rate_rps, 0.0)
        self.assertTrue(command.replan_required)

    def test_stale_or_low_confidence_input_fails_closed(self) -> None:
        for command in (
            self.command(age_s=0.9),
            self.command(localization_confidence=0.2),
            self.command(stair_probability=0.9, stair_direction="unknown"),
        ):
            with self.subTest(reason=command.reason):
                self.assertEqual(command.profile, "safe_stop")
                self.assertEqual(command.forward_speed_mps, 0.0)

    def test_missing_profile_is_rejected(self) -> None:
        broken = json.loads(json.dumps(self.config))
        del broken["profiles"]["stair_down"]
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "broken.json"
            path.write_text(json.dumps(broken), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing required profiles"):
                load_policy_config(path)


if __name__ == "__main__":
    unittest.main()
