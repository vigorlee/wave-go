#!/usr/bin/python3

from __future__ import annotations

import ast
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SEARCH_SOURCE = ROOT / "scripts/go2w_house_mapless_charger_search.py"
sys.path.insert(0, str(ROOT / "scripts"))

import go2w_house_mapless_charger_search as mapless_search  # noqa: E402
from go2w_house_mapless_charger_search import (  # noqa: E402
    CommandState,
    GENERATOR_ACTION_SOURCE,
    GeneratorError,
    LidarScan,
    MAX_LOCKED_HAZARD_VETO_STREAK,
    MaplessChargerSearch,
    MarkerObservation,
    RobotPose,
    SAFETY_STOP_SOURCE,
    adapt_generator_action_chunk,
    adapt_generator_action_prefix,
    advance_locked_hazard_veto_streak,
    av_pose_action_to_twist,
    command_is_live,
    dynamic_search_prefix_steps,
    evaluate_generator_action_prefix,
    front_hazard_consensus_is_safe,
    front_hazard_initial_candidate_budget,
    generator_candidate_seeds,
    generator_prediction_horizon,
    generator_yaw_matches_locked_arc,
    initial_front_hazard_budget_has_safe_consensus,
    limit_generator_twist,
    locked_search_arc_side_unavailable,
    load_config,
    load_generator_config,
    observation_pose_matches,
    rot6d_to_rotation_matrix,
    score_generator_candidate,
    search_turn_phase_active,
    shield_generator_action,
    update_search_arc_yaw_sign,
    validate_generator_action_payload,
    validate_generator_server_info,
)


def generator_payload() -> dict[str, object]:
    return copy.deepcopy(
        json.loads(
            (ROOT / "config/go2w_house_mapless_search.json").read_text(
                encoding="utf-8"
            )
        )["generator"]
    )


def identity_av_action(
    *, camera_x: float = 0.0, camera_y: float = 0.0, camera_z: float = 0.0
) -> list[float]:
    # Translation in optical axes followed by the first two rotation columns.
    return [
        camera_x,
        camera_y,
        camera_z,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]


def scan(
    *,
    front: float = 2.0,
    left: float = 2.0,
    right: float = 2.0,
    received_at: float = 10.0,
) -> LidarScan:
    return LidarScan((), (), front, left, right, received_at)


def pose(
    *,
    x: float = 0.0,
    y: float = 0.0,
    yaw: float = 0.0,
    received_at: float = 10.0,
) -> RobotPose:
    return RobotPose(x, y, 0.40, yaw, 0.0, 0.0, received_at)


def marker(*, error: float = 0.0) -> MarkerObservation:
    return MarkerObservation(
        560,
        (0.5 + error) * 640.0,
        240.0,
        640,
        480,
        0.10,
        ((280.0, 200.0), (360.0, 200.0), (360.0, 280.0), (280.0, 280.0)),
    )


class GeneratorConfigurationContractTest(unittest.TestCase):
    def test_generator_config_accepts_only_the_required_local_av_contract(self) -> None:
        config = load_generator_config(generator_payload())
        self.assertTrue(config.required)
        self.assertEqual(config.server_url, "http://127.0.0.1:8098")
        self.assertEqual(config.domain_name, "av")
        self.assertEqual(config.adapter, "experimental_av_relative_pose")
        self.assertEqual(config.raw_action_dim, 9)
        self.assertEqual(config.candidate_seeds, tuple(range(8)))
        self.assertEqual(config.rotation_scale, 3.00)
        self.assertEqual(config.lateral_yaw_gain, 0.48)
        self.assertEqual(config.max_yaw_rate_rps, 1.00)
        self.assertEqual(config.max_yaw_step_rps, 1.00)
        self.assertEqual(config.execute_prefix_steps, config.action_chunk_size)
        self.assertEqual(config.max_linear_speed_mps, 0.9)
        self.assertLessEqual(config.execute_prefix_steps, config.action_chunk_size)

    def test_generator_config_rejects_missing_and_extra_fields(self) -> None:
        missing = generator_payload()
        missing.pop("fps")
        with self.assertRaisesRegex(ValueError, "missing=.*fps"):
            load_generator_config(missing)

        extra = generator_payload()
        extra["planner_fallback"] = True
        with self.assertRaisesRegex(ValueError, "extra=.*planner_fallback"):
            load_generator_config(extra)

    def test_generator_config_forbids_fallback_remote_urls_and_other_domains(self) -> None:
        invalid_cases = (
            ("required", False, "required must be true"),
            ("server_url", "https://127.0.0.1:8098", "local http"),
            ("server_url", "http://192.168.1.2:8098", "local http"),
            ("server_url", "http://127.0.0.1:8098/predict", "local http"),
            ("domain_name", "go2w", "must be av"),
            ("adapter", "go2w_twist", "unsupported Generator action adapter"),
            ("raw_action_dim", 8, "raw_action_dim"),
        )
        for field, value, error in invalid_cases:
            with self.subTest(field=field, value=value):
                payload = generator_payload()
                payload[field] = value
                with self.assertRaisesRegex(ValueError, error):
                    load_generator_config(payload)

    def test_generator_config_rejects_an_execution_prefix_longer_than_chunk(self) -> None:
        payload = generator_payload()
        payload["action_chunk_size"] = 1
        payload["execute_prefix_steps"] = 2
        with self.assertRaisesRegex(ValueError, "exceeds action_chunk_size"):
            load_generator_config(payload)

    def test_generator_config_requires_unique_bounded_candidate_seeds(self) -> None:
        for seeds in ([], [0], [0, 0], [0, -1], [0, True]):
            with self.subTest(seeds=seeds):
                payload = generator_payload()
                payload["candidate_seeds"] = seeds
                with self.assertRaises(ValueError):
                    load_generator_config(payload)

    def test_generator_server_info_must_match_the_runtime_contract(self) -> None:
        config = load_config().generator
        info = {
            "raw_action_dim": config.raw_action_dim,
            "fps": config.fps,
            "action_chunk_size": config.action_chunk_size,
            "reasoner": True,
            "request_seed_supported": True,
            "checkpoint": "/models/Cosmos3-Edge",
        }
        self.assertIs(validate_generator_server_info(info, config), info)

        for field, value in (
            ("raw_action_dim", 7),
            ("fps", config.fps + 1),
            ("action_chunk_size", config.action_chunk_size + 1),
            ("reasoner", False),
            ("request_seed_supported", False),
            ("checkpoint", ""),
        ):
            with self.subTest(field=field):
                bad = dict(info)
                bad[field] = value
                with self.assertRaises(GeneratorError):
                    validate_generator_server_info(bad, config)


class GeneratorActionProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config().generator

    def valid_payload(self) -> dict[str, object]:
        row = identity_av_action(camera_z=0.2)
        return {"actions": [[list(row) for _ in range(self.config.action_chunk_size)]]}

    def test_action_payload_is_exactly_h_by_nine_and_finite(self) -> None:
        matrix = validate_generator_action_payload(self.valid_payload(), self.config)
        self.assertEqual(
            matrix.shape,
            (self.config.action_chunk_size, self.config.raw_action_dim),
        )
        self.assertTrue(np.isfinite(matrix).all())

    def test_action_payload_rejects_wrong_chunk_count_height_or_width(self) -> None:
        cases: list[object] = [
            {"actions": []},
            {"actions": [[], []]},
            {"actions": [[[0.0] * 9] * (self.config.action_chunk_size - 1)]},
            {"actions": [[[0.0] * 8] * self.config.action_chunk_size]},
            {"actions": [[[0.0] * 10] * self.config.action_chunk_size]},
        ]
        for payload in cases:
            with self.subTest(payload=payload):
                with self.assertRaises(GeneratorError):
                    validate_generator_action_payload(payload, self.config)

    def test_action_payload_rejects_nan_infinity_booleans_and_server_errors(self) -> None:
        for value in (math.nan, math.inf, -math.inf, True, "0"):
            with self.subTest(value=value):
                payload = self.valid_payload()
                payload["actions"][0][0][0] = value  # type: ignore[index]
                with self.assertRaises(GeneratorError):
                    validate_generator_action_payload(payload, self.config)
        with self.assertRaisesRegex(GeneratorError, "server failed"):
            validate_generator_action_payload(
                {"actions": [], "error": "server failed"}, self.config
            )

    def test_rot6d_is_projected_to_a_proper_rotation(self) -> None:
        identity = rot6d_to_rotation_matrix((1.0, 0.0, 0.0, 0.0, 1.0, 0.0))
        np.testing.assert_allclose(identity, np.eye(3), atol=1e-12)

        projected = rot6d_to_rotation_matrix(
            (1.0, 0.1, -0.1, 0.2, 0.9, 0.05)
        )
        np.testing.assert_allclose(projected.T @ projected, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(float(np.linalg.det(projected)), 1.0, places=12)

    def test_rot6d_rejects_degenerate_or_nonfinite_columns(self) -> None:
        for value in (
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0, 2.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0, math.nan, 0.0),
        ):
            with self.subTest(value=value):
                with self.assertRaises(GeneratorError):
                    rot6d_to_rotation_matrix(value)

    def test_av_optical_forward_maps_to_base_forward(self) -> None:
        linear_x, angular_z = av_pose_action_to_twist(
            identity_av_action(camera_z=1.0), self.config
        )
        self.assertAlmostEqual(
            linear_x, self.config.translation_scale * self.config.fps
        )
        self.assertAlmostEqual(angular_z, 0.0)

    def test_av_optical_right_translation_requests_a_base_right_turn(self) -> None:
        linear_x, angular_z = av_pose_action_to_twist(
            identity_av_action(camera_x=1.0), self.config
        )
        self.assertEqual(linear_x, 0.0)
        self.assertLess(angular_z, 0.0)

    def test_av_optical_down_translation_does_not_create_planar_motion(self) -> None:
        linear_x, angular_z = av_pose_action_to_twist(
            identity_av_action(camera_y=1.0), self.config
        )
        self.assertEqual((linear_x, angular_z), (0.0, 0.0))

    def test_av_optical_y_rotation_maps_to_base_yaw_with_expected_sign(self) -> None:
        angle = 0.20
        rotation = np.asarray(
            (
                (math.cos(angle), 0.0, math.sin(angle)),
                (0.0, 1.0, 0.0),
                (-math.sin(angle), 0.0, math.cos(angle)),
            )
        )
        rot6d = np.concatenate((rotation[:, 0], rotation[:, 1])).tolist()
        linear_x, angular_z = av_pose_action_to_twist(
            [0.0, 0.0, 1.0, *rot6d], self.config
        )
        self.assertGreater(linear_x, 0.0)
        self.assertAlmostEqual(
            angular_z,
            -angle * self.config.rotation_scale * self.config.fps,
            places=10,
        )

    def test_chunk_adapter_uses_the_configured_prefix_median(self) -> None:
        chunk = np.asarray(
            [
                identity_av_action(camera_x=1.0, camera_z=0.1),
                identity_av_action(camera_x=-1.0, camera_z=0.2),
                identity_av_action(camera_x=-2.0, camera_z=0.3),
                *[
                    identity_av_action(camera_x=50.0, camera_z=50.0)
                    for _ in range(self.config.action_chunk_size - 3)
                ],
            ],
            dtype=np.float64,
        )
        linear_x, angular_z = adapt_generator_action_chunk(chunk, self.config)
        self.assertAlmostEqual(
            linear_x,
            0.2 * self.config.translation_scale * self.config.fps,
        )
        self.assertGreater(angular_z, 0.0)

class GeneratorSafetyContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.search_config = load_config()
        self.config = self.search_config.generator
        self.now = 10.0

    def shield(
        self,
        linear_x: float,
        angular_z: float,
        *,
        stage: str = "search",
        lidar: LidarScan | None = None,
        robot_pose: RobotPose | None = None,
    ) -> tuple[float, float, tuple[str, ...]]:
        return shield_generator_action(
            linear_x,
            angular_z,
            stage=stage,
            scan=lidar or scan(),
            pose=robot_pose or pose(),
            now=self.now,
            config=self.search_config,
        )

    def test_limiter_forbids_reverse_and_caps_search_and_approach_speed(self) -> None:
        reverse = limit_generator_twist(-1.0, 0.0, (0.0, 0.0), "search", self.config)
        self.assertEqual(reverse[0], 0.0)

        search = limit_generator_twist(
            5.0,
            5.0,
            (self.config.max_linear_speed_mps, self.config.max_yaw_rate_rps),
            "search",
            self.config,
        )
        self.assertEqual(search[0], self.config.max_linear_speed_mps)
        self.assertEqual(search[1], self.config.max_yaw_rate_rps)

        approach = limit_generator_twist(
            5.0,
            0.0,
            (self.config.max_linear_speed_mps, 0.0),
            "approach",
            self.config,
        )
        self.assertEqual(approach[0], self.config.max_approach_speed_mps)

    def test_search_prefix_horizon_uses_only_4_8_or_16_steps(self) -> None:
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=3.0, left=2.0, right=2.0),
                self.search_config,
            ),
            self.config.execute_prefix_steps,
        )
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=3.5, left=1.0, right=1.0),
                self.search_config,
            ),
            8,
        )
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=0.9, left=0.75, right=1.5),
                self.search_config,
            ),
            4,
        )
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(
                    front=self.search_config.search_turn_arc_trigger + 0.10,
                    left=2.0,
                    right=2.0,
                ),
                self.search_config,
                search_turn_only=True,
            ),
            4,
        )

    def test_generator_prediction_drives_4_8_or_16_horizon(self) -> None:
        straight = np.asarray(
            [
                identity_av_action(camera_z=0.10)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        moderate_turn = np.asarray(
            [
                identity_av_action(camera_x=0.005, camera_z=0.10)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        sharp_turn = np.asarray(
            [
                identity_av_action(camera_x=0.10, camera_z=0.10)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        self.assertEqual(generator_prediction_horizon(straight, self.config), 16)
        self.assertEqual(
            generator_prediction_horizon(moderate_turn, self.config), 8
        )
        self.assertEqual(generator_prediction_horizon(sharp_turn, self.config), 4)
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=3.5, left=1.0, right=1.0),
                self.search_config,
                straight,
            ),
            8,
        )

    def test_search_scales_seed_budget_with_hazard_level(self) -> None:
        self.assertEqual(
            [
                generator_candidate_seeds(self.config, "search", request_id)
                for request_id in range(1, 6)
            ],
            [(0,), (1,), (2,), (3,), (4,)],
        )
        self.assertEqual(
            generator_candidate_seeds(
                self.config,
                "search",
                1,
                search_hazard=True,
            ),
            (0, 1),
        )
        self.assertEqual(
            generator_candidate_seeds(
                self.config,
                "search",
                2,
                search_hazard=True,
            ),
            (1, 2),
        )
        self.assertEqual(
            generator_candidate_seeds(
                self.config,
                "search",
                1,
                search_hazard=True,
                front_hazard=True,
            ),
            self.config.candidate_seeds,
        )
        self.assertEqual(
            generator_candidate_seeds(
                self.config,
                "search",
                2,
                search_hazard=True,
                front_hazard=True,
            ),
            (1, 2, 3, 4, 5, 6, 7, 0),
        )
        self.assertEqual(
            generator_candidate_seeds(
                self.config,
                "search",
                2,
                search_hazard=True,
                front_hazard=True,
                preferred_front_hazard_seed=5,
            ),
            (5, 1, 2, 3, 4, 6, 7, 0),
        )
        self.assertEqual(
            generator_candidate_seeds(self.config, "approach", 1),
            self.config.candidate_seeds[:2],
        )

    def test_front_hazard_uses_all_seeds_until_generator_direction_is_locked(
        self,
    ) -> None:
        candidate_count = len(self.config.candidate_seeds)
        self.assertIsNone(
            front_hazard_initial_candidate_budget(
                False, None, candidate_count
            )
        )
        self.assertEqual(
            front_hazard_initial_candidate_budget(
                True, None, candidate_count
            ),
            candidate_count,
        )
        self.assertEqual(
            front_hazard_initial_candidate_budget(
                True, -1.0, candidate_count
            ),
            2,
        )

    def test_locked_hazard_veto_streak_fails_closed_without_unlocking(self) -> None:
        streak = 0
        for expected in range(1, MAX_LOCKED_HAZARD_VETO_STREAK + 1):
            streak = advance_locked_hazard_veto_streak(
                streak,
                front_hazard=True,
                locked_yaw_sign=1.0,
                vetoed=True,
            )
            self.assertEqual(streak, expected)
        self.assertEqual(
            advance_locked_hazard_veto_streak(
                streak,
                front_hazard=False,
                locked_yaw_sign=1.0,
                vetoed=True,
            ),
            0,
        )
        self.assertEqual(
            advance_locked_hazard_veto_streak(
                streak,
                front_hazard=True,
                locked_yaw_sign=None,
                vetoed=True,
            ),
            0,
        )
        self.assertEqual(
            advance_locked_hazard_veto_streak(
                streak,
                front_hazard=True,
                locked_yaw_sign=1.0,
                vetoed=False,
            ),
            0,
        )

    def test_hazard_adapter_uses_three_frame_consensus_without_replaying_noise(
        self,
    ) -> None:
        chunk = np.asarray(
            [
                identity_av_action(camera_x=0.02, camera_z=0.02),
                identity_av_action(camera_x=0.02, camera_z=0.08),
                identity_av_action(camera_x=0.02, camera_z=0.08),
                *[
                    identity_av_action(camera_x=0.02, camera_z=0.08)
                    for _ in range(self.config.action_chunk_size - 3)
                ],
            ],
            dtype=np.float64,
        )
        aggregate_linear, aggregate_yaw = adapt_generator_action_chunk(
            chunk, self.config
        )
        aggregate = self.shield(
            aggregate_linear,
            aggregate_yaw,
            lidar=scan(front=1.80, left=0.75, right=1.50),
        )
        self.assertEqual(aggregate[0], 0.0)
        self.assertLess(aggregate[1], 0.0)

        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=1.80, left=0.75, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.adapter_mode, "hazard_consensus")
        self.assertEqual(evaluation.adapter_output_dim, 2)
        self.assertEqual(evaluation.adapter_support_steps, 3)
        self.assertEqual(evaluation.safe_prefix_steps, 4)
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertEqual(
            len(set(evaluation.nominal_commands)),
            1,
        )
        self.assertTrue(
            all(command[0] == 0.0 for command in evaluation.shielded_commands)
        )
        self.assertTrue(
            all(command[1] < 0.0 for command in evaluation.shielded_commands)
        )

    def test_prefix_adapter_contract_is_h_by_two(self) -> None:
        chunk = np.asarray(
            [
                identity_av_action(camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        for horizon in (4, 8, 16):
            with self.subTest(horizon=horizon):
                adaptation = adapt_generator_action_prefix(
                    chunk,
                    self.config,
                    horizon=horizon,
                )
                self.assertEqual(adaptation.horizon, horizon)
                self.assertEqual(adaptation.output_dim, 2)
                self.assertEqual(adaptation.mode, "framewise")
                self.assertEqual(len(adaptation.commands), horizon)
                self.assertTrue(
                    all(len(command) == 2 for command in adaptation.commands)
                )

    def test_prefix_evaluation_accepts_four_real_safe_arc_frames(self) -> None:
        chunk = np.asarray(
            [
                identity_av_action(camera_x=0.02, camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=1.80, left=0.75, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.safe_prefix_steps, 4)
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertTrue(math.isfinite(evaluation.score))
        self.assertTrue(
            all(
                "search_turn_yaw_only" in reasons
                for reasons in evaluation.shield_reasons
            )
        )

    def test_hazard_consensus_keeps_generator_direction_when_side_is_clear(
        self,
    ) -> None:
        chunk = np.asarray(
            [
                identity_av_action(camera_x=0.02, camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=1.80, left=1.50, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.adapter_mode, "hazard_consensus")
        self.assertEqual(evaluation.adapter_support_steps, 4)
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertTrue(
            all(
                angular_z < 0.0
                for _linear_x, angular_z in evaluation.shielded_commands
            )
        )

    def test_hazard_consensus_accepts_observed_low_rate_generator_arc(
        self,
    ) -> None:
        chunk = np.asarray(
            [
                *[
                    identity_av_action(camera_x=0.009, camera_z=0.08)
                    for _ in range(3)
                ],
                identity_av_action(camera_x=-0.02, camera_z=0.08),
                *[
                    identity_av_action(camera_z=0.08)
                    for _ in range(self.config.action_chunk_size - 4)
                ],
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=1.80, left=0.75, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.adapter_mode, "hazard_consensus")
        self.assertEqual(evaluation.adapter_support_steps, 3)
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertTrue(
            all(
                angular_z <= -self.search_config.search_turn_arc_min_yaw_rate
                for _linear_x, angular_z in evaluation.shielded_commands
            )
        )

    def test_prefix_evaluation_rejects_a_yaw_direction_flip(self) -> None:
        chunk = np.asarray(
            [
                *[
                    identity_av_action(camera_x=0.02, camera_z=0.08)
                    for _ in range(2)
                ],
                *[
                    identity_av_action(camera_x=-0.02, camera_z=0.08)
                    for _ in range(self.config.action_chunk_size - 2)
                ],
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=1.80, left=1.50, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.safe_prefix_steps, 2)
        self.assertEqual(evaluation.execution_steps, 0)
        self.assertEqual(evaluation.adapter_mode, "framewise")
        self.assertEqual(evaluation.adapter_support_steps, 2)
        self.assertEqual(
            evaluation.rejection_reason,
            "candidate_yaw_direction_flip",
        )

    def test_open_search_scores_framewise_yaw_noise_without_rejecting_it(
        self,
    ) -> None:
        chunk = np.asarray(
            [
                *[
                    identity_av_action(camera_x=0.02, camera_z=0.08)
                    for _ in range(2)
                ],
                *[
                    identity_av_action(camera_x=-0.02, camera_z=0.08)
                    for _ in range(self.config.action_chunk_size - 2)
                ],
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=3.0, left=1.5, right=1.0),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.safe_prefix_steps, 4)
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertIsNone(evaluation.rejection_reason)

    def test_prefix_evaluation_shortens_only_to_a_supported_horizon(self) -> None:
        chunk = np.asarray(
            [
                *[
                    identity_av_action(camera_z=0.08)
                    for _ in range(5)
                ],
                *[
                    identity_av_action(camera_z=-0.08)
                    for _ in range(self.config.action_chunk_size - 5)
                ],
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=8,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=3.0, left=2.0, right=2.0),
            pose=pose(),
            now=self.now,
            config=self.search_config,
        )
        self.assertEqual(evaluation.safe_prefix_steps, 5)
        self.assertEqual(evaluation.execution_steps, 4)

    def test_limiter_applies_steps_and_damps_a_yaw_direction_flip(self) -> None:
        linear_x, angular_z = limit_generator_twist(
            1.0, 1.0, (0.0, 0.0), "search", self.config
        )
        self.assertEqual(linear_x, self.config.max_linear_step_mps)
        self.assertEqual(angular_z, self.config.max_yaw_step_rps)

        _linear_x, flipped_yaw = limit_generator_twist(
            0.0, -1.0, (0.0, 0.20), "search", self.config
        )
        self.assertAlmostEqual(flipped_yaw, -self.config.max_yaw_step_rps)

    def test_front_clearance_vetoes_or_reduces_generator_translation(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.20, 0.10, lidar=scan(front=0.50)
        )
        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.0)
        self.assertIn("front_clearance_veto", reasons)
        self.assertIn("near_obstacle_yaw_veto", reasons)

        reduction_config = replace(
            self.search_config,
            search_turn_arc_trigger=0.90,
            search_turn_arc_min_front=0.50,
        )
        linear_x, _angular_z, reasons = shield_generator_action(
            0.50,
            0.0,
            stage="search",
            scan=scan(front=0.95),
            pose=pose(),
            now=self.now,
            config=reduction_config,
        )
        self.assertEqual(linear_x, self.search_config.cautious_speed)
        self.assertIn("front_clearance_reduction", reasons)

    def test_search_executes_generator_yaw_when_commanded_side_is_clear(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.25,
            -0.20,
            lidar=scan(front=1.80, left=0.75, right=1.50),
        )
        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, -0.20)
        self.assertIn("search_turn_yaw_only", reasons)

    def test_search_arc_does_not_replace_generator_direction(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.25,
            0.20,
            lidar=scan(front=1.80, left=1.50, right=1.50),
        )
        self.assertEqual(linear_x, 0.0)
        self.assertEqual(angular_z, 0.20)
        self.assertIn("search_turn_yaw_only", reasons)

    def test_search_arc_never_creates_translation_or_turns_into_low_clearance(
        self,
    ) -> None:
        unsafe_cases = (
            (0.0, -0.20, scan(front=1.80, left=0.75, right=1.50)),
            (0.10, -0.20, scan(front=1.80, left=0.75, right=1.50)),
            (0.25, 0.20, scan(front=1.80, left=1.19, right=1.50)),
            (0.25, -0.10, scan(front=1.80, left=0.75, right=1.50)),
            (0.25, -0.20, scan(front=1.80, left=1.50, right=1.19)),
        )
        for linear_x, angular_z, lidar in unsafe_cases:
            with self.subTest(
                linear_x=linear_x,
                angular_z=angular_z,
                lidar=lidar,
            ):
                shielded_linear, shielded_yaw, reasons = self.shield(
                    linear_x,
                    angular_z,
                    lidar=lidar,
                )
                self.assertEqual((shielded_linear, shielded_yaw), (0.0, 0.0))
                self.assertTrue(
                    "search_turn_arc_translation_veto" in reasons
                    or "search_turn_arc_yaw_veto" in reasons
                )

    def test_search_arc_reserves_command_ttl_front_clearance(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.25,
            0.20,
            lidar=scan(front=1.57, left=1.50, right=1.50),
        )
        self.assertEqual((linear_x, angular_z), (0.0, 0.0))
        self.assertIn("front_clearance_veto", reasons)
        self.assertIn("near_obstacle_yaw_veto", reasons)

        linear_x, angular_z, reasons = self.shield(
            0.25,
            0.20,
            lidar=scan(front=1.59, left=1.50, right=1.50),
        )
        self.assertEqual(
            (linear_x, angular_z),
            (0.0, 0.20),
        )
        self.assertIn("search_turn_yaw_only", reasons)

    def test_front_hazard_arc_lock_is_established_only_by_generator(
        self,
    ) -> None:
        self.assertTrue(generator_yaw_matches_locked_arc(0.20, None))
        self.assertTrue(generator_yaw_matches_locked_arc(0.20, 1.0))
        self.assertTrue(generator_yaw_matches_locked_arc(-0.20, -1.0))
        self.assertFalse(generator_yaw_matches_locked_arc(-0.20, 1.0))
        self.assertFalse(generator_yaw_matches_locked_arc(0.20, -1.0))

        left_open = scan(front=1.80, left=2.0, right=0.8)
        right_open = scan(front=1.80, left=0.8, right=2.0)
        self.assertIsNone(
            update_search_arc_yaw_sign(
                left_open, None, self.search_config
            )
        )
        self.assertIsNone(
            update_search_arc_yaw_sign(
                right_open, None, self.search_config
            )
        )
        self.assertEqual(
            update_search_arc_yaw_sign(
                right_open,
                None,
                self.search_config,
                generator_angular_z=0.20,
            ),
            1.0,
        )
        self.assertEqual(
            update_search_arc_yaw_sign(
                left_open,
                None,
                self.search_config,
                generator_angular_z=-0.20,
            ),
            -1.0,
        )
        self.assertEqual(
            update_search_arc_yaw_sign(
                scan(front=1.80, left=1.00, right=1.01),
                1.0,
                self.search_config,
                generator_angular_z=-0.20,
            ),
            1.0,
        )
        self.assertEqual(
            update_search_arc_yaw_sign(
                scan(front=1.80, left=0.50, right=1.50),
                1.0,
                self.search_config,
            ),
            1.0,
        )
        self.assertIsNone(
            update_search_arc_yaw_sign(
                scan(
                    front=self.search_config.search_turn_arc_trigger + 0.25,
                    left=1.0,
                    right=1.0,
                ),
                1.0,
                self.search_config,
            )
        )

    def test_locked_arc_is_released_only_for_a_safe_generator_reselection(
        self,
    ) -> None:
        self.assertTrue(
            locked_search_arc_side_unavailable(
                scan(front=3.02, left=1.19, right=1.30),
                1.0,
                self.search_config,
            )
        )
        self.assertTrue(
            locked_search_arc_side_unavailable(
                scan(front=3.02, left=1.30, right=1.19),
                -1.0,
                self.search_config,
            )
        )
        self.assertFalse(
            locked_search_arc_side_unavailable(
                scan(front=3.02, left=1.19, right=1.19),
                1.0,
                self.search_config,
            )
        )
        self.assertFalse(
            locked_search_arc_side_unavailable(
                scan(front=3.02, left=1.30, right=1.30),
                1.0,
                self.search_config,
            )
        )
        self.assertFalse(
            locked_search_arc_side_unavailable(
                scan(front=3.02, left=1.19, right=1.30),
                None,
                self.search_config,
            )
        )

        node = object.__new__(MaplessChargerSearch)
        node.search_arc_yaw_sign = 1.0
        node.last_successful_front_hazard_seed = 5
        node.locked_hazard_veto_streak = MAX_LOCKED_HAZARD_VETO_STREAK
        node.config = self.search_config
        events: list[tuple[str, dict[str, object]]] = []
        node.publish_status = lambda state, **details: events.append(
            (state, details)
        )
        self.assertTrue(
            node.release_unavailable_search_arc_lock(
                scan(front=3.02, left=1.19, right=1.30),
                request_id=24,
                reason="test_reselection",
            )
        )
        self.assertIsNone(node.search_arc_yaw_sign)
        self.assertIsNone(node.last_successful_front_hazard_seed)
        self.assertEqual(node.locked_hazard_veto_streak, 0)
        self.assertEqual(events[0][0], "search_arc_direction_released")
        self.assertEqual(events[0][1]["request_id"], 24)

    def test_search_turn_phase_holds_yaw_only_until_hysteresis_release(
        self,
    ) -> None:
        trigger = self.search_config.search_turn_arc_trigger
        self.assertTrue(
            search_turn_phase_active(
                scan(front=trigger - 0.01), None, self.search_config
            )
        )
        self.assertTrue(
            search_turn_phase_active(
                scan(front=trigger + 0.10), 1.0, self.search_config
            )
        )
        self.assertFalse(
            search_turn_phase_active(
                scan(front=trigger + 0.10), None, self.search_config
            )
        )
        self.assertFalse(
            search_turn_phase_active(
                scan(front=trigger + 0.25), 1.0, self.search_config
            )
        )

        chunk = np.asarray(
            [
                identity_av_action(camera_x=-0.02, camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        evaluation = evaluate_generator_action_prefix(
            chunk,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=scan(front=trigger + 0.10, left=1.50, right=1.50),
            pose=pose(),
            now=self.now,
            config=self.search_config,
            preferred_yaw_sign=1.0,
            search_turn_only=True,
        )
        self.assertEqual(evaluation.adapter_mode, "hazard_consensus")
        self.assertEqual(evaluation.execution_steps, 4)
        self.assertTrue(
            all(
                linear_x == 0.0 and angular_z > 0.0
                for linear_x, angular_z in evaluation.shielded_commands
            )
        )
        self.assertTrue(
            all(
                "search_turn_yaw_only" in reasons
                for reasons in evaluation.shield_reasons
            )
        )

    def test_initial_front_hazard_seed_budget_requires_generator_consensus(
        self,
    ) -> None:
        positive_arc = np.asarray(
            [
                identity_av_action(camera_x=-0.02, camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        negative_arc = np.asarray(
            [
                identity_av_action(camera_x=0.02, camera_z=0.08)
                for _ in range(self.config.action_chunk_size)
            ],
            dtype=np.float64,
        )
        blocked_lidar = scan(front=1.80, left=1.50, right=0.75)
        self.assertFalse(
            initial_front_hazard_budget_has_safe_consensus(
                [(1, negative_arc), (6, negative_arc)],
                scan=blocked_lidar,
                pose=pose(),
                now=self.now,
                config=self.search_config,
                required_yaw_sign=None,
            )
        )

        lidar = scan(front=1.80, left=1.50, right=1.50)
        positive_evaluation = evaluate_generator_action_prefix(
            positive_arc,
            predicted_horizon=4,
            previous=(0.0, 0.0),
            stage="search",
            scan=lidar,
            pose=pose(),
            now=self.now,
            config=self.search_config,
            preferred_yaw_sign=1.0,
        )
        self.assertTrue(
            front_hazard_consensus_is_safe(positive_evaluation, 1.0)
        )
        self.assertFalse(
            front_hazard_consensus_is_safe(positive_evaluation, -1.0)
        )
        self.assertTrue(
            initial_front_hazard_budget_has_safe_consensus(
                [(5, positive_arc), (1, negative_arc)],
                scan=lidar,
                pose=pose(),
                now=self.now,
                config=self.search_config,
                required_yaw_sign=1.0,
            )
        )
        self.assertTrue(
            initial_front_hazard_budget_has_safe_consensus(
                [(1, negative_arc), (6, negative_arc)],
                scan=lidar,
                pose=pose(),
                now=self.now,
                config=self.search_config,
                required_yaw_sign=None,
            )
        )
        self.assertFalse(
            initial_front_hazard_budget_has_safe_consensus(
                [(1, negative_arc), (6, negative_arc)],
                scan=lidar,
                pose=pose(),
                now=self.now,
                config=self.search_config,
                required_yaw_sign=1.0,
            )
        )
        self.assertFalse(
            initial_front_hazard_budget_has_safe_consensus(
                [(5, positive_arc)],
                scan=lidar,
                pose=pose(),
                now=self.now,
                config=self.search_config,
                required_yaw_sign=1.0,
            )
        )

        invalid_evaluations = (
            replace(positive_evaluation, predicted_horizon=8),
            replace(positive_evaluation, adapter_mode="framewise"),
            replace(positive_evaluation, adapter_support_steps=2),
            replace(positive_evaluation, safe_prefix_steps=3),
            replace(positive_evaluation, execution_steps=0),
            replace(
                positive_evaluation,
                shielded_commands=(
                    (0.01, positive_evaluation.shielded_commands[0][1]),
                    *positive_evaluation.shielded_commands[1:],
                ),
            ),
            replace(
                positive_evaluation,
                shielded_commands=(
                    (0.0, 0.0),
                    *positive_evaluation.shielded_commands[1:],
                ),
            ),
            replace(
                positive_evaluation,
                shielded_commands=(
                    *positive_evaluation.shielded_commands[:3],
                    (
                        0.0,
                        -positive_evaluation.shielded_commands[3][1],
                    ),
                ),
            ),
        )
        for invalid in invalid_evaluations:
            with self.subTest(invalid=invalid):
                self.assertFalse(front_hazard_consensus_is_safe(invalid, None))

    def test_side_clearance_vetoes_turns_toward_the_wall(self) -> None:
        _linear_x, angular_z, reasons = self.shield(
            0.05, 0.20, lidar=scan(front=3.0, left=0.50)
        )
        self.assertEqual(angular_z, 0.0)
        self.assertIn("left_turn_clearance_veto", reasons)

        _linear_x, angular_z, reasons = self.shield(
            0.05, -0.20, lidar=scan(front=3.0, right=0.50)
        )
        self.assertEqual(angular_z, 0.0)
        self.assertIn("right_turn_clearance_veto", reasons)

    def test_side_wall_reduces_forward_speed_without_creating_a_turn(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.50, 0.0, lidar=scan(front=3.0, left=0.65)
        )
        self.assertEqual(linear_x, self.search_config.cautious_speed)
        self.assertEqual(angular_z, 0.0)
        self.assertIn("side_wall_speed_reduction", reasons)

    def test_hold_and_reacquire_never_translate(self) -> None:
        for stage in ("approach_hold", "reacquire"):
            with self.subTest(stage=stage):
                linear_x, angular_z, reasons = self.shield(
                    0.10, 0.10, stage=stage
                )
                self.assertEqual(linear_x, 0.0)
                self.assertEqual(angular_z, 0.10)
                self.assertIn(f"{stage}_translation_veto", reasons)

    def test_approach_vetoes_motion_that_diverges_from_the_marker(self) -> None:
        left_target = marker(error=-0.40)
        linear_x, angular_z, reasons = shield_generator_action(
            0.10,
            -0.20,
            stage="approach",
            scan=scan(),
            pose=pose(),
            now=self.now,
            config=self.search_config,
            marker=left_target,
            marker_depth_m=2.0,
            last_exact_depth_m=2.0,
        )
        self.assertEqual((linear_x, angular_z), (0.0, 0.0))
        self.assertIn("marker_divergence_yaw_veto", reasons)
        self.assertIn("marker_alignment_forward_veto", reasons)

    def test_approach_uses_guarded_arc_when_stationary_yaw_cannot_align(self) -> None:
        right_target = marker(error=0.40)
        linear_x, angular_z, reasons = shield_generator_action(
            0.25,
            -0.20,
            stage="approach",
            scan=scan(front=1.20, right=1.00),
            pose=pose(),
            now=self.now,
            config=self.search_config,
            marker=right_target,
            marker_depth_m=0.70,
            last_exact_depth_m=0.70,
        )
        self.assertEqual(linear_x, self.search_config.alignment_arc_speed)
        self.assertEqual(angular_z, -0.20)
        self.assertIn("marker_alignment_arc_creep", reasons)
        self.assertNotIn("marker_alignment_forward_veto", reasons)

    def test_alignment_arc_requires_depth_turn_rate_and_target_side_clearance(
        self,
    ) -> None:
        right_target = marker(error=0.40)
        unsafe_cases = (
            {
                "marker_depth_m": (
                    self.search_config.depth_maximum_charging_range
                    + self.search_config.alignment_arc_depth_margin
                    - 0.01
                )
            },
            {"angular_z": -0.03},
            {"lidar": scan(front=1.20, right=0.60)},
        )
        for overrides in unsafe_cases:
            with self.subTest(overrides=overrides):
                linear_x, _angular_z, reasons = shield_generator_action(
                    0.25,
                    overrides.get("angular_z", -0.20),
                    stage="approach",
                    scan=overrides.get(
                        "lidar", scan(front=1.20, right=1.00)
                    ),
                    pose=pose(),
                    now=self.now,
                    config=self.search_config,
                    marker=right_target,
                    marker_depth_m=overrides.get("marker_depth_m", 0.70),
                    last_exact_depth_m=0.70,
                )
                self.assertEqual(linear_x, 0.0)
                self.assertIn("marker_alignment_forward_veto", reasons)

    def test_alignment_arc_requires_an_exact_marker(self) -> None:
        tracked_target = replace(marker(error=0.40), exact_id=False)
        linear_x, _angular_z, reasons = shield_generator_action(
            0.25,
            -0.20,
            stage="approach",
            scan=scan(front=1.20, right=1.00),
            pose=pose(),
            now=self.now,
            config=self.search_config,
            marker=tracked_target,
            marker_depth_m=None,
            last_exact_depth_m=1.20,
        )
        self.assertEqual(linear_x, 0.0)
        self.assertIn("marker_alignment_forward_veto", reasons)

    def test_candidate_scoring_selects_only_generator_motion_toward_open_space(self) -> None:
        lidar = scan(front=0.90, left=4.0, right=0.75)
        left_score = score_generator_candidate(
            0.05,
            0.20,
            stage="search",
            scan=lidar,
            config=self.search_config,
        )
        right_score = score_generator_candidate(
            0.05,
            -0.20,
            stage="search",
            scan=lidar,
            config=self.search_config,
        )
        self.assertGreater(left_score, right_score)
        self.assertEqual(
            score_generator_candidate(
                0.0,
                0.0,
                stage="search",
                scan=lidar,
                config=self.search_config,
            ),
            -math.inf,
        )

    def test_side_wall_does_not_make_near_zero_progress_outscore_forward(self) -> None:
        lidar = scan(front=3.30, left=1.40, right=1.03)
        forward_score = score_generator_candidate(
            0.25,
            0.14,
            stage="search",
            scan=lidar,
            config=self.search_config,
        )
        near_zero_score = score_generator_candidate(
            0.02,
            0.20,
            stage="search",
            scan=lidar,
            config=self.search_config,
        )
        self.assertGreater(forward_score, near_zero_score)

    def test_candidate_scoring_prefers_generator_yaw_toward_visual_target(self) -> None:
        left_target = marker(error=-0.20)
        left_score = score_generator_candidate(
            0.0,
            0.20,
            stage="reacquire",
            scan=scan(),
            config=self.search_config,
            marker=left_target,
        )
        right_score = score_generator_candidate(
            0.0,
            -0.20,
            stage="reacquire",
            scan=scan(),
            config=self.search_config,
            marker=left_target,
        )
        self.assertGreater(left_score, right_score)

    def test_stale_scan_or_unsafe_pose_fails_closed(self) -> None:
        stale = scan(received_at=self.now - 1.01)
        self.assertEqual(
            self.shield(0.10, 0.10, lidar=stale)[:2],
            (0.0, 0.0),
        )
        low = RobotPose(0.0, 0.0, 0.20, 0.0, 0.0, 0.0, self.now)
        self.assertEqual(
            self.shield(0.10, 0.10, robot_pose=low)[:2],
            (0.0, 0.0),
        )

    def test_command_ttl_boundary_and_expired_publish_are_fail_closed(self) -> None:
        state = CommandState(
            0.10,
            0.20,
            GENERATOR_ACTION_SOURCE,
            issued_at=9.0,
            valid_until=10.0,
            request_id=7,
        )
        self.assertTrue(command_is_live(state, 10.0))
        self.assertFalse(command_is_live(state, 10.000001))

        class FakeTwist:
            def __init__(self) -> None:
                self.linear = SimpleNamespace(x=None)
                self.angular = SimpleNamespace(z=None)

        class FakePublisher:
            def __init__(self) -> None:
                self.messages: list[FakeTwist] = []

            def publish(self, message: FakeTwist) -> None:
                self.messages.append(message)

        node = object.__new__(MaplessChargerSearch)
        node.data_lock = threading.Lock()
        node.command = state
        node.command_expiry_reported = False
        node.velocity_publisher = FakePublisher()
        warnings: list[str] = []
        node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
        with patch.object(mapless_search, "Twist", FakeTwist), patch.object(
            mapless_search.time, "monotonic", return_value=10.1
        ):
            MaplessChargerSearch.publish_command(node)
        message = node.velocity_publisher.messages[-1]
        self.assertEqual((message.linear.x, message.angular.z), (0.0, 0.0))
        self.assertEqual(len(warnings), 1)

    def test_set_command_rejects_every_non_generator_nonzero_source(self) -> None:
        node = object.__new__(MaplessChargerSearch)
        node.data_lock = threading.Lock()
        node.generator_config = self.config
        node.command_expiry_reported = False
        with self.assertRaisesRegex(RuntimeError, "only Cosmos3 Generator"):
            MaplessChargerSearch.set_command(
                node, 0.10, 0.0, source=SAFETY_STOP_SOURCE
            )
        with patch.object(mapless_search.time, "monotonic", return_value=10.0):
            MaplessChargerSearch.set_command(
                node, 0.10, 0.0, source=GENERATOR_ACTION_SOURCE, request_id=11
            )
        state = node.command_state_snapshot()
        self.assertEqual(state.source, GENERATOR_ACTION_SOURCE)
        self.assertEqual(state.request_id, 11)
        self.assertAlmostEqual(state.valid_until, 10.0 + self.config.command_ttl_sec)

    def test_late_generator_commit_is_atomically_rejected_after_cancel(self) -> None:
        node = object.__new__(MaplessChargerSearch)
        node.data_lock = threading.Lock()
        node.task_state_lock = threading.Lock()
        node.generator_config = self.config
        node.command_expiry_reported = False
        node.task_generation = 4
        node.cancel_requested = False
        now = 10.0
        node.command = CommandState(
            0.0, 0.0, SAFETY_STOP_SOURCE, now, math.inf
        )
        with patch.object(mapless_search.time, "monotonic", return_value=now):
            self.assertTrue(
                MaplessChargerSearch.commit_generator_command_if_current(
                    node,
                    4,
                    0.10,
                    0.20,
                    request_id=9,
                    chunk_step=0,
                )
            )
            with node.task_state_lock:
                node.cancel_requested = True
                node.task_generation += 1
                MaplessChargerSearch.set_command(node, 0.0, 0.0)
            self.assertFalse(
                MaplessChargerSearch.commit_generator_command_if_current(
                    node,
                    4,
                    0.10,
                    0.20,
                    request_id=10,
                    chunk_step=0,
                )
            )
        self.assertEqual(
            MaplessChargerSearch.command_snapshot(node), (0.0, 0.0)
        )

    def test_pose_drift_uses_translation_and_wrapped_yaw_thresholds(self) -> None:
        captured = pose(yaw=math.pi - 0.01)
        within = pose(
            x=self.config.maximum_stationary_drift_m,
            yaw=-math.pi + 0.01,
        )
        self.assertTrue(observation_pose_matches(captured, within, self.config))

        translated = pose(x=self.config.maximum_stationary_drift_m + 0.001)
        self.assertFalse(observation_pose_matches(pose(), translated, self.config))

        rotated = pose(yaw=self.config.maximum_stationary_drift_rad + 0.001)
        self.assertFalse(observation_pose_matches(pose(), rotated, self.config))


class GeneratorRuntimeSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.tree = ast.parse(SEARCH_SOURCE.read_text(encoding="utf-8"))
        cls.search_class = next(
            node
            for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "MaplessChargerSearch"
        )

    def method(self, name: str) -> ast.FunctionDef:
        return next(
            node
            for node in self.search_class.body
            if isinstance(node, ast.FunctionDef) and node.name == name
        )

    @staticmethod
    def called_names(method: ast.FunctionDef) -> set[str]:
        names: set[str] = set()
        for node in ast.walk(method):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Name):
                names.add(node.func.id)
            elif isinstance(node.func, ast.Attribute):
                names.add(node.func.attr)
        return names

    def test_runtime_search_and_approach_do_not_call_legacy_velocity_planners(self) -> None:
        forbidden = {
            "reactive_velocity",
            "select_exploration_heading",
            "damped_approach_velocity",
            "damped_reacquire_velocity",
            "held_marker_velocity",
        }
        for method_name in ("run_search", "track_and_charge"):
            with self.subTest(method=method_name):
                calls = self.called_names(self.method(method_name))
                self.assertFalse(forbidden & calls, forbidden & calls)
                self.assertIn("generate_and_execute", calls)

    def test_runtime_generator_stages_cover_search_approach_hold_and_reacquire(self) -> None:
        stages: set[str] = set()
        for method_name in ("run_search", "track_and_charge"):
            for node in ast.walk(self.method(method_name)):
                if not isinstance(node, ast.Call):
                    continue
                name = (
                    node.func.attr
                    if isinstance(node.func, ast.Attribute)
                    else node.func.id
                    if isinstance(node.func, ast.Name)
                    else ""
                )
                if name == "generate_and_execute" and node.args:
                    stage = node.args[0]
                    if isinstance(stage, ast.Constant) and isinstance(stage.value, str):
                        stages.add(stage.value)
        self.assertEqual(stages, {"search", "approach", "approach_hold", "reacquire"})

    def test_search_reuses_the_last_exact_marker_as_a_small_marker_hint(self) -> None:
        calls = [
            node
            for node in ast.walk(self.method("run_search"))
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "detect_marker"
        ]
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(calls[0].args), 3)
        self.assertIsInstance(calls[0].args[2], ast.IfExp)

    def test_marker_seen_during_inference_or_chunk_execution_is_not_dropped(self) -> None:
        for method_name in ("generate_and_execute", "execute_search_action_prefix"):
            with self.subTest(method=method_name):
                assigned_attributes = {
                    node.targets[0].attr
                    for node in ast.walk(self.method(method_name))
                    if isinstance(node, ast.Assign)
                    and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Attribute)
                }
                self.assertIn("pending_search_marker", assigned_attributes)
        run_attributes = {
            node.attr
            for node in ast.walk(self.method("run_search"))
            if isinstance(node, ast.Attribute)
        }
        self.assertIn("pending_search_marker", run_attributes)

    def test_prefix_stages_are_committed_only_by_the_stepwise_executors(
        self,
    ) -> None:
        generation = self.method("generate_and_execute")
        guarded_commits = []
        for node in ast.walk(generation):
            if not isinstance(node, ast.If):
                continue
            test = node.test
            if (
                isinstance(test, ast.Compare)
                and isinstance(test.left, ast.Name)
                and test.left.id == "stage"
                and len(test.ops) == 1
                and isinstance(test.ops[0], ast.NotIn)
                and isinstance(test.comparators[0], ast.Set)
                and {
                    element.value
                    for element in test.comparators[0].elts
                    if isinstance(element, ast.Constant)
                }
                == {"search", "approach"}
            ):
                guarded_commits.extend(
                    child
                    for statement in node.body
                    for child in ast.walk(statement)
                    if isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Attribute)
                    and child.func.attr == "commit_generator_command_if_current"
                )
        self.assertEqual(len(guarded_commits), 1)
        for method_name in (
            "execute_search_action_prefix",
            "execute_approach_action_prefix",
        ):
            self.assertIn(
                "commit_generator_command_if_current",
                self.called_names(self.method(method_name)),
            )

    def test_approach_prefix_rechecks_marker_depth_and_shield_each_step(self) -> None:
        calls = self.called_names(self.method("execute_approach_action_prefix"))
        self.assertTrue(
            {
                "create_marker_tracker",
                "detect_marker",
                "estimate_marker_depth",
                "shield_generator_action",
                "commit_generator_command_if_current",
                "tracker_hold_active",
                "update_marker_tracker",
            }.issubset(calls)
        )
        self.assertIn(
            "execute_approach_action_prefix",
            self.called_names(self.method("track_and_charge")),
        )


if __name__ == "__main__":
    unittest.main()
