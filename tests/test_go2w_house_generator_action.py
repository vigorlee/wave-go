#!/usr/bin/python3

from __future__ import annotations

import ast
import copy
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
    MaplessChargerSearch,
    MarkerObservation,
    RobotPose,
    SAFETY_STOP_SOURCE,
    adapt_generator_action_chunk,
    av_pose_action_to_twist,
    command_is_live,
    dynamic_search_prefix_steps,
    generator_candidate_seeds,
    limit_generator_twist,
    load_config,
    load_generator_config,
    observation_pose_matches,
    rot6d_to_rotation_matrix,
    score_generator_candidate,
    shield_generator_action,
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
        self.assertEqual(config.candidate_seeds, (0, 2, 3, 5))
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

    def test_search_prefix_horizon_shrinks_as_clearance_decreases(self) -> None:
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=3.0, left=2.0, right=2.0),
                self.search_config,
            ),
            self.config.execute_prefix_steps,
        )
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=1.5, left=1.0, right=1.0),
                self.search_config,
            ),
            12,
        )
        self.assertEqual(
            dynamic_search_prefix_steps(
                scan(front=0.9, left=0.75, right=1.5),
                self.search_config,
            ),
            8,
        )

    def test_search_rotates_one_seed_and_precision_stages_use_two(self) -> None:
        self.assertEqual(
            [
                generator_candidate_seeds(self.config, "search", request_id)
                for request_id in range(1, 6)
            ],
            [(0,), (2,), (3,), (5,), (0,)],
        )
        self.assertEqual(
            generator_candidate_seeds(self.config, "approach", 1),
            self.config.candidate_seeds[:2],
        )

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
        self.assertEqual(angular_z, 0.10)
        self.assertIn("front_clearance_veto", reasons)

        linear_x, _angular_z, reasons = self.shield(
            0.50, 0.0, lidar=scan(front=0.80)
        )
        self.assertEqual(linear_x, self.search_config.cautious_speed)
        self.assertIn("front_clearance_reduction", reasons)

    def test_side_clearance_vetoes_turns_toward_the_wall(self) -> None:
        _linear_x, angular_z, reasons = self.shield(
            0.05, 0.20, lidar=scan(left=0.50)
        )
        self.assertEqual(angular_z, 0.0)
        self.assertIn("left_turn_clearance_veto", reasons)

        _linear_x, angular_z, reasons = self.shield(
            0.05, -0.20, lidar=scan(right=0.50)
        )
        self.assertEqual(angular_z, 0.0)
        self.assertIn("right_turn_clearance_veto", reasons)

    def test_side_wall_reduces_forward_speed_without_creating_a_turn(self) -> None:
        linear_x, angular_z, reasons = self.shield(
            0.50, 0.0, lidar=scan(left=0.65)
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
