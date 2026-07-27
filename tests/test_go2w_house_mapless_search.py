#!/usr/bin/python3

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import go2w_house_mapless_charger_search as mapless_search  # noqa: E402
from go2w_house_mapless_charger_search import (  # noqa: E402
    CosmosGeneratorClient,
    LidarScan,
    MarkerObservation,
    REASONER_MAX_NEW_TOKENS,
    RobotPose,
    bounded_reacquire_direction,
    candidate_hold_active,
    candidate_stop_velocity,
    charge_ready,
    compute_lidar_scan,
    cosmos_pose_is_stable,
    create_marker_tracker,
    damped_approach_velocity,
    damped_reacquire_velocity,
    depth_guarded_approach_speed,
    decode_depth_frame,
    detect_marker,
    detect_marker_from_hint,
    estimate_marker_depth,
    held_marker_velocity,
    load_config,
    posture_is_upright,
    reactive_velocity,
    select_exploration_heading,
    sensor_is_fresh,
    tracker_hold_active,
    update_marker_tracker,
    validate_cosmos_detection,
)


class MaplessSearchContractTest(unittest.TestCase):
    def test_depth_message_does_not_shadow_pil_image(self) -> None:
        self.assertTrue(callable(mapless_search.Image.open))

    def test_config_contains_no_map_target_or_route(self) -> None:
        payload = json.loads(
            (ROOT / "config/go2w_house_mapless_search.json").read_text(
                encoding="utf-8"
            )
        )
        encoded = json.dumps(payload)
        for forbidden in ('"x"', '"y"', '"yaw_rad"', '"map_target"', '"routes"'):
            self.assertNotIn(forbidden, encoded)
        self.assertEqual(payload["mode"], "mapless_visual_search")

    def test_runtime_source_uses_no_global_navigation_action(self) -> None:
        source = (
            ROOT / "scripts/go2w_house_mapless_charger_search.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("/navigate_to_pose", source)
        self.assertNotIn("ComputePathToPose", source)
        self.assertNotIn("PoseStamped", source)

    def test_validated_charging_marker_is_detected(self) -> None:
        config = load_config()
        marker = np.full((320, 480), 255, dtype=np.uint8)
        rendered = cv2.aruco.drawMarker(
            cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000),
            config.marker_id,
            140,
        )
        marker[90:230, 170:310] = rendered
        ok, encoded = cv2.imencode(".jpg", marker)
        self.assertTrue(ok)
        observation = detect_marker(encoded.tobytes(), config.marker_id)
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertEqual(observation.marker_id, config.marker_id)
        self.assertAlmostEqual(observation.horizontal_error, 0.0, delta=0.03)

    def test_small_floor_marker_is_recovered_from_lower_image_band(self) -> None:
        config = load_config()
        image = np.full((360, 640), 255, dtype=np.uint8)
        rendered = cv2.aruco.drawMarker(
            cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000),
            config.marker_id,
            28,
        )
        image[320:348, 300:328] = rendered
        ok, encoded = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        observation = detect_marker(encoded.tobytes(), config.marker_id)
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertGreater(observation.center_y, 300.0)

    def test_small_floor_marker_survives_double_jpeg_compression(self) -> None:
        config = load_config()
        image = np.full((360, 640, 3), 210, dtype=np.uint8)
        rendered = cv2.aruco.drawMarker(
            cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000),
            config.marker_id,
            14,
        )
        image[244:258, 272:286] = cv2.cvtColor(
            rendered, cv2.COLOR_GRAY2BGR
        )
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88]
        )
        self.assertTrue(ok)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        ok, recompressed = cv2.imencode(
            ".jpg", decoded, [cv2.IMWRITE_JPEG_QUALITY, 88]
        )
        self.assertTrue(ok)
        observation = detect_marker(recompressed.tobytes(), config.marker_id)
        self.assertIsNotNone(observation)

    def test_confirmed_tracker_hint_recovers_perspective_marker_id(self) -> None:
        config = load_config()
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        rendered = cv2.aruco.drawMarker(dictionary, config.marker_id, 240)
        source = np.asarray(
            ((0.0, 0.0), (239.0, 0.0), (239.0, 239.0), (0.0, 239.0)),
            dtype=np.float32,
        )
        destination = np.asarray(
            ((300.0, 170.0), (530.0, 120.0), (575.0, 475.0), (250.0, 430.0)),
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(source, destination)
        warped = cv2.warpPerspective(
            rendered,
            transform,
            (960, 540),
            flags=cv2.INTER_NEAREST,
            borderValue=255,
        )
        mask = cv2.warpPerspective(
            np.full_like(rendered, 255),
            transform,
            (960, 540),
            flags=cv2.INTER_NEAREST,
            borderValue=0,
        )
        image = np.full((540, 960, 3), 235, dtype=np.uint8)
        marker_rgb = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        image[mask > 0] = marker_rgb[mask > 0]
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 88]
        )
        self.assertTrue(ok)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        hint = MarkerObservation(
            config.marker_id,
            412.5,
            297.5,
            960,
            540,
            355.0 / 540.0,
            ((245.0, 110.0), (580.0, 110.0), (580.0, 485.0), (245.0, 485.0)),
            exact_id=False,
        )
        observation = detect_marker_from_hint(
            decoded, config.marker_id, hint, dictionary
        )
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertTrue(observation.exact_id)
        self.assertEqual(observation.marker_id, config.marker_id)
        self.assertAlmostEqual(observation.horizontal_error, -0.07, delta=0.08)

    def make_dark_pedestal_marker(
        self,
        marker_id: int,
        *,
        corrupt_cell: bool = False,
    ) -> tuple[bytes, MarkerObservation]:
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        rendered = cv2.aruco.drawMarker(dictionary, marker_id, 72)
        rendered = np.where(rendered > 0, 175, 2).astype(np.uint8)
        if corrupt_cell:
            rendered[12:24, 12:24] = np.where(
                rendered[12:24, 12:24] > 80,
                2,
                175,
            )
        image = np.full((540, 960, 3), 2, dtype=np.uint8)
        image[346:418, 360:432] = cv2.cvtColor(
            rendered,
            cv2.COLOR_GRAY2BGR,
        )
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, 88],
        )
        self.assertTrue(ok)
        hint = MarkerObservation(
            560,
            396.0,
            382.0,
            960,
            540,
            72.0 / 540.0,
            (
                (360.0, 346.0),
                (432.0, 346.0),
                (432.0, 418.0),
                (360.0, 418.0),
            ),
            exact_id=False,
        )
        return encoded.tobytes(), hint

    def test_confirmed_hint_exactly_decodes_marker_on_dark_pedestal(self) -> None:
        encoded, hint = self.make_dark_pedestal_marker(560)
        self.assertIsNone(detect_marker(encoded, 560))
        observation = detect_marker(encoded, 560, hint=hint)
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertTrue(observation.exact_id)
        self.assertEqual(observation.verification, "target_cell_exact")

    def test_target_cell_decoder_rejects_wrong_id_and_one_bit_error(self) -> None:
        wrong_id, hint = self.make_dark_pedestal_marker(561)
        self.assertIsNone(detect_marker(wrong_id, 560, hint=hint))
        corrupted, hint = self.make_dark_pedestal_marker(
            560,
            corrupt_cell=True,
        )
        self.assertIsNone(detect_marker(corrupted, 560, hint=hint))

    def test_marker_depth_uses_registered_polygon_and_metre_scale(self) -> None:
        config = load_config()
        values = np.full(
            (config.depth_height, config.depth_width), 3.2, dtype=np.float32
        )
        values[95:145, 135:185] = 0.55
        message = type(
            "DepthMessage",
            (),
            {
                "encoding": "32FC1",
                "data": values.tobytes(),
                "width": 0,
                "height": 0,
                "step": 0,
                "is_bigendian": 0,
            },
        )()
        depth = decode_depth_frame(message, 10.0, config)
        marker = MarkerObservation(
            config.marker_id,
            480.0,
            270.0,
            960,
            540,
            120.0 / 540.0,
            ((420.0, 210.0), (540.0, 210.0), (540.0, 330.0), (420.0, 330.0)),
        )
        self.assertAlmostEqual(
            estimate_marker_depth(marker, depth, 10.0, config), 0.55, places=2
        )
        self.assertIsNone(estimate_marker_depth(marker, depth, 11.0, config))

    def test_depth_decoder_rejects_malformed_payload(self) -> None:
        config = load_config()
        message = type(
            "DepthMessage",
            (),
            {
                "encoding": "32FC1",
                "data": b"invalid",
                "width": 0,
                "height": 0,
                "step": 0,
                "is_bigendian": 0,
            },
        )()
        with self.assertRaisesRegex(ValueError, "depth payload"):
            decode_depth_frame(message, 1.0, config)

    def test_verified_marker_tracker_bridges_small_marker_dropout(self) -> None:
        config = load_config()
        rendered = cv2.aruco.drawMarker(
            cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000),
            config.marker_id,
            40,
        )

        def frame(left: int) -> bytes:
            image = np.full((360, 640, 3), 210, dtype=np.uint8)
            image[250:290, left : left + 40] = cv2.cvtColor(
                rendered, cv2.COLOR_GRAY2BGR
            )
            ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90]
            )
            self.assertTrue(ok)
            return encoded.tobytes()

        marker = MarkerObservation(
            config.marker_id,
            320.0,
            270.0,
            640,
            360,
            40.0 / 360.0,
            ((300.0, 250.0), (340.0, 250.0), (340.0, 290.0), (300.0, 290.0)),
        )
        tracker = create_marker_tracker(frame(300), marker)
        self.assertIsNotNone(tracker)
        assert tracker is not None
        observation = update_marker_tracker(tracker, frame(308))
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertFalse(observation.exact_id)
        self.assertAlmostEqual(observation.center_x, 328.0, delta=8.0)

    def test_marker_tracker_rejects_a_partially_cropped_marker(self) -> None:
        class FakeTracker:
            def update(self, _image: np.ndarray) -> tuple[bool, tuple[float, ...]]:
                return True, (-10.0, 100.0, 40.0, 40.0)

        image = np.full((360, 640, 3), 210, dtype=np.uint8)
        ok, encoded = cv2.imencode(".jpg", image)
        self.assertTrue(ok)
        tracker = mapless_search.VisualMarkerTracker(
            tracker=FakeTracker(),
            width=640,
            height=360,
            marker_id=560,
            center_x_fraction=0.5,
            center_y_fraction=0.5,
            marker_width_fraction=1.0,
            marker_height_fraction=1.0,
        )
        self.assertIsNone(update_marker_tracker(tracker, encoded.tobytes()))

    def test_lidar_emergency_clearance_commands_reverse_turn(self) -> None:
        config = load_config()
        points = np.asarray(
            [
                (0.35, 0.0, 0.2),
                (1.5, 1.0, 0.2),
                (0.8, -0.8, 0.2),
            ],
            dtype=np.float32,
        )
        scan = compute_lidar_scan(points, 1.0)
        speed, yaw_rate = reactive_velocity(scan, 0.0, config)
        self.assertLess(speed, 0.0)
        self.assertNotEqual(yaw_rate, 0.0)

    def test_candidate_hold_covers_observed_marker_dropout(self) -> None:
        config = load_config()
        self.assertEqual(config.candidate_lost_hold, 6.0)
        self.assertEqual(config.approach_observation_hold, 1.0)
        self.assertEqual(config.tracker_only_timeout, 4.0)
        self.assertEqual(config.reacquire_direction_flip, 1.5)
        self.assertTrue(candidate_hold_active(10.0, 14.55, config))
        self.assertTrue(candidate_hold_active(10.0, 16.0, config))
        self.assertFalse(candidate_hold_active(10.0, 16.01, config))
        self.assertTrue(tracker_hold_active(10.0, 14.0, config))
        self.assertFalse(tracker_hold_active(10.0, 14.01, config))

    def test_candidate_stop_decelerates_without_a_zero_speed_step(self) -> None:
        config = load_config()
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        linear_x, angular_z = candidate_stop_velocity(
            0.20, -0.30, 0.10, scan, config
        )
        self.assertAlmostEqual(linear_x, 0.10)
        self.assertAlmostEqual(angular_z, -0.21)
        self.assertGreater(linear_x, 0.0)

    def test_candidate_stop_fails_closed_at_a_front_obstacle(self) -> None:
        config = load_config()
        scan = LidarScan((), (), 0.35, 1.5, 0.8, 1.0)
        speed, yaw_rate = candidate_stop_velocity(0.20, 0.0, 0.10, scan, config)
        self.assertEqual((speed, yaw_rate), (0.0, 0.0))

    def test_damped_approach_keeps_advancing_with_moderate_error(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            700.0,
            300.0,
            1000,
            600,
            0.10,
            (),
        )
        scan = LidarScan((), (), 2.72, 2.0, 2.0, 1.0)
        speed, yaw_rate = damped_approach_velocity(marker, scan, 0.0, config)
        self.assertGreater(speed, config.approach_min_forward_speed)
        self.assertEqual(speed, config.creep_speed)
        self.assertAlmostEqual(speed, 0.08)
        self.assertAlmostEqual(yaw_rate, -0.08)

    def test_damped_approach_turns_toward_marker_on_left(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            400.0,
            300.0,
            1000,
            600,
            0.10,
            (),
        )
        scan = LidarScan((), (), 2.72, 2.0, 2.0, 1.0)
        _speed, yaw_rate = damped_approach_velocity(marker, scan, 0.0, config)
        self.assertGreater(yaw_rate, 0.0)

    def test_damped_approach_limits_a_yaw_direction_flip(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            350.0,
            300.0,
            1000,
            600,
            0.04,
            (),
        )
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        _speed, yaw_rate = damped_approach_velocity(
            marker, scan, previous_yaw_rate=-0.12, config=config
        )
        self.assertAlmostEqual(yaw_rate, -0.04)
        self.assertLessEqual(
            abs(yaw_rate - (-0.12)), config.approach_yaw_step_limit
        )

    def test_damped_approach_stops_forward_only_for_large_error(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            900.0,
            300.0,
            1000,
            600,
            0.04,
            (),
        )
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        speed, yaw_rate = damped_approach_velocity(marker, scan, 0.0, config)
        self.assertEqual(speed, 0.0)
        self.assertLessEqual(abs(yaw_rate), config.approach_max_yaw_rate)

    def test_held_marker_stops_forward_motion_during_a_dropout(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            500.0,
            300.0,
            1000,
            600,
            0.04,
            (),
        )
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        speed, yaw_rate = held_marker_velocity(marker, scan, 0.0, config)
        self.assertEqual(speed, 0.0)
        self.assertEqual(yaw_rate, 0.0)

    def test_tracker_advances_only_while_last_exact_depth_is_far(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            500.0,
            300.0,
            1000,
            600,
            0.07,
            (),
            exact_id=False,
        )
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        speed, _yaw_rate = damped_approach_velocity(marker, scan, 0.0, config)
        self.assertGreater(speed, 0.0)
        self.assertEqual(
            depth_guarded_approach_speed(marker, None, 1.00, speed, config), 0.0
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, None, 1.01, speed, config), speed
        )

    def test_rgbd_charging_range_stops_forward_motion_for_alignment(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            500.0,
            300.0,
            1000,
            600,
            0.10,
            (),
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, 0.40, 0.40, 0.08, config), 0.0
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, 0.41, 0.41, 0.08, config), 0.08
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, None, None, 0.08, config), 0.0
        )

    def test_rgbd_far_approach_uses_depth_speed_tiers(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            500.0,
            300.0,
            1000,
            600,
            0.05,
            (),
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, 3.0, 3.0, 0.50, config),
            0.50,
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, 1.20, 1.20, 0.50, config),
            config.cautious_speed,
        )
        self.assertEqual(
            depth_guarded_approach_speed(marker, 0.70, 0.70, 0.50, config),
            config.approach_speed,
        )

    def test_sensor_freshness_rejects_missing_or_stale_timestamps(self) -> None:
        self.assertTrue(sensor_is_fresh(10.0, 10.5))
        self.assertFalse(sensor_is_fresh(0.0, 10.5))
        self.assertFalse(sensor_is_fresh(9.49, 10.5))
        self.assertFalse(sensor_is_fresh(11.0, 10.5))

    def test_damped_reacquire_rotates_in_place_toward_last_bearing(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            300.0,
            300.0,
            1000,
            600,
            0.04,
            (),
        )
        scan = LidarScan((), (), 2.0, 2.0, 2.0, 1.0)
        speed, yaw_rate = damped_reacquire_velocity(marker, scan, 0.0, config)
        self.assertEqual(speed, 0.0)
        self.assertAlmostEqual(yaw_rate, config.approach_yaw_step_limit)
        self.assertLess(yaw_rate, 0.30)

    def test_reacquire_scan_reverses_at_both_yaw_limits(self) -> None:
        config = load_config()
        half_angle = config.reacquire_sweep_half_angle
        self.assertEqual(
            bounded_reacquire_direction(
                half_angle + 0.01, 0.0, 1.0, half_angle
            ),
            -1.0,
        )
        self.assertEqual(
            bounded_reacquire_direction(
                -half_angle - 0.01, 0.0, -1.0, half_angle
            ),
            1.0,
        )

    def test_damped_reacquire_preserves_emergency_reverse(self) -> None:
        config = load_config()
        marker = MarkerObservation(
            config.marker_id,
            300.0,
            300.0,
            1000,
            600,
            0.04,
            (),
        )
        scan = LidarScan((), (), 0.35, 1.5, 0.8, 1.0)
        speed, yaw_rate = damped_reacquire_velocity(marker, scan, 0.0, config)
        self.assertEqual(speed, config.reverse_speed)
        self.assertNotEqual(yaw_rate, 0.0)

    def test_search_turns_away_before_hugging_a_side_wall(self) -> None:
        config = load_config()
        scan = LidarScan(
            angles_rad=(-0.5, 0.0, 0.5),
            clearances_m=(2.0, 3.0, 0.35),
            front_m=3.0,
            left_m=0.35,
            right_m=2.0,
            received_at=1.0,
        )
        speed, yaw_rate = reactive_velocity(scan, 0.0, config)
        self.assertGreater(speed, 0.0)
        self.assertLess(yaw_rate, 0.0)

    def test_exploration_prefers_unvisited_open_sector(self) -> None:
        scan = LidarScan(
            angles_rad=(-0.5, 0.0, 0.5),
            clearances_m=(2.0, 2.5, 2.0),
            front_m=2.5,
            left_m=2.0,
            right_m=2.0,
            received_at=1.0,
        )
        pose = RobotPose(0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0)
        visits = {(2, 0): 5}
        heading = select_exploration_heading(scan, pose, visits)
        self.assertNotEqual(heading, 0.0)

    def test_cosmos_confirmation_requires_marker_and_dock(self) -> None:
        payload = {
            "target_visible": True,
            "target_kind": "robot_charging_dock",
            "marker_visible": True,
            "safe_to_approach": True,
            "confidence": 0.9,
            "reason": "Marker is mounted on the floor charging dock.",
        }
        self.assertTrue(validate_cosmos_detection(payload).confirmed)
        payload["marker_visible"] = False
        detection = validate_cosmos_detection(payload)
        self.assertFalse(detection.confirmed)
        self.assertFalse(detection.safe_to_approach)

    def test_reasoner_requests_full_protocol_budget(self) -> None:
        client = CosmosGeneratorClient(load_config().generator)
        with patch.object(
            client, "_request", return_value={"reasoner_text": '{"ok": true}'}
        ) as request:
            self.assertEqual(client.reason(b"jpeg", "prompt"), '{"ok": true}')
        payload = request.call_args.args[2]
        self.assertEqual(payload["max_new_tokens"], REASONER_MAX_NEW_TOKENS)
        self.assertEqual(REASONER_MAX_NEW_TOKENS, 1024)

    def test_reasoner_prompt_requires_complete_six_field_json_first(self) -> None:
        fixture = type(
            "ReasonerPromptFixture",
            (),
            {"task": "find dock", "config": load_config()},
        )()
        prompt = mapless_search.MaplessChargerSearch.cosmos_prompt(fixture)
        self.assertIn("OUTPUT THE COMPLETE JSON FIRST", prompt)
        for field in (
            "target_visible",
            "target_kind",
            "marker_visible",
            "safe_to_approach",
            "confidence",
            "reason",
        ):
            self.assertIn(f'"{field}"', prompt)

    def test_charge_gate_requires_visual_close_centered_marker_and_safe_lidar(
        self,
    ) -> None:
        config = load_config()
        scan = LidarScan((), (), 0.65, 1.5, 1.5, 1.0)

        def marker(
            *,
            ratio: float = 0.20,
            error: float = 0.0,
            exact_id: bool = True,
        ) -> object:
            width = 1000
            center_x = width * (0.5 + error)
            return type(
                "MarkerFixture",
                (),
                {
                    "marker_id": config.marker_id,
                    "marker_height_ratio": ratio,
                    "horizontal_error": center_x / width - 0.5,
                    "exact_id": exact_id,
                },
            )()

        self.assertFalse(
            charge_ready(marker(ratio=0.05), 0.35, scan, 1.5, config)
        )
        self.assertFalse(charge_ready(marker(), 0.35, scan, 0.8, config))
        self.assertFalse(
            charge_ready(marker(error=0.10), 0.35, scan, 1.5, config)
        )
        self.assertFalse(charge_ready(marker(), None, scan, 1.5, config))
        self.assertFalse(charge_ready(marker(), 0.90, scan, 1.5, config))
        self.assertFalse(charge_ready(marker(), 0.55, scan, 1.5, config))
        self.assertFalse(charge_ready(marker(), 0.10, scan, 1.5, config))
        unsafe_scan = LidarScan((), (), 0.35, 1.5, 1.5, 1.0)
        self.assertFalse(
            charge_ready(marker(), 0.35, unsafe_scan, 1.5, config)
        )
        wall_behind_dock = LidarScan((), (), 3.20, 1.5, 1.5, 1.0)
        self.assertTrue(
            charge_ready(marker(), 0.35, wall_behind_dock, 1.5, config)
        )
        self.assertFalse(
            charge_ready(marker(exact_id=False), 0.35, scan, 1.5, config)
        )
        self.assertTrue(charge_ready(marker(), 0.35, scan, 1.5, config))

    def test_close_gate_does_not_treat_lidar_as_dock_range(self) -> None:
        source = (
            ROOT / "scripts/go2w_house_mapless_charger_search.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("scan.front_m <= config.charging_clearance", source)
        config = load_config()
        self.assertEqual(config.final_height_ratio, 0.10)
        self.assertEqual(config.depth_minimum_charging_range, 0.20)
        self.assertEqual(config.depth_maximum_charging_range, 0.40)
        self.assertEqual(config.tracker_forward_min_depth, 1.00)
        self.assertEqual(config.arrival_stop_hold, 3.0)
        self.assertEqual(config.approach_timeout, 240.0)

    def test_posture_guard_rejects_low_or_tilted_robot(self) -> None:
        config = load_config()
        upright = RobotPose(0.0, 0.0, 0.40, 0.0, 0.0, 0.0, 1.0)
        low = RobotPose(0.0, 0.0, 0.30, 0.0, 0.0, 0.0, 1.0)
        tilted = RobotPose(0.0, 0.0, 0.40, 0.0, 0.0, 0.40, 1.0)
        self.assertTrue(posture_is_upright(upright, config))
        self.assertFalse(posture_is_upright(low, config))
        self.assertFalse(posture_is_upright(tilted, config))

    def test_cosmos_settle_requires_low_speed_and_upright_posture(self) -> None:
        config = load_config()
        stable = RobotPose(
            0.0, 0.0, 0.40, 0.0, 0.0, 0.10, 1.0, 0.02, 0.05
        )
        moving = RobotPose(
            0.0, 0.0, 0.40, 0.0, 0.0, 0.10, 1.0, 0.04, 0.05
        )
        rotating = RobotPose(
            0.0, 0.0, 0.40, 0.0, 0.0, 0.10, 1.0, 0.02, 0.07
        )
        tilted = RobotPose(
            0.0, 0.0, 0.40, 0.0, 0.0, 0.31, 1.0, 0.0, 0.0
        )
        self.assertTrue(cosmos_pose_is_stable(stable, config))
        self.assertFalse(cosmos_pose_is_stable(moving, config))
        self.assertFalse(cosmos_pose_is_stable(rotating, config))
        self.assertFalse(cosmos_pose_is_stable(tilted, config))

    def test_rl_bridge_uses_policy_feedback_for_recovery(self) -> None:
        source = (ROOT / "controllers/go2w_rl_bridge/src/main.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('posture == "recover"', source)
        self.assertIn("Stage::kRecover", source)
        self.assertIn("policy_recovery", source)
        self.assertIn("apply_policy_command({0.0F, 0.0F, 0.0F})", source)


if __name__ == "__main__":
    unittest.main()
