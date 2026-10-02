#!/usr/bin/python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import time
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "scripts"))

from cosmos_vln_mission import (  # noqa: E402
    CosmosVlnMission,
    GENERIC_PROFILE,
    MissionStage,
    RobotPose,
    STAIR_UP_PROFILE,
)


def robot_pose(
    x: float,
    y: float,
    *,
    z: float = 0.4,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    received_at: float | None = None,
) -> RobotPose:
    return RobotPose(
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        received_at=time.monotonic() if received_at is None else received_at,
    )


def mission_stage(
    profile: str,
    x: float,
    y: float,
    *,
    yaw: float = 0.0,
) -> MissionStage:
    return MissionStage(
        stage_id="test_stage",
        profile=profile,
        navigation_mode="up" if profile == STAIR_UP_PROFILE else "avoid",
        frame_id="map",
        x=x,
        y=y,
        yaw_rad=yaw,
    )


class CosmosVlnMissionSupervisorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.supervisor = object.__new__(CosmosVlnMission)
        self.supervisor.odom_max_age_sec = 1.0
        self.supervisor.stair_start_max_lateral = 0.35
        self.supervisor.stair_start_min_y = -0.30
        self.supervisor.stair_start_max_y = 0.45
        self.supervisor.stair_start_max_heading = 0.25
        self.supervisor.stair_down_start_max_lateral = 0.55
        self.supervisor.stair_down_start_min_y = 4.20
        self.supervisor.stair_down_start_min_z = 1.40
        self.supervisor.stair_down_start_max_heading = 0.35
        self.supervisor.stair_max_lateral = 0.65
        self.supervisor.stair_max_heading = 0.55
        self.supervisor.stair_max_roll = 0.60
        self.supervisor.stair_max_pitch = 0.75
        self.supervisor.stair_no_progress_sec = 10.0
        self.supervisor.stair_platform_min_y = 4.20
        self.supervisor.stair_platform_min_z = 1.40
        self.supervisor.stair_platform_settle_sec = 1.0
        self.supervisor.stair_platform_timeout_sec = 15.0
        self.supervisor.stair_down_platform_min_y = 9.60
        self.supervisor.stair_down_platform_max_z = 0.80
        self.supervisor.route_max_lateral = 1.0
        self.supervisor.route_max_heading = 0.35
        self.supervisor.route_max_roll = 0.60
        self.supervisor.route_max_pitch = 0.75
        self.supervisor.route_no_progress_sec = 10.0
        self.supervisor.route_progress_min_displacement = 0.10
        self.supervisor.continuous_pass_radius = 0.68
        self.supervisor.ramp_min_elevation_gain = 0.30
        self.supervisor.ramp_min_pitch = 0.07
        self.supervisor.ramp_crest_min_z = 0.72
        self.supervisor.ramp_exit_max_z = 0.58
        self.supervisor.ramp_approach_z = None
        self.supervisor.ramp_peak_z = -math.inf
        self.supervisor.ramp_exit_z = None
        self.supervisor.ramp_up_max_abs_pitch = 0.0
        self.supervisor.ramp_down_max_abs_pitch = 0.0
        self.supervisor.navigation_active = True
        self.supervisor.awaiting_platform_validation = False
        self.supervisor.platform_stable_since = 0.0
        self.supervisor.platform_validation_started = 0.0
        self.supervisor.route_progress_at = time.monotonic()
        self.supervisor.route_progress_pose = None
        self.supervisor.stage_best_distance = math.inf
        self.aborts: list[tuple[str, dict[str, object]]] = []
        self.completions: list[dict[str, object]] = []
        self.finishes: list[tuple[str, dict[str, object]]] = []
        self.supervisor.abort_stage = (
            lambda reason, **details: self.aborts.append((reason, details))
        )
        self.supervisor.complete_stage = (
            lambda **details: self.completions.append(details)
        )
        self.supervisor.finish = (
            lambda state, **details: self.finishes.append((state, details))
        )

    def set_stage(self, stage: MissionStage, start: RobotPose) -> None:
        self.supervisor.active_stages = (stage,)
        self.supervisor.stage_index = 0
        self.supervisor.stage_start_pose = start
        self.supervisor.robot_pose = start
        self.supervisor.route_progress_pose = start
        self.supervisor.stage_best_distance = math.hypot(
            stage.x - start.x, stage.y - start.y
        )

    def test_generic_stage_start_does_not_treat_goal_yaw_as_entry_heading(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 10.0, 10.0, yaw=0.0)
        self.supervisor.robot_pose = robot_pose(0.0, 0.0, yaw=math.pi)

        safe, reason = self.supervisor.stage_start_is_safe(stage)

        self.assertTrue(safe)
        self.assertEqual(reason, "")

    def test_stair_stage_start_keeps_heading_guard(self) -> None:
        stage = mission_stage(STAIR_UP_PROFILE, 0.0, 4.8, yaw=0.0)
        self.supervisor.robot_pose = robot_pose(0.0, 0.0, yaw=0.50)

        safe, reason = self.supervisor.stage_start_is_safe(stage)

        self.assertFalse(safe)
        self.assertEqual(reason, "stage_start_heading_misaligned")

    def test_generic_stage_allows_nav2_detour_inside_bounded_envelope(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 10.0, 10.0, yaw=0.0)
        start = robot_pose(0.0, 0.0)
        self.set_stage(stage, start)
        now = time.monotonic()
        self.supervisor.robot_pose = robot_pose(1.0, 9.0, yaw=math.pi, received_at=now)

        direct_line_error = abs(
            self.supervisor.stage_lateral_error(stage, self.supervisor.robot_pose)
        )
        result = self.supervisor.check_stage_route(now)

        self.assertGreater(direct_line_error, self.supervisor.route_max_lateral)
        self.assertEqual(
            self.supervisor.generic_stage_corridor_overrun(
                stage, self.supervisor.robot_pose
            ),
            0.0,
        )
        self.assertTrue(result)
        self.assertEqual(self.aborts, [])

    def test_generic_stage_fails_closed_outside_route_envelope(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 10.0, 10.0)
        self.set_stage(stage, robot_pose(0.0, 0.0))
        now = time.monotonic()
        self.supervisor.robot_pose = robot_pose(-1.2, 5.0, received_at=now)

        result = self.supervisor.check_stage_route(now)

        self.assertFalse(result)
        self.assertEqual(self.aborts[0][0], "stage_safety_boundary")
        self.assertAlmostEqual(self.aborts[0][1]["corridor_overrun"], 0.2)

    def test_generic_stage_keeps_odom_and_attitude_guards(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 10.0, 10.0)
        self.set_stage(stage, robot_pose(0.0, 0.0))
        now = time.monotonic()
        self.supervisor.robot_pose = robot_pose(
            1.0, 1.0, roll=0.61, received_at=now
        )

        self.assertFalse(self.supervisor.check_stage_route(now))
        self.assertEqual(self.aborts[-1][0], "stage_safety_boundary")

        self.aborts.clear()
        self.supervisor.robot_pose = robot_pose(1.0, 1.0, received_at=now - 1.1)
        self.assertFalse(self.supervisor.check_stage_route(now))
        self.assertEqual(self.aborts[-1][0], "stage_odom_lost")

    def test_generic_actual_displacement_refreshes_progress_while_detouring(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 0.0, 10.0)
        start = robot_pose(0.0, 0.0)
        self.set_stage(stage, start)
        now = time.monotonic()
        self.supervisor.route_progress_at = now - 11.0
        self.supervisor.robot_pose = robot_pose(1.0, 0.0, received_at=now)

        result = self.supervisor.check_stage_route(now)

        self.assertTrue(result)
        self.assertEqual(self.aborts, [])
        self.assertEqual(self.supervisor.route_progress_at, now)
        self.assertEqual(self.supervisor.route_progress_pose, self.supervisor.robot_pose)
        self.assertGreater(
            math.hypot(
                stage.x - self.supervisor.robot_pose.x,
                stage.y - self.supervisor.robot_pose.y,
            ),
            self.supervisor.stage_best_distance,
        )

    def test_generic_still_aborts_when_neither_motion_nor_goal_progress_occurs(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 0.0, 10.0)
        start = robot_pose(0.0, 0.0)
        self.set_stage(stage, start)
        now = time.monotonic()
        self.supervisor.route_progress_at = now - 11.0
        self.supervisor.robot_pose = robot_pose(0.0, 0.0, received_at=now)

        result = self.supervisor.check_stage_route(now)

        self.assertFalse(result)
        self.assertEqual(self.aborts[0][0], "stage_no_progress")

    def test_stair_no_progress_does_not_accept_sideways_motion(self) -> None:
        stage = mission_stage(STAIR_UP_PROFILE, 0.0, 4.8)
        start = robot_pose(0.0, 0.0)
        self.set_stage(stage, start)
        now = time.monotonic()
        self.supervisor.route_progress_at = now - 11.0
        self.supervisor.robot_pose = robot_pose(0.4, 0.0, received_at=now)

        result = self.supervisor.check_stage_route(now)

        self.assertFalse(result)
        self.assertEqual(self.aborts[0][0], "stage_no_progress")

    def test_stair_runtime_keeps_centerline_heading_guard(self) -> None:
        stage = mission_stage(STAIR_UP_PROFILE, 0.0, 4.8)
        self.set_stage(stage, robot_pose(0.0, 0.0))
        now = time.monotonic()
        self.supervisor.robot_pose = robot_pose(
            0.0, 1.0, yaw=0.56, received_at=now
        )

        result = self.supervisor.check_stage_route(now)

        self.assertFalse(result)
        self.assertEqual(self.aborts[0][0], "stage_safety_boundary")

    def test_continuous_waypoint_is_passed_without_exact_goal_stop(self) -> None:
        stage = mission_stage(GENERIC_PROFILE, 0.0, 1.0)
        self.supervisor.stage_start_pose = robot_pose(0.0, 0.0)

        self.assertTrue(
            self.supervisor.continuous_position_reached(
                stage, robot_pose(0.0, 0.45)
            )
        )
        self.assertTrue(
            self.supervisor.continuous_position_reached(
                stage, robot_pose(0.0, 1.10)
            )
        )

    def test_physical_ramp_evidence_requires_rise_drop_and_attitude(self) -> None:
        self.supervisor.ramp_approach_z = 0.42
        self.supervisor.ramp_peak_z = 0.84
        self.supervisor.ramp_exit_z = 0.43
        self.supervisor.ramp_up_max_abs_pitch = 0.17
        self.supervisor.ramp_down_max_abs_pitch = 0.16

        self.assertTrue(self.supervisor.ramp_physics_details()["passed"])

        self.supervisor.ramp_exit_z = 0.73
        self.assertFalse(self.supervisor.ramp_physics_details()["passed"])


if __name__ == "__main__":
    unittest.main()
