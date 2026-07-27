#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import go2w_house_mission as mission  # noqa: E402
import start_go2w_house_navigation as startup  # noqa: E402
import test_go2w_house_navigation as navigation_test  # noqa: E402


class HouseStartupTest(unittest.TestCase):
    def test_default_spawn_is_shifted_right(self) -> None:
        with patch.object(
            sys,
            "argv",
            ["start_go2w_house_navigation.py", "start"],
        ):
            args = startup.parse_args()
        self.assertEqual(args.initial_x, 1.1)
        self.assertEqual(args.initial_y, -7.2)
        self.assertFalse(args.known_map_nav)
        self.assertEqual(startup.map_source(args), "online_slam")

    def test_known_map_navigation_is_explicit(self) -> None:
        with patch.object(
            sys,
            "argv",
            [
                "start_go2w_house_navigation.py",
                "start",
                "--known-map-nav",
            ],
        ):
            args = startup.parse_args()
        self.assertTrue(args.known_map_nav)
        self.assertEqual(startup.map_source(args), "known_map")

    def test_generator_argv_pins_strict_action_contract(self) -> None:
        argv = startup.cosmos_generator_argv({})
        expected_options = {
            "--num-steps": "4",
            "--fps": "10",
            "--action-chunk-size": "16",
            "--raw-action-dim": "9",
        }
        for option, expected in expected_options.items():
            with self.subTest(option=option):
                self.assertEqual(argv.count(option), 1)
                index = argv.index(option)
                self.assertLess(index + 1, len(argv))
                self.assertEqual(argv[index + 1], expected)

    def test_generator_info_accepts_exact_contract_and_reasoner(self) -> None:
        startup.validate_cosmos_generator_info(
            {
                "num_steps": 4,
                "fps": 10,
                "action_chunk_size": 16,
                "raw_action_dim": 9,
                "reasoner": True,
                "request_seed_supported": True,
            }
        )

    def test_mapless_generator_ready_probe_matches_node_log(self) -> None:
        self.assertEqual(
            startup.MAPLESS_CHARGER_READY_NEEDLE,
            "Mapless NWM-Cosmos3Edge Generator charger search ready",
        )
        node_source = (
            ROOT / "scripts/go2w_house_mapless_charger_search.py"
        ).read_text(encoding="utf-8")
        self.assertIn(startup.MAPLESS_CHARGER_READY_NEEDLE, node_source)

    def test_generator_info_rejects_missing_or_invalid_contract(self) -> None:
        valid = {
            "num_steps": 4,
            "fps": 10,
            "action_chunk_size": 16,
            "raw_action_dim": 9,
            "reasoner": True,
            "request_seed_supported": True,
        }
        invalid_updates = (
            {"num_steps": 30},
            {"reasoner": False},
            {"request_seed_supported": False},
            {"fps": True},
            {"action_chunk_size": True},
            {"raw_action_dim": True},
            {"num_steps": True},
        )
        for update in invalid_updates:
            with self.subTest(update=update):
                info = {**valid, **update}
                with self.assertRaises(startup.RuntimeFailure):
                    startup.validate_cosmos_generator_info(info)
        for missing in ("num_steps", "reasoner", "request_seed_supported"):
            with self.subTest(missing=missing):
                info = dict(valid)
                info.pop(missing)
                with self.assertRaises(startup.RuntimeFailure):
                    startup.validate_cosmos_generator_info(info)

    def test_house_environment_overrides_domain_and_caps_policy_speed(self) -> None:
        args = argparse.Namespace(
            domain_id=90,
            initial_x=-1.8,
            initial_y=-7.2,
            initial_yaw_deg=90.0,
            offscreen=True,
        )
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "89"}, clear=False):
            environment = startup.base_environment(args)
        self.assertEqual(environment["ROS_DOMAIN_ID"], "90")
        self.assertEqual(environment["RMW_IMPLEMENTATION"], "rmw_zenoh_cpp")
        self.assertLessEqual(float(environment["GO2W_RL_MAX_VX"]), 0.35)
        self.assertEqual(environment["GO2W_RL_IDLE_STAND"], "0")
        self.assertEqual(
            environment["GO2W_NAV_MODE_FILE"],
            str(ROOT / ".run/go2w_house/nav_mode"),
        )
        self.assertEqual(
            environment["GO2W_RL_POSTURE_FILE"],
            str(ROOT / ".run/go2w_house/posture"),
        )
        self.assertEqual(
            environment["GO2W_MAPLESS_SEARCH_CONFIG"],
            str(ROOT / "config/go2w_house_mapless_search.json"),
        )
        self.assertEqual(environment["GO2W_HOUSE_MAP_SOURCE"], "online_slam")
        self.assertEqual(environment["GO2W_LCM_LINEAR_DEADZONE_MPS"], "0.01")

    def test_optional_nvidia_compat_path_must_be_absolute_and_existing(self) -> None:
        args = argparse.Namespace(
            domain_id=90,
            initial_x=1.1,
            initial_y=-7.2,
            initial_yaw_deg=90.0,
            offscreen=True,
        )
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(
                os.environ,
                {"GO2W_NVIDIA_COMPAT_LIBRARY_PATH": directory},
                clear=False,
            ):
                environment = startup.base_environment(args)
            self.assertEqual(environment["LD_LIBRARY_PATH"], directory)
        with patch.dict(
            os.environ,
            {"GO2W_NVIDIA_COMPAT_LIBRARY_PATH": "relative/path"},
            clear=False,
        ):
            with self.assertRaises(startup.RuntimeFailure):
                startup.base_environment(args)
        self.assertNotIn("COSMOS_VLN_ROUTES_FILE", environment)
        self.assertNotIn("GO2W_HOUSE_MAP_YAML", environment)
        self.assertNotIn("GO2W_HOUSE_MAP_PGM", environment)

    def test_known_map_environment_includes_static_map_artifacts(self) -> None:
        args = argparse.Namespace(
            domain_id=90,
            initial_x=1.1,
            initial_y=-7.2,
            initial_yaw_deg=90.0,
            offscreen=True,
            known_map_nav=True,
        )
        environment = startup.base_environment(args)
        self.assertEqual(environment["GO2W_HOUSE_MAP_SOURCE"], "known_map")
        self.assertEqual(
            environment["COSMOS_VLN_ROUTES_FILE"],
            str(ROOT / "config/cosmos_vln_house_routes.json"),
        )
        self.assertEqual(
            environment["GO2W_HOUSE_MAP_YAML"],
            str(ROOT / ".run/go2w_house/map/house_map.yaml"),
        )

    def test_online_commands_isolate_tf_and_bridge_mapless_velocity(self) -> None:
        online_forward = startup.robot_forward_argv(False)
        known_forward = startup.robot_forward_argv(True)
        velocity_bridge = startup.mapless_velocity_bridge_argv()
        self.assertIn("/tf_static:=/robot_forward/tf_static", online_forward)
        self.assertNotIn("/tf_static:=/robot_forward/tf_static", known_forward)
        self.assertIn("/cmd_vel:=/cmd_vel_nav", velocity_bridge)

    def test_online_preflight_skips_static_map_generation(self) -> None:
        success = subprocess.CompletedProcess([], 0, "ok\n", "")
        with patch.object(startup, "command_output", return_value=success) as runner:
            startup.run_preflight({}, known_map_nav=False)
        commands = [call.args[0] for call in runner.call_args_list]
        flattened = [str(item) for command in commands for item in command]
        self.assertNotIn(str(ROOT / "scripts/go2w_house_map.py"), flattened)
        self.assertNotIn(
            str(ROOT / "scripts/go2w_house_navigation_strategy.py"), flattened
        )
        self.assertIn("online_slam", flattened)

    def test_known_map_preflight_generates_and_validates_map(self) -> None:
        success = subprocess.CompletedProcess([], 0, "ok\n", "")
        with patch.object(startup, "command_output", return_value=success) as runner:
            startup.run_preflight({}, known_map_nav=True)
        commands = [call.args[0] for call in runner.call_args_list]
        flattened = [str(item) for command in commands for item in command]
        self.assertIn(str(ROOT / "scripts/go2w_house_map.py"), flattened)
        self.assertIn(
            str(ROOT / "scripts/go2w_house_navigation_strategy.py"), flattened
        )
        self.assertIn("known_map", flattened)

    def test_runtime_state_records_online_map_source(self) -> None:
        args = argparse.Namespace(
            domain_id=90,
            with_cosmos=True,
            offscreen=True,
            known_map_nav=False,
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            state_path = Path(temp_dir) / "runtime.json"
            supervisor = startup.Supervisor(args, {})
            observed_info = {
                "num_steps": 4,
                "fps": 10,
                "action_chunk_size": 16,
                "raw_action_dim": 9,
                "reasoner": True,
                "request_seed_supported": True,
                "checkpoint": "/models/NWM-Cosmos3Edge",
            }
            supervisor.cosmos_generator_info = observed_info
            with patch.object(startup, "RUNTIME_STATE", state_path):
                supervisor.write_state(ready=True)
            state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(state["map_source"], "online_slam")
        self.assertIsNone(state["map"])
        self.assertEqual(state["map_topic"], "/map")
        self.assertEqual(
            state["cosmos_generator"]["contract"],
            startup.COSMOS_GENERATOR_CONTRACT,
        )
        self.assertTrue(state["cosmos_generator"]["reasoner_required"])
        self.assertEqual(state["cosmos_generator"]["info"], observed_info)

    def test_default_service_timeout_covers_generator_cold_start(self) -> None:
        with patch.object(
            sys,
            "argv",
            ["start_go2w_house_navigation.py", "start"],
        ):
            args = startup.parse_args()
        self.assertGreater(
            args.start_timeout,
            startup.COSMOS_GENERATOR_START_TIMEOUT_SEC,
        )

    def test_file_only_mode_update_does_not_require_nav2(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            mode_file = Path(temp_dir) / "nav_mode"
            environment = os.environ.copy()
            environment.update(
                {
                    "GO2W_NAV_MODE_FILE": str(mode_file),
                    "GO2W_NAV_MODE_FILE_ONLY": "1",
                }
            )
            result = subprocess.run(
                [str(ROOT / "scripts/set_go2w_nav_mode.sh"), "avoid"],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(mode_file.read_text(encoding="ascii"), "avoid\n")

    def test_domain_id_is_bounded(self) -> None:
        self.assertEqual(startup.domain_id("90"), 90)
        for value in ("-1", "233"):
            with self.assertRaises(argparse.ArgumentTypeError):
                startup.domain_id(value)

    def test_failed_service_cleanup_stops_and_resets_unit(self) -> None:
        stopped = subprocess.CompletedProcess([], 0, "", "")
        with patch.object(startup, "systemctl", return_value=stopped) as systemctl:
            startup.cleanup_service_after_failed_start()
        self.assertEqual(
            systemctl.call_args_list[0].args,
            ("stop", startup.UNIT),
        )
        self.assertEqual(
            systemctl.call_args_list[1].args,
            ("reset-failed", startup.UNIT),
        )

    def test_stop_timeout_forces_only_the_house_unit_and_verifies_state(self) -> None:
        inactive = subprocess.CompletedProcess([], 0, "inactive\n", "")
        success = subprocess.CompletedProcess([], 0, "", "")
        with patch.object(
            startup,
            "systemctl",
            side_effect=[
                subprocess.TimeoutExpired(["systemctl", "stop"], 30.0),
                success,
                inactive,
                success,
            ],
        ) as systemctl:
            stopped, detail = startup.stop_unit_with_fallback()
        self.assertTrue(stopped)
        self.assertIn("timed out", detail)
        self.assertEqual(
            systemctl.call_args_list[1].args,
            (
                "kill",
                "--kill-who=all",
                "--signal=SIGKILL",
                startup.UNIT,
            ),
        )


class HouseSpeedProfileTest(unittest.TestCase):
    def test_door_profile_caps_linear_and_angular_speed(self) -> None:
        commands = mission.speed_profile_commands("door", 0.32, 0.12)
        values = {(node, parameter): value for node, parameter, value in commands}
        self.assertEqual(values[("/controller_server", "FollowPath.vx_max")], "0.120")
        self.assertEqual(values[("/controller_server", "FollowPath.wz_max")], "0.350")
        self.assertEqual(mission.stage_speed_profile("east_door_inside"), "door")
        self.assertEqual(mission.stage_speed_profile("north_lounge"), "normal")

    def test_partial_profile_failure_rolls_back_to_previous_profile(self) -> None:
        calls: list[tuple[str, str, str]] = []
        failed = False

        def runner(node: str, parameter: str, value: str) -> mission.ParameterSetResult:
            nonlocal failed
            calls.append((node, parameter, value))
            if not failed and parameter == "FollowPath.wz_max" and value == "0.350":
                failed = True
                return mission.ParameterSetResult(False, "injected failure")
            return mission.ParameterSetResult(True)

        result = mission.transition_speed_profile(
            "normal", "door", 0.32, 0.12, runner=runner
        )
        self.assertFalse(result.success)
        self.assertTrue(result.rollback_succeeded)
        self.assertEqual(result.applied_profile, "normal")
        self.assertIn(
            ("/controller_server", "FollowPath.vx_max", "0.320"), calls
        )


class HouseNavigationTestModeTest(unittest.TestCase):
    def test_default_mode_is_dry_run(self) -> None:
        with patch.object(sys, "argv", ["test_go2w_house_navigation.py"]):
            args = navigation_test.parse_args()
        self.assertFalse(args.drive)
        self.assertEqual(args.domain_id, 90)

    def test_drive_requires_matching_ready_house_runtime_and_avoid_mode(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            runtime = root / "runtime.json"
            mode = root / "nav_mode"
            runtime.write_text(
                json.dumps(
                    {
                        "scene": "HouseWorld",
                        "scene_id": 6,
                        "ready": True,
                        "ros_domain_id": 90,
                        "map_source": "known_map",
                    }
                ),
                encoding="utf-8",
            )
            mode.write_text("avoid\n", encoding="utf-8")
            self.assertEqual(
                navigation_test.validate_drive_runtime(90, runtime, mode), []
            )
            errors = navigation_test.validate_drive_runtime(89, runtime, mode)
            self.assertTrue(any("runtime ROS domain" in error for error in errors))
            state = json.loads(runtime.read_text(encoding="utf-8"))
            state["map_source"] = "online_slam"
            runtime.write_text(json.dumps(state), encoding="utf-8")
            errors = navigation_test.validate_drive_runtime(90, runtime, mode)
            self.assertTrue(any("--known-map-nav" in error for error in errors))
            mode.write_text("up\n", encoding="utf-8")
            errors = navigation_test.validate_drive_runtime(90, runtime, mode)
            self.assertTrue(any("must be avoid" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
