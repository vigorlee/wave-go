#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import start_go2w_house_navigation as startup  # noqa: E402
import go2w_house_mapless_charger_search as search  # noqa: E402


class HouseDependencyContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = json.loads(
            (ROOT / "config/go2w_house_dependencies.json").read_text(
                encoding="utf-8"
            )
        )

    def test_manifest_pins_every_external_source(self) -> None:
        sources = self.manifest["sources"]
        self.assertEqual(
            set(sources),
            {
                "matrix",
                "roamerx",
                "dreamwaq",
                "cosmos_framework",
                "cosmos_checkpoint",
            },
        )
        for name, source in sources.items():
            with self.subTest(source=name):
                self.assertRegex(source["revision"], r"^[0-9a-f]{40}$")
        self.assertEqual(
            sources["cosmos_checkpoint"]["repository"],
            "nvidia/Cosmos3-Edge",
        )
        self.assertEqual(
            set(sources["dreamwaq"]["required_weights"]),
            {
                "actor_dwaq.pt",
                "encoder_dwaq.pt",
                "latent_mu_dwaq.pt",
                "latent_var_dwaq.pt",
                "vel_mu_dwaq.pt",
                "vel_var_dwaq.pt",
            },
        )

    def test_matrix_release_manifest_pins_houseworld_checksums(self) -> None:
        packages = self.manifest["sources"]["matrix"]["release"]["packages"]
        self.assertEqual(
            set(packages),
            {
                "assets-0.1.2.tar.gz",
                "base-0.1.2.tar.gz",
                "shared-0.1.2.tar.gz",
                "HouseWorld-0.1.2.tar.gz",
            },
        )
        for name, package in packages.items():
            with self.subTest(package=name):
                self.assertGreater(package["size"], 0)
                self.assertRegex(package["sha256"], r"^[0-9a-f]{64}$")

    def test_bootstrap_script_is_valid_and_uses_no_fixed_home(self) -> None:
        script = ROOT / "scripts/bootstrap_go2w_house.sh"
        result = subprocess.run(
            ["bash", "-n", str(script)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        source = script.read_text(encoding="utf-8")
        self.assertNotIn("/home/unitree", source)
        self.assertNotIn("/home/yons", source)
        self.assertIn("sources -> system -> assets -> model -> build", source)
        self.assertIn("verify_matrix_release_archives", source)
        self.assertIn("ros-humble-rmw-zenoh-cpp", source)
        self.assertIn("librmw_zenoh_cpp.so", source)
        self.assertIn(
            "install/robot_navigo/lib/robot_navigo/vel_cmd_lcm_pub",
            source,
        )
        self.assertIn("system_packages.robot_forward", source)

    def test_startup_uses_portable_cosmos_layout_and_recovery_hint(self) -> None:
        self.assertEqual(
            startup.DEFAULT_COSMOS_ROOT,
            ROOT / ".external/cosmos",
        )
        self.assertEqual(
            search.DEFAULT_COSMOS_ROOT,
            ROOT / ".external/cosmos",
        )
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing"
            with self.assertRaisesRegex(
                startup.RuntimeFailure,
                r"bootstrap_go2w_house\.sh check",
            ):
                startup.require_files([missing])


if __name__ == "__main__":
    unittest.main()
