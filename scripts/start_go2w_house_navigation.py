#!/usr/bin/python3
"""Start and supervise the isolated Go2-W HouseWorld navigation stack."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Callable, IO, Sequence
import urllib.error
import urllib.request


ROOT_DIR = Path(__file__).resolve().parents[1]
MATRIX_DIR = ROOT_DIR / "matrix"
ROAMERX_DIR = ROOT_DIR / "genisom_roamerx_open"
RUN_DIR = ROOT_DIR / ".run/go2w_house"
LOG_DIR = ROOT_DIR / "logs/go2w_house"
MAP_DIR = RUN_DIR / "map"
COSMOS_DIR = RUN_DIR / "cosmos"
UNIT = "go2w-house-navigation.service"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
ROAMERX_SETUP = ROAMERX_DIR / "install/setup.bash"
FORWARD_SETUP = Path("/opt/robot/robot-forward/install/setup.bash")
FORWARD_BINARY = Path(
    "/opt/robot/robot-forward/install/robot_forward/lib/robot_forward/robot_forward"
)
ROUTES_FILE = ROOT_DIR / "config/cosmos_vln_house_routes.json"
MAPLESS_SEARCH_CONFIG = ROOT_DIR / "config/go2w_house_mapless_search.json"
NAV_PARAMS = ROOT_DIR / "config/go2w_house_navigo_params.yaml"
POINTCLOUD_TO_SCAN_CONFIG = (
    ROOT_DIR / "config/go2w_house_pointcloud_to_laserscan.yaml"
)
SLAM_PARAMS = ROOT_DIR / "config/go2w_house_slam_toolbox.yaml"
ONLINE_SLAM_RVIZ = ROOT_DIR / "config/go2w_house_online_slam.rviz"
SLAM_TOOLBOX_BINARY = Path(
    "/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node"
)
HOUSE_XML = (
    MATRIX_DIR / "src/robot_mujoco/zsibot_robots/go2w/scene_terrain_house.xml"
)
HOUSE_UE_XML = (
    MATRIX_DIR
    / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_house.xml"
)
HOUSE_LINK = MATRIX_DIR / "src/robot_mujoco/zsibot_robots/xgw/go2w_house"
SUPERVISOR_LOG = LOG_DIR / "supervisor.log"
RUNTIME_STATE = RUN_DIR / "runtime.json"
POSTURE_FILE = RUN_DIR / "posture"
DEFAULT_COSMOS_ROOT = Path("/home/unitree/matrix_g1_lcm_demo")
COSMOS_GENERATOR_SCRIPT = ROOT_DIR / "scripts/cosmos3_go2w_generator_server.py"
COSMOS_GENERATOR_HOST = "127.0.0.1"
COSMOS_GENERATOR_PORT = 8098
COSMOS_GENERATOR_URL = (
    f"http://{COSMOS_GENERATOR_HOST}:{COSMOS_GENERATOR_PORT}"
)
COSMOS_GENERATOR_CONTRACT = {
    "raw_action_dim": 9,
    "fps": 10,
    "action_chunk_size": 16,
    "num_steps": 4,
}
COSMOS_GENERATOR_REASONER_REQUIRED = True
COSMOS_GENERATOR_START_TIMEOUT_SEC = 360.0
DEFAULT_START_TIMEOUT_SEC = 420.0
MAPLESS_CHARGER_READY_NEEDLE = (
    "Mapless NWM-Cosmos3Edge Generator charger search ready"
)


class RuntimeFailure(RuntimeError):
    pass


def ros_argv(command: Sequence[str], setup: Path = ROAMERX_SETUP) -> list[str]:
    return [
        "bash",
        "--noprofile",
        "--norc",
        "-c",
        'set +u; source "$1"; source "$2"; set -u; shift 2; exec "$@"',
        "go2w-house-ros",
        str(ROS_SETUP),
        str(setup),
        *command,
    ]


def command_output(
    command: Sequence[str], *, env: dict[str, str] | None = None, timeout: float = 10.0
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command),
        check=False,
        capture_output=True,
        text=True,
        env=env,
        timeout=timeout,
    )


def systemctl(*args: str, timeout: float = 15.0) -> subprocess.CompletedProcess[str]:
    return command_output(["systemctl", "--user", *args], timeout=timeout)


def unit_active(unit: str) -> bool:
    return systemctl("is-active", "--quiet", unit).returncode == 0


def domain_id(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed <= 232:
        raise argparse.ArgumentTypeError("ROS domain ID must be in [0, 232]")
    return parsed


def positive_seconds(value: str) -> float:
    parsed = float(value)
    if not parsed > 0.0:
        raise argparse.ArgumentTypeError("timeout must be positive")
    return parsed


def ensure_layout() -> None:
    for directory in (RUN_DIR, LOG_DIR, MAP_DIR, COSMOS_DIR / "jobs"):
        directory.mkdir(parents=True, exist_ok=True)


def map_source(args: argparse.Namespace) -> str:
    return "known_map" if getattr(args, "known_map_nav", False) else "online_slam"


def robot_forward_argv(known_map_nav: bool) -> list[str]:
    command = [str(FORWARD_BINARY)]
    if not known_map_nav:
        command.extend(
            [
                "--ros-args",
                "-r",
                "/tf_static:=/robot_forward/tf_static",
            ]
        )
    return ros_argv(command, setup=FORWARD_SETUP)


def mapless_velocity_bridge_argv() -> list[str]:
    return ros_argv(
        [
            "ros2",
            "run",
            "robot_navigo",
            "vel_cmd_lcm_pub",
            "--ros-args",
            "-r",
            "/cmd_vel:=/cmd_vel_nav",
        ]
    )


def cosmos_generator_paths(env: dict[str, str]) -> tuple[Path, Path, Path, Path]:
    cosmos_root = Path(env.get("COSMOS_ROOT", str(DEFAULT_COSMOS_ROOT)))
    framework_dir = Path(
        env.get(
            "COSMOS_VLN_FRAMEWORK",
            str(cosmos_root / "packages/cosmos-framework"),
        )
    )
    checkpoint_dir = Path(
        env.get("COSMOS_VLN_CHECKPOINT", str(cosmos_root / "Cosmos3-Edge"))
    )
    model_config = Path(
        env.get(
            "COSMOS_VLN_MODEL_CONFIG",
            str(
                framework_dir
                / "cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml"
            ),
        )
    )
    return framework_dir, framework_dir / ".venv/bin/python", checkpoint_dir, model_config


def cosmos_generator_argv(env: dict[str, str]) -> list[str]:
    framework_dir, python, checkpoint_dir, model_config = cosmos_generator_paths(env)
    return [
        str(python),
        str(COSMOS_GENERATOR_SCRIPT),
        "--checkpoint-path",
        str(checkpoint_dir),
        "--config-file",
        str(model_config),
        "--output-dir",
        str(COSMOS_DIR / "generator_server"),
        "--host",
        COSMOS_GENERATOR_HOST,
        "--port",
        str(COSMOS_GENERATOR_PORT),
        "--num-steps",
        str(COSMOS_GENERATOR_CONTRACT["num_steps"]),
        "--fps",
        str(COSMOS_GENERATOR_CONTRACT["fps"]),
        "--action-chunk-size",
        str(COSMOS_GENERATOR_CONTRACT["action_chunk_size"]),
        "--raw-action-dim",
        str(COSMOS_GENERATOR_CONTRACT["raw_action_dim"]),
        "--http-400-on-error",
    ]


def fetch_cosmos_generator_info(timeout: float = 2.0) -> dict[str, object] | None:
    request = urllib.request.Request(
        f"{COSMOS_GENERATOR_URL}/info",
        headers={"Accept": "application/json", "Cache-Control": "no-cache"},
        method="GET",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            status = response.getcode()
            body = response.read(65_537)
    except urllib.error.HTTPError as exc:
        raise RuntimeFailure(
            f"Cosmos3 Generator /info returned HTTP {exc.code}"
        ) from exc
    except (urllib.error.URLError, TimeoutError, OSError):
        return None
    if status != 200:
        raise RuntimeFailure(f"Cosmos3 Generator /info returned HTTP {status}")
    if len(body) > 65_536:
        raise RuntimeFailure("Cosmos3 Generator /info response is too large")
    try:
        payload = json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeFailure("Cosmos3 Generator /info returned invalid JSON") from exc
    if not isinstance(payload, dict):
        raise RuntimeFailure("Cosmos3 Generator /info must return a JSON object")
    return payload


def validate_cosmos_generator_info(info: dict[str, object]) -> None:
    mismatches: list[str] = []
    for field, expected in COSMOS_GENERATOR_CONTRACT.items():
        actual = info.get(field)
        if type(actual) is not int or actual != expected:
            mismatches.append(f"{field}=expected {expected}, got {actual!r}")
    if info.get("reasoner") is not COSMOS_GENERATOR_REASONER_REQUIRED:
        mismatches.append(
            "reasoner=expected True, got " + repr(info.get("reasoner"))
        )
    if info.get("request_seed_supported") is not True:
        mismatches.append(
            "request_seed_supported=expected True, got "
            + repr(info.get("request_seed_supported"))
        )
    if mismatches:
        raise RuntimeFailure(
            "Cosmos3 Generator contract mismatch: " + "; ".join(mismatches)
        )


def base_environment(args: argparse.Namespace) -> dict[str, str]:
    env = os.environ.copy()
    nvidia_compat_library_path = os.environ.get(
        "GO2W_NVIDIA_COMPAT_LIBRARY_PATH", ""
    ).strip()
    if nvidia_compat_library_path:
        compat_path = Path(nvidia_compat_library_path)
        if not compat_path.is_absolute() or not compat_path.is_dir():
            raise RuntimeFailure(
                "GO2W_NVIDIA_COMPAT_LIBRARY_PATH must be an existing absolute directory"
            )
        env["LD_LIBRARY_PATH"] = str(compat_path)
    env.update(
        {
            "RMW_IMPLEMENTATION": "rmw_zenoh_cpp",
            "ROS_DOMAIN_ID": str(args.domain_id),
            "ROS2CLI_NO_DAEMON": "1",
            "SDK_CLIENT_IP": "127.0.0.1",
            "COMMUNICATION_TYPE": "LCM",
            "ROAMERX_COMMUNICATION_TYPE": "LCM",
            "GENISOM_ROAMERX_OPEN_WORKSPACE": str(ROAMERX_DIR),
            "MATRIX_SKIP_ENV_CHECK": "1",
            "MATRIX_ROBOT_INITIAL_X": str(args.initial_x),
            "MATRIX_ROBOT_INITIAL_Y": str(args.initial_y),
            "MATRIX_ROBOT_INITIAL_YAW_DEG": str(args.initial_yaw_deg),
            "MATRIX_UE_OFFSCREEN": "1" if args.offscreen else "0",
            "MATRIX_UE_PERFORMANCE_PROFILE": "1",
            "MATRIX_UE_NO_RHI_THREAD": "1",
            "MATRIX_UE_MAX_FPS": os.environ.get("MATRIX_UE_MAX_FPS", "15"),
            "MATRIX_UE_RES_X": os.environ.get("MATRIX_UE_RES_X", "960"),
            "MATRIX_UE_RES_Y": os.environ.get("MATRIX_UE_RES_Y", "540"),
            "MATRIX_CAMERA_WIDTH": os.environ.get("MATRIX_CAMERA_WIDTH", "960"),
            "MATRIX_CAMERA_HEIGHT": os.environ.get("MATRIX_CAMERA_HEIGHT", "540"),
            "MATRIX_CAMERA_FREQUENCY": os.environ.get("MATRIX_CAMERA_FREQUENCY", "2"),
            "MATRIX_DEPTH_WIDTH": os.environ.get("MATRIX_DEPTH_WIDTH", "320"),
            "MATRIX_DEPTH_HEIGHT": os.environ.get("MATRIX_DEPTH_HEIGHT", "240"),
            "MATRIX_DEPTH_FREQUENCY": os.environ.get("MATRIX_DEPTH_FREQUENCY", "2"),
            "MATRIX_DISABLE_IMAGE_SENSORS": "0",
            "MATRIX_MUJOCO_ROBOT_TYPE": "xgw",
            "MATRIX_MUJOCO_SCENE": "go2w_house/scene_terrain_house.xml",
            "MATRIX_UE_SCENE_OVERRIDE": str(HOUSE_UE_XML),
            "MATRIX_WORLD_MODEL_VALIDATOR": str(
                ROOT_DIR / "scripts/validate_go2w_house_world.py"
            ),
            "GO2W_WHEEL_SIGN": "-1",
            "GO2W_RL_HISTORY_MODE": os.environ.get("GO2W_RL_HISTORY_MODE", "train"),
            "GO2W_RL_STOCHASTIC": os.environ.get("GO2W_RL_STOCHASTIC", "0"),
            "GO2W_RL_MAX_VX": "0.35",
            "GO2W_RL_MAX_VY": "0.20",
            "GO2W_RL_MAX_YAW": "0.65",
            "GO2W_RL_IDLE_STAND": "0",
            "GO2W_RL_TERRAIN_GUARD": "0",
            "GO2W_RL_STAIR_ASSIST": "0",
            "GO2W_RL_STAIR_APPROACH_ASSIST": "0",
            "GO2W_LCM_LINEAR_DEADZONE_MPS": "0.01",
            "GO2W_NAV_MODE_FILE": str(RUN_DIR / "nav_mode"),
            "GO2W_RL_POSTURE_FILE": str(POSTURE_FILE),
            "GO2W_RL_CHARGE_HIP": "1.20",
            "GO2W_RL_CHARGE_KNEE": "-2.30",
            "GO2W_RL_POSTURE_RAMP_SECONDS": "2.5",
            "AVOID_NAV_VX_MAX": "0.32",
            "AVOID_NAV_VX_MIN": "-0.10",
            "AVOID_NAV_WZ_MAX": "0.55",
            "AVOID_NAV_MAX_VELOCITY": "[0.32, 0.0, 0.55]",
            "AVOID_NAV_MIN_VELOCITY": "[-0.10, 0.0, -0.55]",
            "AVOID_NAV_MAX_ACCEL": "[0.55, 0.0, 0.70]",
            "AVOID_NAV_MAX_DECEL": "[-0.70, 0.0, -0.70]",
            "GO2W_HOUSE_NAV_SPEED": os.environ.get(
                "GO2W_HOUSE_NAV_SPEED", "0.32"
            ),
            "GO2W_HOUSE_DOOR_SPEED": os.environ.get(
                "GO2W_HOUSE_DOOR_SPEED", "0.12"
            ),
            "GO2W_HOUSE_MODEL_FUSION": os.environ.get(
                "GO2W_HOUSE_MODEL_FUSION", "0"
            ),
            "GO2W_HOUSE_MAP_SOURCE": map_source(args),
            "GO2W_MAPLESS_SEARCH_CONFIG": str(MAPLESS_SEARCH_CONFIG),
            "GO2W_CHARGER_SEARCH_JOBS_DIR": str(COSMOS_DIR / "charger_search"),
            "COSMOS3_GENERATOR_URL": COSMOS_GENERATOR_URL,
            "COSMOS_VLN_JOBS_DIR": str(COSMOS_DIR / "jobs"),
            "COSMOS_VLN_VISUALIZATION_IMAGE": str(
                COSMOS_DIR / "latest_visualization.jpg"
            ),
            "COSMOS_VLN_NAV_MODE_SCRIPT": str(
                ROOT_DIR / "scripts/set_go2w_nav_mode.sh"
            ),
            "COSMOS_VLN_ROUTE_MAX_LATERAL": "1.50",
            "COSMOS_VLN_ROUTE_NO_PROGRESS_SEC": "30.0",
            "COSMOS_VLN_MISSION_TIMEOUT_SEC": "900.0",
        }
    )
    if getattr(args, "known_map_nav", False):
        env["COSMOS_VLN_ROUTES_FILE"] = str(ROUTES_FILE)
        env["GO2W_HOUSE_MAP_YAML"] = str(MAP_DIR / "house_map.yaml")
        env["GO2W_HOUSE_MAP_PGM"] = str(MAP_DIR / "house_map.pgm")
    else:
        env.pop("COSMOS_VLN_ROUTES_FILE", None)
        env.pop("GO2W_HOUSE_MAP_YAML", None)
        env.pop("GO2W_HOUSE_MAP_PGM", None)
    return env


def require_files(paths: Sequence[Path]) -> None:
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise RuntimeFailure("required files are missing: " + ", ".join(missing))


def reject_conflicting_runtime() -> None:
    conflicts = (
        "go2w-navigation.service",
        "oli-matrix-ue.service",
        "nezha2-navigation.service",
    )
    active = [unit for unit in conflicts if unit_active(unit)]
    process_check = command_output(
        [
            "pgrep",
            "-f",
            "robot_mujoco|zsibot_mujoco_ue|go2w_rl_bridge|go2w_lcm_bridge",
        ]
    )
    if active or process_check.returncode == 0:
        detail = ", ".join(active) if active else "simulator/controller process"
        raise RuntimeFailure(
            f"another MATRiX runtime is active ({detail}); stop it before HouseWorld"
        )


def prepare_house_link() -> None:
    expected = Path("../go2w")
    if HOUSE_LINK.is_symlink():
        if Path(os.readlink(HOUSE_LINK)) == expected:
            return
        HOUSE_LINK.unlink()
    elif HOUSE_LINK.exists():
        raise RuntimeFailure(f"House runtime link path is not a symlink: {HOUSE_LINK}")
    HOUSE_LINK.symlink_to(expected)


def run_preflight(env: dict[str, str], known_map_nav: bool) -> None:
    if known_map_nav:
        map_command = [
            "/usr/bin/python3",
            str(ROOT_DIR / "scripts/go2w_house_map.py"),
            "--output-dir",
            str(MAP_DIR),
            "--routes-file",
            str(ROUTES_FILE),
        ]
        result = command_output(map_command, env=env, timeout=60.0)
        if result.returncode != 0:
            raise RuntimeFailure(
                "House map generation failed: "
                + (result.stderr or result.stdout).strip()
            )
        print(result.stdout.strip(), flush=True)

        strategy_command = [
            "/usr/bin/python3",
            str(ROOT_DIR / "scripts/go2w_house_navigation_strategy.py"),
            "--routes-file",
            str(ROUTES_FILE),
            "--map-dir",
            str(MAP_DIR),
            "--output",
            str(RUN_DIR / "strategy_report.json"),
        ]
        result = command_output(strategy_command, env=env, timeout=60.0)
        if result.returncode != 0:
            raise RuntimeFailure(
                "House route strategy validation failed: "
                + (result.stderr or result.stdout).strip()
            )
        print(result.stdout.strip(), flush=True)

    validator = command_output(
        [
            "/usr/bin/python3",
            str(ROOT_DIR / "scripts/validate_go2w_house_world.py"),
            "--map-source",
            "known_map" if known_map_nav else "online_slam",
        ],
        env=env,
        timeout=30.0,
    )
    if validator.returncode != 0:
        raise RuntimeFailure(
            "House world validation failed: "
            + (validator.stderr or validator.stdout).strip()
        )
    print(validator.stdout.strip(), flush=True)


class Supervisor:
    def __init__(self, args: argparse.Namespace, env: dict[str, str]) -> None:
        self.args = args
        self.env = env
        self.processes: list[tuple[str, subprocess.Popen[bytes], IO[bytes], bool]] = []
        self.stop_requested = False
        self.cosmos_generator_info: dict[str, object] | None = None
        self.cosmos_generator_managed = False
        self.cosmos_generator_checked_at = 0.0

    def request_stop(self, _signum: int, _frame: object) -> None:
        self.stop_requested = True

    def start_process(
        self,
        name: str,
        command: Sequence[str],
        *,
        critical: bool = True,
        env: dict[str, str] | None = None,
        cwd: Path = ROOT_DIR,
    ) -> subprocess.Popen[bytes]:
        log_path = LOG_DIR / f"{name}.log"
        # A fresh log prevents readiness probes from matching a previous run.
        log_handle = log_path.open("wb", buffering=0)
        try:
            process = subprocess.Popen(
                list(command),
                cwd=cwd,
                env=env or self.env,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_handle.close()
            raise
        self.processes.append((name, process, log_handle, critical))
        print(f"[INFO] started {name}, pid={process.pid}, log={log_path}", flush=True)
        self.write_state()
        return process

    def write_state(self, ready: bool = False) -> None:
        current_map_source = map_source(self.args)
        state = {
            "scene": "HouseWorld",
            "scene_id": 6,
            "ros_domain_id": self.args.domain_id,
            "with_cosmos": self.args.with_cosmos,
            "offscreen": self.args.offscreen,
            "ready": ready,
            "updated_at": time.time(),
            "map_source": current_map_source,
            "map": (
                str(MAP_DIR / "house_map.yaml")
                if current_map_source == "known_map"
                else None
            ),
            "map_topic": "/map",
            "visualization": str(COSMOS_DIR / "latest_visualization.jpg"),
            "cosmos_generator": (
                {
                    "url": COSMOS_GENERATOR_URL,
                    "managed": self.cosmos_generator_managed,
                    "ready": self.cosmos_generator_info is not None,
                    "checked_at": self.cosmos_generator_checked_at,
                    "contract": dict(COSMOS_GENERATOR_CONTRACT),
                    "reasoner_required": COSMOS_GENERATOR_REASONER_REQUIRED,
                    "info": self.cosmos_generator_info,
                }
                if self.args.with_cosmos
                else None
            ),
            "processes": {
                name: {"pid": process.pid, "running": process.poll() is None}
                for name, process, _log, _critical in self.processes
            },
        }
        temporary = RUNTIME_STATE.with_suffix(".tmp")
        temporary.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
        temporary.replace(RUNTIME_STATE)

    def check_critical_processes(self) -> None:
        for name, process, _log, critical in self.processes:
            code = process.poll()
            if critical and code is not None:
                raise RuntimeFailure(f"critical process {name} exited with status {code}")

    def wait_until(
        self, label: str, predicate: Callable[[], bool], timeout_seconds: float
    ) -> None:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.check_critical_processes()
            if self.stop_requested:
                raise RuntimeFailure(f"startup interrupted while waiting for {label}")
            if predicate():
                print(f"[OK] {label}", flush=True)
                return
            time.sleep(1.0)
        raise RuntimeFailure(f"timed out waiting for {label}")

    def topic_exists(self, topic: str) -> bool:
        try:
            result = command_output(
                ros_argv(["ros2", "topic", "list", "--no-daemon"]),
                env=self.env,
                timeout=6.0,
            )
        except subprocess.TimeoutExpired:
            return False
        return result.returncode == 0 and topic in result.stdout.splitlines()

    def action_exists(self, action: str) -> bool:
        try:
            result = command_output(
                ros_argv(["ros2", "action", "list"]), env=self.env, timeout=6.0
            )
        except subprocess.TimeoutExpired:
            return False
        return result.returncode == 0 and action in result.stdout.splitlines()

    def log_contains(self, filename: str, needle: str) -> bool:
        try:
            return needle in (LOG_DIR / filename).read_text(
                encoding="utf-8", errors="replace"
            )
        except OSError:
            return False

    def cosmos_generator_ready(self) -> bool:
        info = fetch_cosmos_generator_info()
        if info is None:
            return False
        validate_cosmos_generator_info(info)
        self.cosmos_generator_info = info
        self.cosmos_generator_checked_at = time.time()
        return True

    def ensure_cosmos_generator(self) -> None:
        info = fetch_cosmos_generator_info()
        if info is not None:
            validate_cosmos_generator_info(info)
            self.cosmos_generator_info = info
            self.cosmos_generator_checked_at = time.time()
            print(
                "[OK] reusing healthy Cosmos3 Generator "
                f"endpoint={COSMOS_GENERATOR_URL} contract={COSMOS_GENERATOR_CONTRACT}",
                flush=True,
            )
            self.write_state()
            return

        framework_dir, python, checkpoint_dir, model_config = cosmos_generator_paths(
            self.env
        )
        missing = [
            str(path)
            for path in (COSMOS_GENERATOR_SCRIPT, python, model_config)
            if not path.is_file()
        ]
        if not framework_dir.is_dir():
            missing.append(str(framework_dir))
        if not checkpoint_dir.is_dir():
            missing.append(str(checkpoint_dir))
        if missing:
            raise RuntimeFailure(
                "Cosmos3 Generator runtime paths are missing: " + ", ".join(missing)
            )

        generator_env = self.env.copy()
        generator_env.update(
            {
                "COSMOS_TRAINING": "0",
                "LD_LIBRARY_PATH": self.env.get(
                    "GO2W_NVIDIA_COMPAT_LIBRARY_PATH", ""
                ),
                "PYTHONPATH": "",
                "PYTHONUNBUFFERED": "1",
                "PYTORCH_CUDA_ALLOC_CONF": "expandable_segments:True",
                "PYTORCH_ALLOC_CONF": "expandable_segments:True",
            }
        )
        self.cosmos_generator_managed = True
        self.start_process(
            "cosmos_generator",
            cosmos_generator_argv(self.env),
            env=generator_env,
            cwd=framework_dir,
        )
        self.wait_until(
            "Cosmos3 Generator action contract "
            "raw_action_dim=9 fps=10 action_chunk_size=16 "
            "num_steps=4 reasoner=true",
            self.cosmos_generator_ready,
            COSMOS_GENERATOR_START_TIMEOUT_SEC,
        )
        self.write_state()

    def check_cosmos_generator_health(self) -> None:
        info = fetch_cosmos_generator_info(timeout=1.0)
        if info is None:
            raise RuntimeFailure(
                f"Cosmos3 Generator is not reachable at {COSMOS_GENERATOR_URL}"
            )
        validate_cosmos_generator_info(info)
        self.cosmos_generator_info = info
        self.cosmos_generator_checked_at = time.time()

    def run_checked(
        self,
        command: Sequence[str],
        label: str,
        *,
        env: dict[str, str] | None = None,
    ) -> None:
        result = command_output(command, env=env or self.env, timeout=30.0)
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeFailure(f"{label} failed: {detail}")
        if result.stdout.strip():
            print(result.stdout.strip(), flush=True)

    def launch(self) -> None:
        self.start_process(
            "zenohd",
            ros_argv(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]),
        )
        time.sleep(2.0)
        self.check_critical_processes()

        self.start_process(
            "bridge",
            ["stdbuf", "-oL", "-eL", str(ROOT_DIR / "install/bin/go2w_rl_bridge")],
        )
        self.start_process(
            "sim",
            [
                str(MATRIX_DIR / "scripts/run_sim.sh"),
                "go2w",
                "6",
                "1" if self.args.offscreen else "0",
                "0",
                "1",
            ],
            env=self.env,
        )
        self.start_process(
            "lidar_tf",
            ros_argv(
                [
                    "ros2",
                    "run",
                    "tf2_ros",
                    "static_transform_publisher",
                    "--x",
                    "0.13011",
                    "--y",
                    "0.02329",
                    "--z",
                    "0.17598",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "lidar",
                ]
            ),
        )
        self.start_process(
            "lidar_bridge",
            ros_argv(
                [
                    "/usr/bin/python3",
                    str(ROOT_DIR / "scripts/go2w_house_lidar_bridge.py"),
                ]
            ),
        )

        self.wait_until(
            "DreamWaQ policy active",
            lambda: self.log_contains("bridge.log", "Policy control enabled"),
            45.0,
        )
        for topic in (
            "/odom/mujoco_odom",
            "/livox/lidar_raw",
            "/livox/lidar",
            "/image_raw/compressed",
        ):
            self.wait_until(
                f"topic {topic}", lambda topic=topic: self.topic_exists(topic), 120.0
            )

        known_map_nav = getattr(self.args, "known_map_nav", False)
        # robot_forward publishes a hard-coded identity map -> odom on
        # /tf_static. Online SLAM must be the sole map -> odom authority.
        self.start_process(
            "robot_forward",
            robot_forward_argv(known_map_nav),
        )

        if known_map_nav:
            nav_command = ros_argv(
                [
                    "ros2",
                    "launch",
                    "robot_navigo",
                    "navigation_bringup.launch.py",
                    "platform:=UE",
                    "mc_controller_type:=RL_TRACK_VELOCITY",
                    "communication_type:=LCM",
                    "use_composition:=False",
                    f"map:={MAP_DIR / 'house_map.yaml'}",
                    f"params_file:={NAV_PARAMS}",
                ]
            )
            self.start_process("navigation", nav_command)
            self.wait_until(
                "Nav2 /navigate_to_pose action",
                lambda: self.action_exists("/navigate_to_pose"),
                60.0,
            )
            self.wait_until(
                "Nav2 /compute_path_to_pose action",
                lambda: self.action_exists("/compute_path_to_pose"),
                30.0,
            )
            self.run_checked(
                [str(ROOT_DIR / "scripts/set_go2w_nav_mode.sh"), "avoid"],
                "indoor navigation mode",
            )
            self.run_checked(
                [str(ROOT_DIR / "scripts/validate_go2w_local_avoidance.sh")],
                "local obstacle avoidance",
            )
        else:
            self.start_process(
                "pointcloud_to_laserscan",
                ros_argv(
                    [
                        "/usr/bin/python3",
                        str(
                            ROOT_DIR
                            / "scripts/go2w_house_pointcloud_to_laserscan.py"
                        ),
                        "--ros-args",
                        "--params-file",
                        str(POINTCLOUD_TO_SCAN_CONFIG),
                    ]
                ),
            )
            self.wait_until(
                "projected LaserScan /scan",
                lambda: self.topic_exists("/scan"),
                30.0,
            )
            self.start_process(
                "slam_toolbox",
                ros_argv(
                    [
                        str(SLAM_TOOLBOX_BINARY),
                        "--ros-args",
                        "--params-file",
                        str(SLAM_PARAMS),
                    ]
                ),
            )
            self.start_process(
                "vel_cmd_lcm",
                mapless_velocity_bridge_argv(),
            )
            mode_env = self.env.copy()
            mode_env["GO2W_NAV_MODE_FILE_ONLY"] = "1"
            self.run_checked(
                [str(ROOT_DIR / "scripts/set_go2w_nav_mode.sh"), "avoid"],
                "mapless RL navigation mode",
                env=mode_env,
            )
            self.wait_until(
                "online SLAM map topic /map",
                lambda: self.topic_exists("/map"),
                60.0,
            )

        self.run_checked(
            ros_argv(["ros2", "daemon", "stop"]),
            "ROS 2 CLI daemon cleanup",
        )

        if self.args.with_cosmos:
            self.ensure_cosmos_generator()
            if known_map_nav:
                self.start_process(
                    "cosmos_bridge",
                    ros_argv(
                        [
                            "/usr/bin/python3",
                            str(ROOT_DIR / "scripts/cosmos_vln_bridge.py"),
                        ]
                    ),
                )
                self.start_process(
                    "cosmos_mission",
                    ros_argv(
                        [
                            "/usr/bin/python3",
                            str(ROOT_DIR / "scripts/go2w_house_mission.py"),
                        ]
                    ),
                )
            self.start_process(
                "cosmos_visualizer",
                ros_argv(
                    [
                        "/usr/bin/python3",
                        str(ROOT_DIR / "scripts/cosmos_vln_visualizer.py"),
                    ]
                ),
            )
            self.start_process(
                "charger_search",
                ros_argv(
                    [
                        "/usr/bin/python3",
                        str(
                            ROOT_DIR
                            / "scripts/go2w_house_mapless_charger_search.py"
                        ),
                    ]
                ),
            )
            if known_map_nav:
                self.wait_until(
                    "NWM-Cosmos3Edge House route bridge",
                    lambda: self.log_contains(
                        "cosmos_bridge.log",
                        "NWM-Cosmos3Edge route-command bridge ready",
                    ),
                    30.0,
                )
                self.wait_until(
                    "House mission supervisor",
                    lambda: self.log_contains(
                        "cosmos_mission.log", "HouseWorld safety active"
                    ),
                    30.0,
                )
            self.wait_until(
                "NWM-Cosmos3Edge charger search",
                lambda: self.log_contains(
                    "charger_search.log",
                    MAPLESS_CHARGER_READY_NEEDLE,
                ),
                30.0,
            )

        if self.args.rviz:
            rviz_env = self.env.copy()
            for key in tuple(rviz_env):
                if key.startswith("SNAP") or key in {
                    "GTK_EXE_PREFIX",
                    "GTK_IM_MODULE_FILE",
                    "GTK_PATH",
                    "GTK_MODULES",
                }:
                    rviz_env.pop(key, None)
            rviz_env["QT_X11_NO_MITSHM"] = "1"
            self.start_process(
                "rviz",
                ros_argv(
                    [
                        "rviz2",
                        "-d",
                        str(
                            ROOT_DIR / "config/cosmos_vln.rviz"
                            if known_map_nav
                            else ONLINE_SLAM_RVIZ
                        ),
                    ]
                ),
                critical=False,
                env=rviz_env,
            )
            if self.args.with_cosmos:
                self.start_process(
                    "cosmos_image_view",
                    ros_argv(
                        [
                            "/usr/bin/python3",
                            str(ROOT_DIR / "scripts/cosmos_vln_image_view.py"),
                        ]
                    ),
                    critical=False,
                    env=rviz_env,
                )

        self.write_state(ready=True)
        print(
            "[HOUSE_READY] scene=HouseWorld id=6 "
            f"domain={self.args.domain_id} cosmos={int(self.args.with_cosmos)} "
            f"map_source={map_source(self.args)}",
            flush=True,
        )
        if known_map_nav:
            print(
                f"[INFO] map preview: {MAP_DIR / 'house_map_preview.png'}",
                flush=True,
            )
        else:
            print("[INFO] online SLAM map: topic=/map source=/scan", flush=True)
        print(
            f"[INFO] NWM-Cosmos3Edge visualization: "
            f"{COSMOS_DIR / 'latest_visualization.jpg'}",
            flush=True,
        )

    def supervise(self) -> None:
        next_generator_check = 0.0
        while not self.stop_requested:
            self.check_critical_processes()
            now = time.monotonic()
            if self.args.with_cosmos and now >= next_generator_check:
                self.check_cosmos_generator_health()
                next_generator_check = now + 5.0
            self.write_state(ready=True)
            time.sleep(1.0)

    def shutdown(self) -> None:
        print("[INFO] stopping HouseWorld runtime", flush=True)
        for name, process, _log, _critical in reversed(self.processes):
            if process_group_exists(process.pid):
                print(f"[INFO] SIGTERM {name} pgid={process.pid}", flush=True)
                signal_process_group(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            for _name, process, _log, _critical in self.processes:
                process.poll()
            if not any(
                process_group_exists(process.pid)
                for _name, process, _log, _critical in self.processes
            ):
                break
            time.sleep(0.1)
        for name, process, _log, _critical in reversed(self.processes):
            if process_group_exists(process.pid):
                print(f"[WARN] SIGKILL {name} pgid={process.pid}", flush=True)
                signal_process_group(process.pid, signal.SIGKILL)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
        for _name, _process, log_handle, _critical in self.processes:
            log_handle.close()
        if self.cosmos_generator_managed:
            self.cosmos_generator_info = None
            self.cosmos_generator_checked_at = time.time()
        self.write_state(ready=False)


def process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def signal_process_group(process_group_id: int, signum: int) -> None:
    try:
        os.killpg(process_group_id, signum)
    except ProcessLookupError:
        pass


def run_runtime(args: argparse.Namespace) -> int:
    ensure_layout()
    POSTURE_FILE.write_text("stand\n", encoding="ascii")
    env = base_environment(args)
    supervisor = Supervisor(args, env)
    signal.signal(signal.SIGTERM, supervisor.request_stop)
    signal.signal(signal.SIGINT, supervisor.request_stop)
    try:
        required = [
            ROS_SETUP,
            ROAMERX_SETUP,
            FORWARD_SETUP,
            FORWARD_BINARY,
            HOUSE_XML,
            HOUSE_UE_XML,
            MAPLESS_SEARCH_CONFIG,
            ROOT_DIR / "install/bin/go2w_rl_bridge",
            MATRIX_DIR / "scripts/run_sim.sh",
            ROOT_DIR / "scripts/validate_go2w_house_world.py",
            ROOT_DIR / "scripts/go2w_house_lidar_bridge.py",
            ROOT_DIR / "scripts/set_go2w_nav_mode.sh",
        ]
        if args.known_map_nav:
            required.extend(
                (
                    ROUTES_FILE,
                    NAV_PARAMS,
                    ROOT_DIR / "scripts/go2w_house_map.py",
                    ROOT_DIR / "scripts/go2w_house_navigation_strategy.py",
                    ROOT_DIR / "scripts/validate_go2w_local_avoidance.sh",
                )
            )
        else:
            required.extend(
                (
                    POINTCLOUD_TO_SCAN_CONFIG,
                    SLAM_PARAMS,
                    SLAM_TOOLBOX_BINARY,
                    ROOT_DIR / "scripts/go2w_house_pointcloud_to_laserscan.py",
                )
            )
        if args.rviz:
            required.append(
                ROOT_DIR / "config/cosmos_vln.rviz"
                if args.known_map_nav
                else ONLINE_SLAM_RVIZ
            )
        if args.with_cosmos:
            required.extend(
                (
                    COSMOS_GENERATOR_SCRIPT,
                    ROOT_DIR / "scripts/go2w_house_mapless_charger_search.py",
                    ROOT_DIR / "scripts/cosmos_vln_visualizer.py",
                )
            )
            if args.known_map_nav:
                required.extend(
                    (
                        ROOT_DIR / "scripts/cosmos_vln_bridge.py",
                        ROOT_DIR / "scripts/go2w_house_mission.py",
                    )
                )
            if args.rviz:
                required.append(ROOT_DIR / "scripts/cosmos_vln_image_view.py")
        require_files(required)
        reject_conflicting_runtime()
        prepare_house_link()
        run_preflight(env, args.known_map_nav)
        supervisor.launch()
        supervisor.supervise()
        return 0
    except (RuntimeFailure, OSError, subprocess.SubprocessError) as exc:
        print(f"[HOUSE_FAIL] {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        supervisor.shutdown()


def forwarded_run_args(args: argparse.Namespace) -> list[str]:
    forwarded = [
        "/usr/bin/python3",
        str(Path(__file__).resolve()),
        "run",
        "--domain-id",
        str(args.domain_id),
        "--initial-x",
        str(args.initial_x),
        "--initial-y",
        str(args.initial_y),
        "--initial-yaw-deg",
        str(args.initial_yaw_deg),
    ]
    forwarded.append("--offscreen" if args.offscreen else "--onscreen")
    forwarded.append("--with-cosmos" if args.with_cosmos else "--without-cosmos")
    if getattr(args, "known_map_nav", False):
        forwarded.append("--known-map-nav")
    if args.rviz:
        forwarded.append("--rviz")
    return forwarded


def start_service(args: argparse.Namespace) -> int:
    ensure_layout()
    if unit_active(UNIT):
        print(f"[ERROR] {UNIT} is already active", file=sys.stderr)
        return 1
    systemctl("reset-failed", UNIT)
    SUPERVISOR_LOG.write_text("", encoding="utf-8")

    command = [
        "systemd-run",
        "--user",
        "--quiet",
        "--unit=go2w-house-navigation",
        "--collect",
        "--property=Type=simple",
        "--property=KillMode=control-group",
        f"--property=WorkingDirectory={ROOT_DIR}",
        f"--property=StandardOutput=append:{SUPERVISOR_LOG}",
        f"--property=StandardError=append:{SUPERVISOR_LOG}",
    ]
    for key in (
        "DISPLAY",
        "XAUTHORITY",
        "CUDA_VISIBLE_DEVICES",
        "GO2W_NVIDIA_COMPAT_LIBRARY_PATH",
    ):
        if os.environ.get(key):
            command.append(f"--setenv={key}={os.environ[key]}")
    command.extend(forwarded_run_args(args))
    try:
        result = command_output(command, timeout=20.0)
    except (OSError, subprocess.SubprocessError) as exc:
        cleanup_service_after_failed_start()
        print(f"[ERROR] cannot create HouseWorld service: {exc}", file=sys.stderr)
        return 1
    if result.returncode != 0:
        cleanup_service_after_failed_start()
        print((result.stderr or result.stdout).strip(), file=sys.stderr)
        return result.returncode

    deadline = time.monotonic() + args.start_timeout
    while time.monotonic() < deadline:
        if not unit_active(UNIT):
            detail = SUPERVISOR_LOG.read_text(encoding="utf-8", errors="replace")
            print("[ERROR] HouseWorld service exited during startup", file=sys.stderr)
            print("\n".join(detail.splitlines()[-80:]), file=sys.stderr)
            cleanup_service_after_failed_start()
            return 1
        content = SUPERVISOR_LOG.read_text(encoding="utf-8", errors="replace")
        if "[HOUSE_READY]" in content:
            print(f"[OK] HouseWorld runtime is active under {UNIT}")
            print(f"[INFO] log: {SUPERVISOR_LOG}")
            if args.known_map_nav:
                print(f"[INFO] map preview: {MAP_DIR / 'house_map_preview.png'}")
            else:
                print("[INFO] online SLAM map: /map (input /scan)")
            print(f"[INFO] visualization: {COSMOS_DIR / 'latest_visualization.jpg'}")
            return 0
        time.sleep(1.0)
    print(f"[ERROR] startup timed out after {args.start_timeout:.0f}s", file=sys.stderr)
    cleanup_service_after_failed_start()
    return 1


def unit_state(unit: str) -> str:
    try:
        result = systemctl(
            "show", unit, "--property=ActiveState", "--value", timeout=5.0
        )
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    if result.returncode != 0:
        return "inactive"
    return result.stdout.strip() or "inactive"


def stop_unit_with_fallback(timeout: float = 30.0) -> tuple[bool, str]:
    detail = ""
    try:
        result = systemctl("stop", UNIT, timeout=timeout)
    except (OSError, subprocess.SubprocessError) as exc:
        result = None
        detail = str(exc)
    if result is not None and result.returncode in (0, 5):
        systemctl("reset-failed", UNIT)
        return True, ""
    if result is not None:
        detail = (result.stderr or result.stdout).strip() or (
            f"systemctl stop exited with {result.returncode}"
        )

    try:
        kill_result = systemctl(
            "kill",
            "--kill-who=all",
            "--signal=SIGKILL",
            UNIT,
            timeout=10.0,
        )
        if kill_result.returncode not in (0, 5):
            kill_detail = (kill_result.stderr or kill_result.stdout).strip()
            if kill_detail:
                detail = f"{detail}; fallback kill: {kill_detail}" if detail else kill_detail
    except (OSError, subprocess.SubprocessError) as exc:
        detail = f"{detail}; fallback kill: {exc}" if detail else str(exc)

    deadline = time.monotonic() + 10.0
    state = unit_state(UNIT)
    while state in {"active", "activating", "deactivating", "reloading"}:
        if time.monotonic() >= deadline:
            break
        time.sleep(0.2)
        state = unit_state(UNIT)
    systemctl("reset-failed", UNIT)
    if state in {"inactive", "failed"}:
        return True, detail
    return False, f"{detail}; service state is {state}" if detail else f"service state is {state}"


def cleanup_service_after_failed_start() -> None:
    stopped, detail = stop_unit_with_fallback()
    if not stopped:
        print(f"[WARN] failed-start cleanup incomplete: {detail}", file=sys.stderr)


def stop_service() -> int:
    stopped, detail = stop_unit_with_fallback()
    if not stopped:
        print(f"[ERROR] HouseWorld navigation stop failed: {detail}", file=sys.stderr)
        return 1
    if detail:
        print(f"[WARN] graceful stop required fallback cleanup: {detail}")
    print("[OK] HouseWorld navigation stopped")
    return 0


def show_status() -> int:
    active = unit_active(UNIT)
    print(f"service={UNIT} active={str(active).lower()}")
    if RUNTIME_STATE.is_file():
        print(RUNTIME_STATE.read_text(encoding="utf-8").strip())
    if not active:
        return 1
    status = systemctl(
        "show",
        UNIT,
        "--property=ActiveState,SubState,MainPID,ExecMainStatus",
    )
    if status.stdout.strip():
        print(status.stdout.strip())
    return 0


def add_runtime_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--domain-id", type=domain_id, default=90)
    parser.add_argument("--initial-x", type=float, default=1.1)
    parser.add_argument("--initial-y", type=float, default=-7.2)
    parser.add_argument("--initial-yaw-deg", type=float, default=90.0)
    screen = parser.add_mutually_exclusive_group()
    screen.add_argument("--offscreen", dest="offscreen", action="store_true")
    screen.add_argument("--onscreen", dest="offscreen", action="store_false")
    parser.set_defaults(offscreen=True)
    cosmos = parser.add_mutually_exclusive_group()
    cosmos.add_argument("--with-cosmos", dest="with_cosmos", action="store_true")
    cosmos.add_argument(
        "--without-cosmos", dest="with_cosmos", action="store_false"
    )
    parser.set_defaults(with_cosmos=True)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument(
        "--known-map-nav",
        action="store_true",
        help="use the legacy generated map + Nav2 fixed-route navigation chain",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    start = subparsers.add_parser("start", help="start a persistent user service")
    add_runtime_options(start)
    start.add_argument(
        "--start-timeout", type=positive_seconds, default=DEFAULT_START_TIMEOUT_SEC
    )
    run = subparsers.add_parser("run", help="run the foreground supervisor")
    add_runtime_options(run)
    subparsers.add_parser("stop", help="stop the HouseWorld service")
    subparsers.add_parser("status", help="show service and child process status")
    restart = subparsers.add_parser("restart", help="restart the persistent service")
    add_runtime_options(restart)
    restart.add_argument(
        "--start-timeout", type=positive_seconds, default=DEFAULT_START_TIMEOUT_SEC
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "run":
        return run_runtime(args)
    if args.command == "start":
        return start_service(args)
    if args.command == "stop":
        return stop_service()
    if args.command == "status":
        return show_status()
    if args.command == "restart":
        stop_result = stop_service()
        if stop_result != 0:
            return stop_result
        return start_service(args)
    raise AssertionError(args.command)


if __name__ == "__main__":
    raise SystemExit(main())
