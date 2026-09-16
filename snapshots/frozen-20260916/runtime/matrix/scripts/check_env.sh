#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

MODE="runtime"
ROBOT_ARG="xgb"
SCENE_ID="1"
MUJOCO_RUNNING="0"
OFFSCREEN="0"
CUSTOM_URDF=""
WARN_ONLY="${MATRIX_ENV_CHECK_WARN_ONLY:-0}"
CHECK_LDD=1

ERRORS=0
WARNINGS=0

usage() {
    cat <<'EOF'
Usage:
  scripts/check_env.sh [runtime|custom|install|local-install|build|roamerx|all] [options]

Options:
  --robot VALUE         Robot argument passed to run_sim.sh (default: xgb)
  --scene VALUE         Scene id passed to run_sim.sh (default: 1)
  --mujoco VALUE        1 if MuJoCo should be started, otherwise 0
  --offscreen VALUE     1 if UE runs offscreen, otherwise 0
  --custom-urdf PATH    Custom URDF path for custom profile
  --warn-only           Report failures but exit 0
  --skip-ldd            Do not inspect shared-library dependencies
  -h, --help            Show this help

Examples:
  scripts/check_env.sh runtime --robot xgb --scene 1 --mujoco 0
  scripts/check_env.sh custom --custom-urdf /path/to/robot.urdf
  scripts/check_env.sh local-install
EOF
}

if [[ $# -gt 0 && "${1:-}" != --* ]]; then
    MODE="$1"
    shift
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot)
            ROBOT_ARG="${2:-}"
            shift 2
            ;;
        --scene)
            SCENE_ID="${2:-}"
            shift 2
            ;;
        --mujoco)
            MUJOCO_RUNNING="${2:-}"
            shift 2
            ;;
        --offscreen)
            OFFSCREEN="${2:-}"
            shift 2
            ;;
        --custom-urdf)
            CUSTOM_URDF="${2:-}"
            shift 2
            ;;
        --warn-only)
            WARN_ONLY=1
            shift
            ;;
        --skip-ldd)
            CHECK_LDD=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[FAIL] Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

log_ok() {
    echo "[OK] $1"
}

log_warn() {
    WARNINGS=$((WARNINGS + 1))
    echo "[WARN] $1" >&2
    if [[ -n "${2:-}" ]]; then
        echo "       fix: $2" >&2
    fi
}

log_fail() {
    ERRORS=$((ERRORS + 1))
    echo "[FAIL] $1" >&2
    if [[ -n "${2:-}" ]]; then
        echo "       fix: $2" >&2
    fi
}

have_cmd() {
    command -v "$1" >/dev/null 2>&1
}

join_existing_dirs() {
    local joined=""
    local dir
    for dir in "$@"; do
        if [[ -d "$dir" ]]; then
            joined="${joined}${joined:+:}$dir"
        fi
    done
    printf '%s\n' "$joined"
}

ue_runtime_library_path() {
    join_existing_dirs \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux"
}

ros_runtime_library_path() {
    join_existing_dirs \
        "/opt/ros/humble/lib" \
        "/opt/ros/humble/lib/x86_64-linux-gnu" \
        "$PROJECT_ROOT/src/robot_mujoco/simulate/build" \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
}

require_cmd() {
    local cmd="$1"
    local hint="${2:-}"
    if have_cmd "$cmd"; then
        log_ok "command: $cmd"
    else
        log_fail "missing command: $cmd" "$hint"
    fi
}

warn_cmd() {
    local cmd="$1"
    local hint="${2:-}"
    if have_cmd "$cmd"; then
        log_ok "command: $cmd"
    else
        log_warn "missing optional command: $cmd" "$hint"
    fi
}

require_any_cmd() {
    local label="$1"
    local hint="$2"
    shift 2
    local cmd
    for cmd in "$@"; do
        if have_cmd "$cmd"; then
            log_ok "$label: $cmd"
            return 0
        fi
    done
    log_fail "missing $label: one of $*" "$hint"
}

warn_any_cmd() {
    local label="$1"
    local hint="$2"
    shift 2
    local cmd
    for cmd in "$@"; do
        if have_cmd "$cmd"; then
            log_ok "$label: $cmd"
            return 0
        fi
    done
    log_warn "missing optional $label: one of $*" "$hint"
}

require_file() {
    local path="$1"
    local hint="${2:-}"
    if [[ -f "$path" ]]; then
        log_ok "file: ${path#$PROJECT_ROOT/}"
    else
        log_fail "missing file: ${path#$PROJECT_ROOT/}" "$hint"
    fi
}

require_dir() {
    local path="$1"
    local hint="${2:-}"
    if [[ -d "$path" ]]; then
        log_ok "directory: ${path#$PROJECT_ROOT/}"
    else
        log_fail "missing directory: ${path#$PROJECT_ROOT/}" "$hint"
    fi
}

require_executable_file() {
    local path="$1"
    local hint="${2:-}"
    if [[ -x "$path" ]]; then
        log_ok "executable: ${path#$PROJECT_ROOT/}"
    elif [[ -f "$path" ]]; then
        log_warn "file exists but is not executable: ${path#$PROJECT_ROOT/}" "chmod +x ${path#$PROJECT_ROOT/}"
    else
        log_fail "missing executable: ${path#$PROJECT_ROOT/}" "$hint"
    fi
}

python_require_module() {
    local module="$1"
    local hint="$2"
    if python3 - "$module" <<'PY' >/dev/null 2>&1
import importlib.util
import sys

module = sys.argv[1]
raise SystemExit(0 if importlib.util.find_spec(module) is not None else 1)
PY
    then
        log_ok "python module: $module"
    else
        log_fail "missing python module: $module" "$hint"
    fi
}

custom_urdf_uses_reference_profile() {
    local urdf_path="$1"
    local basename_lower
    basename_lower="$(basename "$urdf_path" | tr '[:upper:]' '[:lower:]')"

    case "$basename_lower" in
        xg_b.urdf|xg-b.urdf|xgb.urdf|\
        lite3.urdf|lite3.xml|lite3|\
        xg_wheel.urdf|xg-wheel.urdf|xgw.urdf|\
        zg.urdf|zg.xml|zg|\
        zg_wheel.urdf|zg-wheel.urdf|zgw.urdf|\
        zgws.urdf|zg_wheel_spine.urdf|zg-wheel-spine.urdf|\
        go2.urdf|unitree_go2.urdf|\
        go2w.urdf|unitree_go2w.urdf)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

find_ue_binary() {
    local ue_bin_dir="$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
    local candidate
    for candidate in \
        "$ue_bin_dir/zsibot_mujoco_ue-Linux-Shipping" \
        "$ue_bin_dir/zsibot_mujoco_ue-Linux-Development" \
        "$ue_bin_dir/zsibot_mujoco_ue"
    do
        if [[ -f "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return 0
        fi
    done
    return 1
}

check_ldd_missing() {
    local binary="$1"
    local extra_ld_path="${2:-}"

    if [[ "$CHECK_LDD" != "1" ]]; then
        return 0
    fi
    if [[ ! -f "$binary" ]]; then
        return 0
    fi
    if ! have_cmd ldd; then
        log_warn "cannot inspect shared libraries because ldd is missing" "sudo apt install -y libc-bin"
        return 0
    fi

    local ldd_output
    local missing
    local version_errors

    ldd_output="$(
        LD_LIBRARY_PATH="${extra_ld_path}${extra_ld_path:+:}${LD_LIBRARY_PATH:-}" \
            ldd "$binary" 2>&1 || true
    )"
    missing="$(
        printf '%s\n' "$ldd_output" | awk '/=>[[:space:]]+not found/ {print $1}' | sort -u | tr '\n' ' '
    )"
    version_errors="$(
        printf '%s\n' "$ldd_output" | grep -E "version .+ not found" || true
    )"
    if [[ -n "$missing" ]]; then
        log_fail "missing shared libraries for ${binary#$PROJECT_ROOT/}: $missing" "Run scripts/install_deps.sh; these libraries come from deps/*.deb and are not installed by install_chunks.sh."
    elif [[ -n "$version_errors" ]]; then
        log_fail "incompatible shared library versions for ${binary#$PROJECT_ROOT/}" "Run scripts/install_deps.sh and make sure the release deps/assets match this Ubuntu version."
        printf '%s\n' "$version_errors" | sed "s#${PROJECT_ROOT}/##g; s/^/       ldd: /" >&2
    else
        log_ok "shared libraries: ${binary#$PROJECT_ROOT/}"
    fi
}

check_common_commands() {
    require_cmd bash "sudo apt install -y bash"
    require_cmd sed "sudo apt install -y sed"
    require_cmd awk "sudo apt install -y gawk"
    require_cmd grep "sudo apt install -y grep"
    require_cmd find "sudo apt install -y findutils"
    require_cmd stat "sudo apt install -y coreutils"
    require_cmd readlink "sudo apt install -y coreutils"
    require_cmd realpath "sudo apt install -y coreutils"
    require_cmd dirname "sudo apt install -y coreutils"
    require_cmd basename "sudo apt install -y coreutils"
    require_cmd cp "sudo apt install -y coreutils"
    require_cmd mv "sudo apt install -y coreutils"
    require_cmd mkdir "sudo apt install -y coreutils"
}

robot_to_runtime_name() {
    case "$1" in
        1|xgb) echo "xgb" ;;
        2|xgw) echo "xgw" ;;
        3|zgws) echo "zgws" ;;
        4|go2) echo "go2" ;;
        5|go2w) echo "go2w" ;;
        7|custom) echo "custom" ;;
        *)
            return 1
            ;;
    esac
}

robot_needs_mc() {
    case "$1" in
        4|go2|5|go2w)
            return 1
            ;;
        *)
            return 0
            ;;
    esac
}

check_runtime_env() {
    echo "[INFO] Checking MATRiX runtime environment"
    check_common_commands
    require_cmd jq "sudo apt install -y jq"
    require_cmd pkill "sudo apt install -y procps"
    require_cmd taskset "sudo apt install -y util-linux"

    require_file "$PROJECT_ROOT/config/config.json" "Restore config/config.json or reinstall the assets package."
    require_file "$PROJECT_ROOT/scene/scene.json" "Restore scene/scene.json or reinstall the assets package."
    require_file "$PROJECT_ROOT/src/robot_mujoco/simulate/config.yaml" "Install the MuJoCo runtime files from the assets package."
    require_file "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue.sh" "Install the base package or restore src/UeSim/Linux/zsibot_mujoco_ue.sh."
    require_file "$PROJECT_ROOT/src/UeSim/Linux/Engine/Content/Renderer/TessellationTable.bin" "Reinstall the assets package; UE Nanite initialization requires this Engine runtime file."

    local runtime_robot
    if runtime_robot="$(robot_to_runtime_name "$ROBOT_ARG")"; then
        if [[ "$runtime_robot" != "custom" ]]; then
            require_dir "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/$runtime_robot" "Install base-${VERSION:-0.1.2}.tar.gz or run scripts/release_manager/install_chunks_local.sh."
            require_dir "$PROJECT_ROOT/src/robot_mujoco/zsibot_robots/$runtime_robot" "Install assets/base packages so robot model files are available."
        fi
    else
        log_fail "unsupported robot argument for this release: $ROBOT_ARG" "Use one of: 1/xgb, 2/xgw, 3/zgws, 4/go2, 5/go2w, 7/custom."
    fi

    local ue_binary=""
    if ue_binary="$(find_ue_binary)"; then
        log_ok "UE binary: ${ue_binary#$PROJECT_ROOT/}"
        check_ldd_missing "$ue_binary" "$(ue_runtime_library_path)"
    else
        log_fail "missing UE binary under src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" "Install base/assets packages, then rerun this check."
    fi

    if [[ "$OFFSCREEN" != "1" && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        log_fail "no graphical display detected for UE" "Start from a desktop session, set DISPLAY, or run with offscreen=1."
    fi

    if [[ "$SCENE_ID" == "15" ]]; then
        require_file "$PROJECT_ROOT/dynamicmaps/moonworld.bin" "Install the assets package; MoonWorld needs dynamicmaps/moonworld.bin."
    fi

    if [[ "$MUJOCO_RUNNING" == "1" ]]; then
        local mujoco_bin="$PROJECT_ROOT/src/robot_mujoco/simulate/build/robot_mujoco"
        require_executable_file "$mujoco_bin" "Install the assets package or build robot_mujoco."
        check_ldd_missing "$mujoco_bin" "$(ros_runtime_library_path)"
        if [[ ! -f /opt/ros/humble/setup.bash ]]; then
            log_fail "ROS 2 Humble setup file not found: /opt/ros/humble/setup.bash" "Install ROS 2 Humble before enabling MuJoCo runtime."
        else
            log_ok "ROS 2 Humble: /opt/ros/humble/setup.bash"
        fi
    fi

    if robot_needs_mc "$ROBOT_ARG"; then
        local mc_bin="$PROJECT_ROOT/src/robot_mc/build/export/mc/bin/mc_ctrl"
        require_file "$PROJECT_ROOT/src/robot_mc/run_mc.sh" "Restore src/robot_mc/run_mc.sh."
        require_executable_file "$mc_bin" "Install the assets package; it should provide src/robot_mc/build/export/mc/bin/mc_ctrl."
        check_ldd_missing "$mc_bin" "$PROJECT_ROOT/src/robot_mc/build/export/mc/bin"
    fi
}

check_custom_env() {
    echo "[INFO] Checking custom URDF environment"
    check_common_commands
    require_cmd python3 "sudo apt install -y python3 python3-pip"
    require_cmd jq "sudo apt install -y jq"
    require_file "$PROJECT_ROOT/scripts/validate_xml_contract.py" "Restore scripts/validate_xml_contract.py."

    if [[ -n "$CUSTOM_URDF" ]]; then
        require_file "$CUSTOM_URDF" "Pass an existing URDF path to run_custom_urdf.sh."
        if custom_urdf_uses_reference_profile "$CUSTOM_URDF"; then
            log_ok "custom URDF uses a built-in reference profile; urdf2mjcf is not required"
        else
            python_require_module "urdf2mjcf" "python3 -m pip install urdf2mjcf"
        fi
    else
        python_require_module "urdf2mjcf" "python3 -m pip install urdf2mjcf"
    fi
}

check_install_env() {
    echo "[INFO] Checking release installer environment"
    check_common_commands
    require_cmd tar "sudo apt install -y tar"
    require_cmd gzip "sudo apt install -y gzip"
    require_cmd sha256sum "sudo apt install -y coreutils"
    warn_cmd git "sudo apt install -y git"
    warn_cmd jq "sudo apt install -y jq"
    warn_any_cmd "download command" "sudo apt install -y aria2 wget curl" aria2c axel wget curl
}

check_local_install_env() {
    echo "[INFO] Checking local release installer environment"
    check_common_commands
    require_cmd tar "sudo apt install -y tar"
    require_cmd gzip "sudo apt install -y gzip"
    require_cmd sha256sum "sudo apt install -y coreutils"
    warn_cmd jq "sudo apt install -y jq"
}

check_build_env() {
    echo "[INFO] Checking build environment"
    check_common_commands
    require_cmd gcc "sudo apt install -y gcc"
    require_cmd g++ "sudo apt install -y g++"
    require_cmd cmake "sudo apt install -y cmake"
    require_cmd make "sudo apt install -y make"
    require_cmd qmake "sudo apt install -y qt5-qmake qtbase5-dev-tools"
    require_cmd protoc "sudo apt install -y protobuf-compiler"
    warn_cmd pkg-config "sudo apt install -y pkg-config"
}

check_roamerx_env() {
    echo "[INFO] Checking RoamerX link environment"
    check_common_commands
    require_cmd ros2 "Install and source ROS 2 Humble: source /opt/ros/humble/setup.bash"
    require_cmd rviz2 "sudo apt install -y ros-humble-rviz2"
    warn_cmd ss "sudo apt install -y iproute2"
    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        log_fail "ROS 2 Humble setup file not found: /opt/ros/humble/setup.bash" "Install ROS 2 Humble and source it before starting RoamerX link."
    else
        log_ok "ROS 2 Humble: /opt/ros/humble/setup.bash"
    fi
}

case "$MODE" in
    runtime)
        check_runtime_env
        ;;
    custom)
        check_custom_env
        ;;
    install)
        check_install_env
        ;;
    local-install)
        check_local_install_env
        ;;
    build)
        check_build_env
        ;;
    roamerx)
        check_roamerx_env
        ;;
    all)
        check_runtime_env
        check_custom_env
        check_install_env
        check_build_env
        ;;
    *)
        echo "[FAIL] Unknown mode: $MODE" >&2
        usage >&2
        exit 2
        ;;
esac

echo "[INFO] Environment check finished: errors=$ERRORS warnings=$WARNINGS"

if [[ "$ERRORS" -gt 0 && "$WARN_ONLY" != "1" ]]; then
    exit 1
fi
exit 0
