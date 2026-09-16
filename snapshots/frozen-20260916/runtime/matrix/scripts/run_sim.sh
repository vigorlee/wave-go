#!/usr/bin/env bash
set -euo pipefail

#######################################
# 基础
#######################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

ROBOT_ARG="${1:-xgb}"
SCENE_ID="${2:-1}"
OFFSCREEN="${3:-0}"
PIXELSTREAM="${4:-0}"
MUJOCORUNNING="${5:-0}"
CUSTOM_URDF="${6:-}"
CUSTOM_NAME="${7:-}"

SIM_LAUNCHER_ROOT="${SIM_LAUNCHER_ROOT:-$PROJECT_ROOT}"
CUSTOM_WRAPPER="$SIM_LAUNCHER_ROOT/scripts/run_custom_urdf.sh"

join_ld_library_path() {
    local joined=""
    local dir
    for dir in "$@"; do
        if [[ -d "$dir" ]]; then
            joined="${joined}${joined:+:}$dir"
        fi
    done
    if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
        joined="${joined}${joined:+:}${LD_LIBRARY_PATH}"
    fi
    printf '%s\n' "$joined"
}

setup_runtime_environment() {
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        set +u
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        set -u
    fi
}

mujoco_ld_library_path() {
    join_ld_library_path \
        "$PROJECT_ROOT/src/robot_mujoco/simulate/build" \
        "/opt/ros/humble/lib" \
        "/opt/ros/humble/lib/x86_64-linux-gnu" \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux"
}

ue_ld_library_path() {
    join_ld_library_path \
        "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Binaries/Linux" \
        "$PROJECT_ROOT/src/UeSim/Linux/Engine/Plugins/Runtime/OpenCV/Binaries/ThirdParty/Linux"
}

mc_ld_library_path() {
    join_ld_library_path "$PROJECT_ROOT/src/robot_mc/build/export/mc/bin"
}

setup_runtime_environment

if [[ "${SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER:-0}" != "1" ]] && [[ "$ROBOT_ARG" == "custom" || "$ROBOT_ARG" == "7" ]] && [[ -n "$CUSTOM_URDF" ]]; then
    if [[ -f "$CUSTOM_WRAPPER" ]]; then
        echo "[INFO] Delegating custom URDF setup to $CUSTOM_WRAPPER"
        exec "$CUSTOM_WRAPPER" "$ROBOT_ARG" "$SCENE_ID" "$OFFSCREEN" "$PIXELSTREAM" "$MUJOCORUNNING" "$CUSTOM_URDF" "$CUSTOM_NAME"
    else
        echo "[ERROR] Custom URDF wrapper not found at: $CUSTOM_WRAPPER" >&2
        exit 1
    fi
fi

run_env_check() {
    if [[ "${MATRIX_SKIP_ENV_CHECK:-0}" == "1" ]]; then
        echo "[INFO] Environment check skipped by MATRIX_SKIP_ENV_CHECK=1"
        return 0
    fi

    local checker="$PROJECT_ROOT/scripts/check_env.sh"
    if [[ ! -x "$checker" ]]; then
        echo "[WARN] Environment checker not found or not executable: $checker"
        return 0
    fi

    "$checker" runtime \
        --robot "$ROBOT_ARG" \
        --scene "$SCENE_ID" \
        --mujoco "$MUJOCORUNNING" \
        --offscreen "$OFFSCREEN"
}

run_env_check

#######################################
# 全局 PID 管理
#######################################

PROCESS_PATTERNS=(
    "robot_mujoco"
    "jszr_mujoco_ue"
    "zsibot_mujoco_ue"
    "UnrealGame"
    "UE4Editor"
    "mc_ctrl"
)

kill_known_processes() {
    if [[ "${MATRIX_SKIP_STALE_PROCESS_CLEANUP:-0}" == "1" ]]; then
        # Robot-specific launchers already performed an exact preflight.  The
        # legacy broad pkill patterns can otherwise match a supervising shell
        # and abort a clean Nezha2 demo before the simulator writes its log.
        return 0
    fi
    local signal="$1"
    local pattern
    for pattern in "${PROCESS_PATTERNS[@]}"; do
        pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
    done
}

kill_known_processes TERM


PIDS=()
PROCESS_LABELS=()
WATCHDOG_PID=""
FORCED_CLEANUP_PID=""

schedule_forced_cleanup() {
    (
        trap '' HUP
        sleep 1
        kill_known_processes TERM
        sleep 1
        kill_known_processes KILL
    ) </dev/null >/dev/null 2>&1 &
    FORCED_CLEANUP_PID=$!
}

start_parent_watchdog() {
    local parent_pid="$$"
    (
        trap 'exit 0' TERM INT
        trap '' HUP
        while kill -0 "${parent_pid}" 2>/dev/null; do
            sleep 1
        done

        echo "[INFO] Parent launcher process exited unexpectedly, cleaning child processes..."
        schedule_forced_cleanup
        kill_known_processes TERM
    ) &
    WATCHDOG_PID=$!
}

stop_parent_watchdog() {
    if [[ -n "${WATCHDOG_PID:-}" ]] && kill -0 "${WATCHDOG_PID}" 2>/dev/null; then
        kill -TERM "${WATCHDOG_PID}" 2>/dev/null || true
        wait "${WATCHDOG_PID}" 2>/dev/null || true
    fi
}

cleanup() {
    echo "[INFO] ===== Cleaning up processes ====="

    stop_parent_watchdog
    schedule_forced_cleanup

    # 1. 优雅关闭脚本启动的进程
    for pid in "${PIDS[@]:-}"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            echo "[INFO] SIGTERM PID $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    # 2. 兜底清理（仅限本项目）
    kill_known_processes TERM

    # 3. 最终兜底
    kill_known_processes KILL

    if [[ -n "${FORCED_CLEANUP_PID:-}" ]] && kill -0 "${FORCED_CLEANUP_PID}" 2>/dev/null; then
        kill -TERM "${FORCED_CLEANUP_PID}" 2>/dev/null || true
        wait "${FORCED_CLEANUP_PID}" 2>/dev/null || true
    fi

    echo "[INFO] ===== Cleanup finished ====="
}
trap cleanup EXIT SIGINT SIGTERM SIGHUP
start_parent_watchdog

#######################################
# Offscreen / PixelStreaming
#######################################
USE_OFFSCREEN=""
[[ "$OFFSCREEN" == "1" ]] && USE_OFFSCREEN="-RenderOffScreen"

USE_PIXELSTREAMER=""
[[ "$PIXELSTREAM" == "1" ]] && USE_PIXELSTREAMER="-PixelStreamingURL=ws://127.0.0.1:8888"

UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-30}"
UE_RES_X="${MATRIX_UE_RES_X:-1600}"
UE_RES_Y="${MATRIX_UE_RES_Y:-900}"
UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-0}"
UE_SCREEN_PERCENTAGE="${MATRIX_UE_SCREEN_PERCENTAGE:-75}"
UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-0}"
UE_EXEC_CMDS="t.MaxFPS ${UE_MAX_FPS}"
UE_EXEC_CMDS+="${MATRIX_UE_EXTRA_EXEC_COMMANDS:+,${MATRIX_UE_EXTRA_EXEC_COMMANDS}}"
UE_EXTRA_ARGS=()

if [[ "${UE_PERFORMANCE_PROFILE}" == "1" ]]; then
    if ! [[ "${UE_SCREEN_PERCENTAGE}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "[WARN] Invalid MATRIX_UE_SCREEN_PERCENTAGE='${UE_SCREEN_PERCENTAGE}', using 75" >&2
        UE_SCREEN_PERCENTAGE=75
    fi
    UE_EXEC_CMDS+=",sg.ViewDistanceQuality 1,sg.AntiAliasingQuality 1,sg.ShadowQuality 1,sg.GlobalIlluminationQuality 1,sg.ReflectionQuality 1,sg.PostProcessQuality 1,sg.TextureQuality 2,sg.EffectsQuality 1,sg.FoliageQuality 1,r.ScreenPercentage ${UE_SCREEN_PERCENTAGE},r.RayTracing 0,r.VSync 0"
    UE_EXTRA_ARGS+=("-ResX=${UE_RES_X}" "-ResY=${UE_RES_Y}" -windowed -nosplash)
fi

if [[ "${UE_NO_RHI_THREAD}" == "1" ]]; then
    UE_EXTRA_ARGS+=(-norhithread)
fi

#######################################
# 场景配置
#######################################
SCENE="scene_terrain_wh.xml"
MAPNAME="/Game/Maps/SceneWorld"
WEAPON=""

case "$SCENE_ID" in
    0)  SCENE="scene_terrain_custom.xml"; MAPNAME="/Game/Maps/CustomWorld" ;;
    1)  SCENE="scene_terrain_wh.xml";     MAPNAME="/Game/Maps/SceneWorld" ;;
    2)  SCENE="scene_terrain_t10.xml";    MAPNAME="/Game/Maps/Town10World" ;;
    3)  SCENE="scene_terrain_yard.xml";   MAPNAME="/Game/Maps/YardWorld" ;;
    4)  SCENE="scene_terrain_crowd.xml";  MAPNAME="/Game/Maps/CrowdWorld" ;;
    5)  SCENE="scene_terrain_venice.xml"; MAPNAME="/Game/Maps/VeniceWorld" ;;
    6)  SCENE="scene_terrain_house.xml";  MAPNAME="/Game/Maps/HouseWorld" ;;
    7)  SCENE="scene_terrain_rw.xml";     MAPNAME="/Game/Maps/RunningWorld" ;;
    8)  SCENE="scene_terrain_zombie.xml"; MAPNAME="/Game/Maps/Town10Zombie"; WEAPON="gun" ;;
    9)  SCENE="scene_terrain_flat.xml";   MAPNAME="/Game/Maps/IROSFlatWorld" ;;
    10) SCENE="scene_terrain_sloped.xml"; MAPNAME="/Game/Maps/IROSSlopedWorld" ;;
    11) SCENE="scene_terrain_flat25.xml"; MAPNAME="/Game/Maps/IROSFlatWorld2025" ;;
    12) SCENE="scene_terrain_sloped25.xml"; MAPNAME="/Game/Maps/IROSSloppedWorld2025" ;;
    13) SCENE="scene_terrain_office.xml"; MAPNAME="/Game/Maps/OfficeWorld" ;;
    14) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    16) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    17) SCENE="3dgs.xml";                 MAPNAME="/Game/Maps/3DGSWorld" ;;
    15)
        SCENE="scene_terrain_moon_dynamic.xml"
        MAPNAME="/Game/Maps/MoonWorld"
        mkdir -p src/robot_mujoco/simulate/build src/UeSim/Linux/zsibot_mujoco_ue/Content/model/dynamicmap
        cp dynamicmaps/moonworld.bin src/robot_mujoco/simulate/build/DynamicMapData.bin
        cp dynamicmaps/moonworld.bin src/UeSim/Linux/zsibot_mujoco_ue/Content/model/dynamicmap/moonworld.bin
        ;;
    20) SCENE="scene_terrain_cali.xml"; MAPNAME="/Game/Maps/CaliWorld" ;;
    21) SCENE="scene_terrain_apart2.xml"; MAPNAME="/Game/Maps/ApartmentWorld" ;;
    22) SCENE="scene_terrain_meet.xml"; MAPNAME="/Game/Maps/MeetRoomWorld" ;;
    *)
        echo "[WARN] Unknown scene id $SCENE_ID, using default"
        ;;
esac

MUJOCO_SCENE="${MATRIX_MUJOCO_SCENE:-$SCENE}"
sed -i "s|^robot_scene: .*|robot_scene: \"$MUJOCO_SCENE\"|" src/robot_mujoco/simulate/config.yaml

ROBOT_INITIAL_X="${MATRIX_ROBOT_INITIAL_X:-$(jq -r '.robot.position.x // 0' config/config.json)}"
ROBOT_INITIAL_Y="${MATRIX_ROBOT_INITIAL_Y:-$(jq -r '.robot.position.y // 0' config/config.json)}"
ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-$(jq -r '.robot.rotation.yaw // 0' config/config.json)}"

#######################################
# 机器人类型 & 启动策略
#######################################
TARGET_FILE="src/robot_mc/run_mc.sh"
ENABLE_MUJOCO=false
ENABLE_MC=false
ROBOTTYPE="xgb"
RUNTIME_ROBOTTYPE="xgb"

# MUJOCORUNNING is 1 config/config.json中"mujoco_running": true，否则为 false
if [[ "$MUJOCORUNNING" == "1" ]]; then
    ENABLE_MUJOCO=true
    echo "[INFO] MuJoCo will be enabled. Please ensure you have the proper license and setup."
else
    ENABLE_MUJOCO=false
    echo "[INFO] MuJoCo will be disabled. The simulation will run without physics-based dynamics."
fi


case "$ROBOT_ARG" in
    4|go2)
        ROBOTTYPE="go2"
        RUNTIME_ROBOTTYPE="go2"
        ENABLE_MC=false
        # sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=GO2/' "$TARGET_FILE"
        ;;
    5|go2w)
        ROBOTTYPE="go2w"
        RUNTIME_ROBOTTYPE="go2w"
        ENABLE_MC=false
        # sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=GO2W/' "$TARGET_FILE"
        ;;
    1|xgb)
        ROBOTTYPE="xgb"
        RUNTIME_ROBOTTYPE="xgb"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XG/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg-user-parameters.yaml
        fi
        ;;
    2|xgw)
        ROBOTTYPE="xgw"
        RUNTIME_ROBOTTYPE="xgw"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XGW/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
        fi
        ;;
    3|zgws)
        ROBOTTYPE="zgws"
        RUNTIME_ROBOTTYPE="zgws"
        ENABLE_MC=true
        sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=ZGWS/' "$TARGET_FILE"
        if [[ "$MUJOCORUNNING" == "1" ]]; then
            ENABLE_MUJOCO=true
            sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml
        else
            ENABLE_MUJOCO=false
            sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml
        fi
        ;;
    6|xxg)
        echo "[ERROR] Robot type '$ROBOT_ARG' is not included in this release" >&2
        exit 1
        ;;
    7|custom)
        ROBOTTYPE="custom"
        RUNTIME_ROBOTTYPE="custom"
        ENABLE_MC=true
        # Read reference_profile from manifest to select the correct MC config
        _CUSTOM_MODEL_DIR="${CUSTOM_NAME:-custom}"
        _MANIFEST="src/robot_mujoco/zsibot_robots/custom/_cache/${_CUSTOM_MODEL_DIR}/manifest.json"
        _REF_PROFILE=""
        if [[ -f "$_MANIFEST" ]]; then
            _REF_PROFILE="$(jq -r '.reference_profile // empty' "$_MANIFEST" 2>/dev/null || true)"
        fi
        echo "[INFO] custom robot reference_profile: '${_REF_PROFILE:-none}'"
        if [[ -n "$_REF_PROFILE" ]]; then
            # Keep custom scene/layout handling, but expose the matched native
            # robot type to downstream runtime config.
            RUNTIME_ROBOTTYPE="$_REF_PROFILE"
        fi
        case "${_REF_PROFILE}" in
            xgw|zgw)
                # 16-DOF wheel-leg (xgw/zgw) → XGW MC config
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XGW/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml
                fi
                ;;
            xxg)
                # XXG family → XXG MC config
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XXG/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xxg-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xxg-user-parameters.yaml
                fi
                ;;
            *)
                # xgb / generic / unknown → XG MC config (default)
                sed -i 's/export ROBOT_TYPE=.*/export ROBOT_TYPE=XG/' "$TARGET_FILE"
                if [[ "$MUJOCORUNNING" == "1" ]]; then
                    ENABLE_MUJOCO=true
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 5/' src/robot_mc/build/export/config/xg-user-parameters.yaml
                else
                    ENABLE_MUJOCO=false
                    sed -i 's/motor_platform_type: .*/motor_platform_type: 8/' src/robot_mc/build/export/config/xg-user-parameters.yaml
                fi
                ;;
        esac
        ;;
    *)
        echo "[ERROR] Unknown robot type: $ROBOT_ARG"
        exit 1
        ;;
esac

# Robot-specific RL bridges publish their own low-level MuJoCo commands.  The
# bundled MC executable must not run concurrently, otherwise its generic XG
# policy writes a second command stream to the same simulated robot.
if [[ "${MATRIX_DISABLE_MC:-0}" == "1" ]]; then
    ENABLE_MC=false
    echo "[INFO] MATRiX motor controller disabled; external robot-specific controller owns low-level commands."
fi

MUJOCO_ROBOTTYPE="${MATRIX_MUJOCO_ROBOT_TYPE:-$ROBOTTYPE}"
sed -i "s/^robot: .*/robot: \"$MUJOCO_ROBOTTYPE\"/" src/robot_mujoco/simulate/config.yaml

#######################################
# JSON 同步
#######################################
MUJOCO_RUNNING_JSON=false
if $ENABLE_MUJOCO; then
    MUJOCO_RUNNING_JSON=true
fi

CONFIG_TMP="$(mktemp)"
jq \
    --arg robot_type "$ROBOTTYPE" \
    --arg weapon "$WEAPON" \
    --arg custom_name "$CUSTOM_NAME" \
    --argjson mujoco_running "$MUJOCO_RUNNING_JSON" \
    --argjson performance_profile "${UE_PERFORMANCE_PROFILE}" \
    --argjson camera_width "${MATRIX_CAMERA_WIDTH:-1280}" \
    --argjson camera_height "${MATRIX_CAMERA_HEIGHT:-720}" \
    --argjson camera_frequency "${MATRIX_CAMERA_FREQUENCY:-5}" \
    --argjson camera_x "${MATRIX_CAMERA_X:-29}" \
    --argjson camera_y "${MATRIX_CAMERA_Y:-0}" \
    --argjson camera_z "${MATRIX_CAMERA_Z:-1}" \
    --argjson camera_roll "${MATRIX_CAMERA_ROLL:-0}" \
    --argjson camera_pitch "${MATRIX_CAMERA_PITCH:-15}" \
    --argjson camera_yaw "${MATRIX_CAMERA_YAW:-0}" \
    --argjson depth_width "${MATRIX_DEPTH_WIDTH:-320}" \
    --argjson depth_height "${MATRIX_DEPTH_HEIGHT:-240}" \
    --argjson depth_frequency "${MATRIX_DEPTH_FREQUENCY:-5}" \
    --argjson lidar_x "${MATRIX_LIDAR_X:-$(jq -r '.robot.sensors.lidar.position.x // 13.011' config/config.json)}" \
    --argjson lidar_y "${MATRIX_LIDAR_Y:-$(jq -r '.robot.sensors.lidar.position.y // 2.329' config/config.json)}" \
    --argjson lidar_z "${MATRIX_LIDAR_Z:-$(jq -r '.robot.sensors.lidar.position.z // 17.598' config/config.json)}" \
    --argjson lidar_roll "${MATRIX_LIDAR_ROLL:-$(jq -r '.robot.sensors.lidar.rotation.roll // 0' config/config.json)}" \
    --argjson lidar_pitch "${MATRIX_LIDAR_PITCH:-$(jq -r '.robot.sensors.lidar.rotation.pitch // 0' config/config.json)}" \
    --argjson lidar_yaw "${MATRIX_LIDAR_YAW:-$(jq -r '.robot.sensors.lidar.rotation.yaw // 0' config/config.json)}" \
    --argjson lidar_frequency "${MATRIX_LIDAR_FREQUENCY:-$(jq -r '.robot.sensors.lidar.frequency // 10' config/config.json)}" \
    --argjson ego_view "${MATRIX_UE_EGO_VIEW:-1}" \
    --argjson robot_initial_x "${ROBOT_INITIAL_X}" \
    --argjson robot_initial_y "${ROBOT_INITIAL_Y}" \
    --argjson robot_initial_yaw_deg "${ROBOT_INITIAL_YAW_DEG}" \
    '
    .robot = (.robot // {})
    | .robot.robot_type = $robot_type
    | .robot.weapon = $weapon
    | if $robot_type == "custom" and $custom_name != "" then
        .robot.use_custom_urdf = true
        | .robot.custom_urdf = "custom/scene_terrain_custom.xml"
        | .robot.custom_name = $custom_name
      else . end
    | .robot.mujoco_running = $mujoco_running
    | .robot.state_port = (.robot.state_port // 25001)
    | .robot.cmd_port = (.robot.cmd_port // 25002)
    | .robot.EgoView = $ego_view
    | .robot.position = (.robot.position // {"x": 0, "y": 0, "z": 0})
    | .robot.position.x = $robot_initial_x
    | .robot.position.y = $robot_initial_y
    | .robot.rotation = (.robot.rotation // {"roll": 0, "pitch": 0, "yaw": 0})
    | .robot.rotation.yaw = $robot_initial_yaw_deg
    | if $performance_profile == 1 then
        .robot.sensors.camera.width = $camera_width
        | .robot.sensors.camera.height = $camera_height
        | .robot.sensors.camera.frequency = $camera_frequency
        | .robot.sensors.camera.position.x = $camera_x
        | .robot.sensors.camera.position.y = $camera_y
        | .robot.sensors.camera.position.z = $camera_z
        | .robot.sensors.camera.rotation.roll = $camera_roll
        | .robot.sensors.camera.rotation.pitch = $camera_pitch
        | .robot.sensors.camera.rotation.yaw = $camera_yaw
        | .robot.sensors.depth_sensor.width = $depth_width
        | .robot.sensors.depth_sensor.height = $depth_height
        | .robot.sensors.depth_sensor.frequency = $depth_frequency
        | .robot.sensors.lidar.position.x = $lidar_x
        | .robot.sensors.lidar.position.y = $lidar_y
        | .robot.sensors.lidar.position.z = $lidar_z
        | .robot.sensors.lidar.rotation.roll = $lidar_roll
        | .robot.sensors.lidar.rotation.pitch = $lidar_pitch
        | .robot.sensors.lidar.rotation.yaw = $lidar_yaw
        | .robot.sensors.lidar.frequency = $lidar_frequency
      else . end
    ' config/config.json > "$CONFIG_TMP" && mv "$CONFIG_TMP" config/config.json

mkdir -p src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config
mkdir -p src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder
SCENE_JSON_SOURCE="scene/scene.json"
if [[ -n "${MATRIX_SCENE_JSON_OVERRIDE:-}" ]]; then
    if [[ ! -f "${MATRIX_SCENE_JSON_OVERRIDE}" ]]; then
        echo "[ERROR] MATRiX scene JSON override not found: ${MATRIX_SCENE_JSON_OVERRIDE}" >&2
        exit 1
    fi
    SCENE_JSON_SOURCE="${MATRIX_SCENE_JSON_OVERRIDE}"
    echo "[INFO] Using custom scene JSON override: ${SCENE_JSON_SOURCE}"
fi
cp config/config.json src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json
cp "${SCENE_JSON_SOURCE}" src/UeSim/Linux/zsibot_mujoco_ue/Content/model/SceneLoder/scene.json

if [[ "${MATRIX_DISABLE_IMAGE_SENSORS:-0}" == "1" ]]; then
    UE_CONFIG="src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json"
    CONFIG_TMP="$(mktemp)"
    jq 'del(.robot.sensors.camera, .robot.sensors.depth_sensor)' \
        "$UE_CONFIG" > "$CONFIG_TMP"
    mv "$CONFIG_TMP" "$UE_CONFIG"
    echo "[INFO] Disabled UE RGB/depth capture; viewport and LiDAR remain enabled."
fi

#######################################
# UE 场景入口同步
#######################################
# UE 运行时会从固定入口文件读取模型布局：
# - 非 custom 机器人: Content/model/<runtime_robot>/scene_terrain.xml
# - custom 机器人:   Content/model/custom/scene_terrain_custom.xml
# launcher 选中的场景变体需要同步覆盖到该入口，否则 UE 会继续读取默认场景。
sync_ue_runtime_scene() {
    local ue_model_root="src/UeSim/Linux/zsibot_mujoco_ue/Content/model"

    if [[ "$ROBOTTYPE" == "custom" ]]; then
        local custom_scene_entry="$ue_model_root/custom/scene_terrain_custom.xml"
        # SceneWorld and other UE maps select their model by the scene
        # variant name (for example scene_terrain_wh.xml).  Keep the fixed
        # custom entry for compatibility and mirror the same Nezha2 scene to
        # the active variant so the renderer never falls back to a proxy.
        local custom_scene_variant="$ue_model_root/custom/$SCENE"
        if [[ -n "${MATRIX_UE_CUSTOM_SCENE:-}" ]]; then
            if [[ ! -f "${MATRIX_UE_CUSTOM_SCENE}" ]]; then
                echo "[ERROR] MATRiX custom UE scene override not found: ${MATRIX_UE_CUSTOM_SCENE}" >&2
                exit 1
            fi
            if [[ -n "${MATRIX_UE_CUSTOM_INCLUDE_REWRITE:-}" ]]; then
                # A custom robot may reuse a robot-specific scene source.
                # Rewrite only its MJCF include for the fixed custom entry;
                # this keeps the UE model, physical scene, and asset paths in
                # one consistent robot family instead of falling back to an
                # unrelated custom proxy model.
                CUSTOM_SCENE_TMP="$(mktemp)"
                sed "s|<include file=\"[^\"]*\" />|<include file=\"${MATRIX_UE_CUSTOM_INCLUDE_REWRITE}\" />|" \
                    "${MATRIX_UE_CUSTOM_SCENE}" >"${CUSTOM_SCENE_TMP}"
                mv "${CUSTOM_SCENE_TMP}" "$custom_scene_entry"
            else
                cp "${MATRIX_UE_CUSTOM_SCENE}" "$custom_scene_entry"
            fi
            if [[ "$custom_scene_variant" != "$custom_scene_entry" ]]; then
                cp "$custom_scene_entry" "$custom_scene_variant"
            fi
            echo "[INFO] Synced custom UE scene override: ${MATRIX_UE_CUSTOM_SCENE} -> $custom_scene_entry"
            return
        fi
        if [[ "$SCENE" != "scene_terrain_custom.xml" ]]; then
            echo "[WARN] Custom runtime uses fixed entry custom/scene_terrain_custom.xml; requested '$SCENE' is not available for active custom layout"
            return
        fi
        if [[ -f "$custom_scene_entry" ]]; then
            echo "[INFO] Custom runtime scene entry ready: $custom_scene_entry"
        else
            echo "[WARNING] Custom runtime scene entry not found: $custom_scene_entry"
        fi
        return
    fi

    local runtime_dir="$ue_model_root/$RUNTIME_ROBOTTYPE"
    local source_scene="$runtime_dir/$SCENE"
    if [[ -n "${MATRIX_UE_SCENE_OVERRIDE:-}" ]]; then
        source_scene="${MATRIX_UE_SCENE_OVERRIDE}"
    fi
    local target_scene="$runtime_dir/scene_terrain.xml"
    # YardWorld is selected by MAPNAME and the UE loader resolves the map
    # variant (for example scene_terrain_yard.xml) directly.  Keep the fixed
    # compatibility entry above, but also synchronize the active variant when
    # a scene override is supplied; otherwise UE silently renders the stock
    # YardWorld while MuJoCo uses the extended XML.
    local target_variant="$runtime_dir/$SCENE"

    if [[ ! -d "$runtime_dir" ]]; then
        echo "[WARNING] UE runtime model directory not found: $runtime_dir"
        return
    fi
    if [[ ! -f "$source_scene" ]]; then
        echo "[WARNING] UE scene variant not found: $source_scene"
        return
    fi
    if [[ ! "$source_scene" -ef "$target_scene" ]]; then
        cp "$source_scene" "$target_scene"
        echo "[INFO] Synced UE runtime scene: $source_scene -> $target_scene"
    else
        echo "[INFO] UE runtime scene already points to: $target_scene"
    fi
    if [[ ! "$source_scene" -ef "$target_variant" ]]; then
        cp "$source_scene" "$target_variant"
        echo "[INFO] Synced UE active map variant: $source_scene -> $target_variant"
    fi
}

sync_ue_runtime_scene

# Optional upper-layer contract check. Robot-specific launchers use this to
# stop before physics starts if the active world belongs to another robot.
if [[ -n "${MATRIX_WORLD_MODEL_VALIDATOR:-}" ]]; then
    if [[ ! -x "${MATRIX_WORLD_MODEL_VALIDATOR}" ]]; then
        echo "[ERROR] World-model validator is not executable: ${MATRIX_WORLD_MODEL_VALIDATOR}" >&2
        exit 1
    fi
    "${MATRIX_WORLD_MODEL_VALIDATOR}"
fi

#######################################
# 机器人初始位姿
#######################################
ROBOT_X=$(jq -r '.robot.position.x' config/config.json)
ROBOT_Y=$(jq -r '.robot.position.y' config/config.json)
ROBOT_YAW_DEG=$(jq -r '.robot.rotation.yaw // 0' config/config.json)
read -r ROBOT_QW ROBOT_QZ < <(
    awk -v yaw_deg="${ROBOT_YAW_DEG}" 'BEGIN {
        pi = atan2(0, -1);
        half_yaw = yaw_deg * pi / 360.0;
        printf "%.12f %.12f\n", cos(half_yaw), sin(half_yaw);
    }'
)

if [[ "$ROBOTTYPE" == "custom" ]]; then
    CUSTOM_MODEL_DIR="${CUSTOM_NAME:-custom}"
    XML_FILE="src/robot_mujoco/zsibot_robots/custom/_cache/${CUSTOM_MODEL_DIR}/${CUSTOM_MODEL_DIR}.xml"
    if [[ -f "$XML_FILE" ]]; then
        echo "[INFO] Custom robot detected, skipping built-in XML position update for ${XML_FILE}"
    else
        echo "[WARNING] Custom robot XML not found: $XML_FILE"
    fi
else
    XML_FILE="src/robot_mujoco/zsibot_robots/${ROBOTTYPE}/${ROBOTTYPE}.xml"
    sed -E -i \
        "s|<body name=\"base_link\" pos=\"[^\"]*\"( quat=\"[^\"]*\")?|<body name=\"base_link\" pos=\"${ROBOT_X} ${ROBOT_Y} 0.65\" quat=\"${ROBOT_QW} 0 0 ${ROBOT_QZ}\"|" \
        "$XML_FILE"
    echo "[INFO] Robot initial pose: x=${ROBOT_X}, y=${ROBOT_Y}, yaw=${ROBOT_YAW_DEG} deg"
fi

#######################################
# 启动流程
#######################################
echo "[INFO] Starting processes..."

cd src/robot_mujoco/simulate/build
if $ENABLE_MUJOCO; then
    echo "[INFO] Starting MuJoCo"
    MUJOCO_ENV=()
    if [[ "${GO2W_MJ_STATE_RELAY:-0}" == "1" ]]; then
        TEE_SOURCE="${PROJECT_ROOT}/../scripts/go2w_state_tee.c"
        TEE_LIBRARY="${PROJECT_ROOT}/../.run/libgo2w_state_tee.so"
        mkdir -p "$(dirname "${TEE_LIBRARY}")"
        if [[ ! -f "${TEE_LIBRARY}" || "${TEE_SOURCE}" -nt "${TEE_LIBRARY}" ]]; then
            cc -O2 -Wall -Wextra -shared -fPIC "${TEE_SOURCE}" -o "${TEE_LIBRARY}" -ldl -pthread
        fi
        MUJOCO_ENV+=("LD_PRELOAD=${TEE_LIBRARY}${LD_PRELOAD:+:${LD_PRELOAD}}")
    fi
    MUJOCO_TRACE=()
    if [[ "${MATRIX_MUJOCO_EXIT_TRACE:-0}" == "1" ]]; then
        MUJOCO_TRACE=(strace -ff -tt -k -e trace=exit_group,kill -e signal=all -o "$PROJECT_ROOT/../.run/mujoco_exit_trace")
    fi
    "${MUJOCO_TRACE[@]}" env "${MUJOCO_ENV[@]}" LD_LIBRARY_PATH="$(mujoco_ld_library_path)" ./robot_mujoco > robot_mujoco.log 2>&1 &
    PIDS+=($!)
    PROCESS_LABELS+=("MuJoCo")
fi

cd ../../../UeSim/Linux
if [[ "${MATRIX_DISABLE_UE:-0}" == "1" ]]; then
    echo "[INFO] UE disabled; MATRiX MuJoCo physics remains active."
else
    echo "[INFO] Starting UE"
    echo "[INFO] UE profile: fps=${UE_MAX_FPS}, resolution=${UE_RES_X}x${UE_RES_Y}, performance=${UE_PERFORMANCE_PROFILE}, no_rhi_thread=${UE_NO_RHI_THREAD}"
    VK_ICD_FILENAMES="${MATRIX_VK_ICD:-/usr/share/vulkan/icd.d/nvidia_icd.json}" \
    CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}" \
    LD_LIBRARY_PATH="$(ue_ld_library_path)" \
    ./zsibot_mujoco_ue.sh -game "$MAPNAME" "-ExecCmds=${UE_EXEC_CMDS}" \
        "${UE_EXTRA_ARGS[@]}" $USE_OFFSCREEN $USE_PIXELSTREAMER > zsibot_mujoco_ue.log 2>&1 &
    PIDS+=($!)
    PROCESS_LABELS+=("UE")
fi

sleep 7

cd ../../robot_mc
if $ENABLE_MC; then
    echo "[INFO] Starting MC"
    export SDK_CLIENT_IP="${SDK_CLIENT_IP:-127.0.0.1}"
    ROAMERX_STATE_FILE="${PROJECT_ROOT}/bin/roamerx_link.state"
    if [[ -f "${ROAMERX_STATE_FILE}" ]]; then
        ROAMERX_TARGET_IP="${SDK_CLIENT_IP}"
        SDK_CONFIG_FILE="${PWD}/build/export/config/sdk_config.yaml"
        if [[ -f "${SDK_CONFIG_FILE}" ]]; then
            sed -i "s/^target_ip: .*/target_ip: \"${ROAMERX_TARGET_IP}\"/" "${SDK_CONFIG_FILE}"
        fi
        echo "[INFO] RoamerX link detected, starting MC with UDP target ${ROAMERX_TARGET_IP}:43988 and highlevel port 43997"
        LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r 25001 25002 43988 43997 25005 > run_mc.log 2>&1 &
    else
        LD_LIBRARY_PATH="$(mc_ld_library_path)" ./run_mc.sh r mc_enable=true > run_mc.log 2>&1 &
    fi
    PIDS+=($!)
    PROCESS_LABELS+=("MC")
fi

# echo "[INFO] Starting ROS2 pub_tf.launch.py"
# ros2 launch pub_tf pub_tf.launch.py tf_type:=mujoco_tf > pub_tf.log 2>&1 &
# PIDS+=($!)

#######################################
# 阻塞等待
#######################################
echo "[INFO] All components started."
component_status=0
exited_pid=""
wait -n -p exited_pid "${PIDS[@]}" || component_status=$?
exited_label="unknown"
for index in "${!PIDS[@]}"; do
    if [[ "${PIDS[$index]}" == "${exited_pid:-}" ]]; then
        exited_label="${PROCESS_LABELS[$index]}"
        break
    fi
done
echo "[INFO] Simulator component exited: name=${exited_label} pid=${exited_pid:-unknown} status=${component_status}; stopping the remaining stack."
exit "${component_status}"
