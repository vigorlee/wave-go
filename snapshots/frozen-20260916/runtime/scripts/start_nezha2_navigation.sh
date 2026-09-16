#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run/nezha2"
LOG_DIR="${ROOT_DIR}/logs/nezha2"
FOREGROUND=0
START_RVIZ=1
DEMO_ONLY="${NEZHA2_MATRIX_DEMO_ONLY:-0}"
for arg in "$@"; do
  case "$arg" in
    --foreground) FOREGROUND=1 ;;
    --no-rviz) START_RVIZ=0 ;;
    *) echo "Usage: $0 [--foreground] [--no-rviz]" >&2; exit 2 ;;
  esac
done

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
export SDK_CLIENT_IP="127.0.0.1"
export COMMUNICATION_TYPE="LCM"
export ROAMERX_COMMUNICATION_TYPE="LCM"
export GENISOM_ROAMERX_OPEN_WORKSPACE="${ROAMERX_DIR}"
# A scene-specific static map may be supplied by a launcher.  Keeping the
# normal map as the default preserves the existing stair/YardWorld workflow.
export ROAMERX_MAP="${ROAMERX_MAP:-${ROAMERX_DIR}/map/map.yaml}"
export MATRIX_STATE_PORT="${MATRIX_STATE_PORT:-25401}"
export MATRIX_CMD_PORT="${MATRIX_CMD_PORT:-25402}"
export MATRIX_DISABLE_MC=1
# The exact ``matrix_stack_conflict`` audit below protects this launcher from
# a genuine concurrent simulator.  Skip MATRiX's broad pattern-based cleanup,
# which can kill the supervising process before a Nezha2 demo starts.
export MATRIX_SKIP_STALE_PROCESS_CLEANUP=1
export MATRIX_MUJOCO_ROBOT_TYPE=xgw
export MATRIX_NEZHA2_SCENE="${MATRIX_NEZHA2_SCENE:-scene_terrain_yard.xml}"
export MATRIX_MUJOCO_SCENE="nezha2/${MATRIX_NEZHA2_SCENE}"
export MATRIX_UE_CUSTOM_URDF="nezha2/${MATRIX_NEZHA2_SCENE}"
if [[ "${MATRIX_NEZHA2_SCENE}" == "scene_terrain_stairs_obstacles.xml" ]]; then
  export MATRIX_UE_CUSTOM_SCENE="${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/scene_terrain_nezha2_stairs_obstacles.xml"
fi
export MATRIX_ROBOT_INITIAL_YAW_DEG=90
export MATRIX_KEEP_MUJOCO_ON_UE_EXIT="${MATRIX_KEEP_MUJOCO_ON_UE_EXIT:-1}"
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-1}"
export MATRIX_DISABLE_IMAGE_SENSORS="${MATRIX_DISABLE_IMAGE_SENSORS:-1}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}"
export ROAMERX_RVIZ_READ_ONLY="${ROAMERX_RVIZ_READ_ONLY:-0}"
export ROAMERX_DISABLE_RVIZ="${ROAMERX_DISABLE_RVIZ:-$((1 - START_RVIZ))}"
export ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-${ROOT_DIR}/config/nezha2_navigation.rviz}"
export NEZHA2_RL_SUCCESS_AXIS=y
export NEZHA2_RL_SUCCESS_POSITION=4.7
export NEZHA2_RL_SUCCESS_HEIGHT=0.8
MODEL_DIR="${ROOT_DIR}/third_party/nezha2_rl_sar/rl_sar_nezha2_0115/src/rl_sar/models/nezha/robot_lab"
# This is the unmodified Nezha2 stair policy used by the checked closed-loop
# MuJoCo runner.  Do not silently substitute a different exported checkpoint.
VALIDATED_STAIR_MODEL="${MODEL_DIR}/policy_zyl_0115_2.pt"
FLAT_SOURCE_MODEL="${MODEL_DIR}/policy_zyl_1209.pt"
FLAT_DETERMINISTIC_MODEL="${MODEL_DIR}/policy_zyl_1209_deterministic.pt"
export NEZHA2_RL_ACTION_ALPHA="${NEZHA2_RL_ACTION_ALPHA:-1.0}"
export NEZHA2_NAV_MODE_FILE="${NEZHA2_NAV_MODE_FILE:-${RUN_DIR}/nav_mode}"

if [[ -z "${NEZHA2_RL_MODEL:-}" ]]; then
  export NEZHA2_RL_MODEL="${VALIDATED_STAIR_MODEL}"
fi
if [[ ! -f "${NEZHA2_RL_MODEL}" ]]; then
  echo "[ERROR] Nezha2 RL model not found: ${NEZHA2_RL_MODEL}" >&2
  exit 1
fi
if [[ "${NEZHA2_RL_ENABLE_FLAT_BLEND:-0}" == "1" ]]; then
  if [[ ! -f "${FLAT_DETERMINISTIC_MODEL}" ]]; then
    TORCH_PYTHON="${NEZHA2_TORCH_PYTHON:-/home/unitree/isaacsim/env/bin/python}"
    "${TORCH_PYTHON}" "${ROOT_DIR}/scripts/export_nezha2_deterministic_policy.py" \
      --source "${FLAT_SOURCE_MODEL}" --output "${FLAT_DETERMINISTIC_MODEL}"
  fi
  export NEZHA2_RL_FLAT_MODEL="${NEZHA2_RL_FLAT_MODEL:-${FLAT_DETERMINISTIC_MODEL}}"
else
  # A single validated terrain policy is used until the base state/control
  # contract has passed the motion smoke test.
  export NEZHA2_RL_FLAT_MODEL=""
fi

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

pid_alive() {
  [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
}

pid_matches() {
  local pid_file="$1" pattern="$2" pid command
  pid_alive "${pid_file}" || return 1
  pid="$(cat "${pid_file}")"
  command="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  [[ "${command}" =~ ${pattern} ]]
}

start_tracked() {
  local name="$1"
  shift
  nohup setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/${name}.pid"
  echo "[INFO] started ${name}, pid=$(cat "${RUN_DIR}/${name}.pid")"
}

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "[OK] topic ready: ${topic}"
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] timed out waiting for ${topic}" >&2
  return 1
}

wait_for_action() {
  local action="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 action list 2>/dev/null | grep -Fxq "$action"; then
      echo "[OK] action ready: ${action}"
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] timed out waiting for action ${action}" >&2
  return 1
}

wait_for_tf() {
  local parent="$1" child="$2" timeout_seconds="$3" output
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    output="$(timeout 4 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null || true)"
    if grep -q Translation <<<"${output}"; then
      echo "[OK] TF ready: ${parent} -> ${child}"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] timed out waiting for TF ${parent} -> ${child}" >&2
  return 1
}

fail_stop() {
  "${ROOT_DIR}/scripts/stop_nezha2_navigation.sh" >/dev/null 2>&1 || true
}

# Do not use ``pgrep -f 'robot_mujoco|...'`` here.  It can match the parent
# shell when a launcher or diagnostic command happens to contain that regular
# expression, producing a false "simulator is running" failure.  Inspect the
# actual executable command lines instead.
matrix_stack_conflict() {
  local proc pid command
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    [[ "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
    [[ -r "${proc}/cmdline" ]] || continue
    command="$(tr '\0' ' ' <"${proc}/cmdline" 2>/dev/null || true)"
    case "${command}" in
      *'/install/bin/nezha2_rl_bridge '*|\
      *'/simulate/build/robot_mujoco '*|\
      *'/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue '*)
        echo "[INFO] active MATRiX process: pid=${pid} cmd=${command}" >&2
        return 0
        ;;
    esac
  done
  return 1
}

stop_stale_nezha2_lidar_tf() {
  local proc pid command
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    [[ "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
    [[ -r "${proc}/cmdline" ]] || continue
    command="$(tr '\0' ' ' <"${proc}/cmdline" 2>/dev/null || true)"
    case "${command}" in
      *'static_transform_publisher --x 0.13011 --y 0.02329 --z 0.17598 '*|\
      *'--frame-id base_link --child-frame-id lidar '*)
        kill -TERM "${pid}" 2>/dev/null || true
        ;;
    esac
  done
}

if pid_matches "${RUN_DIR}/bridge.pid" 'nezha2_rl_bridge' &&
   ! pid_matches "${RUN_DIR}/sim.pid" 'matrix/scripts/run_sim\.sh'; then
  echo "[WARN] removing an incomplete previous Nezha2 runtime"
  "${ROOT_DIR}/scripts/stop_nezha2_navigation.sh" >/dev/null 2>&1 || true
fi
if matrix_stack_conflict; then
  echo "[ERROR] another MATRiX simulator/controller is running; stop it first." >&2
  exit 1
fi
stop_stale_nezha2_lidar_tf
UE_SCENE_SOURCE="${MATRIX_UE_CUSTOM_SCENE:-${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/nezha2/${MATRIX_NEZHA2_SCENE}}"
if [[ ! -x "${ROOT_DIR}/install/bin/nezha2_rl_bridge" ||
      ! -f "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/nezha2/${MATRIX_NEZHA2_SCENE}" ||
      ! -f "${UE_SCENE_SOURCE}" ||
      ! -f "${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/nezha2/nezha2.xml" ]]; then
  echo "[ERROR] Nezha2 build/model output missing; run scripts/build_demo.sh and scripts/prepare_nezha2_matrix.py." >&2
  exit 1
fi

ln -sfn ../nezha2 "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw/nezha2"
if ss -ltn '( sport = :7447 )' 2>/dev/null | grep -q 7447; then
  echo "[INFO] reusing existing Zenoh router on port 7447"
else
  start_tracked zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
  sleep 2
  if ! pid_alive "${RUN_DIR}/zenohd.pid"; then
    echo "[ERROR] rmw_zenohd failed; see ${LOG_DIR}/zenohd.log" >&2
    exit 1
  fi
fi

start_tracked bridge stdbuf -oL -eL "${ROOT_DIR}/install/bin/nezha2_rl_bridge"
nohup setsid "${MATRIX_DIR}/scripts/run_sim.sh" custom "${MATRIX_SCENE_ID:-3}" 0 0 1 "" nezha2 \
  >"${LOG_DIR}/sim.log" 2>&1 &
echo "$!" >"${RUN_DIR}/sim.pid"
echo "[INFO] started Nezha2 MATRiX scene ${MATRIX_NEZHA2_SCENE}, pid=$(cat "${RUN_DIR}/sim.pid")"

start_tracked lidar_tf ros2 run tf2_ros static_transform_publisher \
  --x 0.13011 --y 0.02329 --z 0.17598 \
  --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id lidar

# MATRiX publishes the actual UE ray-cast cloud on /livox/lidar_raw.  The
# RoamerX navigation contract uses /livox/lidar, so the real-scene launcher
# enables a transparent relay instead of inserting model-derived points.
if [[ "${NEZHA2_RELAY_RAW_LIDAR:-0}" == "1" ]]; then
  start_tracked lidar_relay /usr/bin/python3 "${ROOT_DIR}/scripts/nezha2_lidar_relay.py"
fi

if ! wait_for_topic /odom/mujoco_odom 90; then
  fail_stop
  exit 1
fi

deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  if grep -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
    echo "[OK] Nezha2 policy active"
    break
  fi
  sleep 1
done
if ! grep -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
  echo "[ERROR] Nezha2 policy did not activate" >&2
  fail_stop
  exit 1
fi

if [[ "${DEMO_ONLY}" == "1" ]]; then
  echo "[INFO] MATRiX demo-only mode: physical simulation and policy bridge are active; ROS navigation is intentionally not started."
elif ! wait_for_topic /livox/lidar 90; then
  fail_stop
  exit 1
fi

if [[ "${DEMO_ONLY}" != "1" ]]; then
  ROAMERX_COMMUNICATION_TYPE=LCM \
    bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" start
  if ! wait_for_action /navigate_to_pose 45 || ! wait_for_tf odom base_link 30; then
    fail_stop
    exit 1
  fi

  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" "${ROOT_DIR}/scripts/set_nezha2_stair_nav_mode.sh" avoid
  "${ROOT_DIR}/scripts/set_nezha2_navigation_view.sh" || true
  echo "[OK] Nezha2 navigation ready: domain=${ROS_DOMAIN_ID}, ports=${MATRIX_STATE_PORT}/${MATRIX_CMD_PORT}."
fi
echo "[OK] policy=${NEZHA2_RL_MODEL}; action_alpha=${NEZHA2_RL_ACTION_ALPHA}"
if (( START_RVIZ )) && [[ "${DEMO_ONLY}" != "1" ]]; then
  echo "[OK] RViz interactive goals enabled: use 2D Goal Pose in the map view."
fi
echo "[INFO] Send a stair goal with: scripts/send_nezha2_goal.sh 0.65 4.8 1.570796 --feedback"

if (( FOREGROUND )); then
  trap '"${ROOT_DIR}/scripts/stop_nezha2_navigation.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
  while pid_alive "${RUN_DIR}/sim.pid" && pid_alive "${RUN_DIR}/bridge.pid"; do sleep 1; done
  echo "[ERROR] simulator or Nezha2 bridge exited" >&2
  "${ROOT_DIR}/scripts/stop_nezha2_navigation.sh" >/dev/null 2>&1 || true
  exit 1
fi

sim_pid="$(cat "${RUN_DIR}/sim.pid")"
bridge_pid="$(cat "${RUN_DIR}/bridge.pid")"
nohup setsid bash -c '
  sim_pid="$1"
  bridge_pid="$2"
  stop_script="$3"
  while kill -0 "${sim_pid}" 2>/dev/null && kill -0 "${bridge_pid}" 2>/dev/null; do
    sleep 1
  done
  NEZHA2_SKIP_WATCHDOG_STOP=1 "${stop_script}" >/dev/null 2>&1 || true
' _ "${sim_pid}" "${bridge_pid}" "${ROOT_DIR}/scripts/stop_nezha2_navigation.sh" \
  >"${LOG_DIR}/runtime_watchdog.log" 2>&1 &
echo "$!" >"${RUN_DIR}/runtime_watchdog.pid"
