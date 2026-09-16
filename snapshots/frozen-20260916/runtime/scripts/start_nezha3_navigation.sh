#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
FOREGROUND=0
for arg in "$@"; do
  case "$arg" in
    --foreground) FOREGROUND=1 ;;
    *) echo "Usage: $0 [--foreground]" >&2; exit 2 ;;
  esac
done

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
export SDK_CLIENT_IP="127.0.0.1"
export COMMUNICATION_TYPE="LCM"
export ROAMERX_COMMUNICATION_TYPE="LCM"
export GENISOM_ROAMERX_OPEN_WORKSPACE="${ROAMERX_DIR}"
export ROAMERX_MAP="${ROAMERX_DIR}/map/map.yaml"
export MATRIX_STATE_PORT="${MATRIX_STATE_PORT:-25301}"
export MATRIX_CMD_PORT="${MATRIX_CMD_PORT:-25302}"
export MATRIX_DISABLE_MC=1
export MATRIX_MUJOCO_ROBOT_TYPE=xgw
export MATRIX_MUJOCO_SCENE=nezha3/scene_terrain_yard.xml
export MATRIX_UE_CUSTOM_URDF=custom/scene_terrain_yard.xml
export NEZHA3_RL_SUCCESS_AXIS=y
export NEZHA3_RL_SUCCESS_POSITION=4.7
export NEZHA3_RL_SUCCESS_HEIGHT=0.8
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-1}"
export MATRIX_DISABLE_IMAGE_SENSORS="${MATRIX_DISABLE_IMAGE_SENSORS:-1}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}"
export ROAMERX_RVIZ_READ_ONLY="${ROAMERX_RVIZ_READ_ONLY:-1}"
export ROAMERX_DISABLE_RVIZ="${ROAMERX_DISABLE_RVIZ:-1}"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

pid_alive() {
  [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
}

start_tracked() {
  local name="$1"
  shift
  nohup "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
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
  echo "[ERROR] timed out waiting for ${action}" >&2
  return 1
}

wait_for_tf() {
  local parent="$1"
  local child="$2"
  local timeout_seconds="$3"
  local output
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    output="$(timeout 4 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null || true)"
    if grep -q 'Translation' <<<"${output}"; then
      echo "[OK] TF ready: ${parent} -> ${child}"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] timed out waiting for TF ${parent} -> ${child}" >&2
  return 1
}

if pid_alive "${RUN_DIR}/sim.pid" || pgrep -f 'robot_mujoco|nezha3_rl_bridge|zsibot_mujoco_ue' >/dev/null; then
  echo "[ERROR] another MATRiX simulator/controller is running; stop it first." >&2
  exit 1
fi
if [[ ! -x "${ROOT_DIR}/install/bin/nezha3_rl_bridge" ||
      ! -f "${ROAMERX_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] build output missing; run scripts/build_demo.sh first." >&2
  exit 1
fi

ln -sfn ../custom "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw/nezha3"
start_tracked zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
sleep 2
if ! pid_alive "${RUN_DIR}/zenohd.pid"; then
  echo "[ERROR] rmw_zenohd failed; see ${LOG_DIR}/zenohd.log" >&2
  exit 1
fi

start_tracked bridge stdbuf -oL -eL "${ROOT_DIR}/install/bin/nezha3_rl_bridge"
# YardWorld contains the Mid360 actor that publishes /livox/lidar. MuJoCo still
# uses MATRIX_MUJOCO_SCENE above, so the Nezha3 stair physics are unchanged.
nohup "${MATRIX_DIR}/scripts/run_sim.sh" custom 3 0 0 1 "" nezha3 \
  >"${LOG_DIR}/sim.log" 2>&1 &
echo "$!" >"${RUN_DIR}/sim.pid"
echo "[INFO] started Nezha3 MATRiX custom simulator, pid=$(cat "${RUN_DIR}/sim.pid")"

start_tracked lidar_tf ros2 run tf2_ros static_transform_publisher \
  --x 0.13011 --y 0.02329 --z 0.17598 \
  --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id lidar

if ! wait_for_topic /odom/mujoco_odom 90 || ! wait_for_topic /livox/lidar 90; then
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

deadline=$((SECONDS + 25))
while (( SECONDS < deadline )); do
  if rg -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
    echo "[OK] Nezha3 stair policy active"
    break
  fi
  sleep 1
done
if ! rg -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
  echo "[ERROR] Nezha3 policy did not activate" >&2
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

ROAMERX_COMMUNICATION_TYPE=LCM \
  bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" start
if ! wait_for_action /navigate_to_pose 45; then
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi
if ! wait_for_tf odom base_link 30; then
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

"${ROOT_DIR}/scripts/set_g1_stair_nav_mode.sh" up
echo "[OK] Nezha3 navigation stack ready (ports ${MATRIX_STATE_PORT}/${MATRIX_CMD_PORT})."
echo "[INFO] Send the stair goal with: scripts/send_nezha3_stair_goal.sh --feedback"

if (( FOREGROUND )); then
  trap '"${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
  while pid_alive "${RUN_DIR}/sim.pid" && pid_alive "${RUN_DIR}/bridge.pid"; do sleep 1; done
  echo "[ERROR] simulator or Nezha3 bridge exited" >&2
  exit 1
fi
