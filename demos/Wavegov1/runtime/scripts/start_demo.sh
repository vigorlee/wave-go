#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
FOREGROUND=0

for arg in "$@"; do
  case "${arg}" in
    --foreground) FOREGROUND=1 ;;
    *)
      echo "Usage: $0 [--foreground]" >&2
      exit 2
      ;;
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
pkill -KILL -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 action send_goal /navigate_to_pose ' \
  2>/dev/null || true
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-0}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-3}"
export MATRIX_DEPTH_WIDTH="${MATRIX_DEPTH_WIDTH:-320}"
export MATRIX_DEPTH_HEIGHT="${MATRIX_DEPTH_HEIGHT:-240}"
export MATRIX_DEPTH_FREQUENCY="${MATRIX_DEPTH_FREQUENCY:-2}"
export MATRIX_DISABLE_IMAGE_SENSORS="${MATRIX_DISABLE_IMAGE_SENSORS:-1}"
export MATRIX_UE_OFFSCREEN="${MATRIX_UE_OFFSCREEN:-0}"
export MATRIX_ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-0}"
# Keep the canonical YardWorld as the default, but allow a launcher to select
# a paired Go2-W scene variant without editing this stable entry point.
export MATRIX_MUJOCO_SCENE="${MATRIX_MUJOCO_SCENE:-go2w_lcm/scene_terrain_yard.xml}"
# Other robot demos reuse the public UE `go2w` dispatch directory and may
# replace its YardWorld variant. Always source the Go2-W runtime scene from the
# canonical MuJoCo model so UE cannot accidentally render an OLI/G1 proxy.
export MATRIX_UE_SCENE_OVERRIDE="${MATRIX_UE_SCENE_OVERRIDE:-${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_go2w.xml}"
export GO2W_WORLD_SCENE="${GO2W_WORLD_SCENE:-${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard.xml}"
export MATRIX_WORLD_MODEL_VALIDATOR="${MATRIX_WORLD_MODEL_VALIDATOR:-${ROOT_DIR}/scripts/validate_go2w_world.py}"
export GO2W_WHEEL_SIGN="-1"
export GO2W_MAX_LINEAR="0.45"
export GO2W_MAX_ANGULAR="0.8"
export GO2W_RL_HISTORY_MODE="${GO2W_RL_HISTORY_MODE:-train}"
export GO2W_RL_STOCHASTIC="${GO2W_RL_STOCHASTIC:-1}"
export GO2W_RL_MAX_VX="${GO2W_RL_MAX_VX:-1.2}"
export GO2W_RL_MAX_VY="${GO2W_RL_MAX_VY:-0.45}"
export GO2W_RL_MAX_YAW="${GO2W_RL_MAX_YAW:-1.0}"
export GO2W_RL_TERRAIN_GUARD="${GO2W_RL_TERRAIN_GUARD:-1}"
export GO2W_RL_STAIR_ASSIST="${GO2W_RL_STAIR_ASSIST:-1}"
export GO2W_RL_STAIR_APPROACH_ASSIST="${GO2W_RL_STAIR_APPROACH_ASSIST:-1}"
export GO2W_RL_STAIR_ASSIST_VX="${GO2W_RL_STAIR_ASSIST_VX:-1.10}"
export GO2W_RL_STAIR_ASSIST_ACCEL="${GO2W_RL_STAIR_ASSIST_ACCEL:-3.0}"
export GO2W_RL_STAIR_ASSIST_DECEL="${GO2W_RL_STAIR_ASSIST_DECEL:-2.0}"
export GO2W_RL_STAIR_ENTRY_VX="${GO2W_RL_STAIR_ENTRY_VX:-0.90}"
export GO2W_RL_STAIR_EXIT_VX="${GO2W_RL_STAIR_EXIT_VX:-0.80}"
export GO2W_RL_STAIR_APPROACH_MAX_LATERAL="${GO2W_RL_STAIR_APPROACH_MAX_LATERAL:-2.0}"
export GO2W_RL_STAIR_APPROACH_MAX_HEADING="${GO2W_RL_STAIR_APPROACH_MAX_HEADING:-0.50}"
export GO2W_RL_STAIR_ALIGNMENT_MAX_LATERAL="${GO2W_RL_STAIR_ALIGNMENT_MAX_LATERAL:-0.50}"
export GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING="${GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING:-0.20}"
export GO2W_RL_STAIR_ALIGNMENT_VX="${GO2W_RL_STAIR_ALIGNMENT_VX:-0.35}"
export GO2W_RL_TEST_LATERAL_GAIN="${GO2W_RL_TEST_LATERAL_GAIN:-0.8}"
export GO2W_RL_TEST_HEADING_GAIN="${GO2W_RL_TEST_HEADING_GAIN:-2.0}"
export GO2W_RL_DOWNHILL_MAX_VX="${GO2W_RL_DOWNHILL_MAX_VX:-0.50}"
export GO2W_RL_DOWNHILL_MAX_VY="${GO2W_RL_DOWNHILL_MAX_VY:-0.18}"
export GO2W_RL_DOWNHILL_MAX_YAW="${GO2W_RL_DOWNHILL_MAX_YAW:-0.45}"
export GO2W_RL_DOWNHILL_RELEASE_PITCH="${GO2W_RL_DOWNHILL_RELEASE_PITCH:-0.08}"
export GO2W_RL_TERRAIN_EXIT_VX="${GO2W_RL_TERRAIN_EXIT_VX:-0.18}"
export GO2W_NAV_MODE_FILE="${GO2W_NAV_MODE_FILE:-${RUN_DIR}/go2w/nav_mode}"
GO2W_CONTROLLER="${GO2W_CONTROLLER:-rl}"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if [[ -e "${RUN_DIR}/oli_navigation.armed" ]] || \
   systemctl --user is-active --quiet oli-matrix-ue.service 2>/dev/null; then
  echo "[ERROR] OLI runtime is active; stop it before starting Go2-W." >&2
  exit 1
fi
touch "${RUN_DIR}/go2w_navigation.lock"

pid_alive() {
  [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
}

stop_domain_zenohd() {
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null | \
       grep -Fxq "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -x rmw_zenohd 2>/dev/null || true)
}

start_tracked() {
  local name="$1"
  shift
  nohup "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  echo "${pid}" >"${RUN_DIR}/${name}.pid"
  echo "[INFO] started ${name}, pid=${pid}, log=${LOG_DIR}/${name}.log"
}

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "[OK] topic ready: ${topic}"
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] timed out waiting for ${topic}" >&2
  return 1
}

wait_for_action() {
  local action_name="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if timeout 3 ros2 action list 2>/dev/null | grep -Fxq "${action_name}"; then
      echo "[OK] action ready: ${action_name}"
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] timed out waiting for action ${action_name}" >&2
  return 1
}

wait_for_standing() {
  local timeout_seconds="$1"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if [[ "${GO2W_CONTROLLER}" == "rl" ]] && \
       grep -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
      echo "[OK] Go2-W DreamWaQ policy is active"
      return 0
    fi
    if [[ "${GO2W_CONTROLLER}" == "pd" ]] && \
       grep -q 'stand=1.000 drive=1' "${LOG_DIR}/bridge.log" 2>/dev/null; then
      echo "[OK] Go2-W PD standing controller is stable"
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] Go2-W did not reach a stable standing state. See ${LOG_DIR}/bridge.log." >&2
  return 1
}

if pid_alive "${RUN_DIR}/sim.pid"; then
  echo "[ERROR] demo is already running; use scripts/stop_demo.sh first." >&2
  exit 1
fi

if [[ "${GO2W_CONTROLLER}" != "rl" && "${GO2W_CONTROLLER}" != "pd" ]]; then
  echo "[ERROR] GO2W_CONTROLLER must be 'rl' or 'pd'." >&2
  exit 1
fi

BRIDGE_BINARY="${ROOT_DIR}/install/bin/go2w_${GO2W_CONTROLLER}_bridge"
if [[ "${GO2W_CONTROLLER}" == "pd" ]]; then
  BRIDGE_BINARY="${ROOT_DIR}/install/bin/go2w_lcm_bridge"
fi

if [[ ! -x "${BRIDGE_BINARY}" || ! -f "${ROAMERX_DIR}/install/setup.bash" ]]; then
  echo "[ERROR] build output missing; run scripts/build_demo.sh first." >&2
  exit 1
fi

ln -sfn ../go2w "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw/go2w_lcm"

stop_domain_zenohd
ros2 daemon stop >/dev/null 2>&1 || true
start_tracked zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
sleep 2
if ! pid_alive "${RUN_DIR}/zenohd.pid"; then
  echo "[ERROR] rmw_zenohd failed to start. See ${LOG_DIR}/zenohd.log." >&2
  exit 1
fi

start_tracked bridge stdbuf -oL -eL "${BRIDGE_BINARY}"

MATRIX_MUJOCO_ROBOT_TYPE=xgw \
MATRIX_MUJOCO_SCENE="${MATRIX_MUJOCO_SCENE}" \
nohup "${MATRIX_DIR}/scripts/run_sim.sh" go2w "${MATRIX_MAP_ID:-3}" "${MATRIX_UE_OFFSCREEN}" 0 1 \
  >"${LOG_DIR}/sim.log" 2>&1 &
echo "$!" >"${RUN_DIR}/sim.pid"
echo "[INFO] started MATRiX Go2-W YardWorld, pid=$(cat "${RUN_DIR}/sim.pid")"

LIDAR_MOUNT="$(/usr/bin/python3 "$ROOT_DIR/scripts/go2w_lidar_mount.py")"
read -r LIDAR_X LIDAR_Y LIDAR_Z LIDAR_ROLL LIDAR_PITCH LIDAR_YAW <<< "$LIDAR_MOUNT"
start_tracked lidar_tf ros2 run tf2_ros static_transform_publisher \
  --x "$LIDAR_X" --y "$LIDAR_Y" --z "$LIDAR_Z" \
  --roll "$LIDAR_ROLL" --pitch "$LIDAR_PITCH" --yaw "$LIDAR_YAW" \
  --frame-id base_link --child-frame-id lidar

start_tracked world_cloud /usr/bin/python3 "${ROOT_DIR}/scripts/go2w_world_obstacle_cloud.py"

if ! wait_for_topic /odom/mujoco_odom 90 || \
   ! wait_for_topic /livox/lidar_raw 90 || ! wait_for_topic /livox/lidar 90; then
  echo "[ERROR] MATRiX did not reach the PDF topic checkpoint. See ${LOG_DIR}/sim.log." >&2
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

if ! wait_for_standing 20; then
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

ROAMERX_COMMUNICATION_TYPE=LCM \
  bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" start

if ! wait_for_action /navigate_to_pose 30 || ! wait_for_action /navigate_through_poses 30; then
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

# Never inherit the stair filter (min height 0.65 m) into a normal navigation
# session.  It intentionally ignores low steps and would also ignore the
# 0.44 m cylinder course shown in the MATRiX demo.
if ! "${ROOT_DIR}/scripts/set_go2w_nav_mode.sh" avoid; then
  echo "[ERROR] Failed to arm Go2-W local obstacle avoidance." >&2
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi
if ! "${ROOT_DIR}/scripts/validate_go2w_local_avoidance.sh"; then
  echo "[ERROR] Go2-W local obstacle-avoidance validation failed closed." >&2
  "${ROOT_DIR}/scripts/stop_demo.sh" || true
  exit 1
fi

echo "[OK] MATRiX + Go2-W (${GO2W_CONTROLLER}) + RoamerX LCM demo started."
echo "[INFO] Run scripts/check_demo.sh for the PDF topic/TF checkpoints."

if (( FOREGROUND )); then
  echo "[INFO] foreground supervisor active; press Ctrl-C to stop the demo."
  trap '"${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
  while pid_alive "${RUN_DIR}/sim.pid" && pid_alive "${RUN_DIR}/bridge.pid"; do
    sleep 1
  done
  echo "[ERROR] simulator or controller exited." >&2
  exit 1
fi
