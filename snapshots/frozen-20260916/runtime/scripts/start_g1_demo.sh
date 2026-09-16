#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
FOREGROUND=0

if [[ "${1:-}" == "--foreground" ]]; then
  FOREGROUND=1
elif [[ -n "${1:-}" ]]; then
  echo "Usage: $0 [--foreground]" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${G1_ROS_DOMAIN_ID:-89}"
export SDK_CLIENT_IP="127.0.0.1"
export COMMUNICATION_TYPE="LCM"
export ROAMERX_COMMUNICATION_TYPE="LCM"
export GENISOM_ROAMERX_OPEN_WORKSPACE="${ROAMERX_DIR}"
export G1_RL_DEVICE="${G1_RL_DEVICE:-cpu}"
export G1_RL_POLICY_TYPE="${G1_RL_POLICY_TYPE:-dreamwaq}"
export G1_RL_STAND_SECONDS="${G1_RL_STAND_SECONDS:-0.8}"
export G1_RL_MAX_VX="${G1_RL_MAX_VX:-1.0}"
export G1_RL_MAX_VY="${G1_RL_MAX_VY:-0.35}"
export G1_RL_MAX_YAW="${G1_RL_MAX_YAW:-0.8}"
export G1_RL_IDLE_LINEAR_THRESHOLD="${G1_RL_IDLE_LINEAR_THRESHOLD:-0.02}"
export G1_RL_IDLE_YAW_THRESHOLD="${G1_RL_IDLE_YAW_THRESHOLD:-0.02}"
export G1_RL_IDLE_DELAY_SECONDS="${G1_RL_IDLE_DELAY_SECONDS:-0.25}"
export G1_RL_IDLE_TRANSITION_SECONDS="${G1_RL_IDLE_TRANSITION_SECONDS:-1.0}"
case "${G1_RL_POLICY_TYPE}" in
  dreamwaq)
    DEFAULT_G1_MODEL="${ROOT_DIR}/third_party/HumanoidDreamWaQ_G1/pre_train/g1/dreamwaq_level9.pt"
    ;;
  official)
    DEFAULT_G1_MODEL="${ROOT_DIR}/third_party/unitree_rl_gym_g1/pre_train/g1/motion.pt"
    ;;
  *)
    echo "[ERROR] G1_RL_POLICY_TYPE must be dreamwaq or official." >&2
    exit 2
    ;;
esac
export G1_RL_MODEL="${G1_RL_MODEL:-${DEFAULT_G1_MODEL}}"
if [[ ! -f "${G1_RL_MODEL}" ]]; then
  echo "[ERROR] G1 policy model is missing: ${G1_RL_MODEL}" >&2
  exit 1
fi
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-1}"
export MATRIX_DISABLE_IMAGE_SENSORS="${MATRIX_DISABLE_IMAGE_SENSORS:-1}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-3}"
export MATRIX_DEPTH_WIDTH="${MATRIX_DEPTH_WIDTH:-320}"
export MATRIX_DEPTH_HEIGHT="${MATRIX_DEPTH_HEIGHT:-240}"
export MATRIX_DEPTH_FREQUENCY="${MATRIX_DEPTH_FREQUENCY:-2}"
export MATRIX_ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-0}"
export MATRIX_STATE_PORT="${MATRIX_STATE_PORT:-25201}"
export MATRIX_CMD_PORT="${MATRIX_CMD_PORT:-25202}"
# The packaged UE release has no g1 dispatch entry and skips all ROS sensors for
# robot_type=g1. Keep G1 in MuJoCo and use the supported Go2-W UE sensor proxy.
export MATRIX_UE_ROBOT_TYPE="${MATRIX_UE_ROBOT_TYPE:-go2w}"
export MATRIX_UE_RUNTIME_ROBOT_TYPE="${MATRIX_UE_RUNTIME_ROBOT_TYPE:-go2w}"
export MATRIX_MUJOCO_ROBOT_TYPE="g1"
export MATRIX_LIDAR_X_CM="${MATRIX_LIDAR_X_CM:-10.0}"
export MATRIX_LIDAR_Y_CM="${MATRIX_LIDAR_Y_CM:-0.0}"
export MATRIX_LIDAR_Z_CM="${MATRIX_LIDAR_Z_CM:-32.0}"
export G1_YARD_STAIR_PROFILE="${G1_YARD_STAIR_PROFILE:-original}"
export G1_OIL_UPPER_BODY="${G1_OIL_UPPER_BODY:-0}"
export MATRIX_SKIP_ENV_CHECK="${MATRIX_SKIP_ENV_CHECK:-1}"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

cleanup_on_error() {
  local status=$?
  if (( status != 0 )); then
    "${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true
  fi
  return "${status}"
}
trap cleanup_on_error EXIT

pid_alive() {
  [[ -f "$1" ]] && kill -0 "$(cat "$1")" 2>/dev/null
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
  local deadline=$((SECONDS + $2))
  while (( SECONDS < deadline )); do
    if ! pid_alive "${RUN_DIR}/sim.pid" ||
       ! pid_alive "${RUN_DIR}/bridge.pid" ||
       ! pid_alive "${RUN_DIR}/zenohd.pid"; then
      echo "[ERROR] core G1 process exited while waiting for ${topic}." >&2
      return 1
    fi
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
  local deadline=$((SECONDS + $2))
  while (( SECONDS < deadline )); do
    if ! pid_alive "${RUN_DIR}/sim.pid" ||
       ! pid_alive "${RUN_DIR}/bridge.pid" ||
       ! pid_alive "${RUN_DIR}/zenohd.pid"; then
      echo "[ERROR] core G1 process exited while waiting for ${action_name}." >&2
      return 1
    fi
    if timeout 3 ros2 action list 2>/dev/null | grep -Fxq "${action_name}"; then
      echo "[OK] action ready: ${action_name}"
      return 0
    fi
    sleep 2
  done
  echo "[ERROR] timed out waiting for action ${action_name}" >&2
  return 1
}

if pgrep -f 'robot_mujoco|go2w_rl_bridge|g1_rl_bridge|mc_ctrl|zsibot_mujoco_ue' >/dev/null; then
  echo "[INFO] stopping the previous simulator/controller stack"
  "${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true
  pkill -TERM -f 'robot_mujoco|go2w_rl_bridge|g1_rl_bridge|mc_ctrl|zsibot_mujoco_ue' \
    2>/dev/null || true
  sleep 2
fi

if [[ ! -x "${ROOT_DIR}/install/bin/g1_rl_bridge" ]]; then
  echo "[ERROR] G1 RL bridge missing; run scripts/build_demo.sh first." >&2
  exit 1
fi
if [[ ! -x "${MATRIX_DIR}/src/robot_mujoco/simulate/build/robot_mujoco_g1" ]]; then
  "${ROOT_DIR}/scripts/prepare_g1_mujoco.sh"
fi
G1_WORLD_PROFILE="${G1_WORLD_PROFILE:-yard}"
case "${G1_WORLD_PROFILE}:${G1_YARD_STAIR_PROFILE}" in
  office:*)
    "${ROOT_DIR}/scripts/build_g1_office_navigation_assets.py" --root "${ROOT_DIR}" >/dev/null
    G1_PHYSICS_SCENE="${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/g1/scene_terrain_office.xml"
    export MATRIX_MUJOCO_SCENE="scene_terrain_office.xml"
    ;;
  yard:policy)
    G1_PHYSICS_SCENE="$("${ROOT_DIR}/scripts/prepare_g1_yard_policy_scene.sh")"
    export MATRIX_MUJOCO_SCENE="$(basename "${G1_PHYSICS_SCENE}")"
    ;;
  yard:original)
    G1_PHYSICS_SCENE="${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/g1/scene_terrain_yard.xml"
    export MATRIX_MUJOCO_SCENE="scene_terrain_yard.xml"
    ;;
  *)
    echo "[ERROR] supported G1 profiles are office or yard:{policy,original}." >&2
    exit 2
    ;;
esac
if [[ "${G1_OIL_UPPER_BODY}" == "1" ]]; then
  G1_PHYSICS_SCENE="$("${ROOT_DIR}/scripts/prepare_g1_oil_upper.sh" "${G1_PHYSICS_SCENE}")"
  export MATRIX_MUJOCO_SCENE="$(basename "${G1_PHYSICS_SCENE}")"
elif [[ "${G1_OIL_UPPER_BODY}" != "0" ]]; then
  echo "[ERROR] G1_OIL_UPPER_BODY must be 0 or 1." >&2
  exit 2
fi
G1_UE_PROXY_SCENE="$("${ROOT_DIR}/scripts/prepare_g1_ue_proxy.sh" "${G1_PHYSICS_SCENE}")"
export MATRIX_UE_SCENE_OVERRIDE="${MATRIX_UE_SCENE_OVERRIDE:-${G1_UE_PROXY_SCENE}}"
echo "[INFO] G1 YardWorld stair profile=${G1_YARD_STAIR_PROFILE}, scene=${MATRIX_MUJOCO_SCENE}"

ros2 daemon stop >/dev/null 2>&1 || true
start_tracked zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
sleep 2
start_tracked bridge stdbuf -oL -eL "${ROOT_DIR}/install/bin/g1_rl_bridge"
echo "[INFO] G1 policy=${G1_RL_POLICY_TYPE}, model=${G1_RL_MODEL}"

bridge_deadline=$((SECONDS + 15))
while (( SECONDS < bridge_deadline )); do
  if grep -q 'LCM vel_cmd_lcm_data -> policy -> eCAL mujoco_cmd' \
    "${LOG_DIR}/bridge.log" 2>/dev/null; then
    break
  fi
  if ! pid_alive "${RUN_DIR}/bridge.pid"; then
    echo "[ERROR] G1 bridge exited before communication was ready." >&2
    tail -80 "${LOG_DIR}/bridge.log" >&2 || true
    exit 1
  fi
  sleep 0.2
done
if ! grep -q 'LCM vel_cmd_lcm_data -> policy -> eCAL mujoco_cmd' \
  "${LOG_DIR}/bridge.log" 2>/dev/null; then
  echo "[ERROR] G1 bridge communication did not become ready." >&2
  exit 1
fi

sed -i 's/^enable_elastic_band: .*/enable_elastic_band: 0 # Disabled after paused startup handoff/' \
  "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
: >"${LOG_DIR}/g1_demo_pause.log"

start_tracked sim env MATRIX_MUJOCO_BINARY="./robot_mujoco_g1" \
  "${MATRIX_DIR}/scripts/run_sim.sh" go2w "${G1_SCENE_ID:-3}" 0 0 1
echo "[INFO] G1 MuJoCo started in deterministic pause mode."

deadline=$((SECONDS + 30))
physics_resumed=0
policy_ready=0
while (( SECONDS < deadline )); do
  if (( ! physics_resumed )) && \
    grep -q '^STAND_COMMAND_ACTIVE$' "${LOG_DIR}/bridge.log" 2>/dev/null; then
    if ! "${ROOT_DIR}/scripts/mujoco_key.py" space --wait 5 \
      >>"${LOG_DIR}/g1_demo_pause.log" 2>&1; then
      echo "[ERROR] G1 stand command is active, but MuJoCo physics could not be resumed." >&2
      tail -40 "${LOG_DIR}/g1_demo_pause.log" >&2 || true
      "${ROOT_DIR}/scripts/stop_g1.sh" || true
      exit 1
    fi
    physics_resumed=1
    echo "[INFO] MuJoCo physics resumed after stand-command handoff."
  fi
  if (( physics_resumed )) && grep -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
    echo "[OK] G1 policy is active"
    policy_ready=1
    break
  fi
  if ! pid_alive "${RUN_DIR}/sim.pid" || ! pid_alive "${RUN_DIR}/bridge.pid"; then
    echo "[ERROR] G1 simulator or bridge exited during startup." >&2
    exit 1
  fi
  sleep 0.05
done
if (( ! policy_ready )); then
  echo "[ERROR] G1 policy did not become active." >&2
  tail -100 "${LOG_DIR}/bridge.log" >&2 || true
  "${ROOT_DIR}/scripts/stop_g1.sh" || true
  exit 1
fi

"${ROOT_DIR}/scripts/cleanup_lidar_tf.sh"
start_tracked lidar_tf /opt/ros/humble/lib/tf2_ros/static_transform_publisher \
  --x 0.10 --y 0 --z 0.32 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id lidar

if ! wait_for_topic /odom/mujoco_odom 90 || ! wait_for_topic /livox/lidar 90; then
  "${ROOT_DIR}/scripts/stop_g1.sh" || true
  exit 1
fi

ROAMERX_COMMUNICATION_TYPE=LCM \
  bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" start

if ! wait_for_action /navigate_to_pose 40; then
  "${ROOT_DIR}/scripts/stop_g1.sh" || true
  exit 1
fi

if ! "${ROOT_DIR}/scripts/mujoco_key.py" --ue --raise-only --wait 5; then
  echo "[WARN] UE is running, but its scene window could not be raised." >&2
fi

# Restore the morning stair workflow: keep the static map for walls, but do not
# mark the known stair geometry from the LiDAR point cloud as a local obstacle.
if [[ "${G1_WORLD_PROFILE}" == "yard" ]]; then
  "${ROOT_DIR}/scripts/set_g1_stair_nav_mode.sh" up
fi

echo "[OK] MATRiX + G1 + RoamerX LCM demo started."
echo "[INFO] Use RViz 2D Goal Pose or: scripts/send_goal.sh X Y YAW"

if (( FOREGROUND )); then
  trap '"${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
  while pid_alive "${RUN_DIR}/sim.pid" &&
        pid_alive "${RUN_DIR}/bridge.pid" &&
        pid_alive "${RUN_DIR}/zenohd.pid"; do
    sleep 1
  done
  echo "[ERROR] G1 simulator or bridge exited." >&2
  exit 1
fi
