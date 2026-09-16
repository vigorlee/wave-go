#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
TEST_MODE=0
HEADLESS=0
FOREGROUND=0
STAIR_HEIGHT_CM="${G1_STAIR_HEIGHT_CM:-}"

for arg in "$@"; do
  case "${arg}" in
    --test) TEST_MODE=1 ;;
    --headless) HEADLESS=1 ;;
    --foreground) FOREGROUND=1 ;;
    --4cm) STAIR_HEIGHT_CM=4 ;;
    --5cm) STAIR_HEIGHT_CM=5 ;;
    *)
      echo "Usage: $0 [--test] [--headless] [--foreground] [--4cm|--5cm]" >&2
      exit 2
      ;;
  esac
done

set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
export MATRIX_STATE_PORT="${MATRIX_STATE_PORT:-25201}"
export MATRIX_CMD_PORT="${MATRIX_CMD_PORT:-25202}"
export G1_RL_DEVICE="${G1_RL_DEVICE:-cpu}"
export G1_RL_POLICY_TYPE="${G1_RL_POLICY_TYPE:-dreamwaq}"
export G1_RL_STAND_SECONDS="${G1_RL_STAND_SECONDS:-0.8}"
export G1_RL_MAX_VX="${G1_RL_MAX_VX:-1.0}"
export G1_RL_MAX_VY="${G1_RL_MAX_VY:-0.35}"
export G1_RL_MAX_YAW="${G1_RL_MAX_YAW:-0.8}"
export G1_RL_IDLE_TRANSITION_SECONDS="${G1_RL_IDLE_TRANSITION_SECONDS:-1.0}"

case "${G1_RL_POLICY_TYPE}" in
  dreamwaq)
    DEFAULT_G1_MODEL="${ROOT_DIR}/third_party/HumanoidDreamWaQ_G1/pre_train/g1/dreamwaq_level9.pt"
    [[ -n "${STAIR_HEIGHT_CM}" ]] || STAIR_HEIGHT_CM=5
    ;;
  official)
    DEFAULT_G1_MODEL="${ROOT_DIR}/third_party/unitree_rl_gym_g1/pre_train/g1/motion.pt"
    [[ -n "${STAIR_HEIGHT_CM}" ]] || STAIR_HEIGHT_CM=4
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

case "${STAIR_HEIGHT_CM}" in
  4)
    STAIR_SCENE="scene_terrain_stairs_rl_4cm.xml"
    STAIR_SCENE_ID=31
    ;;
  5)
    STAIR_SCENE="scene_terrain_stairs_rl.xml"
    STAIR_SCENE_ID=30
    ;;
  *)
    echo "[ERROR] stair height must be 4 or 5 cm." >&2
    exit 2
    ;;
esac

if (( TEST_MODE )); then
  export G1_RL_TEST_VX="${G1_RL_TEST_VX:-0.70}"
  export G1_RL_TEST_DELAY="${G1_RL_TEST_DELAY:-0}"
  export G1_RL_TEST_DURATION="${G1_RL_TEST_DURATION:-25.0}"
  export G1_RL_TEST_LATERAL_GAIN="${G1_RL_TEST_LATERAL_GAIN:-0.4}"
  export G1_RL_TEST_HEADING_GAIN="${G1_RL_TEST_HEADING_GAIN:-0.8}"
  export G1_RL_TEST_MAX_VY="${G1_RL_TEST_MAX_VY:-0.15}"
  export G1_RL_TEST_MAX_YAW="${G1_RL_TEST_MAX_YAW:-0.35}"
  export G1_RL_TEST_VY_STEP="${G1_RL_TEST_VY_STEP:-0.02}"
  export G1_RL_TEST_YAW_STEP="${G1_RL_TEST_YAW_STEP:-0.04}"
  export G1_RL_TEST_DECEL_STEP="${G1_RL_TEST_DECEL_STEP:-0.04}"
  export G1_RL_TEST_STOP_X="${G1_RL_TEST_STOP_X:-7.2}"
  export G1_RL_TEST_POST_HOLD_SECONDS="${G1_RL_TEST_POST_HOLD_SECONDS:-2.0}"
fi

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

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

start_tracked g1_stair_zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
sleep 2
start_tracked g1_stair_bridge stdbuf -oL -eL "${ROOT_DIR}/install/bin/g1_rl_bridge"

bridge_deadline=$((SECONDS + 15))
while (( SECONDS < bridge_deadline )); do
  if grep -q 'LCM vel_cmd_lcm_data -> policy -> eCAL mujoco_cmd' \
    "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
    break
  fi
  if ! pid_alive "${RUN_DIR}/g1_stair_bridge.pid"; then
    echo "[ERROR] G1 bridge exited before communication was ready." >&2
    tail -80 "${LOG_DIR}/g1_stair_bridge.log" >&2 || true
    exit 1
  fi
  sleep 0.2
done
if ! grep -q 'LCM vel_cmd_lcm_data -> policy -> eCAL mujoco_cmd' \
  "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
  echo "[ERROR] G1 bridge communication did not become ready." >&2
  exit 1
fi

sed -i 's/^enable_elastic_band: .*/enable_elastic_band: 0 # Disabled after paused startup handoff/' \
  "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
: >"${LOG_DIR}/g1_stair_pause.log"

if (( HEADLESS )); then
  sed -i 's/^robot: .*/robot: "g1"/' "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
  sed -i "s|^robot_scene: .*|robot_scene: \"${STAIR_SCENE}\"|" \
    "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
  MUJOCO_BUILD="${MATRIX_DIR}/src/robot_mujoco/simulate/build"
  MUJOCO_LD_PATH="${MUJOCO_BUILD}:/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux:${MATRIX_DIR}/src/UeSim/Linux/Engine/Binaries/Linux"
  (
    cd "${MUJOCO_BUILD}"
    nohup env LD_LIBRARY_PATH="${MUJOCO_LD_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
      ./robot_mujoco_g1 >"${LOG_DIR}/g1_stair_mujoco.log" 2>&1 &
    echo "$!" >"${RUN_DIR}/g1_stair_sim.pid"
  )
  echo "[INFO] started G1 MuJoCo benchmark without UE, pid=$(cat "${RUN_DIR}/g1_stair_sim.pid")"
else
  start_tracked g1_stair_sim env \
    MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}" \
    MATRIX_UE_NO_RHI_THREAD="${MATRIX_UE_NO_RHI_THREAD:-1}" \
    MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-20}" \
    MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}" \
    MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}" \
    MATRIX_UE_MAP="${MATRIX_UE_MAP:-/Game/Maps/YardWorld}" \
    MATRIX_ROBOT_INITIAL_YAW_DEG="0" \
    MATRIX_MUJOCO_BINARY="./robot_mujoco_g1" \
    "${MATRIX_DIR}/scripts/run_sim.sh" g1 "${STAIR_SCENE_ID}" 0 0 1
fi
echo "[INFO] G1 policy=${G1_RL_POLICY_TYPE}, stairs=${STAIR_HEIGHT_CM}cm, model=${G1_RL_MODEL}"
echo "[INFO] G1 MuJoCo started in deterministic pause mode."

deadline=$((SECONDS + 30))
physics_resumed=0
while (( SECONDS < deadline )); do
  if (( ! physics_resumed )) && \
    grep -q '^STAND_COMMAND_ACTIVE$' "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
    if ! "${ROOT_DIR}/scripts/mujoco_key.py" space --wait 5 \
      >>"${LOG_DIR}/g1_stair_pause.log" 2>&1; then
      echo "[ERROR] G1 stand command is active, but MuJoCo physics could not be resumed." >&2
      tail -40 "${LOG_DIR}/g1_stair_pause.log" >&2 || true
      "${ROOT_DIR}/scripts/stop_g1.sh" || true
      exit 1
    fi
    physics_resumed=1
    echo "[INFO] MuJoCo physics resumed after stand-command handoff."
  fi
  if (( physics_resumed )) && \
    grep -q 'Policy control enabled' "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
    echo "[OK] G1 policy is controlling the robot."
    echo "[INFO] Run scripts/check_g1_stairs.sh to inspect progress."
    if (( FOREGROUND )); then
      trap '"${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
      while pid_alive "${RUN_DIR}/g1_stair_sim.pid" && pid_alive "${RUN_DIR}/g1_stair_bridge.pid"; do
        if rg -q '^STAIR_FAILED |Fall protection latched' \
          "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
          echo "[ERROR] G1 stair benchmark failed." >&2
          "${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true
          exit 1
        fi
        if (( HEADLESS && TEST_MODE )) && \
          rg -q '^STAIR_SUCCESS ' "${LOG_DIR}/g1_stair_bridge.log" 2>/dev/null; then
          echo "[OK] G1 stair benchmark completed."
          "${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true
          exit 0
        fi
        sleep 1
      done
      echo "[ERROR] G1 simulator or bridge exited." >&2
      exit 1
    fi
    exit 0
  fi
  if ! pid_alive "${RUN_DIR}/g1_stair_sim.pid" || ! pid_alive "${RUN_DIR}/g1_stair_bridge.pid"; then
    echo "[ERROR] G1 simulator or bridge exited during startup." >&2
    tail -80 "${LOG_DIR}/g1_stair_bridge.log" >&2 || true
    tail -80 "${LOG_DIR}/g1_stair_sim.log" >&2 || true
    tail -80 "${LOG_DIR}/g1_stair_mujoco.log" >&2 || true
    exit 1
  fi
  sleep 0.05
done

echo "[ERROR] G1 stand/policy handoff did not complete within 30 seconds." >&2
tail -100 "${LOG_DIR}/g1_stair_bridge.log" >&2 || true
exit 1
