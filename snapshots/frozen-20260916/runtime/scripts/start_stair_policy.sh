#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
TEST_MODE=0
FOREGROUND=0

for arg in "$@"; do
  case "${arg}" in
    --test) TEST_MODE=1 ;;
    --deterministic) export GO2W_RL_STOCHASTIC=0 ;;
    --foreground) FOREGROUND=1 ;;
    *)
      echo "Usage: $0 [--test] [--deterministic] [--foreground]" >&2
      exit 2
      ;;
  esac
done

set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
export GO2W_RL_MAX_VX="${GO2W_RL_MAX_VX:-1.2}"
export GO2W_RL_STAIR_ASSIST_VX="${GO2W_RL_STAIR_ASSIST_VX:-1.10}"
export GO2W_RL_STAIR_ASSIST_ACCEL="${GO2W_RL_STAIR_ASSIST_ACCEL:-3.0}"
export GO2W_RL_STAIR_ASSIST_DECEL="${GO2W_RL_STAIR_ASSIST_DECEL:-2.0}"
export GO2W_RL_STAIR_ENTRY_VX="${GO2W_RL_STAIR_ENTRY_VX:-0.90}"
export GO2W_RL_STAIR_EXIT_VX="${GO2W_RL_STAIR_EXIT_VX:-0.80}"
export GO2W_RL_DOWNHILL_MAX_VX="${GO2W_RL_DOWNHILL_MAX_VX:-0.50}"
export GO2W_RL_DOWNHILL_RELEASE_PITCH="${GO2W_RL_DOWNHILL_RELEASE_PITCH:-0.08}"

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

if pgrep -f 'robot_mujoco|go2w_lcm_bridge|go2w_rl_bridge|mc_ctrl|zsibot_mujoco_ue' >/dev/null; then
  echo "[ERROR] another simulator/controller is running; stop it before the stair benchmark." >&2
  exit 1
fi

if [[ ! -x "${ROOT_DIR}/install/bin/go2w_rl_bridge" ]]; then
  echo "[ERROR] RL bridge missing; run scripts/build_demo.sh first." >&2
  exit 1
fi

ln -sfn ../go2w "${MATRIX_DIR}/src/robot_mujoco/zsibot_robots/xgw/go2w_rl"
sed -i 's/^robot: .*/robot: "xgw"/' "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"
sed -i 's|^robot_scene: .*|robot_scene: "go2w_rl/scene_terrain_stairs_rl.xml"|' \
  "${MATRIX_DIR}/src/robot_mujoco/simulate/config.yaml"

if ! pgrep -f '/rmw_zenohd($| )' >/dev/null; then
  start_tracked stair_zenohd ros2 run rmw_zenoh_cpp rmw_zenohd
  sleep 2
fi

if (( TEST_MODE )); then
  export GO2W_RL_TEST_VX="${GO2W_RL_TEST_VX:-0.80}"
  export GO2W_RL_TEST_DELAY="${GO2W_RL_TEST_DELAY:-1.0}"
  export GO2W_RL_TEST_DURATION="${GO2W_RL_TEST_DURATION:-28.0}"
fi

start_tracked stair_bridge stdbuf -oL -eL "${ROOT_DIR}/install/bin/go2w_rl_bridge"

MUJOCO_BUILD="${MATRIX_DIR}/src/robot_mujoco/simulate/build"
MUJOCO_LD_PATH="${MUJOCO_BUILD}:/opt/ros/humble/opt/zenoh_cpp_vendor/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:${MATRIX_DIR}/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux:${MATRIX_DIR}/src/UeSim/Linux/Engine/Binaries/Linux"
(
  cd "${MUJOCO_BUILD}"
  nohup env LD_LIBRARY_PATH="${MUJOCO_LD_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
    ./robot_mujoco >"${LOG_DIR}/stair_mujoco.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/stair_sim.pid"
)
echo "[INFO] started stair_sim, pid=$(cat "${RUN_DIR}/stair_sim.pid"), log=${LOG_DIR}/stair_mujoco.log"

deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  if grep -q 'Policy control enabled' "${LOG_DIR}/stair_bridge.log" 2>/dev/null; then
    echo "[OK] DreamWaQ policy is controlling Go2-W."
    echo "[INFO] Run scripts/check_stair_policy.sh to inspect progress."
    if (( FOREGROUND )); then
      echo "[INFO] foreground supervisor active; press Ctrl-C to stop."
      trap '"${ROOT_DIR}/scripts/stop_stair_policy.sh" >/dev/null 2>&1 || true; exit 0' INT TERM
      while pid_alive "${RUN_DIR}/stair_sim.pid" && pid_alive "${RUN_DIR}/stair_bridge.pid"; do
        sleep 1
      done
      echo "[ERROR] stair simulator or RL bridge exited." >&2
      exit 1
    fi
    exit 0
  fi
  if ! pid_alive "${RUN_DIR}/stair_sim.pid" || ! pid_alive "${RUN_DIR}/stair_bridge.pid"; then
    echo "[ERROR] stair simulator or RL bridge exited during startup." >&2
    tail -80 "${LOG_DIR}/stair_mujoco.log" >&2 || true
    tail -80 "${LOG_DIR}/stair_bridge.log" >&2 || true
    exit 1
  fi
  sleep 1
done

echo "[ERROR] policy did not become active within 20 seconds." >&2
tail -80 "${LOG_DIR}/stair_bridge.log" >&2 || true
exit 1
