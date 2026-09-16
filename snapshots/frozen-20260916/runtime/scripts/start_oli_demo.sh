#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${ROOT_DIR}/third_party/limx_humanoid_mujoco_sim"
CONTROL_DIR="${ROOT_DIR}/third_party/limx_humanoid_rl_deploy_python"
PYTHON="${ROOT_DIR}/.venv-oli38/bin/python"
MATRIX_PYTHON="${ROOT_DIR}/.venv-oli310/bin/python"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
DEFAULT_WALK_POLICY_DIR="${CONTROL_DIR}/controllers/HU_D04_01/walk_controller/policy/default"
FOREGROUND=0

if [[ "${1:-}" == "--foreground" ]]; then
  FOREGROUND=1
elif [[ -n "${1:-}" ]]; then
  echo "Usage: $0 [--foreground]" >&2
  exit 2
fi

for required in \
  "${PYTHON}" \
  "${MATRIX_PYTHON}" \
  "${SIM_DIR}/simulator.py" \
  "${CONTROL_DIR}/main.py" \
  "${DEFAULT_WALK_POLICY_DIR}/policy.onnx"; do
  if [[ ! -e "${required}" ]]; then
    echo "[ERROR] Missing OLI runtime file: ${required}" >&2
    exit 1
  fi
done

mkdir -p "${RUN_DIR}" "${LOG_DIR}"
GO2W_LOCK="${RUN_DIR}/go2w_navigation.lock"
if [[ -e "${GO2W_LOCK}" ]]; then
  echo "[ERROR] Go2-W navigation is active; stop it before starting OLI." >&2
  exit 1
fi

export ROBOT_TYPE="HU_D04_01"
export ROBOT_IP="127.0.0.1"
export MROS_IP_LIST="127.0.0.x"
export PATH="${ROOT_DIR}/.venv-oli38/bin:${PATH}"
# Navigation always starts from the vendor flat-ground policy unless an
# explicitly validated policy directory is supplied by the caller.
export OLI_WALK_POLICY_DIR="${OLI_WALK_POLICY_DIR:-${DEFAULT_WALK_POLICY_DIR}}"
rm -f "${RUN_DIR}/oli_stair_mode"

cleanup_on_error() {
  local status=$?
  if (( status != 0 )); then
    "${ROOT_DIR}/scripts/stop_oli_demo.sh" >/dev/null 2>&1 || true
  fi
  return "${status}"
}
trap cleanup_on_error EXIT

start_tracked() {
  local name="$1"
  local cwd="$2"
  local command
  shift 2
  printf -v command '%q ' "$@"
  (
    cd "${cwd}"
    # LimX MROS initialization expects a terminal-like file descriptor. `script`
    # supplies a private PTY while preserving normal background operation.
    nohup setsid script --quiet --return --command "${command}" /dev/null \
      >"${LOG_DIR}/${name}.log" 2>&1 &
    echo "$!" >"${RUN_DIR}/${name}.pid"
  )
  echo "[INFO] started ${name}, pid=$(cat "${RUN_DIR}/${name}.pid")"
}

"${ROOT_DIR}/scripts/stop_oli_demo.sh" >/dev/null 2>&1 || true

OLI_MODEL_PATH="$("${PYTHON}" "${ROOT_DIR}/scripts/build_oli_flat_scene.py" | tail -n 1)"
if [[ ! -f "${OLI_MODEL_PATH}" ]]; then
  echo "[ERROR] Failed to build the flat OLI navigation scene." >&2
  exit 1
fi

OLI_MATRIX_UE_STATE=0
OLI_MATRIX_UE_PROXY_MODEL=""
if [[ "${OLI_START_MATRIX_UE:-1}" == "1" ]]; then
  OLI_MATRIX_UE_STATE=1
  OLI_MATRIX_UE_PROXY_MODEL="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/custom/scene_terrain_custom.xml"
  "${MATRIX_PYTHON}" "${ROOT_DIR}/scripts/prepare_oli_matrix_ue.py" \
    >"${LOG_DIR}/oli_matrix_ue_prepare.log"
  if [[ ! -f "${OLI_MATRIX_UE_PROXY_MODEL}" ]]; then
    echo "[ERROR] Failed to build the OLI MATRIX render proxy." >&2
    exit 1
  fi
fi

if pgrep -f 'robot_mujoco|go2w_rl_bridge|g1_rl_bridge|zsibot_mujoco_ue|scripts/start_demo.sh --foreground' \
    >/dev/null 2>&1; then
  echo "[INFO] stopping the previous MATRIX demo stack"
  pkill -TERM -f 'scripts/start_demo.sh --foreground' 2>/dev/null || true
  "${ROOT_DIR}/scripts/stop_demo.sh" >/dev/null 2>&1 || true
  sleep 2
fi

# The communication runtime initializes reliably with the ability manager first.
start_tracked oli_control "${CONTROL_DIR}" env \
  ROBOT_TYPE="${ROBOT_TYPE}" ROBOT_IP="${ROBOT_IP}" \
  MROS_IP_LIST="${MROS_IP_LIST}" PATH="${PATH}" "${PYTHON}" main.py

deadline=$((SECONDS + 30))
while ! grep -q 'Ability loaded: damping' "${LOG_DIR}/oli_control.log" 2>/dev/null || \
      ! grep -q 'Ability loaded: walk' "${LOG_DIR}/oli_control.log" 2>/dev/null; do
  if ! kill -0 "$(cat "${RUN_DIR}/oli_control.pid")" 2>/dev/null; then
    echo "[ERROR] OLI controller exited while loading abilities." >&2
    tail -n 80 "${LOG_DIR}/oli_control.log" >&2 || true
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[ERROR] Timed out while loading OLI controller abilities." >&2
    tail -n 80 "${LOG_DIR}/oli_control.log" >&2 || true
    exit 1
  fi
  sleep 0.2
done

if grep -q 'Error loading ability walk' "${LOG_DIR}/oli_control.log" 2>/dev/null; then
  echo "[ERROR] OLI walk policy failed to load." >&2
  tail -n 80 "${LOG_DIR}/oli_control.log" >&2 || true
  exit 1
fi

# Let the middleware discovery table settle before a second Humanoid endpoint
# joins on the same loopback transport.
sleep 2

# Publish the initial upright state without advancing gravity. Stand will release
# physics only after its first valid command reaches the simulator.
start_tracked oli_sim "${SIM_DIR}" env \
  ROBOT_TYPE="${ROBOT_TYPE}" MROS_IP_LIST="${MROS_IP_LIST}" \
  OLI_MODEL_PATH="${OLI_MODEL_PATH}" \
  OLI_INITIAL_YAW_DEG="${OLI_INITIAL_YAW_DEG:-90}" \
  LIMX_WAIT_FOR_COMMAND=1 OLI_ROS_STATE=1 OLI_ROS_STATE_PORT=15002 \
  OLI_MATRIX_UE_STATE="${OLI_MATRIX_UE_STATE}" OLI_MATRIX_UE_STATE_PORT=9999 \
  OLI_MATRIX_UE_PROXY_MODEL="${OLI_MATRIX_UE_PROXY_MODEL}" \
  LIBGL_ALWAYS_SOFTWARE=1 \
  __GLX_VENDOR_LIBRARY_NAME=mesa "${PYTHON}" simulator.py

deadline=$((SECONDS + 45))
while ! grep -q 'Model File Loaded' "${LOG_DIR}/oli_sim.log" 2>/dev/null; do
  if ! kill -0 "$(cat "${RUN_DIR}/oli_sim.pid")" 2>/dev/null; then
    echo "[ERROR] OLI simulator exited before loading the model." >&2
    tail -n 80 "${LOG_DIR}/oli_sim.log" >&2 || true
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "[ERROR] Timed out while loading the OLI model." >&2
    exit 1
  fi
  sleep 0.2
done

stand_ready=0
deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  if ! kill -0 "$(cat "${RUN_DIR}/oli_sim.pid")" 2>/dev/null || \
     ! kill -0 "$(cat "${RUN_DIR}/oli_control.pid")" 2>/dev/null; then
    echo "[ERROR] OLI simulator or controller exited during startup." >&2
    tail -n 80 "${LOG_DIR}/oli_sim.log" "${LOG_DIR}/oli_control.log" >&2 || true
    exit 1
  fi

  if "${PYTHON}" -m limxsdk.ability.cli \
      switch 'mimic damping walk' 'stand' >/dev/null 2>&1; then
    stand_ready=1
    break
  fi
  sleep 0.5
done

if (( ! stand_ready )); then
  echo "[ERROR] Timed out while activating the OLI Stand controller." >&2
  exit 1
fi

deadline=$((SECONDS + 10))
while (( SECONDS < deadline )); do
  if grep -q 'First valid robot command received; physics started' \
      "${LOG_DIR}/oli_sim.log" 2>/dev/null; then
    echo "[OK] Full OLI HU_D04_01 is standing under active control."
    if [[ "${OLI_START_MATRIX_UE:-1}" == "1" ]]; then
      "${ROOT_DIR}/scripts/start_oli_matrix_ue.sh"
    else
      echo "[INFO] MATRIX UE startup skipped (OLI_START_MATRIX_UE=0)."
    fi
    echo "[INFO] MuJoCo log: ${LOG_DIR}/oli_sim.log"
    echo "[INFO] Controller log: ${LOG_DIR}/oli_control.log"
    if (( FOREGROUND )); then
      trap '"${ROOT_DIR}/scripts/stop_oli_demo.sh"; exit 0' INT TERM
      while kill -0 "$(cat "${RUN_DIR}/oli_sim.pid")" 2>/dev/null && \
            kill -0 "$(cat "${RUN_DIR}/oli_control.pid")" 2>/dev/null; do
        sleep 1
      done
    fi
    exit 0
  fi
  sleep 0.1
done

echo "[ERROR] Stand was selected, but MuJoCo received no valid command." >&2
tail -n 80 "${LOG_DIR}/oli_sim.log" "${LOG_DIR}/oli_control.log" >&2 || true
exit 1
