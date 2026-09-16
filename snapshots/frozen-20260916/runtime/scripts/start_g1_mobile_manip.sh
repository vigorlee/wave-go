#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${MATRIX_G1_PYTHON:-/home/unitree/miniconda3/envs/unitree_lerobot/bin/python}"
RUNTIME="${ROOT_DIR}/matrix/src/g1_mobile_manip/g1_mobile_manip.py"
DEFAULT_MISSION="${ROOT_DIR}/config/g1_mobile_manip_stairs.json"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
PID_FILE="${RUN_DIR}/g1_mobile_manip.pid"
STATUS_FILE="${RUN_DIR}/g1_mobile_manip_status.json"
LOG_FILE="${LOG_DIR}/g1_mobile_manip.log"

FOREGROUND=0
HEADLESS=0
REPLACE=0
MISSION="${DEFAULT_MISSION}"
DURATION="30"
COMMAND_PORT="15151"
EXTRA_ARGS=()

while (($#)); do
  case "$1" in
    --foreground) FOREGROUND=1; shift ;;
    --headless) HEADLESS=1; shift ;;
    --replace) REPLACE=1; shift ;;
    --manual) MISSION=""; shift ;;
    --mission)
      [[ $# -ge 2 ]] || { echo "[ERROR] --mission requires a path" >&2; exit 2; }
      MISSION="$2"; shift 2 ;;
    --duration)
      [[ $# -ge 2 ]] || { echo "[ERROR] --duration requires seconds" >&2; exit 2; }
      DURATION="$2"; shift 2 ;;
    --command-port)
      [[ $# -ge 2 ]] || { echo "[ERROR] $1 requires a value" >&2; exit 2; }
      COMMAND_PORT="$2"; EXTRA_ARGS+=("$1" "$2"); shift 2 ;;
    --policy-delay|--action-scale)
      [[ $# -ge 2 ]] || { echo "[ERROR] $1 requires a value" >&2; exit 2; }
      EXTRA_ARGS+=("$1" "$2"); shift 2 ;;
    --realtime) EXTRA_ARGS+=("$1"); shift ;;
    *)
      echo "Usage: $0 [--foreground] [--headless] [--replace] [--manual|--mission FILE] [--duration SECONDS] [--realtime]" >&2
      exit 2 ;;
  esac
done

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] G1 mobile-manipulation runtime is already running, pid=$(cat "${PID_FILE}")" >&2
  exit 1
fi

if pgrep -f 'robot_mujoco_g1|g1_rl_bridge' >/dev/null 2>&1; then
  if (( REPLACE )); then
    "${ROOT_DIR}/scripts/stop_g1.sh" >/dev/null 2>&1 || true
  else
    echo "[ERROR] The existing MATRIX G1 stack is running." >&2
    echo "[ERROR] Stop it with scripts/stop_g1.sh or start this runtime with --replace." >&2
    exit 1
  fi
fi

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "[ERROR] Python runtime not found: ${PYTHON_BIN}" >&2
  exit 1
fi
if [[ -n "${MISSION}" && ! -f "${MISSION}" ]]; then
  echo "[ERROR] Mission file not found: ${MISSION}" >&2
  exit 1
fi
if command -v ss >/dev/null 2>&1 &&
   ss -lun 2>/dev/null | grep -Eq ":${COMMAND_PORT}([[:space:]]|$)"; then
  echo "[ERROR] UDP command port ${COMMAND_PORT} is already in use." >&2
  exit 1
fi

"${PYTHON_BIN}" - <<'PY'
import mujoco, numpy, torch
print(f"[OK] mujoco={mujoco.__version__} numpy={numpy.__version__} torch={torch.__version__}")
PY

COMMAND=("${PYTHON_BIN}" "${RUNTIME}" --duration "${DURATION}" --status-file "${STATUS_FILE}")
(( HEADLESS )) && COMMAND+=(--headless)
[[ -n "${MISSION}" ]] && COMMAND+=(--mission "${MISSION}")
COMMAND+=("${EXTRA_ARGS[@]}")

rm -f "${STATUS_FILE}"
if (( FOREGROUND )); then
  exec "${COMMAND[@]}"
fi

nohup env PYTHONUNBUFFERED=1 "${COMMAND[@]}" >"${LOG_FILE}" 2>&1 &
pid=$!
echo "${pid}" >"${PID_FILE}"

ready=0
for _ in {1..100}; do
  if ! kill -0 "${pid}" 2>/dev/null; then
    break
  fi
  if ss -lun 2>/dev/null | grep -Eq ":${COMMAND_PORT}([[:space:]]|$)"; then
    ready=1
    break
  fi
  sleep 0.1
done
if (( ! ready )); then
  echo "[ERROR] G1 mobile-manipulation runtime failed to bind UDP port ${COMMAND_PORT}" >&2
  tail -80 "${LOG_FILE}" >&2 || true
  kill -TERM "${pid}" 2>/dev/null || true
  rm -f "${PID_FILE}"
  exit 1
fi

echo "[OK] MATRIX G1 mobile-manipulation runtime started, pid=${pid}"
echo "[INFO] mission=${MISSION:-manual}"
echo "[INFO] log=${LOG_FILE}"
echo "[INFO] commands: scripts/send_g1_mobile_manip.py --help"
