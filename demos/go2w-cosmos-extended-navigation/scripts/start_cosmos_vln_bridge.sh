#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"
require_runtime
source_ros

PID_FILE="${RUN_DIR}/cosmos_vln_bridge.pid"
CAMERA_TOPIC="${COSMOS_VLN_CAMERA_TOPIC:-/image_raw/compressed}"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos bridge already running, pid=$(cat "${PID_FILE}")." >&2
  exit 1
fi
if [[ ! -x "${COSMOS_VLN_FRAMEWORK}/.venv/bin/python" ]]; then
  echo "[ERROR] Cosmos framework environment missing: ${COSMOS_VLN_FRAMEWORK}/.venv/bin/python" >&2
  exit 1
fi

deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "${CAMERA_TOPIC}"; then
    break
  fi
  sleep 1
done
if ! timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "${CAMERA_TOPIC}"; then
  echo "[ERROR] Camera topic is unavailable: ${CAMERA_TOPIC}" >&2
  exit 1
fi

nohup setsid /usr/bin/python3 "${DEMO_ROOT}/scripts/cosmos_vln_bridge.py" \
  >"${LOG_DIR}/cosmos_vln_bridge.log" 2>&1 &
echo "$!" >"${PID_FILE}"
sleep 2
if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos bridge failed to start." >&2
  tail -n 80 "${LOG_DIR}/cosmos_vln_bridge.log" >&2 || true
  exit 1
fi
echo "[OK] Cosmos route-command bridge started, pid=$(cat "${PID_FILE}")."
