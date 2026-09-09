#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"
require_runtime
source_ros

PID_FILE="${RUN_DIR}/cosmos_vln_mission.pid"
mkdir -p "${RUN_DIR}" "${LOG_DIR}"
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos mission supervisor already running." >&2
  exit 1
fi

nohup setsid /usr/bin/python3 "${DEMO_ROOT}/scripts/cosmos_vln_mission.py" \
  >"${LOG_DIR}/cosmos_vln_mission.log" 2>&1 &
echo "$!" >"${PID_FILE}"
sleep 1
if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos mission supervisor failed to start." >&2
  tail -n 80 "${LOG_DIR}/cosmos_vln_mission.log" >&2 || true
  exit 1
fi
echo "[OK] Cosmos mission supervisor started, pid=$(cat "${PID_FILE}")."
