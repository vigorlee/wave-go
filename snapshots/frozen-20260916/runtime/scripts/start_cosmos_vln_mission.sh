#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
PID_FILE="${ROOT_DIR}/.run/cosmos_vln_mission.pid"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

mkdir -p "${ROOT_DIR}/.run" "${ROOT_DIR}/logs"
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN mission node is already running." >&2
  exit 1
fi

nohup setsid /usr/bin/python3 "${ROOT_DIR}/scripts/cosmos_vln_mission.py" \
  >"${ROOT_DIR}/logs/cosmos_vln_mission.log" 2>&1 &
echo "$!" >"${PID_FILE}"
sleep 1
if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN mission node failed to start." >&2
  tail -n 80 "${ROOT_DIR}/logs/cosmos_vln_mission.log" >&2 || true
  exit 1
fi
echo "[OK] Cosmos VLN high-level mission node started, pid=$(cat "${PID_FILE}")."
