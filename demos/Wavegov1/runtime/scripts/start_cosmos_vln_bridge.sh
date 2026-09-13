#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
LOG_DIR="${ROOT_DIR}/logs"
PID_FILE="${RUN_DIR}/cosmos_vln_bridge.pid"
CAMERA_TOPIC="${COSMOS_VLN_CAMERA_TOPIC:-/image_raw/compressed}"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN bridge is already running, pid=$(cat "${PID_FILE}")." >&2
  exit 1
fi

if [[ ! -x /home/unitree/matrix_g1_lcm_demo/packages/cosmos-framework/.venv/bin/python ]]; then
  echo "[ERROR] Cosmos inference environment is missing." >&2
  exit 1
fi

deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "${CAMERA_TOPIC}"; then
    break
  fi
  sleep 1
done
if ! timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "${CAMERA_TOPIC}"; then
  echo "[ERROR] Camera topic ${CAMERA_TOPIC} is unavailable." >&2
  echo "[INFO] Start Go2-W with scripts/start_go2w_cosmos_vln.sh so image sensors are enabled." >&2
  exit 1
fi

nohup setsid /usr/bin/python3 "${ROOT_DIR}/scripts/cosmos_vln_bridge.py" \
  >"${LOG_DIR}/cosmos_vln_bridge.log" 2>&1 &
echo "$!" >"${PID_FILE}"
sleep 2
if ! kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[ERROR] Cosmos VLN bridge failed to start." >&2
  tail -n 80 "${LOG_DIR}/cosmos_vln_bridge.log" >&2 || true
  exit 1
fi

echo "[OK] Cosmos VLN bridge started, pid=$(cat "${PID_FILE}")."
echo "[INFO] Submit a high-level task through /cosmos_vln/mission."
