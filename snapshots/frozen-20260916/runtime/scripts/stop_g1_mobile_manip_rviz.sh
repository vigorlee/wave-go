#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
BRIDGE_PID_FILE="${ROOT_DIR}/.run/g1_mobile_manip_ros_bridge.pid"

"${ROOT_DIR}/scripts/send_g1_mobile_manip.py" stop >/dev/null 2>&1 || true

if [[ -f "${BRIDGE_PID_FILE}" ]]; then
  bridge_pid="$(cat "${BRIDGE_PID_FILE}")"
  if kill -0 "${bridge_pid}" 2>/dev/null; then
    kill -TERM "${bridge_pid}" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "${bridge_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${bridge_pid}" 2>/dev/null; then
      kill -KILL "${bridge_pid}" 2>/dev/null || true
    fi
    echo "[OK] stopped G1 ROS bridge, pid=${bridge_pid}"
  fi
  rm -f "${BRIDGE_PID_FILE}"
fi

ROAMERX_SKIP_PUBTF=1 ROAMERX_SKIP_FORWARD=1 \
  bash "${ROOT_DIR}/matrix/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" stop \
  >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/stop_g1_mobile_manip.sh"
echo "[OK] G1 ROS 2 / RoamerX / RViz stack stopped."
