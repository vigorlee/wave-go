#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"

stop_pid_file() {
  local file="$1"
  [[ -f "${file}" ]] || return 0
  local pid
  pid="$(cat "${file}" 2>/dev/null || true)"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${file}"
}

stop_pid_file "${RUN_DIR}/cosmos_vln_image_view.pid"
stop_pid_file "${RUN_DIR}/cosmos_vln_rviz.pid"
stop_pid_file "${RUN_DIR}/cosmos_vln_visualizer.pid"
stop_pid_file "${RUN_DIR}/cosmos_vln_mission.pid"
stop_pid_file "${RUN_DIR}/cosmos_vln_bridge.pid"

if [[ -x "${RUNTIME_ROOT}/scripts/stop_go2w_systemd.sh" ]]; then
  "${RUNTIME_ROOT}/scripts/stop_go2w_systemd.sh" >/dev/null 2>&1 || true
fi
rm -rf "${RUN_DIR}" 2>/dev/null || true
echo "[OK] Extended demo processes stopped."
