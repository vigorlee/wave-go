#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

stop_pid_file() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] || return 0
  local pid
  pid="$(cat "${pid_file}")"
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
}

stop_pid_file "${ROOT_DIR}/.run/cosmos_vln_rqt_image.pid"
stop_pid_file "${ROOT_DIR}/.run/cosmos_vln_image_view.pid"
stop_pid_file "${ROOT_DIR}/.run/cosmos_vln_rviz.pid"
stop_pid_file "${ROOT_DIR}/.run/cosmos_vln_visualizer.pid"
echo "[OK] Cosmos VLN visualization stopped."
