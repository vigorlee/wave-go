#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"

stop_pid() {
  local name="$1"
  local pid_file="${RUN_DIR}/${name}.pid"
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

stop_pid stair_sim
stop_pid stair_bridge
stop_pid stair_zenohd
echo "[OK] stair policy processes stopped."
