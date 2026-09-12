#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT_DIR}/.run/cosmos_vln_bridge.pid"

"${ROOT_DIR}/scripts/stop_cosmos_vln_mission.sh" >/dev/null 2>&1 || true

if [[ -f "${PID_FILE}" ]]; then
  pid="$(cat "${PID_FILE}")"
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
fi

echo "[OK] Cosmos VLN bridge stopped."
