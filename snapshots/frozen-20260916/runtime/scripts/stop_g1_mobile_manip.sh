#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT_DIR}/.run/g1_mobile_manip.pid"

if [[ ! -f "${PID_FILE}" ]]; then
  echo "[INFO] G1 mobile-manipulation runtime is not tracked."
  exit 0
fi

pid="$(cat "${PID_FILE}")"
if kill -0 "${pid}" 2>/dev/null; then
  kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  echo "[OK] stopped G1 mobile-manipulation runtime, pid=${pid}"
else
  echo "[INFO] tracked G1 mobile-manipulation process already exited, pid=${pid}"
fi
rm -f "${PID_FILE}"
