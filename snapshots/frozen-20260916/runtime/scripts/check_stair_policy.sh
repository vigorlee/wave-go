#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"
LOG_FILE="${ROOT_DIR}/logs/stair_bridge.log"
failed=0

check_pid() {
  local label="$1"
  local file="$2"
  if [[ -f "${file}" ]] && kill -0 "$(cat "${file}")" 2>/dev/null; then
    echo "[OK] process: ${label}"
  else
    echo "[FAIL] process: ${label}"
    failed=1
  fi
}

check_pid "MuJoCo" "${RUN_DIR}/stair_sim.pid"
check_pid "DreamWaQ RL bridge" "${RUN_DIR}/stair_bridge.pid"

if grep -q 'Policy control enabled' "${LOG_FILE}" 2>/dev/null; then
  echo "[OK] policy: active"
else
  echo "[FAIL] policy: not active"
  failed=1
fi

if grep -q 'STAIR_SUCCESS' "${LOG_FILE}" 2>/dev/null; then
  echo "[OK] stair result: success"
  grep 'STAIR_SUCCESS' "${LOG_FILE}" | tail -1
elif grep -q 'stage=safety' "${LOG_FILE}" 2>/dev/null; then
  echo "[FAIL] stair result: safety stop"
  failed=1
else
  echo "[INFO] stair result: running or not yet at the platform"
fi

echo "[INFO] latest telemetry:"
grep 'stage=' "${LOG_FILE}" 2>/dev/null | tail -5 || true
exit "${failed}"
