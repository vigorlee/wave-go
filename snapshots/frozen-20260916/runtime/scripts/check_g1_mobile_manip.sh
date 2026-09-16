#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="${ROOT_DIR}/.run/g1_mobile_manip.pid"
STATUS_FILE="${ROOT_DIR}/.run/g1_mobile_manip_status.json"
LOG_FILE="${ROOT_DIR}/logs/g1_mobile_manip.log"

if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[OK] runtime running, pid=$(cat "${PID_FILE}")"
else
  echo "[INFO] runtime not running"
fi

if [[ -f "${STATUS_FILE}" ]]; then
  echo "[INFO] status=${STATUS_FILE}"
  python3 -m json.tool "${STATUS_FILE}"
fi

if [[ -f "${LOG_FILE}" ]]; then
  echo "[INFO] recent log=${LOG_FILE}"
  tail -12 "${LOG_FILE}"
fi
