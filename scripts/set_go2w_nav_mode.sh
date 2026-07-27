#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export STAIR_NAV_MODE_FILE="${GO2W_NAV_MODE_FILE:-${ROOT_DIR}/.run/go2w/nav_mode}"

# Online SLAM/mapless HouseWorld intentionally has no Nav2 parameter servers,
# while the RL bridge still consumes this mode file.
if [[ "${GO2W_NAV_MODE_FILE_ONLY:-0}" == "1" ]]; then
  mode="${1:-status}"
  case "${mode}" in
    avoid|up|down|flat) ;;
    *)
      echo "Usage: $0 {avoid|up|down|flat|status}" >&2
      exit 2
      ;;
  esac
  mkdir -p "$(dirname "${STAIR_NAV_MODE_FILE}")"
  printf '%s\n' "${mode}" >"${STAIR_NAV_MODE_FILE}"
  echo "[OK] Go2-W navigation mode file: ${mode}"
  exit 0
fi

exec "${ROOT_DIR}/scripts/set_g1_stair_nav_mode.sh" "${1:-status}"
