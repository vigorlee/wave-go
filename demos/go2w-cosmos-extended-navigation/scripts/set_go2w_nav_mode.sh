#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/demo_env.sh"
require_runtime
source_ros

mode="${1:-status}"
if [[ "${mode}" == "status" ]]; then
  if [[ -f "${GO2W_NAV_MODE_FILE}" ]]; then
    cat "${GO2W_NAV_MODE_FILE}"
  else
    echo "unknown"
  fi
  exit 0
fi
exec /usr/bin/python3 "${DEMO_ROOT}/scripts/set_go2w_nav_mode.py" "${mode}"
