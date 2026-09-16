#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
# A small lateral offset prevents both wheel pairs from meeting aligned stair
# risers at the same instant.  Callers can still provide any explicit goal.
"${ROOT_DIR}/scripts/set_nezha2_stair_nav_mode.sh" up
exec "${ROOT_DIR}/scripts/send_goal.sh" "${1:-0.65}" "${2:-4.8}" "${3:-1.570796}" "${4:-}"
