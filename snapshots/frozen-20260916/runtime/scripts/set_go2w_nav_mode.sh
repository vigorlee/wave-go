#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
export GO2W_NAV_MODE_FILE="${GO2W_NAV_MODE_FILE:-${ROOT_DIR}/.run/go2w/nav_mode}"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export ROS2CLI_NO_DAEMON="1"

if [[ "${1:-status}" == "status" ]]; then
  exec "${ROOT_DIR}/scripts/set_g1_stair_nav_mode.sh" status
fi
exec /usr/bin/python3 "${ROOT_DIR}/scripts/set_go2w_nav_mode.py" "${1}"
