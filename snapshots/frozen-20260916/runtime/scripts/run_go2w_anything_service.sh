#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/genisom_roamerx_open/install/setup.bash"
set -u
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
exec "$ROOT_DIR/.venv-anything/bin/python" scripts/go2w_anything_live.py --output "$1"
