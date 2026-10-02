#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
set +u
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
exec /usr/bin/python3 scripts/go2w_manual_control.py
