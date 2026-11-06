#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/genisom_roamerx_open/install/setup.bash"
set -u
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
/usr/bin/python3 scripts/go2w_dense_depth.py &
depth_pid=$!
if [[ "${GO2W_PRETRAINED:-0}" == 1 ]]; then
  trap 'kill -TERM "$depth_pid" 2>/dev/null || true' EXIT
  wait "$depth_pid"
  exit $?
fi
/usr/bin/python3 scripts/go2w_live_perception.py &
perception_pid=$!
trap 'kill -TERM "$depth_pid" "$perception_pid" 2>/dev/null || true' EXIT
wait -n "$depth_pid" "$perception_pid"
