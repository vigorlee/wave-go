#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"

for name in oli_sim oli_control oli_cmd_vel_bridge oli_nav; do
  if [[ ! -f "${RUN_DIR}/${name}.pid" ]]; then
    echo "[FAIL] ${name}: no pid file"
    continue
  fi
  pid="$(cat "${RUN_DIR}/${name}.pid")"
  if kill -0 "${pid}" 2>/dev/null; then
    echo "[OK] ${name}: pid=${pid}"
  else
    echo "[FAIL] ${name}: stale pid=${pid}"
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/genisom_roamerx_open/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

timeout 5 ros2 topic echo /odom/mujoco_odom --once >/dev/null
timeout 5 ros2 topic echo /livox/lidar --once >/dev/null
timeout 5 ros2 action list | grep -Fxq /navigate_to_pose
echo "[OK] odometry, lidar, and /navigate_to_pose are ready."

if [[ -f "${ROOT_DIR}/artifacts/oli_navigation/latest_result.json" ]]; then
  cat "${ROOT_DIR}/artifacts/oli_navigation/latest_result.json"
fi
