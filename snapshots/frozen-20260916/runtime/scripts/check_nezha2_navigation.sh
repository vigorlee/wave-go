#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
LOG_DIR="${ROOT_DIR}/logs/nezha2"
set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
failed=0

check_process() {
  if pgrep -f "$2" >/dev/null; then echo "[OK] process: $1"; else echo "[FAIL] process: $1"; failed=1; fi
}
check_topic() {
  if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "$1"; then echo "[OK] topic: $1"; else echo "[FAIL] topic: $1"; failed=1; fi
}
check_tf() {
  local output
  output="$(timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null || true)"
  if grep -q Translation <<<"${output}"; then echo "[OK] TF: $1 -> $2"; else echo "[FAIL] TF: $1 -> $2"; failed=1; fi
}

check_process "MATRiX MuJoCo" '^./robot_mujoco$'
check_process "MATRiX UE" '/zsibot_mujoco_ue .*YardWorld'
check_process "Nezha2 bridge" '/install/bin/nezha2_rl_bridge$'
check_process "robot_forward" '/robot_forward/lib/robot_forward/robot_forward$'
check_process "navigation" 'ros2 launch robot_navigo navigation_bringup.launch.py '
check_topic /odom/mujoco_odom
check_topic /livox/lidar
check_topic /plan
check_topic /cmd_vel_nav
check_topic /cmd_vel
check_tf odom base_link
check_tf base_link lidar

if grep -q 'Policy control enabled' "${LOG_DIR}/bridge.log" 2>/dev/null; then
  echo "[OK] Nezha2 policy active"
else
  echo "[FAIL] Nezha2 policy inactive"
  failed=1
fi

odom_info="$(timeout 5 ros2 topic info /odom/mujoco_odom -v 2>/dev/null || true)"
publisher_count="$(sed -n 's/^Publisher count: //p' <<<"${odom_info}" | head -1)"
if [[ "${publisher_count}" == "1" ]]; then
  echo "[OK] odometry publisher count: 1"
else
  echo "[FAIL] odometry publisher count: ${publisher_count:-unknown}"
  failed=1
fi

echo "[INFO] ROS domain=${ROS_DOMAIN_ID}; expected debug ports=25401/25402"
exit "${failed}"
