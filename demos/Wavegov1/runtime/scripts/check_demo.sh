#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

failed=0

check_process() {
  local label="$1"
  local pattern="$2"
  if pgrep -f "${pattern}" >/dev/null; then
    echo "[OK] process: ${label}"
  else
    echo "[FAIL] process: ${label}"
    failed=1
  fi
}

check_topic() {
  local topic="$1"
  if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "[OK] topic: ${topic}"
  else
    echo "[FAIL] topic: ${topic}"
    failed=1
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  local output
  output="$(timeout 6 ros2 run tf2_ros tf2_echo "${parent}" "${child}" 2>/dev/null || true)"
  if grep -q 'Translation' <<<"${output}"; then
    echo "[OK] TF: ${parent} -> ${child}"
  else
    echo "[FAIL] TF: ${parent} -> ${child}"
    failed=1
  fi
}

check_process "MATRiX MuJoCo" "robot_mujoco"
check_process "MATRiX UE5" "zsibot_mujoco_ue"
check_process "Go2-W controller bridge" "go2w_(lcm|rl)_bridge"
check_process "Zenoh router" "rmw_zenohd"
check_process "robot_forward" "robot_forward"
check_process "LCM velocity publisher" "vel_cmd_lcm_pub"

check_topic /odom/mujoco_odom
check_topic /livox/lidar
check_topic /plan
check_topic /cmd_vel_nav
check_topic /cmd_vel

check_tf odom base_link
check_tf base_link lidar

if "${ROOT_DIR}/scripts/validate_go2w_local_avoidance.sh"; then
  :
else
  failed=1
fi

echo
echo "ROS nodes: $(timeout 5 ros2 node list 2>/dev/null | wc -l)"
echo "LCM: udpm://239.255.76.67:7667?ttl=255 / vel_cmd_lcm_data"
exit "${failed}"
