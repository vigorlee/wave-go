#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
BRIDGE_PID_FILE="${ROOT_DIR}/.run/g1_mobile_manip_ros_bridge.pid"
BRIDGE_LOG="${ROOT_DIR}/logs/g1_mobile_manip_ros_bridge.log"

"${ROOT_DIR}/scripts/check_g1_mobile_manip.sh"

if [[ -f "${BRIDGE_PID_FILE}" ]] && kill -0 "$(cat "${BRIDGE_PID_FILE}")" 2>/dev/null; then
  echo "[OK] ROS bridge running, pid=$(cat "${BRIDGE_PID_FILE}")"
else
  echo "[ERROR] ROS bridge not running"
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

echo "[INFO] required ROS interfaces"
topic_list="$(timeout 5 ros2 topic list 2>/dev/null || true)"
action_list="$(timeout 5 ros2 action list 2>/dev/null || true)"
for interface in \
  /cmd_vel /odom/mujoco_odom /joint_states /livox/lidar /imu/data \
  /tf /tf_static /goal_pose /navigate_to_pose \
  /g1/upper_pose /g1/joint_targets /g1/payload_action; do
  if grep -Fqx -- "${interface}" <<<"${topic_list}"; then
    echo "[OK] ${interface}"
  elif [[ "${interface}" == "/navigate_to_pose" ]] &&
       grep -Fqx -- "${interface}" <<<"${action_list}"; then
    echo "[OK] ${interface} (action)"
  else
    echo "[WARN] missing ${interface}"
  fi
done

echo "[INFO] one odometry sample"
timeout 4 ros2 topic echo /odom/mujoco_odom --once \
  --field pose.pose.position 2>/dev/null || true
echo "[INFO] TF map -> base_link"
timeout 4 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | head -12 || true

if [[ -f "${BRIDGE_LOG}" ]]; then
  echo "[INFO] recent bridge log=${BRIDGE_LOG}"
  tail -12 "${BRIDGE_LOG}"
fi
