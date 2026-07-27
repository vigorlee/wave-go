#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

fail() {
  echo "[LOCAL_AVOIDANCE_FAIL] $*" >&2
  exit 1
}

topic_type="$(timeout 5 ros2 topic type --no-daemon /livox/lidar 2>/dev/null || true)"
[[ "${topic_type}" == "sensor_msgs/msg/PointCloud2" ]] || \
  fail "/livox/lidar type is '${topic_type:-missing}', expected PointCloud2"

cloud_sample="$(timeout 8 ros2 topic echo /livox/lidar --no-daemon --once \
  --field header.frame_id \
  --qos-reliability best_effort --qos-durability volatile \
  2>/dev/null || true)"
grep -q 'lidar' <<<"${cloud_sample}" || fail "no live LiDAR sample in frame lidar"

tf_sample="$(timeout 8 ros2 run tf2_ros tf2_echo base_link lidar 2>/dev/null || true)"
grep -q 'Translation' <<<"${tf_sample}" || fail "TF base_link -> lidar is unavailable"

param_value() {
  timeout 5 ros2 param get --no-daemon "$1" "$2" 2>/dev/null | awk -F': ' 'NF > 1 {print $NF}'
}

enabled="$(param_value /local_costmap/local_costmap obstacle_layer.enabled)"
min_height="$(param_value /local_costmap/local_costmap obstacle_layer.min_obstacle_height)"
max_height="$(param_value /local_costmap/local_costmap obstacle_layer.max_obstacle_height)"
vx_max="$(param_value /controller_server FollowPath.vx_max)"
cost_critic="$(param_value /controller_server FollowPath.CostCritic.enabled)"

[[ "${enabled,,}" == "true" ]] || fail "local obstacle layer is disabled"
awk -v v="${min_height}" 'BEGIN {exit !(v >= 0.09 && v <= 0.11)}' || \
  fail "min obstacle height is ${min_height:-missing}, expected 0.10 m"
awk -v v="${max_height}" 'BEGIN {exit !(v >= 1.19 && v <= 1.21)}' || \
  fail "max obstacle height is ${max_height:-missing}, expected 1.20 m"
awk -v v="${vx_max}" 'BEGIN {exit !(v > 0.0 && v <= 0.45)}' || \
  fail "avoidance speed cap is ${vx_max:-missing}, expected <= 0.45 m/s"
[[ "${cost_critic,,}" == "true" ]] || fail "MPPI CostCritic is disabled"

mode_file="${GO2W_NAV_MODE_FILE:-${ROOT_DIR}/.run/go2w/nav_mode}"
[[ -f "${mode_file}" ]] || fail "navigation mode file is missing"
[[ "$(<"${mode_file}")" == "avoid" ]] || fail "navigation mode is not avoid"

echo "[LOCAL_AVOIDANCE_OK] lidar=PointCloud2 tf=base_link->lidar height=0.10..1.20m vx_max=${vx_max} MPPI_CostCritic=on"
