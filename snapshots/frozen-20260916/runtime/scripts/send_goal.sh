#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
X="${1:-2.0}"
Y="${2:-1.0}"
YAW="${3:-0.1}"
FEEDBACK_ARG=()
if [[ "${4:-}" == "--feedback" ]]; then
  FEEDBACK_ARG=(--feedback)
elif [[ -n "${4:-}" ]]; then
  echo "Usage: $0 [x] [y] [yaw_radians] [--feedback]" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
# Preserve the active robot domain. Go2-W defaults to 89 while Nezha2 uses
# 90, so hard-coding 89 here silently sends the goal to a different graph.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"

# A ros2 action CLI can survive a simulator restart while waiting for its old result.
# Remove those stale clients so they cannot replay or preempt the new navigation goal.
pkill -KILL -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 action send_goal /navigate_to_pose ' \
  2>/dev/null || true
sleep 0.2

QZ="$(awk -v yaw="${YAW}" 'BEGIN { printf "%.9f", sin(yaw / 2.0) }')"
QW="$(awk -v yaw="${YAW}" 'BEGIN { printf "%.9f", cos(yaw / 2.0) }')"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}}}" \
  "${FEEDBACK_ARG[@]}"
