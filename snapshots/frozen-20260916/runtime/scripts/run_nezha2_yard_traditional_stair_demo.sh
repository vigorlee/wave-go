#!/usr/bin/env bash
# Record an actual YardWorld RGB/LiDAR run while the traditional stair A*
# planner owns /cmd_vel_nav.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${1:-${root_dir}/artifacts/nezha2_yard_traditional_stair_${stamp}}"
duration="${NEZHA2_YARD_STAIR_DURATION_SEC:-55}"
mkdir -p "${out_dir}"

"${root_dir}/scripts/start_nezha2_matrix_yard_traditional_stair_demo.sh" --no-rviz \
  >"${out_dir}/startup.log" 2>&1

set +u
source /opt/ros/humble/setup.bash
source "${root_dir}/genisom_roamerx_open/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
"${root_dir}/scripts/set_nezha2_stair_nav_mode.sh" up >"${out_dir}/stair_mode.log" 2>&1

/usr/bin/python3 "${root_dir}/scripts/record_nezha2_ue_camera.py" \
  "${out_dir}/ue_yard_stair.mp4" --duration "${duration}" --fps 5 \
  >"${out_dir}/camera.log" 2>&1 &
camera_pid=$!
/usr/bin/python3 "${root_dir}/scripts/nezha2_traditional_stair_planner_ros2.py" \
  --output-dir "${out_dir}" >"${out_dir}/planner.log" 2>&1 &
planner_pid=$!

cleanup() {
  kill -TERM "${planner_pid}" 2>/dev/null || true
  wait "${planner_pid}" 2>/dev/null || true
  ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
wait "${camera_pid}"
cleanup
trap - EXIT INT TERM

if [[ ! -s "${out_dir}/ue_stair_lidar_snapshot.npz" || ! -s "${out_dir}/traditional_stair_planner.jsonl" ]]; then
  echo "[ERROR] traditional stair evidence was not recorded" >&2
  exit 1
fi
echo "[OK] YardWorld traditional stair demo: ${out_dir}"
