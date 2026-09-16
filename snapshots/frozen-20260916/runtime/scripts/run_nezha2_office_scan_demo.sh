#!/usr/bin/env bash
# Reproducible real-MATRiX OfficeWorld + SCAN-style planning demo.
# UE publishes the RGB and raw LiDAR; the planner consumes /livox/lidar and
# exclusively writes /cmd_vel_nav while the Nav2 controller is idle.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${1:-${root_dir}/artifacts/nezha2_office_scan_${stamp}}"
duration="${NEZHA2_OFFICE_SCAN_DURATION_SEC:-55}"
mkdir -p "${out_dir}"

"${root_dir}/scripts/start_nezha2_matrix_office_scan_demo.sh" --no-rviz \
  >"${out_dir}/startup.log" 2>&1

set +u
source /opt/ros/humble/setup.bash
source "${root_dir}/genisom_roamerx_open/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"

# Record the UE sensor directly, not the desktop viewport.
/usr/bin/python3 "${root_dir}/scripts/record_nezha2_ue_camera.py" \
  "${out_dir}/ue_office_scan.mp4" --duration "${duration}" --fps 5 \
  >"${out_dir}/camera.log" 2>&1 &
camera_pid=$!

/usr/bin/python3 "${root_dir}/scripts/nezha2_scan_planner_ros2.py" \
  --map "${root_dir}/genisom_roamerx_open/map/nezha2_office.yaml" \
  --log "${out_dir}/scan_runtime.jsonl" \
  >"${out_dir}/scan_planner.log" 2>&1 &
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

if [[ ! -s "${out_dir}/ue_lidar_scan_snapshot.npz" || ! -s "${out_dir}/scan_runtime.jsonl" ]]; then
  echo "[ERROR] SCAN evidence was not recorded; see ${out_dir}/scan_planner.log" >&2
  exit 1
fi
echo "[OK] OfficeWorld SCAN demo: ${out_dir}"
