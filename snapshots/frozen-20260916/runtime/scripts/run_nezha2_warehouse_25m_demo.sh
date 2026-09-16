#!/usr/bin/env bash
# Reproducible MATRiX WarehouseWorld navigation evidence.
# This records the official no-pedestrian warehouse renderer while Nezha2
# follows a 25 m Nav2 goal.  It does not fabricate images or point clouds.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date +%Y%m%d_%H%M%S)"
out_dir="${1:-${root_dir}/artifacts/nezha2_warehouse_25m_${stamp}}"
mkdir -p "${out_dir}"

"${root_dir}/scripts/start_nezha2_matrix_warehouse_long_navigation_demo.sh" --no-rviz \
  >"${out_dir}/startup.log" 2>&1

ue_window=""
for _ in {1..30}; do
  ue_window="$(wmctrl -lp 2>/dev/null | awk '/zsibot_mujoco_ue/ {print $1; exit}')"
  [[ -n "${ue_window}" ]] && break
  sleep 1
done
if [[ -z "${ue_window}" ]]; then
  echo "[ERROR] MATRiX UE window was not found; see ${out_dir}/startup.log" >&2
  exit 1
fi

# Subscribe directly to the UE RGB sensor rather than recording the desktop
# viewport.  This is the real SceneWorld camera stream, free of GUI overlays
# and the old in-chassis view artefact.
/usr/bin/python3 "${root_dir}/scripts/record_nezha2_ue_camera.py" \
  "${out_dir}/ue_warehouse_25m_sensor.mp4" --duration 78 --fps 5 \
  >"${out_dir}/capture.log" 2>&1 &
capture_pid=$!

set +u
source /opt/ros/humble/setup.bash
source "${root_dir}/genisom_roamerx_open/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"
if timeout 85 bash "${root_dir}/scripts/send_goal.sh" 25.0 0.0 0.0 --feedback \
  >"${out_dir}/goal_25m.log" 2>&1; then
  goal_status=0
else
  goal_status=$?
fi

wait "${capture_pid}" || true
printf 'goal_exit_status=%s\n' "${goal_status}" >"${out_dir}/result.txt"
[[ "${goal_status}" -eq 0 ]]
