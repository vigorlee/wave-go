#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

"${ROOT_DIR}/scripts/stop_cosmos_vln_visualization.sh" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/stop_cosmos_vln_bridge.sh" >/dev/null 2>&1 || true

pkill -KILL -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 action send_goal /navigate_to_pose ' \
  2>/dev/null || true
pkill -KILL -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 action send_goal /navigate_through_poses ' \
  2>/dev/null || true

bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" stop >/dev/null 2>&1 || true

stop_pid() {
  local name="$1"
  local pid_file="${RUN_DIR}/${name}.pid"
  [[ -f "${pid_file}" ]] || return 0
  local pid
  pid="$(cat "${pid_file}")"
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
}

stop_domain_zenohd() {
  local pid
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null | \
       grep -Fxq "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -x rmw_zenohd 2>/dev/null || true)
}

stop_pid lidar_tf
stop_pid world_cloud
stop_pid sim
stop_pid bridge
stop_pid zenohd
stop_pid stair_sim
stop_pid stair_bridge
stop_pid stair_zenohd
stop_domain_zenohd
ros2 daemon stop >/dev/null 2>&1 || true
rm -f "${RUN_DIR}/go2w_navigation.lock"

echo "[OK] demo processes stopped."
