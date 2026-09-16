#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run"
set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"
failed=0

pid_is_live() {
  local pid="$1"
  local state

  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != Z* ]]
}

check_tracked_process() {
  local label="$1"
  local pid_file="$2"
  local fallback_pattern="$3"
  local pid=""

  if [[ -e "${pid_file}" ]]; then
    IFS= read -r pid <"${pid_file}" || true
    if pid_is_live "${pid}"; then
      echo "[OK] process: ${label} (tracked pid=${pid})"
    else
      echo "[FAIL] process: ${label} (stale PID file: ${pid_file})"
      failed=1
    fi
    return
  fi

  if pgrep -f -- "${fallback_pattern}" >/dev/null; then
    echo "[OK] process: ${label} (untracked fallback)"
  else
    echo "[FAIL] process: ${label}"
    failed=1
  fi
}

check_process() {
  local label="$1"
  local pattern="$2"

  if pgrep -f -- "${pattern}" >/dev/null; then
    echo "[OK] process: ${label}"
  else
    echo "[FAIL] process: ${label}"
    failed=1
  fi
}

check_topic() {
  if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "$1"; then
    echo "[OK] topic: $1"
  else
    echo "[FAIL] topic: $1"
    failed=1
  fi
}

check_tracked_process "MATRiX simulator stack" \
  "${RUN_DIR}/sim.pid" \
  '[r]obot_mujoco(_g1)?([[:space:]]|$)|[r]un_sim\.sh[[:space:]]+g1[[:space:]]+3'
check_process "MATRiX UE5" '[z]sibot_mujoco_ue'
check_tracked_process "G1 RL bridge" \
  "${RUN_DIR}/bridge.pid" '[g]1_rl_bridge([[:space:]]|$)'
check_tracked_process "Zenoh router" \
  "${RUN_DIR}/zenohd.pid" '[r]mw_zenohd([[:space:]]|$)'
check_process "RViz2" '[r]viz2([[:space:]]|$)'
check_process "LCM velocity publisher" '[v]el_cmd_lcm_pub([[:space:]]|$)'
check_topic /odom/mujoco_odom
check_topic /livox/lidar
check_topic /plan
check_topic /cmd_vel

if timeout 5 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose; then
  echo "[OK] action: /navigate_to_pose"
else
  echo "[FAIL] action: /navigate_to_pose"
  failed=1
fi

echo "LCM: udpm://239.255.76.67:7667?ttl=255 / vel_cmd_lcm_data"
exit "${failed}"
