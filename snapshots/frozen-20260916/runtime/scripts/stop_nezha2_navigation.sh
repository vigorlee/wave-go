#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MATRIX_DIR="${ROOT_DIR}/matrix"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
RUN_DIR="${ROOT_DIR}/.run/nezha2"

set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="${NEZHA2_ROS_DOMAIN_ID:-90}"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  bash "${MATRIX_DIR}/scripts/run_roamerx_lite_link.sh" "${ROAMERX_DIR}" stop \
  >/dev/null 2>&1 || true

stop_pid() {
  local name="$1"
  local expected="$2"
  local pid_file="${RUN_DIR}/${name}.pid"
  local pid command
  [[ -f "${pid_file}" ]] || return 0
  pid="$(cat "${pid_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    command="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
    if [[ ! "${command}" =~ ${expected} ]]; then
      echo "[WARN] ignoring reused PID ${pid} for ${name}: ${command}" >&2
      rm -f "${pid_file}"
      return 0
    fi
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
}

stop_matching() {
  local pattern="$1" pid
  while read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    kill -TERM "${pid}" 2>/dev/null || true
  done < <(pgrep -f "${pattern}" 2>/dev/null || true)
}

if [[ "${NEZHA2_SKIP_WATCHDOG_STOP:-0}" != "1" ]]; then
  stop_pid runtime_watchdog 'bash -c .*runtime_watchdog|bash -c .*sim_pid'
fi
stop_pid lidar_tf 'static_transform_publisher'
stop_pid sim 'matrix/scripts/run_sim\.sh'
stop_pid bridge 'nezha2_rl_bridge'
# Port 7447 may be a shared router reused by several robot domains.  Do not
# stop it from a robot-specific shutdown path.
rm -f "${RUN_DIR}/zenohd.pid"

# Recover from stale or prematurely removed PID files without touching other
# MATRiX robot bridges.  These patterns are specific to the Nezha2 launcher.
stop_matching '^.*/install/bin/nezha2_rl_bridge$'
stop_matching '^bash .*/matrix/scripts/run_sim\.sh custom (1|3) 0 0 1  nezha2$'
sleep 1
stop_matching '^\./robot_mujoco$'
stop_matching '^.*/zsibot_mujoco_ue .* /Game/Maps/(YardWorld|SceneWorld)( |$)'
echo "[OK] Nezha2 navigation processes stopped."
