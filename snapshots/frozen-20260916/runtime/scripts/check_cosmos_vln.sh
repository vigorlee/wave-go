#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROAMERX_DIR="${ROOT_DIR}/genisom_roamerx_open"
PID_FILE="${ROOT_DIR}/.run/cosmos_vln_bridge.pid"
VISUALIZER_PID_FILE="${ROOT_DIR}/.run/cosmos_vln_visualizer.pid"
MISSION_PID_FILE="${ROOT_DIR}/.run/cosmos_vln_mission.pid"
MISSION_SCRIPT="${ROOT_DIR}/scripts/cosmos_vln_mission.py"

set +u
source /opt/ros/humble/setup.bash
source "${ROAMERX_DIR}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_zenoh_cpp"
export ROS_DOMAIN_ID="89"

failed=0
if [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null; then
  echo "[OK] process: cosmos_vln_bridge"
else
  echo "[FAIL] process: cosmos_vln_bridge"
  failed=1
fi

for topic in /image_raw/compressed /cosmos_vln/instruction /cosmos_vln/route_command \
             /cosmos_vln/target_pose /cosmos_vln/future_prediction \
             /cosmos_vln/status; do
  if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
    echo "[OK] topic: ${topic}"
  else
    echo "[FAIL] topic: ${topic}"
    failed=1
  fi
done

bridge_info="$(timeout 5 ros2 node info /cosmos_vln_bridge 2>/dev/null || true)"
if grep -Eq '/cmd_vel|/cmd_vel_nav|/navigate_to_pose|vel_cmd_lcm_data' \
     <<<"${bridge_info}"; then
  echo "[FAIL] Cosmos bridge exposes a forbidden motion-control interface"
  failed=1
else
  echo "[OK] Cosmos bridge is command-only (no velocity, trajectory, or Nav2 action)"
fi

if timeout 4 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose; then
  echo "[OK] action: /navigate_to_pose"
else
  echo "[FAIL] action: /navigate_to_pose"
  failed=1
fi

mission_pids=()
while IFS= read -r candidate_pid; do
  [[ -r "/proc/${candidate_pid}/cmdline" ]] || continue
  candidate_cmdline="$(tr '\0' ' ' < "/proc/${candidate_pid}/cmdline")"
  candidate_cmdline="${candidate_cmdline% }"
  if [[ "${candidate_cmdline}" == "/usr/bin/python3 ${MISSION_SCRIPT}" ]]; then
    mission_pids+=("${candidate_pid}")
  fi
done < <(pgrep -f 'cosmos_vln_mission[.]py' 2>/dev/null || true)
mission_pid="$(cat "${MISSION_PID_FILE}" 2>/dev/null || true)"
if [[ "${#mission_pids[@]}" -eq 1 && \
      "${mission_pids[0]}" == "${mission_pid}" ]] && \
   kill -0 "${mission_pid}" 2>/dev/null; then
  echo "[OK] process: cosmos_vln_mission"
  for topic in /cosmos_vln/mission /cosmos_vln/mission_status; do
    if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "[OK] topic: ${topic}"
    else
      echo "[FAIL] topic: ${topic}"
      failed=1
    fi
  done
else
  echo "[FAIL] process: cosmos_vln_mission (expected exactly one, found ${#mission_pids[@]})"
  failed=1
fi

if [[ -x /home/unitree/matrix_g1_lcm_demo/packages/cosmos-framework/.venv/bin/python ]]; then
  echo "[OK] Cosmos inference environment"
else
  echo "[FAIL] Cosmos inference environment"
  failed=1
fi

if [[ -f "${VISUALIZER_PID_FILE}" ]] && \
   kill -0 "$(cat "${VISUALIZER_PID_FILE}")" 2>/dev/null; then
  echo "[OK] process: cosmos_vln_visualizer"
  for topic in /cosmos_vln/annotated_image /cosmos_vln/markers; do
    if timeout 4 ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "[OK] topic: ${topic}"
    else
      echo "[FAIL] topic: ${topic}"
      failed=1
    fi
  done
else
  echo "[INFO] optional visualization is not running"
fi

exit "${failed}"
