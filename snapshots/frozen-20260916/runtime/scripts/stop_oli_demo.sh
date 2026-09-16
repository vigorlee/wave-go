#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${ROOT_DIR}/.run"
rm -f "${RUN_DIR}/oli_navigation.armed"
rm -f "${RUN_DIR}/oli_stair_mode"

for name in oli_rviz oli_nav oli_robot_state_publisher oli_cmd_vel_bridge oli_zenohd oli_matrix_ue oli_control oli_sim; do
  pid_file="${RUN_DIR}/${name}.pid"
  if [[ -f "${pid_file}" ]]; then
    pid="$(cat "${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
      for _ in {1..30}; do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
      done
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
    rm -f "${pid_file}"
  fi
done

pkill -TERM -f '/third_party/limx_humanoid_mujoco_sim/prebuild/kinematic_projection' \
  2>/dev/null || true
pkill -TERM -f '/.venv-oli310/bin/python simulator.py' 2>/dev/null || true
pkill -TERM -f '/.venv-oli310/bin/python main.py' 2>/dev/null || true
pkill -TERM -f '/.venv-oli38/bin/python simulator.py' 2>/dev/null || true
pkill -TERM -f '/.venv-oli38/bin/python main.py' 2>/dev/null || true
pkill -TERM -f '/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue' 2>/dev/null || true

# PID files may be gone after an interrupted launch. These processes still
# conflict with Go2-W on ROS domain 89 and the UE state port, so clean them by
# their OLI-specific command lines as well.
OLI_PROCESS_PATTERNS=(
  'robot_state_publisher.*HU_D04_description'
  'limxsdk\.ability\.cli load.*HU_D04_01'
  'script --quiet.*ROBOT_TYPE=HU_D04_01'
)
for pattern in "${OLI_PROCESS_PATTERNS[@]}"; do
  pkill -TERM -f "${pattern}" 2>/dev/null || true
done
sleep 0.5
for pattern in "${OLI_PROCESS_PATTERNS[@]}"; do
  pkill -KILL -f "${pattern}" 2>/dev/null || true
done

systemctl --user stop oli-matrix-ue.service >/dev/null 2>&1 || true

echo "[OK] OLI simulator and controller stopped."
