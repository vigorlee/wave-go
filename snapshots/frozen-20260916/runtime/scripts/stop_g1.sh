#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${ROOT_DIR}/scripts/stop_demo.sh" || true
pkill -TERM -f '/install/bin/g1_rl_bridge($| )' 2>/dev/null || true
pkill -TERM -f 'robot_mujoco|zsibot_mujoco_ue' 2>/dev/null || true
sleep 1
pkill -KILL -f '/install/bin/g1_rl_bridge($| )' 2>/dev/null || true
echo "[OK] G1 simulation stopped."
