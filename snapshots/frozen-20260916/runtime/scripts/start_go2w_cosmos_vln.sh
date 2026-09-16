#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-1}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_ROBOT_INITIAL_X="${MATRIX_ROBOT_INITIAL_X:-0.2}"
export MATRIX_ROBOT_INITIAL_Y="${MATRIX_ROBOT_INITIAL_Y:-0.0}"
export MATRIX_ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-90}"
export ROAMERX_RVIZ_READ_ONLY="${ROAMERX_RVIZ_READ_ONLY:-1}"
# Cosmos publishes only an allowlisted route command. The mission supervisor
# resolves that route to a global target and is the sole owner of Nav2 goals.

"${ROOT_DIR}/scripts/start_go2w_systemd.sh"
"${ROOT_DIR}/scripts/start_cosmos_vln_bridge.sh"
"${ROOT_DIR}/scripts/start_cosmos_vln_mission.sh"

echo "[OK] Go2-W navigation, Cosmos VLN bridge, and mission node are running."
