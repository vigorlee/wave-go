#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
/usr/bin/python3 scripts/generate_go2w_native_people_scene.py
export GO2W_NATIVE_PEOPLE=1 MATRIX_MAP_ID=4
export MATRIX_UE_SCENE_OVERRIDE="$ROOT_DIR/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_native_people.xml"
export MATRIX_ROBOT_INITIAL_X="${MATRIX_ROBOT_INITIAL_X:-0.2}" MATRIX_ROBOT_INITIAL_Y="${MATRIX_ROBOT_INITIAL_Y:-19}"
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/native_people_$(date +%Y%m%d_%H%M%S)}"
exec bash scripts/start_go2w_anything_preview.sh
