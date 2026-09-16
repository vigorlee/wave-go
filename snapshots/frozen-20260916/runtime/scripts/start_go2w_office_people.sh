#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
export GO2W_NATIVE_PEOPLE=1 GO2W_MJ_STATE_RELAY=0 MATRIX_MAP_ID=13
export GO2W_WORLD_SCENE="$ROOT_DIR/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office.xml"
export MATRIX_MUJOCO_SCENE=go2w_lcm/scene_terrain_office.xml
export MATRIX_UE_SCENE_OVERRIDE="$ROOT_DIR/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_office.xml"
cp "$GO2W_WORLD_SCENE" "$MATRIX_UE_SCENE_OVERRIDE"
export MATRIX_WORLD_MODEL_VALIDATOR="$ROOT_DIR/scripts/validate_go2w_office.py"
export ROAMERX_MAP="$ROOT_DIR/genisom_roamerx_open/map/g1_office.yaml"
export MATRIX_SCENE_JSON_OVERRIDE="$ROOT_DIR/matrix/scene/scene_nezha2_office.json"
export MATRIX_ROBOT_INITIAL_X=0 MATRIX_ROBOT_INITIAL_Y=0 MATRIX_ROBOT_INITIAL_YAW_DEG=180
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/office_people_$(date +%Y%m%d_%H%M%S)}"
exec bash scripts/start_go2w_anything_preview.sh
