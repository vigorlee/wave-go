#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
/usr/bin/python3 scripts/build_go2w_office_course.py
export GO2W_RL_STOCHASTIC=1
export GO2W_NATIVE_PEOPLE=1 GO2W_MJ_STATE_RELAY=0 MATRIX_MAP_ID=13
export GO2W_WORLD_SCENE="$ROOT_DIR/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_course.xml"
export MATRIX_MUJOCO_SCENE=go2w_lcm/scene_terrain_office_course.xml
export MATRIX_UE_SCENE_OVERRIDE="$ROOT_DIR/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_office_course.xml"
export MATRIX_WORLD_MODEL_VALIDATOR="$ROOT_DIR/scripts/validate_go2w_office_course.py"
export ROAMERX_MAP="$ROOT_DIR/config/go2w_office_course.yaml"
export MATRIX_SCENE_JSON_OVERRIDE="$ROOT_DIR/matrix/scene/scene_go2w_office_course.json"
export MATRIX_ROBOT_INITIAL_X=-3.2 MATRIX_ROBOT_INITIAL_Y=-6.0 MATRIX_ROBOT_INITIAL_YAW_DEG=90
export GO2W_RL_STAIR_ROUTE_CENTER_X=-3.2
export GO2W_RL_STAIR_OUTBOUND_MIN_Y=-6.2 GO2W_RL_STAIR_OUTBOUND_MAX_Y=-2.2
export GO2W_RL_STAIR_OUTBOUND_FAST_Y=-4.8 GO2W_RL_STAIR_OUTBOUND_SLOW_Y=-3.3
export GO2W_RL_STAIR_RETURN_MIN_Y=.2 GO2W_RL_STAIR_RETURN_MAX_Y=4.2
export GO2W_RL_STAIR_OBSTACLE_MIN_Y=-3.45 GO2W_RL_STAIR_OBSTACLE_MAX_Y=-2.3
export GO2W_RL_STAIR_EXIT_VX=.8
export GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING=.40 GO2W_RL_STAIR_ALIGNMENT_VX=.90
export MATRIX_LIDAR_TYPE=mid360 MATRIX_LIDAR_DRAW_POINTS=0 MATRIX_LIDAR_RANDOM_SCAN=1
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/office_course_$(date +%Y%m%d_%H%M%S)}"
exec bash scripts/start_go2w_anything_preview.sh
