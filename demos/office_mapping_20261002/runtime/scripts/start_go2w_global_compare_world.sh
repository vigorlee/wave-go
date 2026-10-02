#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export GO2W_RL_STOCHASTIC=1 GO2W_NATIVE_PEOPLE=1 GO2W_MJ_STATE_RELAY=0 MATRIX_MAP_ID=13
export GO2W_WORLD_SCENE="$PWD/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml"
export MATRIX_MUJOCO_SCENE=go2w_lcm/scene_terrain_office_inline.xml
export MATRIX_UE_SCENE_OVERRIDE="$PWD/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_office_inline.xml"
export MATRIX_WORLD_MODEL_VALIDATOR="$PWD/scripts/validate_go2w_office_inline.py"
export ROAMERX_MAP="$PWD/config/go2w_office_course.yaml"
export MATRIX_SCENE_JSON_OVERRIDE="$PWD/matrix/scene/scene_go2w_global_compare.json"
export MATRIX_ROBOT_INITIAL_X="${INLINE_START_X:-8}" MATRIX_ROBOT_INITIAL_Y="${INLINE_START_Y:-0}" MATRIX_ROBOT_INITIAL_YAW_DEG="${INLINE_START_YAW:-180}"
export GO2W_RL_STAIR_FRAME_FILE="$PWD/.run/go2w/stair_frame"
export GO2W_RL_DYNAMIC_YIELD_FILE="$PWD/.run/go2w/native_dynamic_yield"
mkdir -p .run/go2w
printf 'idle 0 0 0 1 10\n' > .run/go2w/stair_frame
export GO2W_RL_STAIR_ALIGNMENT_MAX_HEADING=.35 GO2W_RL_STAIR_ALIGNMENT_VX=.7
export GO2W_RL_STAIR_OBSTACLE_AVOIDANCE=0 GO2W_RL_STAIR_OBSTACLE_MIN_Y=100 GO2W_RL_STAIR_OBSTACLE_MAX_Y=101
export MATRIX_LIDAR_TYPE=mid360 MATRIX_LIDAR_DRAW_POINTS=0 MATRIX_LIDAR_RANDOM_SCAN=1
exec bash scripts/start_go2w_anything_preview.sh
