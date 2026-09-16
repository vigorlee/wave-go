#!/usr/bin/env bash
# Real MATRiX CustomWorld: visible short stairs, a third-tread obstacle, and
# the Nezha2 policy/UE LiDAR use the same scene definitions.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ue_content_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content"
ue_model_root="${ue_content_root}/model"
for suffix in pak ucas utoc; do
  [[ -s "${ue_content_root}/Paks/pakchunk24-Linux.${suffix}" ]] || { echo "[ERROR] CustomWorld chunk missing" >&2; exit 1; }
done

"${root_dir}/scripts/prepare_nezha2_matrix_custom_cache.py" >/dev/null
export MATRIX_NEZHA2_SCENE="scene_terrain_stairs_obstacles.xml"
export MATRIX_SCENE_ID=0
export MATRIX_UE_CUSTOM_SCENE="${ue_model_root}/custom/scene_terrain_nezha2_stairs_obstacles.xml"
export MATRIX_UE_CUSTOM_INCLUDE_REWRITE="current.xml"
export MATRIX_SCENE_JSON_OVERRIDE="${root_dir}/matrix/scene/scene_nezha2_stair_obstacles.json"
export MATRIX_DISABLE_UE=0 MATRIX_DISABLE_IMAGE_SENSORS=0 MATRIX_UE_EGO_VIEW=1
export MATRIX_CAMERA_X="${MATRIX_CAMERA_X:-80}" MATRIX_CAMERA_Y="${MATRIX_CAMERA_Y:-0}" MATRIX_CAMERA_Z="${MATRIX_CAMERA_Z:-55}"
export MATRIX_CAMERA_ROLL=0 MATRIX_CAMERA_PITCH=0 MATRIX_CAMERA_YAW=0
export MATRIX_UE_PERFORMANCE_PROFILE=1 MATRIX_UE_RES_X=1280 MATRIX_UE_RES_Y=720 MATRIX_UE_SCREEN_PERCENTAGE=100 MATRIX_UE_MAX_FPS=15
export MATRIX_CAMERA_WIDTH=1280 MATRIX_CAMERA_HEIGHT=720 MATRIX_CAMERA_FREQUENCY=5
# CustomWorld reports point coordinates in UE centimetres.  Put the test
# LiDAR above and 1.2 m ahead of the base so it can observe the fifth tread
# rather than being occluded by the first riser.
export MATRIX_LIDAR_X="${MATRIX_LIDAR_X:-120}" MATRIX_LIDAR_Y="${MATRIX_LIDAR_Y:-0}" MATRIX_LIDAR_Z="${MATRIX_LIDAR_Z:-70}"
export MATRIX_LIDAR_PITCH="${MATRIX_LIDAR_PITCH:-0}" MATRIX_LIDAR_YAW="${MATRIX_LIDAR_YAW:-0}" MATRIX_LIDAR_FREQUENCY=10
export NEZHA2_RELAY_RAW_LIDAR=1 NEZHA2_RL_ENABLE_FLAT_BLEND=0 NEZHA2_RL_ACTION_ALPHA=1.0
export NEZHA2_RL_LEGACY_LEG_ORDER=1 NEZHA2_RL_TEST_VX="${NEZHA2_RL_TEST_VX:-0.62}" NEZHA2_RL_TEST_DELAY=5
export NEZHA2_STAIR_RISE=0.05 NEZHA2_STAIR_COUNT=10 NEZHA2_STAIR_GOAL_X=0.65 NEZHA2_STAIR_GOAL_Y=4.80 NEZHA2_STAIR_MAP_Y_MAX=5.30
export NEZHA2_UE_LIDAR_SCALE=0.01
exec "${root_dir}/scripts/start_nezha2_navigation.sh" "$@"
