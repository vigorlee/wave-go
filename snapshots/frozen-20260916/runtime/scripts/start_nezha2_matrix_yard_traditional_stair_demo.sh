#!/usr/bin/env bash
# Real MATRiX YardWorld stairs with live UE LiDAR and the traditional A*
# terrain-aware local planner.  This is intentionally not the standalone
# MuJoCo evidence runner.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ue_content_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content"
ue_model_root="${ue_content_root}/model"

for suffix in pak ucas utoc; do
  if [[ ! -s "${ue_content_root}/Paks/pakchunk13-Linux.${suffix}" ]]; then
    echo "[ERROR] YardWorld chunk is missing: pakchunk13-Linux.${suffix}" >&2
    exit 1
  fi
done

"${root_dir}/scripts/prepare_nezha2_matrix_custom_cache.py" >/dev/null

export MATRIX_NEZHA2_SCENE="scene_terrain_stairs_obstacles.xml"
export MATRIX_SCENE_ID=3
export MATRIX_UE_CUSTOM_SCENE="${ue_model_root}/custom/scene_terrain_nezha2_stairs_obstacles.xml"
export MATRIX_UE_CUSTOM_INCLUDE_REWRITE="current.xml"
export MATRIX_SCENE_JSON_OVERRIDE="${root_dir}/matrix/scene/scene_nezha2_stair_obstacles.json"
export MATRIX_DISABLE_UE=0
export MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_UE_EGO_VIEW=1
export MATRIX_CAMERA_X="${MATRIX_CAMERA_X:-80}"
export MATRIX_CAMERA_Y="${MATRIX_CAMERA_Y:-0}"
export MATRIX_CAMERA_Z="${MATRIX_CAMERA_Z:-55}"
export MATRIX_CAMERA_ROLL="${MATRIX_CAMERA_ROLL:-0}"
export MATRIX_CAMERA_PITCH="${MATRIX_CAMERA_PITCH:-0}"
export MATRIX_CAMERA_YAW="${MATRIX_CAMERA_YAW:-0}"
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-1280}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-720}"
export MATRIX_UE_SCREEN_PERCENTAGE="${MATRIX_UE_SCREEN_PERCENTAGE:-100}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-1280}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-720}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export NEZHA2_RELAY_RAW_LIDAR=1
export NEZHA2_RL_ACTION_ALPHA="${NEZHA2_RL_ACTION_ALPHA:-1.0}"
export NEZHA2_RL_ENABLE_FLAT_BLEND=0
# The terrain policy was validated at 0.62 m/s.  Keep this longitudinal
# command fixed for stair contact; the traditional A* planner still controls
# the lateral route through its yaw command and can stop on no-path.
export NEZHA2_RL_TEST_VX="${NEZHA2_RL_TEST_VX:-0.62}"
export NEZHA2_RL_TEST_DELAY="${NEZHA2_RL_TEST_DELAY:-5.0}"

exec "${root_dir}/scripts/start_nezha2_navigation.sh" "$@"
