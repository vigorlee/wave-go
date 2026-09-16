#!/usr/bin/env bash
# Run the Nezha2 wheel-legged robot in MATRiX YardWorld with the real
# long-range obstacle layout, UE LiDAR, and RoamerX/Nav2 planning.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ue_model_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"

# Both the MuJoCo physics world and the UE world derive from this YardWorld
# scene. The custom runtime entry is rewritten to include the Nezha2 model,
# rather than the unrelated model/custom/current.xml proxy.
export MATRIX_NEZHA2_SCENE="scene_terrain_yard.xml"
export MATRIX_SCENE_ID=3
export MATRIX_UE_CUSTOM_SCENE="${ue_model_root}/nezha2/scene_terrain_yard.xml"
export MATRIX_UE_CUSTOM_INCLUDE_REWRITE="../nezha2/nezha2_yard.xml"
export MATRIX_DISABLE_UE=0
export MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-960}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-540}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-2}"
export NEZHA2_MATRIX_DEMO_ONLY=0
export NEZHA2_RELAY_RAW_LIDAR=1
export NEZHA2_RL_TEST_VX=0
export NEZHA2_RL_DEMO_SUCCESS_HOLD=0
export NEZHA2_RL_SUCCESS_AXIS=y

exec "${root_dir}/scripts/start_nezha2_navigation.sh"
