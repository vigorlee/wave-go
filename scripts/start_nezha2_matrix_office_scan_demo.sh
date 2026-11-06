#!/usr/bin/env bash
# Launch the real MATRiX OfficeWorld scene for Nezha2 and the ROS2 SCAN-style
# planner bridge.  The UE renderer, ray-cast LiDAR and physics scene use the
# same OfficeWorld layout.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ue_model_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
office_pak_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks"

for suffix in pak ucas utoc; do
  if [[ ! -s "${office_pak_root}/pakchunk23-Linux.${suffix}" ]]; then
    echo "[ERROR] OfficeWorld UE package is missing: pakchunk23-Linux.${suffix}" >&2
    echo "[INFO] Install the validated OfficeWorld chunk before launching this demo." >&2
    exit 1
  fi
done

"${root_dir}/scripts/build_nezha2_office_navigation_assets.py" --root "${root_dir}"
"${root_dir}/scripts/prepare_nezha2_matrix_custom_cache.py" >/dev/null

export MATRIX_NEZHA2_SCENE="scene_terrain_office.xml"
export MATRIX_SCENE_ID=13
export MATRIX_UE_CUSTOM_SCENE="${ue_model_root}/xgw/scene_terrain_office.xml"
export MATRIX_UE_CUSTOM_INCLUDE_REWRITE="current.xml"
export MATRIX_SCENE_JSON_OVERRIDE="${root_dir}/matrix/scene/scene_nezha2_office.json"
export ROAMERX_MAP="${root_dir}/genisom_roamerx_open/map/nezha2_office.yaml"
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
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-1280}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-720}"
export MATRIX_UE_SCREEN_PERCENTAGE="${MATRIX_UE_SCREEN_PERCENTAGE:-100}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-1280}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-720}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export NEZHA2_RELAY_RAW_LIDAR=1
export NEZHA2_RL_ENABLE_FLAT_BLEND=1
export NEZHA2_RL_ACTION_ALPHA="${NEZHA2_RL_ACTION_ALPHA:-0.80}"
export NEZHA2_RL_HYBRID_BLEND_STEP="${NEZHA2_RL_HYBRID_BLEND_STEP:-0.025}"

exec "${root_dir}/scripts/start_nezha2_navigation.sh" "$@"
