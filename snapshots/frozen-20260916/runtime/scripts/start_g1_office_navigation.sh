#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PAK_ROOT="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/Paks"
for suffix in pak ucas utoc; do
  [[ -s "${PAK_ROOT}/pakchunk23-Linux.${suffix}" ]] || {
    echo "[ERROR] OfficeWorld UE package missing: pakchunk23-Linux.${suffix}" >&2
    exit 1
  }
done

"${ROOT_DIR}/scripts/build_g1_office_navigation_assets.py" --root "${ROOT_DIR}"

export G1_WORLD_PROFILE=office
export G1_ROS_DOMAIN_ID="${G1_ROS_DOMAIN_ID:-90}"
export G1_SCENE_ID=13
export ROAMERX_MAP="${ROOT_DIR}/genisom_roamerx_open/map/g1_office.yaml"
export MATRIX_SCENE_JSON_OVERRIDE="${ROOT_DIR}/matrix/scene/scene_nezha2_office.json"
export MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_UE_EGO_VIEW=1
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-10}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-1280}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-720}"
export MATRIX_ROBOT_INITIAL_YAW_DEG="${MATRIX_ROBOT_INITIAL_YAW_DEG:-180}"

exec "${ROOT_DIR}/scripts/start_g1_demo.sh" "$@"
