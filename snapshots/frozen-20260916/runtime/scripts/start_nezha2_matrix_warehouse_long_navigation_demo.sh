#!/usr/bin/env bash
# Long-range Nezha2 navigation in MATRiX's no-pedestrian WarehouseWorld.
# Physics, UE rendering, static map, and LiDAR all use the same warehouse
# collision layout; this is intentionally not the pedestrian YardWorld.
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ue_model_root="${root_dir}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"

"${root_dir}/scripts/build_nezha2_warehouse_navigation_assets.py" --root "${root_dir}"
"${root_dir}/scripts/prepare_nezha2_matrix_custom_cache.py"

export MATRIX_NEZHA2_SCENE="scene_terrain_warehouse.xml"
export MATRIX_SCENE_ID=1
export MATRIX_UE_CUSTOM_SCENE="${ue_model_root}/xgw/scene_terrain_wh.xml"
# MATRiX custom rendering resolves ``custom/current.xml`` rather than the
# cache path alone.  prepare_nezha2_matrix_custom_cache.py synchronizes this
# official active entry with the 24-body Nezha2 model before every run.
export MATRIX_UE_CUSTOM_INCLUDE_REWRITE="current.xml"
# The stock scene JSON contains three human walkers.  This warehouse scenario
# intentionally has no dynamic human actors; all navigable obstacles come
# from the 209 collidable warehouse geoms used for physics and the static map.
export MATRIX_SCENE_JSON_OVERRIDE="${root_dir}/matrix/scene/scene_nezha2_warehouse.json"
export ROAMERX_MAP="${root_dir}/genisom_roamerx_open/map/nezha2_warehouse.yaml"
export MATRIX_DISABLE_UE=0
export MATRIX_DISABLE_IMAGE_SENSORS=0
# The old RGB sensor was placed *inside* the 1.1 m Nezha2 chassis
# (x=0.29 m, z=0.01 m), clipping the body into the image.  This genuine UE
# RGB sensor is mounted ahead of the chassis and above the wheel line, giving
# a stable navigation view while leaving the UE ray-cast LiDAR unchanged.
export MATRIX_UE_EGO_VIEW=1
export MATRIX_CAMERA_X="${MATRIX_CAMERA_X:-80}"
export MATRIX_CAMERA_Y="${MATRIX_CAMERA_Y:-0}"
export MATRIX_CAMERA_Z="${MATRIX_CAMERA_Z:-55}"
export MATRIX_CAMERA_ROLL="${MATRIX_CAMERA_ROLL:-0}"
export MATRIX_CAMERA_PITCH="${MATRIX_CAMERA_PITCH:-0}"
export MATRIX_CAMERA_YAW="${MATRIX_CAMERA_YAW:-0}"
export MATRIX_UE_PERFORMANCE_PROFILE="${MATRIX_UE_PERFORMANCE_PROFILE:-1}"
export MATRIX_UE_MAX_FPS="${MATRIX_UE_MAX_FPS:-15}"
# Deliver the robot model at native window resolution.  The previous 75%
# render scale made thin wheel-leg details soft even after the correct
# Nezha2 mesh hierarchy had been loaded.
export MATRIX_UE_RES_X="${MATRIX_UE_RES_X:-1280}"
export MATRIX_UE_RES_Y="${MATRIX_UE_RES_Y:-720}"
export MATRIX_UE_SCREEN_PERCENTAGE="${MATRIX_UE_SCREEN_PERCENTAGE:-100}"
export MATRIX_CAMERA_WIDTH="${MATRIX_CAMERA_WIDTH:-960}"
export MATRIX_CAMERA_HEIGHT="${MATRIX_CAMERA_HEIGHT:-540}"
export MATRIX_CAMERA_FREQUENCY="${MATRIX_CAMERA_FREQUENCY:-5}"
export NEZHA2_MATRIX_DEMO_ONLY=0
export NEZHA2_RELAY_RAW_LIDAR=1
export NEZHA2_RL_TEST_VX=0
export NEZHA2_RL_DEMO_SUCCESS_HOLD=0
export NEZHA2_RL_SUCCESS_AXIS=y
# WarehouseWorld is level ground.  Start from the deterministic flat policy
# and retain the validated stair policy only when the navigation mode requests
# a stair transition or the body pitch proves that a slope was entered.
# Mild action filtering prevents the first local-plan turn from injecting a
# discontinuous wheel command into the wheel-legged base.
export NEZHA2_RL_ENABLE_FLAT_BLEND=1
export NEZHA2_RL_ACTION_ALPHA="${NEZHA2_RL_ACTION_ALPHA:-0.80}"
export NEZHA2_RL_HYBRID_BLEND_STEP="${NEZHA2_RL_HYBRID_BLEND_STEP:-0.025}"

exec "${root_dir}/scripts/start_nezha2_navigation.sh" "$@"
