#!/usr/bin/env bash
# Shared paths for the public demo recipe.
#
# The repository intentionally does not vendor Matrix/HouseWorld, ROS 2
# workspaces, DreamWaQ weights, or Cosmos3-Edge checkpoints. Point this recipe
# at an existing workstation snapshot with WAVE_GO_RUNTIME_ROOT, or use the
# defaults documented in README.md.

set -euo pipefail

DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_ROOT="${WAVE_GO_RUNTIME_ROOT:-/home/unitree/matrix_go2w_lcm_demo}"
COSMOS_ROOT="${COSMOS_VLN_ROOT:-/home/unitree/matrix_g1_lcm_demo}"
ROAMERX_DIR="${RUNTIME_ROOT}/genisom_roamerx_open"
RUN_DIR="${DEMO_ROOT}/.run"
LOG_DIR="${DEMO_ROOT}/logs"
PHYSICS_SCENE="${DEMO_ROOT}/scene/scene_terrain_yard_extended_nav.xml"
UE_SCENE="${DEMO_ROOT}/scene/scene_terrain_yard_extended_nav_ue.xml"
SCENE_JSON="${DEMO_ROOT}/scene/scene_go2w_extended_nav.json"
ROUTE_FILE="${DEMO_ROOT}/config/cosmos_vln_routes.json"
ROUTE_ID="farthest_end_via_stairs_ramps_cylinders_pedestrians"

export PYTHONPATH="${DEMO_ROOT}/scripts${PYTHONPATH:+:${PYTHONPATH}}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export ROS2CLI_NO_DAEMON="${ROS2CLI_NO_DAEMON:-1}"
export COSMOS_VLN_ROUTES_FILE="${COSMOS_VLN_ROUTES_FILE:-${ROUTE_FILE}}"
export COSMOS_VLN_FRAMEWORK="${COSMOS_VLN_FRAMEWORK:-${COSMOS_ROOT}/packages/cosmos-framework}"
export COSMOS_VLN_CHECKPOINT="${COSMOS_VLN_CHECKPOINT:-${COSMOS_ROOT}/Cosmos3-Edge}"
export COSMOS_VLN_MODEL_CONFIG="${COSMOS_VLN_MODEL_CONFIG:-${COSMOS_VLN_FRAMEWORK}/cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml}"
export GO2W_WORLD_SCENE="${GO2W_WORLD_SCENE:-${PHYSICS_SCENE}}"
export GO2W_SCENE_JSON="${GO2W_SCENE_JSON:-${SCENE_JSON}}"
export MATRIX_WORLD_MODEL_VALIDATOR="${MATRIX_WORLD_MODEL_VALIDATOR:-${DEMO_ROOT}/scripts/validate_go2w_extended_world.py}"
export MATRIX_UE_SCENE_OVERRIDE="${MATRIX_UE_SCENE_OVERRIDE:-${UE_SCENE}}"
export MATRIX_SCENE_JSON_OVERRIDE="${MATRIX_SCENE_JSON_OVERRIDE:-${SCENE_JSON}}"
export MATRIX_MUJOCO_SCENE="${MATRIX_MUJOCO_SCENE:-go2w_lcm/scene_terrain_yard_extended_nav.xml}"
export GO2W_NAV_MODE_FILE="${GO2W_NAV_MODE_FILE:-${RUN_DIR}/go2w/nav_mode}"
export COSMOS_VLN_RVIZ_CONFIG="${COSMOS_VLN_RVIZ_CONFIG:-${DEMO_ROOT}/config/cosmos_vln_extended.rviz}"
export ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-${DEMO_ROOT}/config/go2w_extended_navigation.rviz}"

require_runtime() {
  if [[ ! -d "${RUNTIME_ROOT}" ]]; then
    echo "[ERROR] WAVE_GO_RUNTIME_ROOT does not exist: ${RUNTIME_ROOT}" >&2
    echo "[INFO] Set WAVE_GO_RUNTIME_ROOT to the Matrix/Go2-W runtime workspace." >&2
    return 1
  fi
  if [[ ! -f "${ROAMERX_DIR}/install/setup.bash" ]]; then
    echo "[ERROR] RoamerX install missing: ${ROAMERX_DIR}/install/setup.bash" >&2
    return 1
  fi
}

source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${ROAMERX_DIR}/install/setup.bash"
  set -u
}
