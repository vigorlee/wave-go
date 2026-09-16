#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
G1_DIR="${ROOT_DIR}/matrix/src/robot_mujoco/zsibot_robots/g1"
SOURCE_MODEL="${G1_DIR}/g1_12dof.xml"
OUTPUT_MODEL="${G1_DIR}/g1_oil_upper.xml"
SOURCE_SCENE="${1:-${G1_DIR}/scene_terrain_yard.xml}"
OUTPUT_SCENE="${SOURCE_SCENE%.xml}_oil_upper.xml"
OIL_CONFIG="${ROOT_DIR}/third_party/oil_upper_body/upper_body/arm/D04_01/arm_config.yaml"
OLI_DESCRIPTION="${ROOT_DIR}/third_party/limx_humanoid_mujoco_sim/humanoid-description/HU_D04_description"
OLI_MODEL="${OLI_DESCRIPTION}/xml/HU_D04_01.xml"
OLI_MESHES="${OLI_DESCRIPTION}/meshes/HU_D04_01"
UE_G1_DIR="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/g1"

if [[ ! -f "${SOURCE_MODEL}" || ! -f "${SOURCE_SCENE}" || ! -f "${OIL_CONFIG}" || ! -f "${OLI_MODEL}" ]]; then
  echo "[ERROR] G1 model/scene or official Oli HU_D04_01 assets are missing." >&2
  exit 1
fi

python3 "${ROOT_DIR}/scripts/build_g1_oil_upper.py" \
  "${SOURCE_MODEL}" "${OLI_MODEL}" "${OLI_MESHES}" \
  "${OUTPUT_MODEL}" "${G1_DIR}/meshes/oil"

python3 "${ROOT_DIR}/scripts/build_g1_oil_upper.py" \
  "${UE_G1_DIR}/g1_12dof.xml" "${OLI_MODEL}" "${OLI_MESHES}" \
  "${UE_G1_DIR}/g1_oil_upper.xml" "${UE_G1_DIR}/meshes/oil"

sed 's|<include file="g1_12dof.xml"/>|<include file="g1_oil_upper.xml"/>|' \
  "${SOURCE_SCENE}" >"${OUTPUT_SCENE}"

echo "${OUTPUT_SCENE}"
