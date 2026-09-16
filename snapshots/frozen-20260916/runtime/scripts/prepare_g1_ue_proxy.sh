#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_ROOT="${ROOT_DIR}/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
G1_DIR="${MODEL_ROOT}/g1"
PROXY_DIR="${MODEL_ROOT}/go2w"
PROXY_MODEL="${PROXY_DIR}/g1_12dof_proxy.xml"
PROXY_SCENE="${PROXY_DIR}/scene_terrain_yard_g1_proxy.xml"
SOURCE_SCENE="${1:-${G1_DIR}/scene_terrain_yard.xml}"
SOURCE_MODEL="${G1_DIR}/g1_12dof.xml"

if grep -q '<include file="g1_oil_upper.xml"/>' "${SOURCE_SCENE}" 2>/dev/null; then
  SOURCE_MODEL="${G1_DIR}/g1_oil_upper.xml"
  PROXY_MODEL="${PROXY_DIR}/g1_oil_upper_proxy.xml"
  PROXY_SCENE="${PROXY_DIR}/scene_terrain_yard_g1_oil_upper_proxy.xml"
fi

if [[ ! -f "${SOURCE_MODEL}" || ! -f "${SOURCE_SCENE}" ]]; then
  echo "[ERROR] G1 UE model files are missing." >&2
  exit 1
fi

# MuJoCo resolves hfield files through the proxy model's meshdir.  OfficeWorld
# uses these two assets even though its floor is otherwise static geometry.
for image_name in height_field.png unitree_hfield.png; do
  image_source="$(dirname "${SOURCE_SCENE}")/${image_name}"
  if [[ -f "${image_source}" ]]; then
    cp "${image_source}" "${G1_DIR}/meshes/${image_name}"
  fi
done

# The packaged UE binary initializes sensors only for supported dispatch names.
# These derived files let its go2w dispatch parse the actual G1 MJCF and meshes.
sed 's|meshdir="meshes/"|meshdir="../g1/meshes/"|' \
  "${SOURCE_MODEL}" >"${PROXY_MODEL}"
sed -E -e 's|<include file="g1_12dof.xml"[[:space:]]*/>|<include file="g1_12dof_proxy.xml"/>|' \
    -e 's|<include file="g1_oil_upper.xml"[[:space:]]*/>|<include file="g1_oil_upper_proxy.xml"/>|' \
    -e 's|file="../height_field.png"|file="height_field.png"|g' \
    -e 's|file="../unitree_hfield.png"|file="unitree_hfield.png"|g' \
  "${SOURCE_SCENE}" >"${PROXY_SCENE}"

echo "${PROXY_SCENE}"
