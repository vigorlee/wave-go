#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
G1_DIR="${ROOT_DIR}/matrix/src/robot_mujoco/zsibot_robots/g1"
SOURCE_SCENE="${G1_DIR}/scene_terrain_yard.xml"
OUTPUT_SCENE="${G1_DIR}/scene_terrain_yard_policy.xml"

[[ -f "${SOURCE_SCENE}" ]] || {
  echo "[ERROR] G1 YardWorld scene is missing: ${SOURCE_SCENE}" >&2
  exit 1
}

# HumanoidDreamWaQ trains stairs with roughly 0.30 m treads. The packaged
# YardWorld uses 0.20 m treads, so derive a scene with matching horizontal
# spacing while preserving its 0.15 m risers and surrounding environment.
sed \
  -e 's/name="box_OBS2" pos="0.0 1.7 /name="box_OBS2" pos="0.0 1.8 /' \
  -e 's/name="box_OBS3" pos="0.0 1.9 /name="box_OBS3" pos="0.0 2.1 /' \
  -e 's/name="box_OBS4" pos="0.0 2.1 /name="box_OBS4" pos="0.0 2.4 /' \
  -e 's/name="box_OBS5" pos="0.0 2.3 /name="box_OBS5" pos="0.0 2.7 /' \
  -e 's/name="box_OBS6" pos="0.0 2.5 /name="box_OBS6" pos="0.0 3.0 /' \
  -e 's/name="box_OBS7" pos="0.0 2.7 /name="box_OBS7" pos="0.0 3.3 /' \
  -e 's/name="box_OBS8" pos="0.0 2.9 /name="box_OBS8" pos="0.0 3.6 /' \
  -e 's/name="box_OBS9" pos="0.0 3.1 /name="box_OBS9" pos="0.0 3.9 /' \
  -e 's/name="box_OBS10" pos="0.0 3.3 /name="box_OBS10" pos="0.0 4.2 /' \
  -e 's/name="box2_OBS1" pos="0.0 8.5 /name="box2_OBS1" pos="0.0 10.3 /' \
  -e 's/name="box2_OBS2" pos="0.0 8.3 /name="box2_OBS2" pos="0.0 10.0 /' \
  -e 's/name="box2_OBS3" pos="0.0 8.1 /name="box2_OBS3" pos="0.0 9.7 /' \
  -e 's/name="box2_OBS4" pos="0.0 7.9 /name="box2_OBS4" pos="0.0 9.4 /' \
  -e 's/name="box2_OBS5" pos="0.0 7.7 /name="box2_OBS5" pos="0.0 9.1 /' \
  -e 's/name="box2_OBS6" pos="0.0 7.5 /name="box2_OBS6" pos="0.0 8.8 /' \
  -e 's/name="box2_OBS7" pos="0.0 7.3 /name="box2_OBS7" pos="0.0 8.5 /' \
  -e 's/name="box2_OBS8" pos="0.0 7.1 /name="box2_OBS8" pos="0.0 8.2 /' \
  -e 's/name="box2_OBS9" pos="0.0 6.9 /name="box2_OBS9" pos="0.0 7.9 /' \
  -e 's/name="box2_OBS10" pos="0.0 6.7 /name="box2_OBS10" pos="0.0 7.6 /' \
  -e 's/name="box2_OBS11" pos="0.0 5.0 /name="box2_OBS11" pos="0.0 5.9 /' \
  "${SOURCE_SCENE}" >"${OUTPUT_SCENE}"

echo "${OUTPUT_SCENE}"
