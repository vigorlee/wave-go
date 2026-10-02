#!/usr/bin/env bash
set -eo pipefail
scene_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${SCENE_ROS_SETUP:-/home/unitree/flycar/tools/setup_noetic.bash}"
mkdir -p "$scene_root/.runtime/native"
scene_output="$scene_root/.runtime/native/libwavego_labels.so"
if [[ ! -f "$scene_output" || "$scene_root/native/chinese_labels.cpp" -nt "$scene_output" ]]; then
  read -r -a scene_flags <<< "$(pkg-config --cflags --libs Qt5Widgets OGRE)"
  g++ -std=c++17 -shared -fPIC -O2 "$scene_root/native/chinese_labels.cpp" \
    -o "$scene_output.tmp" "${scene_flags[@]}" -lrviz
  mv "$scene_output.tmp" "$scene_output"
fi
