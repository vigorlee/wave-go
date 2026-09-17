#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "$0")/common.sh"
sdk="${WAVEGO_V2_SDK_PREFIX:-$V2_ROOT/.runtime/sdk}"
if [[ ! -f "$sdk/lib/liblivox_lidar_sdk_static.a" ]]; then
  mkdir -p "$V2_ROOT/.runtime"
  if [[ ! -d "$V2_ROOT/.runtime/Livox-SDK2/.git" ]]; then git clone https://github.com/Livox-SDK/Livox-SDK2.git "$V2_ROOT/.runtime/Livox-SDK2"; fi
  git -C "$V2_ROOT/.runtime/Livox-SDK2" checkout 08f523c930b2f0ba1e98a6afaa8d7476bf479908
  cmake -S "$V2_ROOT/.runtime/Livox-SDK2" -B "$V2_ROOT/.runtime/sdk-build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX="$sdk"
  cmake --build "$V2_ROOT/.runtime/sdk-build" -j4
  cmake --install "$V2_ROOT/.runtime/sdk-build"
fi
mkdir -p "$V2_ROOT/workspace/src" "$V2_ROOT/.run"
cd "$V2_ROOT/workspace"
catkin build --workspace "$PWD" --no-status -j4 -p2 --cmake-args -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DPYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
