#!/usr/bin/env bash
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$SCENE_ROOT"
source /opt/ros/humble/setup.bash
source /home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916/genisom_roamerx_open/install/setup.bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV
CMAKE_BUILD_PARALLEL_LEVEL=3 /usr/bin/colcon --log-base .runtime/level_lio_logs build --base-paths level_lio_workspace/src --build-base .runtime/level_lio_build --install-base .runtime/level_lio_install --packages-select robot_slam --allow-overriding robot_slam --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt
