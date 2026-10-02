#! /bin/bash

set -e

# ROS 2 Humble system packages require the distro Python and its apt modules.
export PATH="/usr/bin:/bin:${PATH}"

source /opt/ros/humble/setup.bash

usage() {
  echo "Usage: $0 [clean] all [debug]"
  echo "./build.sh all             [build all project packages]"
  echo "./build.sh all debug       [build all packages in Debug mode]"
  echo "./build.sh clean all       [clean build/install/log, then build all]"
  echo "./build.sh clean all debug [clean then build all in Debug mode]"
}

# 检查是否传入了参数
if [ $# -eq 0 ]; then
  usage
  exit 1
fi

DO_CLEAN=0
BUILD_TARGET=""
CMAKE_DEBUG_ARGS=""

for arg in "$@"; do
  case "$arg" in
    clean)
      DO_CLEAN=1
      ;;
    all)
      BUILD_TARGET="all"
      ;;
    debug)
      CMAKE_DEBUG_ARGS="-DCMAKE_BUILD_TYPE=Debug"
      ;;
    *)
      echo "Unknown argument: $arg"
      usage
      exit 1
      ;;
  esac
done

if [ -z "$BUILD_TARGET" ]; then
  usage
  exit 1
fi

if [ "$DO_CLEAN" -eq 1 ]; then
  echo "[jszr shell log] => clean build/install/log ..."
  rm -rf build install log
fi

case "$BUILD_TARGET" in
all)
  echo "[jszr shell log] => will compile all project..."
  echo "[jszr shell log] => "

  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON $CMAKE_DEBUG_ARGS --packages-select \
    robots_dog_msgs \
    robot_slam \
    navigo_behavior_tree \
    navigo_behaviors \
    navigo_bt_navigator \
    navigo_collision_monitor \
    navigo_core \
    navigo_costmap_2d \
    navigo_map_server \
    navigo_mppi_controller \
    navigo_path_controller \
    navigo_navfn_planner \
    navigo_path_planner \
    navigo_util \
    navigo_velocity_optimizer \
    navigo_waypoint_follower \
    fast_gicp \
    ndt_omp \
    localization \
    robot_navigo  --parallel-workers 8
  echo "[zsibot shell log] => "
  echo "[zsibot shell log] => OK"
  ;;
*)
  usage
  exit 1
  ;;
esac
