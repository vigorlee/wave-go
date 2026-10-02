#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../.."
# The local RoamerX overlay contains a different nav2_planner ABI. Keep the
# upstream planning process isolated from that overlay's shared libraries.
systemctl --user stop go2w-global-planner.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-global-planner \
 --property="WorkingDirectory=$PWD" \
 /usr/bin/env -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
 -u COLCON_PREFIX_PATH -u PYTHONPATH /bin/bash -c \
 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec ros2 run nav2_planner planner_server --ros-args -r __ns:=/global_compare --params-file config/global_compare/planners.yaml'
