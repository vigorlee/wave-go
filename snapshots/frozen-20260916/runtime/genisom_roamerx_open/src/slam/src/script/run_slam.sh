#! /bin/bash
#source 目录需确定
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source $PACKAGE_DIR/install/setup.bash
ros2 launch robot_slam slam.launch.py
