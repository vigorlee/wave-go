#! /bin/bash
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source $PACKAGE_DIR/install/setup.bash
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState "{data: 5}"