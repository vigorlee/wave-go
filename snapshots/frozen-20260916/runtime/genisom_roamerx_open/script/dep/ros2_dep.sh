#! /bin/bash

set -e

# ros control 的一些依赖
install_control_dep() {
  sudo apt-get install -y ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-xacro
}

# pcl 的一些依赖
install_pcl_dep() {
  sudo apt-get install -y ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-pcl-ros
}

# 其他依赖
install_append_dep() {
  sudo apt-get install -y ros-humble-slam-toolbox \
    ros-humble-behaviortree-cpp \
    ros-humble-navigation2 \
    libsdl2-dev \
    libsdl2-ttf-dev \
    libompl-dev \
    ros-humble-rviz2 \
    ros-humble-vision-msgs \
    gcc g++ ninja-build make build-essential git
}

install_ros_dep() {
  install_control_dep
  install_pcl_dep
  install_append_dep
}

install_ros_dep
