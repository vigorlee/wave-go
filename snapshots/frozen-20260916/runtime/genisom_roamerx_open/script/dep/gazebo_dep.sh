#! /bin/bash

# install gazebo
install_gazebo_dep() {
  sudo apt-get install -y ros-humble-gazebo-dev \
    ros-humble-gazebo-model-attachment-plugin \
    ros-humble-gazebo-model-attachment-plugin-msgs \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-no-physics-plugin \
    ros-humble-gazebo-planar-move-plugin \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros2-control-demos \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-set-joint-positions-plugin \
    ros-humble-gazebo-video-monitor-interfaces \
    ros-humble-gazebo-video-monitor-plugins \
    ros-humble-gazebo-video-monitors
}

install_gazebo_dep