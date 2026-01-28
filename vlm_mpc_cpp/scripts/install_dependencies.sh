#!/usr/bin/env bash

# Installs all system and ROS 2 Humble dependencies required by social_mpc_nav.
# Tested on Ubuntu 22.04 with the official ROS 2 apt repositories enabled.

set -euo pipefail

sudo apt update

sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-vcstool \
  ros-humble-desktop \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-interfaces \
  ros-humble-ros-gz-bridge \
  ros-humble-rclcpp \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-tf2-ros \
  ros-humble-std-msgs \
  ros-humble-builtin-interfaces

echo "Dependencies installed. Remember to source ROS 2 before building:"
echo "  source /opt/ros/humble/setup.bash"

