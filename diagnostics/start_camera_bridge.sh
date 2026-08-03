#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

gz_camera_topic="/tracking_camera/image"

exec ros2 run ros_gz_image image_bridge "${gz_camera_topic}" --ros-args \
  -r "${gz_camera_topic}:=/tracking_camera/image"
