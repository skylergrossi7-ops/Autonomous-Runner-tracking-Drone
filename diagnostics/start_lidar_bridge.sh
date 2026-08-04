#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

exec ros2 run ros_gz_bridge parameter_bridge \
  '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
