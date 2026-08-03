#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
follower_enabled="${FOLLOWER_ENABLED:-true}"

exec ros2 run drone_control runner_follower \
  --ros-args \
  --params-file /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/drone_control/share/drone_control/config/follower.yaml \
  -p enabled:="${follower_enabled}" \
  -p forward_commands_enabled:=true \
  -p publish_to_mavros:=true
