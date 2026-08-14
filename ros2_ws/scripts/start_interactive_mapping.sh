#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash
exec ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=false start_rviz:=true use_live_odometry:=false \
  sky_mask_enabled:=true
