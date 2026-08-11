#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >/tmp/obstacle_geometry.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT
sleep 30
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/inspect_depth_costmap.py
