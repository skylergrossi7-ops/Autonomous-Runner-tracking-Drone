#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

log_file=/tmp/dynamic_runner_cutout.log
setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >"${log_file}" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 30
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/check_dynamic_runner_cutout.py
