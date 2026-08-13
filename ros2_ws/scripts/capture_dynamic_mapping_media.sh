#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

media_dir=/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/test_artifacts/costmap_persistence
mkdir -p "${media_dir}"
setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >/tmp/dynamic_mapping_media.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 25
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/record_dynamic_mapping_demo.py \
  --output "${media_dir}/stable_dynamic_costmap.mp4" \
  --screenshot "${media_dir}/stable_dynamic_costmap.png" \
  --duration 20 --fps 3
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/inspect_depth_costmap.py \
  | tee "${media_dir}/costmap_validation.txt"
