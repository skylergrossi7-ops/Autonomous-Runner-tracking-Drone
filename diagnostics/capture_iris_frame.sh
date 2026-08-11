#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

output=/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/test_artifacts/iris_camera.png
mkdir -p "$(dirname "${output}")"

setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >/tmp/iris_frame_capture.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 35
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/capture_ros_image.py \
  --output "${output}" --timeout 25
echo "${output}"
