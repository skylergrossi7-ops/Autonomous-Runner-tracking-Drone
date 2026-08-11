#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash
set -u

log_file=/tmp/iris_ai_navigation_validation.log
setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  for _attempt in 1 2 3 4 5; do
    kill -0 "${launch_pid}" 2>/dev/null || return
    sleep 1
  done
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 40

echo "=== GAZEBO MODELS ==="
timeout 10s gz model --list || true

echo "=== RUNNER POSE A ==="
timeout 6s gz topic -e -t /world/iris_runway/pose/info \
  | sed -n '/name: "runner"/,+12p' | head -n 13 || true
sleep 8
echo "=== RUNNER POSE B (8 SECONDS LATER) ==="
timeout 6s gz topic -e -t /world/iris_runway/pose/info \
  | sed -n '/name: "runner"/,+12p' | head -n 13 || true

echo "=== ROS MESSAGE DELIVERY ==="
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/check_integration_topics.py

echo "=== IMPORTANT LAUNCH EVENTS ==="
grep -E "Loading SDF|Managed nodes are active|Metric Depth|Perception node started|YOLO Masking|Error|ERROR" \
  "${log_file}" || true
