#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

log_file=/tmp/moving_actor_obstacle_validation.log
setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false >"${log_file}" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT -- "-${launch_pid}" 2>/dev/null || true
  sleep 4
  kill -TERM -- "-${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 35
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/inspect_depth_costmap.py &
inspector_pid=$!
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/check_actor_motion_costmap.py
motion_status=$?
wait "${inspector_pid}"

echo "=== WORLD AND MAPPING EVENTS ==="
grep -E "Loading SDF|Managed nodes are active|Metric Depth|Perception node started|Error|ERROR" \
  "${log_file}" || true
exit "${motion_status}"
