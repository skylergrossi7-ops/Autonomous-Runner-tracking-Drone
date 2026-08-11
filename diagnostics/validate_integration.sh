#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash
set -u
log_file=/tmp/drone_integration_validation.log
setsid ros2 launch drone_sim simulation.launch.py headless:=true >"${log_file}" 2>&1 &
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

sleep 45

echo "=== MESSAGE DELIVERY ==="
python3 /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/check_integration_topics.py

echo "=== RECENT LAUNCH LOG ==="
tail -n 80 "${log_file}"
