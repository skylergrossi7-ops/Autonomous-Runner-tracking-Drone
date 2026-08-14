#!/usr/bin/env bash
set -e

root=/mnt/c/Users/skyle/OneDrive/Documents/Drone
workspace=${root}/ros2_ws
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
export ARDUCOPTER_OVERLAY=${root}/diagnostics/follow_validation.parm

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2>/dev/null || true; done
  sleep 3
  pkill -x arducopter 2>/dev/null || true
}
trap cleanup EXIT

setsid ros2 launch drone_simulation iris_ai_navigation.launch.py \
  headless:=true start_rviz:=false use_live_odometry:=false \
  sky_mask_enabled:=true >/tmp/sky_corridor_launch.log 2>&1 &
pids+=("$!")
sleep 15
setsid bash "${root}/diagnostics/start_arducopter.sh" \
  >/tmp/sky_corridor_arducopter.log 2>&1 &
pids+=("$!")
sleep 30

/usr/bin/python3 "${workspace}/scripts/record_dynamic_mapping_demo.py" \
  --output "${workspace}/test_artifacts/sky_corridor_release/sky_corridor_mapping.mp4" \
  --screenshot "${workspace}/test_artifacts/sky_corridor_release/sky_corridor_mapping.png" \
  --duration 50 --fps 5
