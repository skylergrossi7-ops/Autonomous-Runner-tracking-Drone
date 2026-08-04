#!/usr/bin/env bash
set -eo pipefail

root="/mnt/c/Users/skyle/OneDrive/Documents/Drone"
pids=()

cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]}"; do
    kill -KILL -- "-${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid bash "${root}/diagnostics/start_gazebo_headless.sh" \
  >/tmp/preview_gazebo.log 2>&1 &
pids+=("$!")

for _ in {1..30}; do
  topics=$(gz topic -l 2>/dev/null || true)
  if [[ "${topics}" == *"/tracking_camera/image"* ]]; then
    break
  fi
  sleep 2
done

gz service -s /world/iris_runway/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 \
  --req "sdf_filename: '${root}/diagnostics/showcase_camera.sdf', name: 'showcase_camera_rig'"
sleep 5

setsid bash "${root}/diagnostics/start_camera_bridge.sh" \
  >/tmp/preview_tracking_bridge.log 2>&1 &
pids+=("$!")
setsid bash "${root}/diagnostics/start_showcase_camera_bridge.sh" \
  >/tmp/preview_showcase_bridge.log 2>&1 &
pids+=("$!")
sleep 8

source /opt/ros/jazzy/setup.bash
source /home/skyler/venvs/drone/bin/activate
timeout 30 python "${root}/diagnostics/capture_ros_image.py" \
  /tracking_camera/image "${root}/diagnostics/preview_tracking_camera.png"
timeout 30 python "${root}/diagnostics/capture_ros_image.py" \
  /showcase_camera/image "${root}/diagnostics/preview_showcase_camera.png"
