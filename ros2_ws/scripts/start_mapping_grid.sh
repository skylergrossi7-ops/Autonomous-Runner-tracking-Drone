#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash
exec /usr/bin/python3 \
  /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/scripts/record_dynamic_mapping_demo.py \
  --output /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/test_artifacts/interactive_footprints.mp4 \
  --screenshot /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/test_artifacts/interactive_footprints.png \
  --duration 0 --fps 5
