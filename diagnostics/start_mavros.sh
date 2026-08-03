#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

exec ros2 launch mavros node.launch \
  fcu_url:="tcp-l://0.0.0.0:5770" \
  gcs_url:="udp://@127.0.0.1:14550" \
  tgt_system:="1" \
  tgt_component:="1" \
  pluginlists_yaml:="/mnt/c/Users/skyle/OneDrive/Documents/Drone/diagnostics/mavros_follow_plugins.yaml" \
  config_yaml:="/opt/ros/jazzy/share/mavros/launch/apm_config.yaml"
