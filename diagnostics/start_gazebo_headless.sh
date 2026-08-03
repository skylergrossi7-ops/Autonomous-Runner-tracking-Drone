#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/setup.bash

package_share="/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws/install/drone_simulation/share/drone_simulation"
export GZ_SIM_RESOURCE_PATH="${package_share}/models:${HOME}/ardupilot_gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

# Server-only mode leaves more CPU for camera inference and flight control.
exec gz sim -s -v4 -r "${package_share}/worlds/runner_tracking.sdf"
