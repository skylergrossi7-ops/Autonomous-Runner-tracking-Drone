# Gazebo camera and ROS 2 image workflow

This guide records the working process for viewing the Iris camera in ROS 2
Jazzy. Gazebo, the image bridge, perception, and the viewer are separate
processes. Keep each one running in its own terminal.

## Prerequisites

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-image \
  ros-jazzy-rqt-image-view
```

## Terminal 1: build and start the custom world

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"
colcon build --symlink-install --packages-select drone_simulation
source install/setup.bash

ros2 launch drone_simulation runner_tracking.launch.py
```

Ensure Gazebo is playing rather than paused.

The launch file adds both the installed custom-model directory and the standard
`~/ardupilot_gazebo/models` directory to `GZ_SIM_RESOURCE_PATH`. The latter must
contain ArduPilot's `runway`, `iris_with_standoffs`, and `gimbal_small_3d`
directories.

## Terminal 2: discover and bridge the camera

```bash
source /opt/ros/jazzy/setup.bash
gz topic -l | grep "/camera/image$"
```

The Iris gimbal camera topic is currently:

```text
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Start the image bridge:

```bash
ros2 run ros_gz_image image_bridge \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Leave this terminal running. Restart this bridge whenever Gazebo is restarted.

## Terminal 3: verify and view the raw camera

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep image
```

Verify frames are arriving:

```bash
ros2 topic hz \
  /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

Open the viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select the full `/world/iris_runway/.../camera/image` topic from the dropdown.
A ROS topic name is not a Bash command and should not be typed by itself.

## Terminal 4: start runner perception

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/drone/bin/activate
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"
source install/setup.bash

ros2 run drone_perception perception_node --ros-args \
  -p image_topic:="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
```

In `rqt_image_view`, select:

```text
/perception/annotated_image
```

## Troubleshooting

### The camera topic is unknown

The image bridge is not running, Gazebo is closed, or the simulation is paused.
Start Gazebo first, then restart the bridge.

### Only `/parameter_events` and `/rosout` appear

No relevant ROS publisher is running in that ROS domain. Confirm that the image
bridge remains open and that all terminals use the same `ROS_DOMAIN_ID`.

### Gazebo cannot find `model://runway` or the lidar model

First confirm that the ArduPilot models exist:

```bash
ls ~/ardupilot_gazebo/models/runway/model.sdf
ls ~/ardupilot_gazebo/models/iris_with_standoffs/model.sdf
ls ~/ardupilot_gazebo/models/gimbal_small_3d/model.sdf
```

Then rebuild the package and launch it through ROS:

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"
colcon build --symlink-install --packages-select drone_simulation
source install/setup.bash
ros2 launch drone_simulation runner_tracking.launch.py
```

Do not run `gz sdf -p` as the normal launch command. Its empty
`sdf::findFile()` callback warning can appear while expanding unresolved
`model://` includes outside the Gazebo server.

### `rqt_image_view: command not found`

Install the Jazzy package:

```bash
sudo apt install ros-jazzy-rqt-image-view
```

Then start it with:

```bash
ros2 run rqt_image_view rqt_image_view
```

### `/dev/video0` or `/sys/class/video4linux` is missing

WSL cannot see the Windows laptop camera as a Linux Video4Linux device. This is
why the simulation uses the Gazebo camera and `ros_gz_image` instead of
`usb_cam`.
