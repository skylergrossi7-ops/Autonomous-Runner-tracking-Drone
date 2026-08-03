# Runner following in Gazebo

This stage connects camera perception to safe velocity commands. It intentionally
keeps arming and takeoff manual. The follower stops if the target disappears,
the LiDAR data becomes stale, or an obstacle is closer than the configured
safety distance.

## Topic flow

```text
Gazebo camera
  -> ros_gz_image
  -> /perception/runner_target
  -> runner_follower
  -> /tracking/cmd_vel
  -> MAVROS raw body-NED setpoint (only when enabled)
  -> ArduPilot SITL
```

`/perception/runner_target` is a `geometry_msgs/Vector3Stamped`:

- `vector.x`: normalized horizontal offset; `-1` is left and `+1` is right
- `vector.y`: normalized vertical offset; `-1` is top and `+1` is bottom
- `vector.z`: runner bounding-box area divided by image area

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/drone/bin/activate
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select drone_perception drone_control drone_simulation
source install/setup.bash
```

## Terminal 1: Gazebo

```bash
source /opt/ros/jazzy/setup.bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"
source install/setup.bash
bash ../diagnostics/start_gazebo.sh
```

For repeatable automated tests, use the server-only launcher to leave more CPU
for camera inference and flight control:

```bash
bash ../diagnostics/start_gazebo_headless.sh
```

## Terminal 2: ArduPilot SITL

The custom model is based on ArduPilot's Iris-with-gimbal parameters:

```bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone"
bash diagnostics/start_arducopter.sh
```

Wait until the EKF and vehicle initialize. Do not arm yet.

## Terminal 3: camera bridge

The custom sensor publishes the fixed topic `/tracking_camera/image`:

```bash
source /opt/ros/jazzy/setup.bash
bash "/mnt/c/Users/skyle/OneDrive/Documents/Drone/diagnostics/start_camera_bridge.sh"
```

Leave the bridge running.

## Terminal 4: LiDAR bridge

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
```

Leave the bridge running.

## Stage 1: verify the perception target

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/drone/bin/activate
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone/ros2_ws"
source install/setup.bash

ros2 launch drone_control tracking_stack.launch.py \
  image_topic:=/tracking_camera/image
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /perception/runner_target
ros2 topic echo /tracking/cmd_vel
```

At this stage `linear.x` stays zero. Moving the actor left or right should change
`angular.z`. If the sign is reversed for the camera mounting, change `yaw_gain`
from `0.8` to `-0.8` in `drone_control/config/follower.yaml`.

## Stage 2: allow calculated forward commands without moving the drone

Restart the tracking launch:

```bash
ros2 launch drone_control tracking_stack.launch.py \
  image_topic:="$CAMERA_TOPIC" \
  forward_commands_enabled:=true
```

Watch `/tracking/cmd_vel`. The drone still does not move because MAVROS output
is disabled.

Expected behavior:

- target far away: positive `linear.x`
- target at desired size: `linear.x` near zero
- target too close: small negative `linear.x`
- target absent for 0.6 seconds: zero command
- obstacle under 2 metres ahead: forward command blocked

## Stage 3: connect MAVROS

Install MAVROS if it is not already installed:

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
```

This project uses a local TCP connection. Start the MAVROS listener before
MAVProxy; reversing these two commands makes MAVProxy exit with connection
refused.

```bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone"
bash diagnostics/start_mavros.sh
```

Then, in the next terminal:

```bash
cd "/mnt/c/Users/skyle/OneDrive/Documents/Drone"
bash diagnostics/start_mavproxy.sh
```

Verify the connection before continuing:

```bash
ros2 topic echo /mavros/state --once
```

The result must contain `connected: true`.

The follower publishes `mavros_msgs/PositionTarget` on
`/mavros/setpoint_raw/local`. Every message explicitly uses frame `8`
(`FRAME_BODY_NED`), matching the original working DroneKit implementation.
The optional `/mavros/setpoint_velocity/mav_frame` service is not required.

## Stage 4: manual arm and takeoff

In the MAVProxy console from Terminal 2:

```text
mode guided
arm throttle
takeoff 3
```

Wait until the Iris is stable near 3 metres. Keep the tracking-to-MAVROS output
disabled during takeoff.

## Stage 5: follow the actor

Stop the previous tracking launch with `Ctrl+C`, then start it with both motion
switches enabled:

```bash
ros2 launch drone_control tracking_stack.launch.py \
  image_topic:="$CAMERA_TOPIC" \
  forward_commands_enabled:=true \
  publish_to_mavros:=true
```

Keep the MAVProxy console and Gazebo emergency-stop controls visible. Start with
the actor well ahead of the drone and an empty flight path.

Before arming, confirm the raw plugin has a subscriber:

```bash
ros2 topic info /mavros/setpoint_raw/local
```

When the tracking stack is enabled, inspect one outgoing command:

```bash
ros2 topic echo /mavros/setpoint_raw/local --once
```

It must show `coordinate_frame: 8`.

To stop autonomous commands immediately, press `Ctrl+C` in the tracking-stack
terminal. Then use MAVProxy:

```text
mode loiter
```

When testing is finished:

```text
land
```

## Tuning

All gains and limits are in:

```text
ros2_ws/src/drone_control/config/follower.yaml
```

Tune one value at a time. Start with yaw only, then enable forward motion. Keep
`maximum_forward_speed` low until the complete obstacle costmap is available.
