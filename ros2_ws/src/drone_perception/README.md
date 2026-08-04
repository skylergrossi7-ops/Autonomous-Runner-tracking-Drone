# ROS 2 Jazzy runner perception

This package wraps the tested standalone perception classes in a ROS 2 node.
It does not control the drone.

## Topics

Input:

- `/camera/image_raw` — `sensor_msgs/Image`

Outputs:

- `/perception/annotated_image` — `sensor_msgs/Image`
- `/perception/detections` — `vision_msgs/Detection2DArray`
- `/perception/runner` — `vision_msgs/Detection2D`

## Ubuntu 24.04 setup

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-rqt-image-view \
  ros-jazzy-usb-cam \
  python3-opencv \
  python3-venv

python3 -m venv --system-site-packages ~/venvs/drone
source ~/venvs/drone/bin/activate
python3 -m pip install -r \
  src/drone_perception/requirements.txt
```

## Build

From `ros2_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/drone/bin/activate
cd ros2_ws
source install/setup.bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="/dev/video0" \
  -r image_raw:=/camera/image_raw
```

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/drone/bin/activate
cd ros2_ws
source install/setup.bash
ros2 launch drone_perception perception.launch.py
```

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install/setup.bash
rqt_image_view /perception/annotated_image
```

If Ubuntu runs in WSL and `/dev/video0` does not exist, use a recorded ROS bag
or a video/image publisher instead of `usb_cam`.
