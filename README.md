# Autonomous Runner Tracking Drone

This project is being developed incrementally from laptop-camera perception
into an autonomous ROS 2 / Gazebo runner-following drone. The current milestone
takes off, detects a moving person with YOLO, regulates a chosen trailing
distance, and lands through MAVROS and ArduPilot SITL.

## Current progress

- Laptop-camera capture with explicit resource cleanup
- YOLO person detection and persistent object tracking
- Highest-confidence initial runner selection
- Stable runner lock using the YOLO tracking ID
- Two-second timeout before releasing a missing runner
- Bounding box, confidence, tracking state, and normalized image offsets
- Separate classes for camera input, detection, tracking, annotation, and
  pipeline coordination
- Unit coverage for runner selection and target-loss behavior
- ROS 2 Jazzy perception, simulation, and flight-control packages
- Gazebo Iris model with tracking camera and forward LiDAR
- Explicit Boolean motion gate and stale-target stop
- Configurable trailing distance with forward and reverse corrections
- MAVROS body-frame velocity control with bounded speed and yaw
- Protected takeoff-follow-land validation with an independent LAND watchdog
- Passing 2.5 m distance-follow result with 0.121 m final error

The original single-file prototype remains as the historical foundation.
Git commits, pull requests, and `PROJECT_HISTORY.md` show how the project moved
from that prototype to modular perception, Gazebo integration, and autonomous
distance-controlled following.

## Latest milestone: distance-controlled following

![YOLO runner detection in Gazebo](docs/media/runner_detection_gazebo.png)

![Passing distance-follow telemetry](docs/media/distance_follow_validation.png)

▶️ [Watch the 15-second annotated drone-camera follow video](docs/media/distance_follow_gazebo_15s.mp4)

The protected Gazebo validation achieved:

- desired trailing distance: **2.5 m**
- final estimated distance: **2.621 m**
- final error: **0.121 m**
- horizontal drone path: **0.357 m**
- fresh target ratio: **100%**
- result: **`FOLLOW_VALIDATION_PASS`**

The raw CSV is available at
[`docs/evidence/distance_follow_validation.csv`](docs/evidence/distance_follow_validation.csv).

The linked MP4 contains 145 authentic ROS camera frames, is encoded at
640×480, and has a verified duration of exactly 15.000 seconds.

## Run the current perception milestone

Use a Windows Python environment so OpenCV can access the laptop camera:

```powershell
py -m pip install -r requirements.txt
py run_camera.py
```

The first run may download the YOLO model weights. Stand in view of the camera
and press `q` in the preview window to exit.

## Structure

```text
perception/
  camera_source.py
  person_detector.py
  runner_tracker.py
  frame_annotator.py
  pipeline.py
  models.py
tests/
  test_runner_tracker.py
run_camera.py
PERCEPTION_ARCHITECTURE.md
ros2_ws/src/drone_perception/
ros2_ws/src/drone_control/
ros2_ws/src/drone_simulation/
diagnostics/
docs/media/
docs/evidence/
```

See `PERCEPTION_ARCHITECTURE.md` for the component boundaries, test gates, and
planned path toward ROS 2, depth association, point clouds, costmaps, planning,
and isolated flight control.

See `GAZEBO_SIMULATION.md` for the multi-terminal Gazebo camera, ROS image
bridge, perception-node, and image-viewer workflow.

See `RUNNER_FOLLOWING.md` for the simulation flight workflow and
`PROJECT_HISTORY.md` for the chronological development record.
