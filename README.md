# Autonomous Runner Tracking Drone

This project is being developed incrementally, beginning with a standalone
perception pipeline that detects and tracks a runner using a laptop camera,
OpenCV, and Ultralytics YOLO.

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

The original single-file prototype remains in the repository as the foundation
and historical reference. The new implementation separates its working logic
into testable components before ROS 2, point-cloud mapping, planning, or flight
control are added.

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
```

See `PERCEPTION_ARCHITECTURE.md` for the component boundaries, test gates, and
planned path toward ROS 2, depth association, point clouds, costmaps, planning,
and isolated flight control.
