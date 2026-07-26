# Runner Perception Architecture

## First objective

The first working milestone is:

> Read a camera frame, detect people with YOLO, lock onto one runner, draw the
> runner's bounding box, and display the annotated frame.

This milestone does **not** control the drone, estimate distance, create a point
cloud, or send MAVLink commands. Those features will consume perception results
later, after runner detection works reliably.

## Preserve the working foundation

The original script already has a useful detection and tracking sequence:

1. Read a frame with `cv2.VideoCapture`.
2. Resize it to `960 x 540`.
3. Call `model.track(..., persist=True)`.
4. Filter detections to YOLO class `0` (`person`).
5. Lock onto the highest-confidence tracked person.
6. Keep the same YOLO tracking ID between frames.
7. Release the target after it is missing for two seconds.
8. Draw the target bounding box and tracking error line.
9. Show the annotated frame with OpenCV.

That behavior should remain the foundation. Initially, we will move it into
small classes without changing its detection decisions.

## Data flow

```text
Laptop camera
      |
      v
CameraSource
  OpenCV BGR frame
      |
      v
PersonDetector
  list of person detections
      |
      v
RunnerTracker
  selected runner or no runner
      |
      +----------------------+
      |                      |
      v                      v
FrameAnnotator        PerceptionResult
  bounding box          clean output for ROS 2
  target ID             and future nodes
  center/error line
      |
      v
OpenCV preview
```

## Shared data objects

These are plain data structures, not ROS messages. Keeping the internal model
independent from ROS makes each class easier to test.

### `Detection`

Represents one object returned by YOLO:

```text
bounding box: x1, y1, x2, y2
class ID
class name
confidence
optional tracking ID
```

### `PerceptionResult`

Represents the output for one frame:

```text
timestamp
frame width and height
all person detections
selected runner, if visible
normalized horizontal offset
normalized vertical offset
tracking state
```

Suggested tracking states:

- `SEARCHING`: no runner is locked.
- `TRACKING`: the locked runner is visible.
- `TEMPORARILY_LOST`: the runner is missing but the two-second timeout has not
  expired.
- `LOST`: the lock expired and a new runner may be selected.

## Classes, in implementation order

We will implement and test one class at a time.

### 1. `CameraSource`

Responsibility:

- Open the laptop camera using `cv2.VideoCapture(0)`.
- Check that the camera opened.
- Return the newest BGR frame.
- Resize frames to the configured resolution.
- Release the camera during shutdown.

Foundation from the original code:

- `cv2.VideoCapture`
- `video_feed.isOpened()`
- `video_feed.read()`
- `cv2.resize(frame, (960, 540))`
- `video_feed.release()`

This class knows nothing about YOLO, runners, ROS, or flight control.

### 2. `PersonDetector`

Responsibility:

- Load the Ultralytics YOLO model once.
- Run YOLO tracking on each frame.
- Request only class `0` (`person`) initially.
- Convert Ultralytics boxes into a list of `Detection` objects.

Foundation from the original code:

```python
model = YOLO(MODEL_PATH)
results = model.track(
    frame,
    persist=True,
    verbose=False,
    conf=CONFIDENCE_THRESHOLD,
    classes=[PERSON_CLASS_ID],
)
```

Initial configuration:

```text
model path: yolov8n.pt
person class ID: 0
confidence threshold: 0.45
tracking enabled: true
```

This class detects people but does not decide which person is the runner.

### 3. `RunnerTracker`

Responsibility:

- Select the highest-confidence tracked person when no runner is locked.
- Store that person's tracking ID.
- Find the same ID in later frames.
- Track when the runner was last seen.
- Release the lock after `2.0` seconds without a match.

Foundation from the original code:

- `select_initial_target`
- `find_existing_target`
- `target_runner_id`
- `target_last_seen_time`
- `MAX_TARGET_LOST_SECONDS`

Important first-version rule:

> A YOLO tracking ID is required before a person can become the runner.

This preserves the original behavior and prevents the target from jumping
between visible people on every frame.

Later improvements may add appearance matching or explicit operator selection,
but they are outside the first milestone.

### 4. `FrameAnnotator`

Responsibility:

- Draw the locked runner's bounding box.
- Show its confidence and tracking ID.
- Mark the runner's center.
- Draw a line from the camera center to the runner center.
- Draw the current tracking state.

Foundation from the original code:

- `draw_target`
- `calculate_target_offset`
- `cv2.rectangle`
- `cv2.putText`
- `cv2.line`

Normalized offsets remain:

```text
offset_x = -1 at the left edge, 0 at center, +1 at the right edge
offset_y = -1 at the top edge, 0 at center, +1 at the bottom edge
```

The annotator only draws information. It does not choose the runner or command
the drone.

### 5. `PerceptionPipeline`

Responsibility:

- Coordinate the four classes above for one frame.
- Return a `PerceptionResult`.
- Keep the application loop small and readable.

Conceptual operation:

```python
frame = camera.read()
detections = detector.detect(frame)
tracking_result = tracker.update(detections)
result = build_perception_result(frame, detections, tracking_result)
annotated_frame = annotator.draw(frame, result)
```

This is where the pieces are assembled, but the individual algorithms remain in
their own classes.

### 6. `PerceptionNode` (ROS 2 integration)

This class will be created only after the non-ROS pipeline works with the laptop
camera.

Responsibility:

- Receive `sensor_msgs/Image`, or use a separate camera node.
- Convert ROS images to OpenCV frames with `cv_bridge`.
- Run `PerceptionPipeline`.
- Publish the annotated image.
- Publish bounding boxes as `vision_msgs/Detection2DArray`.
- Publish the selected runner and normalized offsets.

The ROS node should be a thin adapter. It should not contain YOLO selection,
tracking, or drawing algorithms.

## Configuration

Keep configuration separate from algorithms:

```text
camera index: 0
frame width: 960
frame height: 540
model path: yolov8n.pt
person class ID: 0
confidence threshold: 0.45
maximum target-lost time: 2.0 seconds
```

These will start as constructor arguments. They can become ROS parameters after
the standalone perception pipeline works.

## File structure to create gradually

Do not create all of these files at once. Add them as each class is implemented:

```text
Drone/
  perception/
    __init__.py
    models.py              # Detection and PerceptionResult
    camera_source.py       # CameraSource
    person_detector.py     # PersonDetector
    runner_tracker.py      # RunnerTracker
    frame_annotator.py     # FrameAnnotator
    pipeline.py            # PerceptionPipeline
  tests/
    test_runner_tracker.py
    test_offsets.py
  run_perception.py        # standalone laptop-camera test
```

Later, these modules can be moved into a ROS 2 `ament_python` package without
rewriting the working perception algorithms.

## Testing gates

Each class must pass its gate before the next class is added.

### Camera gate

- Laptop camera opens.
- Frames are received continuously.
- The camera is released cleanly.

### Detection gate

- YOLO loads once.
- People are detected in live frames.
- Confidence filtering works.
- Detection objects contain valid boxes and optional tracking IDs.

### Tracking gate

- The highest-confidence person is selected initially.
- The same tracking ID remains locked.
- Other people do not steal the lock.
- The target unlocks after two seconds of absence.

### Annotation gate

- Bounding boxes match the runner.
- Labels and confidence are readable.
- The center-to-target line is correct.
- Pressing `q` closes the preview cleanly.

### ROS gate

- Camera images arrive on the expected topic.
- Detection messages match the displayed boxes.
- Annotated images are visible in `rqt_image_view`.
- Perception continues without any flight controller connection.

## Future interfaces

After perception is stable:

1. A depth-association component will synchronize detections with calibrated
   depth data.
2. It will select 3D points inside each detection instead of inventing a
   distance from image pixels.
3. A point-cloud node will publish obstacles in a known TF frame.
4. A mapping node will convert those obstacles into an occupancy map or costmap.
5. A planner will find a low-cost collision-free path.
6. A separate safety/control node will be the only component permitted to send
   commands to the autopilot.

The original `fake_lidar_distance = 10` must not be part of perception because
it is not a real measurement.
