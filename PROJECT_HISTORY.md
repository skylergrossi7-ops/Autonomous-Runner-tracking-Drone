# Project history

This file summarizes the major working milestones. Git commits and pull
requests remain the authoritative record and preserve every earlier version.

## 1. Original YOLO tracking prototype

Commit `58d5565` preserved the original single-file implementation. It proved
that OpenCV video input, YOLO person detection, bounding boxes, and image-offset
calculations could work together before the code was reorganized.

## 2. Modular laptop-camera perception

Commit `35b0d53` separated camera input, person detection, runner selection,
tracking, annotation, and pipeline coordination into testable classes.

## 3. Gazebo camera and ROS 2 bridge

Commit `a23ebf4` documented the working Gazebo camera-to-ROS workflow. It
established the simulated Iris camera topic, ROS image bridge, perception node,
and image-viewer process.

## 4. Autonomous distance-controlled runner following

This milestone added three ROS 2 packages:

- `drone_perception`: asynchronous YOLO reacquisition and fast box tracking
- `drone_control`: runner centering, distance regulation, MAVROS commands, and
  safety gating
- `drone_simulation`: moving actor, custom Iris camera, and LiDAR world

The protected simulation completed takeoff, moving-person detection, bounded
following, and landing. Its first passing validation held a selected 2.5 m
distance with 0.121 m final error. The matching chart, CSV, and short-distance
camera recording remain committed as historical evidence.

## 5. Full-body extended-route demonstration

The actor mesh was centered around its waist, so its original `z=0` trajectory
placed its lower half below the runway. The trajectory now applies the measured
1.3 m ground offset, keeping the complete animated person above the surface.
The route was extended from 2.0 m to 9.5 m of travel.

The recording harness now counts unique simulated camera messages instead of
duplicating frames according to wall time when Gazebo runs slower than real
time. A low-rate side camera is spawned only after takeoff, preventing its
renderer from delaying SITL initialization or MAVLink traffic.

### Verified result

| Measurement | Result |
|---|---:|
| Desired trailing distance | 2.500 m |
| Final estimated distance | 2.918 m |
| Final distance error | 0.418 m |
| Horizontal drone path | 4.224 m |
| Fresh runner targets | 99.7% |
| Minimum validation altitude | 2.180 m |
| Automated result | `FOLLOW_VALIDATION_PASS` |

![Full-body Gazebo YOLO detection](docs/media/runner_detection_full_body.png)

[Annotated drone-camera follow video](docs/media/full_body_runner_follow_15s.mp4)

[External Gazebo drone-travel video](docs/media/gazebo_drone_travel_15s.mp4)

Raw telemetry is stored in
[`docs/evidence/full_body_follow_validation.csv`](docs/evidence/full_body_follow_validation.csv).

### Safety boundaries

- Flight movement requires an explicit activation message.
- Missing or stale person detections produce zero velocity.
- Forward motion is blocked by stale LiDAR data or a close obstacle.
- Forward, reverse, and yaw rates are capped in configuration.
- The validation harness requires a detected runner before enabling motion.
- Main-flow and independent-watchdog LAND commands protect every test.

## 6. Monocular AI depth, runner cutout, and local voxel costmap

The forward RGB camera now feeds Depth Anything V2 on the laptop. The resulting
depth image becomes an XYZ point cloud, while YOLO supplies the moving runner
box. A masking node projects the points into the image and removes the runner
surface before publishing an environment-only cloud to Nav2's voxel layer.

Three static hazards beside the runway exercise the costmap without blocking
the runner. High-bandwidth streams use depth-1 sensor QoS, the cloud is
spatially sampled, and Nav2 / RViz retain observations across CPU inference
gaps.

### Verified result

| Measurement | Result |
|---|---:|
| Moving-runner detections | 19 |
| Bounding-box height change | 94.1 px |
| Bounding-box center change | 42.6 px |
| Raw points in runner box (median) | 382 |
| Filtered points in runner box (median) | 187 |
| Runner-silhouette point reduction | 51.0% |
| Occupied costmap cells | 1,283–2,035 |
| Dynamic validation | `PASS` |

![Dynamic AI-depth and costmap dashboard](docs/media/dynamic_depth_costmap_demo.png)

[Watch the 20-second dynamic depth/costmap demonstration](docs/media/dynamic_depth_costmap_demo.mp4)

All earlier detection and autonomous-following media remain in `docs/media/`
as chronological evidence of the project's development.

## Next milestone

Connect live vehicle odometry to the rolling voxel costmap and add reactive
obstacle avoidance while keeping runner tracking and flight safety isolated.
