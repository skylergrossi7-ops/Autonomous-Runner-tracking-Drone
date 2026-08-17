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

## 7. Physical following with a live rolling voxel map

The follower now streams body-frame MAVROS velocity targets at a steady
wall-clock rate, without overriding ArduPilot's takeoff command while tracking
is disabled. Live MAVROS pose drives the `odom -> base_link` transform, allowing
the filtered AI-depth cloud and Nav2 voxel costmap to move with the aircraft.

The simulation uses a resource-aware YOLO setting of 320 pixels and a 0.08
threshold because the stylized Gazebo actor scores unusually low at that input
size. Real-camera operation should use a substantially higher threshold and
temporal confirmation.

| Recorded validation metric | Result |
| --- | ---: |
| Hover altitude | 1.22 m |
| Drone horizontal travel | 2.92 m |
| Nonzero MAVROS setpoints | 496 / 496 |
| Runner target samples | 12 |
| Filtered cloud samples | 22 |
| Maximum cloud gap | 4.04 s |
| Occupied costmap cells | 15,782–18,965 |
| Rolling costmap travel | 2.90 m |

![Physical following with live map updates](docs/media/follow_with_live_costmap.png)

[Watch the synchronized 50-second validation](docs/media/follow_with_live_costmap.mp4)

All earlier videos and screenshots remain in `docs/media/` to preserve the
project's development history.

## 8. Runner-speed prediction and ground-filtered obstacle mapping

The controller now estimates forward runner velocity from consecutive metric
target vectors and adds bounded feed-forward to its distance correction. A
filtered-cloud safety corridor calculates a braking-distance speed limit and
stops motion when obstacle data becomes stale.

The point-cloud pipeline removes the runner, fits a ground plane with RANSAC,
and suppresses residual lower-image runway points produced by monocular depth.
Nav2 requires multiple marked voxels, uses less inflation and displays free,
inflated, high-cost and lethal regions in distinct colors.

| Recorded runner-speed metric | Result |
| --- | ---: |
| Hover altitude | 2.22 m |
| Maximum forward command | 2.50 m/s |
| Drone horizontal travel | 9.05 m |
| Nonzero MAVROS setpoints | 487 / 487 |
| Runner target samples | 11 |
| Filtered cloud samples | 25 |
| Maximum cloud gap | 5.49 s |
| Occupied costmap cells | 5,084–9,941 |
| Rolling costmap travel | 8.70 m |

![Runner-speed follow and sparse colored costmap](docs/media/runner_speed_follow_with_costmap.png)

[Watch the synchronized runner-speed validation](docs/media/runner_speed_follow_with_costmap.mp4)

The 2.5 m/s cap is validated in simulation only. Physical testing must start
at a lower limit and retune braking distance for the actual airframe.

## 9. Stable CPU-limited AI-depth voxel mapping

The AI-depth publisher now processes only the newest camera frame in a worker
thread, applies spatial and temporal smoothing, and removes isolated cloud
speckles. This prevents a backlog of stale inference frames while Gazebo and
YOLO share the laptop CPU.

The empty local-costmap display was traced to a timing and range mismatch:
Depth Anything sometimes produced 4–6 second cloud gaps, Nav2 retained points
for only 0.5 seconds, and several simulated obstacles were estimated just past
the old 14 m cutoff. The voxel layer now uses six-second observation
persistence, a 20 m measured range, and a one-voxel marking threshold.

| Recorded mapping metric | Result |
| --- | ---: |
| Maximum filtered points | 17,413 |
| Points eligible for voxel marking | 1,598 |
| Observed Nav2 cost values | 0, 84, 99, 100 |
| Occupied cells in captured frame | 3,254 |
| Moving actor and obstacles visible | Yes |

![Stable AI-depth local costmap](docs/media/stable_dynamic_costmap.png)

[Watch the 20-second stable mapping validation](docs/media/stable_dynamic_costmap.mp4)

Earlier media remains unchanged to preserve the visible development history.

## 10. Final runner-following baseline before universal pathfinding

A conservative fixed-horizon mask now removes false Gazebo sky depth only from
the generated cloud; the diagnostic depth image remains raw. The mask is off by
default for real cameras and enabled explicitly for the fixed simulation view.

The first combined regression exposed CPU starvation: depth inference restarted
immediately after slow frames and MAVROS repeatedly lost heartbeats. Rate
limiting now begins after inference completes, and the flight profile uses the
validated 1 Hz, stride-4 cloud configuration with eight-second freshness gates.

| Final protected-flight metric | Result |
| --- | ---: |
| Takeoff altitude | 2.20 m |
| Drone horizontal travel | 7.55 m |
| Maximum forward command | 2.50 m/s |
| Nonzero raw MAVROS setpoints | 273 / 491 |
| Runner target samples | 5 |
| Runner box-height change | 187.1 px |
| Filtered cloud samples | 18 |
| Maximum cloud gap | 4.53 s |
| Occupied costmap cells | 5,280–19,658 |
| Rolling costmap travel | 7.60 m |
| Moving actor | PASS |
| Drone following | PASS |
| Mapping while following | PASS |
| LAND command | Successful |

![Final sky-filtered following baseline](docs/media/final_runner_follow_baseline.png)

[Watch the synchronized 50-second validation](docs/media/final_runner_follow_baseline.mp4)

This is the frozen runner-following baseline for the next phase: universal
local pathfinding and reactive obstacle avoidance. All prior media is retained.

## 11. TF-aligned local costmap origin

The rolling Nav2 costmap previously started one grid cell right and above the
drone. The default zero origin caused the first negative rolling-window shift
to truncate by one cell. The map now starts with a symmetric half-width origin,
while continuing to roll from the live `odom -> base_link` transform.

The diagnostic recorder no longer assumes that the drone is at the image
midpoint. It calculates the real grid cell from TF and the published
`OccupancyGrid` origin, rotates around that cell, and reports the centering
error directly. RViz now follows `base_link` with a zero-centered focal point.

| Alignment validation | Result |
| --- | ---: |
| Grid frame | `odom` |
| Grid dimensions tested | 200 x 200 at 0.10 m |
| Previous center error | 0.141 m |
| Corrected center error | 0.0000002 m |
| Live samples passed | 5 / 5 |

![TF-aligned local costmap](docs/media/costmap_origin_centering.png)

[Watch the centered local-costmap demonstration](docs/media/costmap_origin_centering.mp4)

All earlier screenshots and recordings remain in `docs/media/` to preserve the
project's development history.
