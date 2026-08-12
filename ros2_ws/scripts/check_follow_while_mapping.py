#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray


class FollowMappingCheck(Node):
    def __init__(self):
        super().__init__("follow_while_mapping_check")
        self.poses = []
        self.forward_commands = []
        self.target_distances = []
        self.boxes = []
        self.cloud_times = []
        self.costmap_min = None
        self.costmap_max = 0
        self.map_origins = []
        self.raw_setpoints = []
        self.accepted_targets = []
        self._topic_subscriptions = [
            self.create_subscription(
                PoseStamped, "/mavros/local_position/pose", self.pose_cb,
                qos_profile_sensor_data
            ),
            self.create_subscription(Twist, "/tracking/cmd_vel", self.command_cb, 10),
            self.create_subscription(
                PositionTarget, "/mavros/setpoint_raw/local",
                self.raw_setpoint_cb, 10
            ),
            self.create_subscription(
                PositionTarget, "/mavros/setpoint_raw/target_local",
                self.accepted_target_cb, qos_profile_sensor_data
            ),
            self.create_subscription(
                Vector3Stamped, "/perception/runner_target", self.target_cb, 10
            ),
            self.create_subscription(
                Detection2DArray, "/perception/detections", self.detection_cb, 10
            ),
            self.create_subscription(
                PointCloud2, "/camera/depth_ai/filtered_points",
                self.cloud_cb, qos_profile_sensor_data
            ),
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap", self.costmap_cb, 10
            ),
        ]

    def pose_cb(self, message):
        self.poses.append((message.pose.position.x, message.pose.position.y))

    def command_cb(self, message):
        self.forward_commands.append(message.linear.x)

    def raw_setpoint_cb(self, message):
        self.raw_setpoints.append((time.monotonic(), message.velocity.x))

    def accepted_target_cb(self, message):
        self.accepted_targets.append((time.monotonic(), message.velocity.x))

    def target_cb(self, message):
        self.target_distances.append(math.hypot(message.vector.x, message.vector.y))

    def detection_cb(self, message):
        if message.detections:
            box = max(message.detections, key=lambda item: item.bbox.size_y).bbox
            self.boxes.append((box.center.position.y, box.size_y))

    def cloud_cb(self, _message):
        self.cloud_times.append(time.monotonic())

    def costmap_cb(self, message):
        occupied = sum(1 for value in message.data if value >= 50)
        self.costmap_min = occupied if self.costmap_min is None else min(
            self.costmap_min, occupied
        )
        self.costmap_max = max(self.costmap_max, occupied)
        self.map_origins.append((
            message.info.origin.position.x,
            message.info.origin.position.y,
        ))


def planar_change(samples):
    if len(samples) < 2:
        return 0.0
    start = samples[0]
    return max(math.hypot(x - start[0], y - start[1]) for x, y in samples)


def main():
    rclpy.init()
    node = FollowMappingCheck()
    deadline = time.monotonic() + 50.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    drone_travel = planar_change(node.poses)
    origin_travel = planar_change(node.map_origins)
    maximum_forward = max(node.forward_commands, default=0.0)
    raw_nonzero = sum(abs(value) >= 0.1 for _, value in node.raw_setpoints)
    accepted_nonzero = sum(
        abs(value) >= 0.1 for _, value in node.accepted_targets
    )
    box_change = (
        max(value[1] for value in node.boxes) - min(value[1] for value in node.boxes)
        if node.boxes else 0.0
    )
    cloud_gap = max(
        (later - earlier for earlier, later in zip(node.cloud_times, node.cloud_times[1:])),
        default=float("inf"),
    )

    following = (
        len(node.target_distances) >= 3
        and maximum_forward >= 0.1
        and raw_nonzero >= 5
        and drone_travel >= 0.5
    )
    actor_moving = len(node.boxes) >= 3 and box_change >= 5.0
    mapping = (
        len(node.cloud_times) >= 3
        and cloud_gap <= 10.0
        and node.costmap_max > 0
        and origin_travel >= 0.2
    )

    print(f"Pose samples: {len(node.poses)}")
    print(f"Drone horizontal travel: {drone_travel:.3f} m")
    print(f"Maximum forward command: {maximum_forward:.3f} m/s")
    print(f"Raw MAVROS setpoints: {len(node.raw_setpoints)} ({raw_nonzero} nonzero)")
    print(
        "Accepted MAVROS velocity targets: "
        f"{len(node.accepted_targets)} ({accepted_nonzero} nonzero)"
    )
    print(f"Runner target samples: {len(node.target_distances)}")
    print(f"Runner box-height change: {box_change:.1f} px")
    print(f"Filtered cloud samples: {len(node.cloud_times)}")
    print(f"Maximum filtered-cloud gap: {cloud_gap:.2f} s")
    print(f"Occupied costmap cells: min={node.costmap_min}, max={node.costmap_max}")
    print(f"Rolling costmap origin travel: {origin_travel:.3f} m")
    print(f"{'PASS' if actor_moving else 'FAIL'} moving actor")
    print(f"{'PASS' if following else 'FAIL'} drone following")
    print(f"{'PASS' if mapping else 'FAIL'} mapping while following")

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if actor_moving and following and mapping else 1)


if __name__ == "__main__":
    main()
