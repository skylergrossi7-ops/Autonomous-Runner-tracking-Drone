#!/usr/bin/env python3
"""Verify that Nav2's rolling local costmap is centered on base_link."""

import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CostmapAlignmentCheck(Node):
    def __init__(self):
        super().__init__("costmap_alignment_check")
        self.samples = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.callback, qos
        )

    def callback(self, message):
        try:
            transform = self.tf_buffer.lookup_transform(
                message.header.frame_id or "odom", "base_link", Time()
            ).transform
        except TransformException:
            return

        origin = message.info.origin
        quaternion = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
        half_x = 0.5 * message.info.width * message.info.resolution
        half_y = 0.5 * message.info.height * message.info.resolution
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        center_x = origin.position.x + cosine * half_x - sine * half_y
        center_y = origin.position.y + sine * half_x + cosine * half_y
        error_x = transform.translation.x - center_x
        error_y = transform.translation.y - center_y
        self.samples.append({
            "frame": message.header.frame_id,
            "width": message.info.width,
            "height": message.info.height,
            "resolution": message.info.resolution,
            "origin_x": origin.position.x,
            "origin_y": origin.position.y,
            "base_x": transform.translation.x,
            "base_y": transform.translation.y,
            "error_x": error_x,
            "error_y": error_y,
            "error_m": math.hypot(error_x, error_y),
        })


def main():
    rclpy.init()
    node = CostmapAlignmentCheck()
    deadline = time.monotonic() + 20.0
    while rclpy.ok() and time.monotonic() < deadline and len(node.samples) < 5:
        rclpy.spin_once(node, timeout_sec=0.25)

    latest = node.samples[-1] if node.samples else None
    maximum_error = max(
        (sample["error_m"] for sample in node.samples), default=float("inf")
    )
    passed = bool(
        latest
        and latest["frame"] == "odom"
        and latest["width"] == latest["height"]
        and maximum_error <= 0.15
    )
    result = {
        "samples": len(node.samples),
        "latest": latest,
        "maximum_centering_error_m": maximum_error,
        "passed": passed,
    }
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
