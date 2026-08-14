#!/usr/bin/env python3
"""Check that the runner corridor is clear while side obstacles remain."""

import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Check(Node):
    def __init__(self):
        super().__init__("center_corridor_check")
        self.samples = []
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self.callback, qos)

    def callback(self, message):
        grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        resolution = message.info.resolution
        origin_x = message.info.origin.position.x
        origin_y = message.info.origin.position.y
        xs = origin_x + (np.arange(message.info.width) + 0.5) * resolution
        ys = origin_y + (np.arange(message.info.height) + 0.5) * resolution
        xx, yy = np.meshgrid(xs, ys)
        high = grid >= 80
        # Validate the immediate path segment to the tracked runner. Beyond
        # the runner, the tall post legitimately lies close to the centerline.
        corridor = (np.abs(xx) <= 0.45) & (yy >= 1.5) & (yy <= 4.2)
        sides = (np.abs(xx) >= 0.65) & (np.abs(xx) <= 2.2) & (yy >= 3.0) & (yy <= 12.0)
        self.samples.append((int(np.count_nonzero(high & corridor)),
                             int(np.count_nonzero(high & sides))))


rclpy.init()
node = Check()
deadline = time.monotonic() + 35
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.25)
clear = sum(center <= 4 and side >= 3 for center, side in node.samples)
print(f"Samples: {len(node.samples)}")
print(f"Center/side high-cost cells: {node.samples}")
passed = bool(node.samples) and clear >= max(2, len(node.samples) // 2)
print(f"{'PASS' if passed else 'FAIL'} clear runner corridor with side obstacles")
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if passed else 1)
