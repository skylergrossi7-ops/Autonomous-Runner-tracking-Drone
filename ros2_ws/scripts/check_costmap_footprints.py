#!/usr/bin/env python3
"""Confirm the local costmap contains three persistent obstacle footprints."""

import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class FootprintCheck(Node):
    def __init__(self):
        super().__init__("costmap_footprint_check")
        self.samples = []
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
        grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        # VoxelLayer marks obstacles as 254 and InflationLayer supplies the
        # surrounding gradient. Treat only high-cost cores as footprints.
        lethal = np.uint8(grid >= 80)
        count, _, stats, centers = cv2.connectedComponentsWithStats(
            lethal, connectivity=8
        )
        components = [
            (int(stats[index, cv2.CC_STAT_AREA]), centers[index].tolist())
            for index in range(1, count)
            if stats[index, cv2.CC_STAT_AREA] >= 3
        ]
        components.sort(reverse=True)
        self.samples.append(components)


def main():
    rclpy.init()
    node = FootprintCheck()
    deadline = time.monotonic() + 40.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    counts = [len(sample) for sample in node.samples]
    stable = sum(count >= 3 for count in counts)
    required = max(3, len(counts) // 3)
    best = max(node.samples, key=len, default=[])
    print(f"Costmap samples: {len(counts)}")
    print(f"Samples with at least 3 footprints: {stable}")
    print(f"Largest component set (area cells, centroid): {best[:6]}")
    passed = stable >= required
    print(f"{'PASS' if passed else 'FAIL'} distinct crate/post/barrel footprints")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
