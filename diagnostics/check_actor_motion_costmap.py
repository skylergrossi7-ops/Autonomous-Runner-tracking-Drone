#!/usr/bin/env python3
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class MotionCostmapCheck(Node):
    def __init__(self):
        super().__init__("motion_costmap_check")
        self.box_samples = []
        self.maximum_occupied_cells = 0
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            "/perception/detections",
            self.detection_callback,
            10,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.costmap_callback,
            10,
        )

    def detection_callback(self, message):
        if not message.detections:
            return
        box = max(message.detections, key=lambda detection: detection.bbox.size_y).bbox
        self.box_samples.append(
            (float(box.center.position.x), float(box.center.position.y),
             float(box.size_x), float(box.size_y))
        )

    def costmap_callback(self, message):
        occupied = sum(1 for value in message.data if value >= 50)
        self.maximum_occupied_cells = max(self.maximum_occupied_cells, occupied)


def main():
    rclpy.init()
    node = MotionCostmapCheck()
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)

    if len(node.box_samples) >= 2:
        heights = [sample[3] for sample in node.box_samples]
        centers_y = [sample[1] for sample in node.box_samples]
        height_change = max(heights) - min(heights)
        center_change = max(centers_y) - min(centers_y)
    else:
        height_change = 0.0
        center_change = 0.0

    actor_moved = len(node.box_samples) >= 2 and (
        height_change >= 4.0 or center_change >= 4.0
    )
    obstacles_mapped = node.maximum_occupied_cells > 0

    print(f"Detection samples: {len(node.box_samples)}")
    print(f"Bounding-box height change: {height_change:.1f} px")
    print(f"Bounding-box center change: {center_change:.1f} px")
    print(f"Maximum occupied costmap cells: {node.maximum_occupied_cells}")
    print(f"{'PASS' if actor_moved else 'FAIL'} moving actor")
    print(f"{'PASS' if obstacles_mapped else 'FAIL'} mapped obstacles")

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if actor_moved and obstacles_mapped else 1)


if __name__ == "__main__":
    main()
