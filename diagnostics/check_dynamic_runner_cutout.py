#!/usr/bin/env python3
import time

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DynamicCutoutCheck(Node):
    def __init__(self):
        super().__init__("dynamic_runner_cutout_check")
        self.intrinsics = None
        self.box = None
        self.box_samples = []
        self.raw_inside = []
        self.filtered_inside = []
        self.costmap_minimum = None
        self.costmap_maximum = 0
        self._topic_subscriptions = [
            self.create_subscription(
                CameraInfo, "/camera/camera_info", self.info_callback, SENSOR_QOS
            ),
            self.create_subscription(
                Detection2DArray, "/perception/detections",
                self.detection_callback, RELIABLE_QOS
            ),
            self.create_subscription(
                PointCloud2, "/camera/depth_ai/points",
                lambda message: self.cloud_callback(message, False), SENSOR_QOS
            ),
            self.create_subscription(
                PointCloud2, "/camera/depth_ai/filtered_points",
                lambda message: self.cloud_callback(message, True), SENSOR_QOS
            ),
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap",
                self.costmap_callback, RELIABLE_QOS
            ),
        ]

    def info_callback(self, message):
        self.intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def detection_callback(self, message):
        if not message.detections:
            return
        detection = max(message.detections, key=lambda item: item.bbox.size_y)
        box = detection.bbox
        self.box = (
            box.center.position.x,
            box.center.position.y,
            box.size_x,
            box.size_y,
        )
        self.box_samples.append(self.box)

    def cloud_callback(self, message, filtered):
        if self.intrinsics is None or self.box is None:
            return
        points = pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        if len(points) == 0:
            count = 0
        else:
            x, y, z = points["x"], points["y"], points["z"]
            valid = z > 0.1
            x, y, z = x[valid], y[valid], z[valid]
            fx, fy, cx, cy = self.intrinsics
            u = x * fx / z + cx
            v = y * fy / z + cy
            box_cx, box_cy, width, height = self.box
            inside = (
                (u >= box_cx - width / 2.0)
                & (u <= box_cx + width / 2.0)
                & (v >= box_cy - height / 2.0)
                & (v <= box_cy + height / 2.0)
            )
            count = int(np.count_nonzero(inside))
        (self.filtered_inside if filtered else self.raw_inside).append(count)

    def costmap_callback(self, message):
        occupied = sum(1 for value in message.data if value >= 50)
        self.costmap_maximum = max(self.costmap_maximum, occupied)
        self.costmap_minimum = (
            occupied if self.costmap_minimum is None
            else min(self.costmap_minimum, occupied)
        )


def median(values):
    return float(np.median(values)) if values else 0.0


def main():
    rclpy.init()
    node = DynamicCutoutCheck()
    deadline = time.monotonic() + 70.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    heights = [sample[3] for sample in node.box_samples]
    centers = [sample[1] for sample in node.box_samples]
    height_change = max(heights) - min(heights) if heights else 0.0
    center_change = max(centers) - min(centers) if centers else 0.0
    moved = len(node.box_samples) >= 4 and (
        height_change >= 8.0 or center_change >= 8.0
    )

    raw_median = median(node.raw_inside)
    filtered_median = median(node.filtered_inside)
    reduction = (
        1.0 - filtered_median / raw_median if raw_median > 0.0 else 0.0
    )
    cutout_worked = (
        len(node.raw_inside) >= 4
        and len(node.filtered_inside) >= 4
        and reduction >= 0.15
    )
    costmap_stable = (
        node.costmap_minimum is not None
        and node.costmap_minimum > 0
        and node.costmap_maximum > 0
    )

    print(f"Runner detections: {len(node.box_samples)}")
    print(f"Bounding-box height change: {height_change:.1f} px")
    print(f"Bounding-box center change: {center_change:.1f} px")
    print(f"Median raw points in runner box: {raw_median:.0f}")
    print(f"Median filtered points in runner box: {filtered_median:.0f}")
    print(f"Silhouette point reduction: {100.0 * reduction:.1f}%")
    print(
        f"Occupied costmap cells: min={node.costmap_minimum}, "
        f"max={node.costmap_maximum}"
    )
    print(f"{'PASS' if moved else 'FAIL'} dynamic runner motion")
    print(f"{'PASS' if cutout_worked else 'FAIL'} Depth Anything silhouette cutout")
    print(f"{'PASS' if costmap_stable else 'FAIL'} local costmap continuity")

    success = moved and cutout_worked and costmap_stable
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
