#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class DemoRecorder(Node):
    def __init__(self):
        super().__init__("dynamic_mapping_demo_recorder")
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.filtered = None
        self.costmap = None
        self.vehicle_yaw = 0.0
        self.intrinsics = None
        self._topic_subscriptions = [
            self.create_subscription(
                Image, "/perception/debug_image", self.rgb_callback, SENSOR_QOS
            ),
            self.create_subscription(
                Image, "/camera/depth_ai/depth_image", self.depth_callback, SENSOR_QOS
            ),
            self.create_subscription(
                PointCloud2, "/camera/depth_ai/filtered_points",
                self.cloud_callback, SENSOR_QOS
            ),
            self.create_subscription(
                CameraInfo, "/camera/camera_info", self.info_callback, SENSOR_QOS
            ),
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap", self.map_callback, MAP_QOS
            ),
            self.create_subscription(
                PoseStamped, "/mavros/local_position/pose", self.pose_callback, 10
            ),
        ]

    def rgb_callback(self, message):
        self.rgb = self.bridge.imgmsg_to_cv2(message, "bgr8").copy()

    def depth_callback(self, message):
        depth = self.bridge.imgmsg_to_cv2(message, "32FC1")
        normalized = np.nan_to_num(depth, nan=20.0, posinf=20.0, neginf=0.0)
        normalized = np.clip(normalized, 0.0, 20.0)
        normalized = np.uint8(255.0 * (1.0 - normalized / 20.0))
        self.depth = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

    def info_callback(self, message):
        self.intrinsics = (message.k[0], message.k[4], message.k[2], message.k[5])

    def cloud_callback(self, message):
        if self.intrinsics is None:
            return
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)
        points = pc2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        if len(points):
            x, y, z = points["x"], points["y"], points["z"]
            valid = z > 0.1
            x, y, z = x[valid], y[valid], z[valid]
            fx, fy, cx, cy = self.intrinsics
            u = np.asarray(x * fx / z + cx, dtype=np.int32)
            v = np.asarray(y * fy / z + cy, dtype=np.int32)
            visible = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
            colors = np.clip(255.0 * (1.0 - z[visible] / 20.0), 0, 255).astype(np.uint8)
            canvas[v[visible], u[visible], 1] = 180 + colors // 4
            canvas[v[visible], u[visible], 0] = colors // 3
        self.filtered = canvas

    def map_callback(self, message):
        grid = np.asarray(message.data, dtype=np.int16).reshape(
            message.info.height, message.info.width
        )
        grid = np.flipud(grid)
        image = np.zeros((*grid.shape, 3), dtype=np.uint8)
        image[grid < 0] = (70, 70, 70)       # unknown: grey
        image[grid == 0] = (35, 35, 35)      # free: dark grey
        image[(grid > 0) & (grid < 50)] = (0, 180, 255)  # inflated: amber
        image[(grid >= 50) & (grid < 100)] = (0, 80, 255)  # high: orange
        image[grid >= 100] = (0, 0, 255)     # lethal: red
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_NEAREST)
        occupied = int(np.count_nonzero(grid >= 50))
        center = (image.shape[1] // 2, image.shape[0] // 2)
        cv2.circle(image, center, 8, (255, 255, 255), 2)
        endpoint = (
            int(center[0] + 42 * np.cos(self.vehicle_yaw)),
            int(center[1] - 42 * np.sin(self.vehicle_yaw)),
        )
        cv2.arrowedLine(image, center, endpoint, (255, 255, 255), 3, tipLength=0.3)
        cv2.putText(image, "DRONE", (center[0] + 12, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(image, f"occupied cells: {occupied}", (14, 466),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        self.costmap = image

    def pose_callback(self, message):
        q = message.pose.orientation
        self.vehicle_yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


def panel(image, title):
    if image is None:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(image, "Waiting for topic...", (120, 250),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 220, 220), 2)
    resized = cv2.resize(image, (640, 480))
    cv2.rectangle(resized, (0, 0), (640, 42), (20, 20, 20), -1)
    cv2.putText(resized, title, (14, 29), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, (255, 255, 255), 2)
    return resized


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", default="test_artifacts/live_grid.mp4"
    )
    parser.add_argument(
        "--screenshot", default="test_artifacts/live_grid.png"
    )
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds to run; 0 keeps the display open")
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    for path in (args.output, args.screenshot):
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)

    rclpy.init()
    node = DemoRecorder()
    writer = cv2.VideoWriter(
        args.output,
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (1280, 960),
    )
    if not writer.isOpened():
        raise RuntimeError("Could not open MP4 writer")

    deadline = (
        time.monotonic() + args.duration if args.duration > 0.0 else None
    )
    next_frame = time.monotonic()
    last_frame = None
    while deadline is None or time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now < next_frame:
            continue
        top = np.hstack((
            panel(node.rgb, "YOLO runner tracking"),
            panel(node.depth, "Depth Anything V2 (near = warm)"),
        ))
        bottom = np.hstack((
            panel(node.filtered, "Filtered cloud (runner cutout)"),
            panel(node.costmap, "Nav2 local voxel costmap"),
        ))
        last_frame = np.vstack((top, bottom))
        writer.write(last_frame)
        cv2.imshow("Runner tracking and local mapping", last_frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
            break
        next_frame += 1.0 / args.fps

    writer.release()
    cv2.destroyAllWindows()
    if last_frame is not None:
        cv2.imwrite(args.screenshot, last_frame)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
