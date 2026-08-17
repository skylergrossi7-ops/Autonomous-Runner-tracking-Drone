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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.obstacle_components = 0
        self.intrinsics = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
        valid = depth[np.isfinite(depth) & (depth >= 1.0) & (depth <= 15.0)]
        if valid.size < 10:
            self.depth = np.zeros((*depth.shape, 3), dtype=np.uint8)
            return
        low = max(1.0, float(np.percentile(valid, 2.0)))
        high = min(15.0, float(np.percentile(valid, 98.0)))
        high = max(high, low + 0.5)
        clipped = np.clip(np.nan_to_num(depth, nan=high), low, high)
        normalized = np.uint8(255.0 * (1.0 - (clipped - low) / (high - low)))
        normalized = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(
            normalized
        )
        self.depth = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        self.depth[~np.isfinite(depth)] = 0
        # Depth Anything is intentionally smooth. Overlay RGB edges only in
        # the diagnostic view so obstacle outlines remain readable without
        # altering the depth or point cloud used by navigation.
        if self.rgb is not None and self.rgb.shape[:2] == depth.shape:
            grey = cv2.cvtColor(self.rgb, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(grey, 60, 140)
            self.depth[edges > 0] = (245, 245, 245)
        cv2.putText(self.depth, f"range {low:.1f}-{high:.1f} m", (12, 468),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

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
            visible_depth = z[visible]
            low = max(1.0, float(np.percentile(visible_depth, 2.0)))
            high = min(15.0, float(np.percentile(visible_depth, 98.0)))
            high = max(high, low + 0.5)
            values = np.uint8(
                255.0 * (1.0 - np.clip((visible_depth - low) / (high - low), 0, 1))
            )
            colors = cv2.applyColorMap(values.reshape(-1, 1), cv2.COLORMAP_TURBO)[:, 0]
            # Blend depth colour with optical-frame height: taller points are brighter.
            height = -y[visible] * np.cos(0.20) + visible_depth * np.sin(0.20)
            brightness = np.clip(0.65 + 0.20 * (height + 1.0), 0.45, 1.0)[:, None]
            colors = np.uint8(np.clip(colors * brightness, 0, 255))
            for px, py, color in zip(u[visible], v[visible], colors):
                cv2.circle(canvas, (int(px), int(py)), 1,
                           tuple(int(channel) for channel in color), -1)
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
        map_frame = message.header.frame_id or "odom"
        robot_x = None
        robot_y = None
        robot_yaw = self.vehicle_yaw
        try:
            transform = self.tf_buffer.lookup_transform(
                map_frame, "base_link", Time()
            ).transform
            robot_x = float(transform.translation.x)
            robot_y = float(transform.translation.y)
            quaternion = transform.rotation
            robot_yaw = float(np.arctan2(
                2.0 * (
                    quaternion.w * quaternion.z
                    + quaternion.x * quaternion.y
                ),
                1.0 - 2.0 * (
                    quaternion.y * quaternion.y
                    + quaternion.z * quaternion.z
                ),
            ))
        except TransformException:
            pass

        origin = message.info.origin
        origin_quaternion = origin.orientation
        origin_yaw = float(np.arctan2(
            2.0 * (
                origin_quaternion.w * origin_quaternion.z
                + origin_quaternion.x * origin_quaternion.y
            ),
            1.0 - 2.0 * (
                origin_quaternion.y * origin_quaternion.y
                + origin_quaternion.z * origin_quaternion.z
            ),
        ))
        if robot_x is None or robot_y is None:
            robot_grid_x = image.shape[1] / 2.0
            robot_grid_y = image.shape[0] / 2.0
        else:
            delta_x = robot_x - float(origin.position.x)
            delta_y = robot_y - float(origin.position.y)
            cosine = np.cos(origin_yaw)
            sine = np.sin(origin_yaw)
            robot_grid_x = (
                cosine * delta_x + sine * delta_y
            ) / message.info.resolution
            robot_grid_y_up = (
                -sine * delta_x + cosine * delta_y
            ) / message.info.resolution
            robot_grid_y = image.shape[0] - robot_grid_y_up

        robot_grid = (float(robot_grid_x), float(robot_grid_y))
        yaw_in_grid = robot_yaw - origin_yaw
        rotation = cv2.getRotationMatrix2D(
            robot_grid, 90.0 - np.degrees(yaw_in_grid), 1.0
        )
        image = cv2.warpAffine(
            image, rotation, (image.shape[1], image.shape[0]),
            flags=cv2.INTER_NEAREST, borderValue=(35, 35, 35)
        )
        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_NEAREST)
        center = (
            int(round(robot_grid_x * 640.0 / message.info.width)),
            int(round(robot_grid_y * 480.0 / message.info.height)),
        )
        center = (
            int(np.clip(center[0], 0, image.shape[1] - 1)),
            int(np.clip(center[1], 0, image.shape[0] - 1)),
        )
        occupied = int(np.count_nonzero(grid >= 50))
        lethal = np.uint8(grid >= 80)
        labels, _, statistics, _ = cv2.connectedComponentsWithStats(
            lethal, connectivity=8
        )
        # Ignore isolated AI-depth speckles; a physical hazard must occupy at
        # least 4 cells at the configured 8 cm costmap resolution.
        self.obstacle_components = sum(
            1 for label in range(1, labels)
            if statistics[label, cv2.CC_STAT_AREA] >= 3
        )
        cv2.circle(image, center, 8, (255, 255, 255), 2)
        endpoint = (center[0], center[1] - 42)
        cv2.arrowedLine(image, center, endpoint, (255, 255, 255), 3, tipLength=0.3)
        cv2.putText(image, "DRONE", (center[0] + 12, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        center_offset_m = message.info.resolution * float(np.hypot(
            robot_grid_x - message.info.width / 2.0,
            robot_grid_y - message.info.height / 2.0,
        ))
        cv2.putText(
            image, f"base_link centering error: {center_offset_m:.2f} m",
            (14, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
            (255, 255, 255), 1,
        )
        cv2.putText(image, f"occupied cells: {occupied}", (14, 466),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(image, f"distinct footprints: {self.obstacle_components}",
                    (320, 466), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)
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
