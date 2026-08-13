#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray


class YoloMaskingNode(Node):

    def __init__(self):
        super().__init__("yolo_masking_node")

        # Declare parameters with standard defaults
        self.declare_parameter("point_cloud_topic", "/camera/depth_ai/points")
        self.declare_parameter("bbox_topic", "/yolov8/bounding_boxes")
        self.declare_parameter("filtered_cloud_topic", "/camera/depth_ai/filtered_points")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # Camera intrinsic defaults (will be overwritten if camera_info is received)
        self.declare_parameter("fx", 525.0)
        self.declare_parameter("fy", 525.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)

        # Filtering parameters
        self.declare_parameter("min_z", 0.1)
        self.declare_parameter("max_z", 10.0)
        self.declare_parameter("depth_tolerance", 1.0)  # meters around the median depth
        # CPU-only simulation can take a few seconds to run YOLO and monocular
        # depth concurrently. The image timestamps still keep the mask bounded.
        self.declare_parameter("max_bbox_age", 4.0)
        self.declare_parameter("remove_ground_plane", True)
        self.declare_parameter("ground_distance_threshold", 0.18)
        self.declare_parameter("ground_normal_tolerance", 0.35)
        self.declare_parameter("ground_ransac_iterations", 50)
        self.declare_parameter("ground_min_inliers", 250)
        self.declare_parameter("camera_upward_pitch_radians", 0.2)
        self.declare_parameter("ground_horizon_fraction", 0.58)
        self.declare_parameter("ground_keep_height_fraction", 0.28)
        self.declare_parameter("output_frame", "camera_optical_frame")
        self.declare_parameter("remove_isolated_points", True)
        self.declare_parameter("isolation_voxel_size", 0.20)
        self.declare_parameter("isolation_min_points", 3)

        # Retrieve parameter values
        self.point_cloud_topic = self.get_parameter("point_cloud_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.filtered_cloud_topic = self.get_parameter("filtered_cloud_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        self.fx = self.get_parameter("fx").value
        self.fy = self.get_parameter("fy").value
        self.cx = self.get_parameter("cx").value
        self.cy = self.get_parameter("cy").value

        self.min_z = self.get_parameter("min_z").value
        self.max_z = self.get_parameter("max_z").value
        self.depth_tolerance = self.get_parameter("depth_tolerance").value
        self.max_bbox_age = self.get_parameter("max_bbox_age").value
        self.remove_ground_plane = bool(
            self.get_parameter("remove_ground_plane").value
        )

        # State variables
        self.latest_bboxes = None
        self.latest_bbox_time = None

        # Set up QoS profiles compatible with typical best-effort / transient-local camera streams
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.point_cloud_topic,
            self.point_cloud_callback,
            qos_profile
        )

        self.bbox_sub = self.create_subscription(
            Detection2DArray,
            self.bbox_topic,
            self.bbox_callback,
            detection_qos
        )

        # Also support alternate/flexible subscriptions
        self.alternate_bbox_sub = self.create_subscription(
            Detection2DArray,
            "/perception/detections",
            self.bbox_callback,
            detection_qos
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile
        )

        # Create publisher
        self.pc_pub = self.create_publisher(
            PointCloud2,
            self.filtered_cloud_topic,
            qos_profile
        )

        self.get_logger().info(
            f"YOLO Masking Node initialized.\n"
            f"Subscribing to point cloud: {self.point_cloud_topic}\n"
            f"Subscribing to bboxes: {self.bbox_topic} and /perception/detections\n"
            f"Publishing filtered points to: {self.filtered_cloud_topic}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        # K matrix elements: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def bbox_callback(self, msg: Detection2DArray):
        self.latest_bboxes = msg.detections
        # Store time as a float (seconds) using the message header stamp
        self.latest_bbox_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def point_cloud_callback(self, msg: PointCloud2):
        # Determine current message timestamp in seconds
        pc_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Without a fresh runner box, retain the environmental cloud. Empty
        # output would hide real obstacles from Nav2.
        if self.latest_bboxes is None or self.latest_bbox_time is None:
            self.pc_pub.publish(msg)
            return

        bbox_age = pc_time - self.latest_bbox_time
        if bbox_age > self.max_bbox_age:
            self.get_logger().warn(
                f"YOLO bounding boxes are too stale (age: {bbox_age:.2f}s, threshold: {self.max_bbox_age:.2f}s). "
                f"Passing through the cloud for Nav2 safety.",
                throttle_duration_sec=2.0
            )
            self.pc_pub.publish(msg)
            return

        # Read the points from the cloud message
        # Returns a structured numpy array with fields 'x', 'y', 'z' (and possibly others)
        try:
            points_arr = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f"Failed to read point cloud: {e}")
            return

        if len(points_arr) == 0:
            self.publish_empty_cloud(msg.header)
            return

        x = points_arr['x']
        y = points_arr['y']
        z = points_arr['z']

        # Pre-filter by depth range
        valid_depth_mask = (z > self.min_z) & (z < self.max_z)
        if not np.any(valid_depth_mask):
            self.publish_empty_cloud(msg.header)
            return

        x_val = x[valid_depth_mask]
        y_val = y[valid_depth_mask]
        z_val = z[valid_depth_mask]

        # Project valid 3D points to 2D image coordinates (u, v)
        u = (x_val * self.fx) / z_val + self.cx
        v = (y_val * self.fy) / z_val + self.cy

        # Begin with all environmental points and remove runner points.
        retained_mask = np.ones_like(z_val, dtype=bool)

        for detection in self.latest_bboxes:
            # bbox center and sizes
            cx_box = detection.bbox.center.position.x
            cy_box = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y

            xmin = cx_box - size_x / 2.0
            xmax = cx_box + size_x / 2.0
            ymin = cy_box - size_y / 2.0
            ymax = cy_box + size_y / 2.0

            # Find points projecting inside this bounding box
            in_box = (u >= xmin) & (u <= xmax) & (v >= ymin) & (v <= ymax)
            if np.any(in_box):
                # Filter out background clutter and ground plane noise using median depth
                box_depths = z_val[in_box]
                median_depth = np.median(box_depths)

                # Check which points inside the box are within the depth tolerance
                close_to_median = np.abs(box_depths - median_depth) <= self.depth_tolerance

                # Remove the foreground runner surface while keeping background
                # geometry that merely projects through the same rectangle.
                in_box_indices = np.where(in_box)[0]
                retained_mask[in_box_indices[close_to_median]] = False

        # Extract the filtered 3D points
        # Monocular depth often lifts the entire runway above the fitted plane.
        # Suppress points in the lower image region unless they extend far
        # enough above the local ground contact region to be a real obstacle.
        horizon = float(self.get_parameter("ground_horizon_fraction").value)
        keep_height = float(
            self.get_parameter("ground_keep_height_fraction").value
        )
        image_height = max(1.0, 2.0 * self.cy)
        lower_image = v >= image_height * horizon
        projected_height = image_height - v
        ground_image = lower_image & (
            projected_height <= image_height * keep_height
        )
        retained_mask &= ~ground_image
        filtered_points = np.column_stack((
            x_val[retained_mask], y_val[retained_mask], z_val[retained_mask]
        ))
        filtered_points = self.remove_ground(filtered_points)
        filtered_points = self.remove_isolated(filtered_points)

        # Re-create and publish the PointCloud2 message
        try:
            filtered_points = np.ascontiguousarray(filtered_points, dtype=np.float32)
            filtered_cloud_msg = PointCloud2()
            filtered_cloud_msg.header = msg.header
            filtered_cloud_msg.header.frame_id = str(
                self.get_parameter("output_frame").value
            )
            filtered_cloud_msg.height = 1
            filtered_cloud_msg.width = len(filtered_points)
            filtered_cloud_msg.fields = msg.fields[:3]
            filtered_cloud_msg.is_bigendian = False
            filtered_cloud_msg.point_step = 12
            filtered_cloud_msg.row_step = 12 * len(filtered_points)
            filtered_cloud_msg.is_dense = True
            filtered_cloud_msg.data = filtered_points.tobytes()
            self.pc_pub.publish(filtered_cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to create/publish filtered point cloud: {e}")

    def remove_ground(self, points):
        """Remove only a dominant plane whose normal matches level ground."""
        if not self.remove_ground_plane or len(points) < 3:
            return points
        threshold = float(
            self.get_parameter("ground_distance_threshold").value
        )
        minimum = int(self.get_parameter("ground_min_inliers").value)
        iterations = int(
            self.get_parameter("ground_ransac_iterations").value
        )
        pitch = float(
            self.get_parameter("camera_upward_pitch_radians").value
        )
        expected = np.array(
            [0.0, -np.cos(pitch), np.sin(pitch)], dtype=np.float32
        )
        tolerance = float(
            self.get_parameter("ground_normal_tolerance").value
        )
        sample_count = min(len(points), 6000)
        sample_indices = np.linspace(
            0, len(points) - 1, sample_count, dtype=np.int32
        )
        sample = points[sample_indices]
        rng = np.random.default_rng(7)
        best_mask = None
        best_count = 0
        for _ in range(iterations):
            selected = sample[rng.choice(sample_count, 3, replace=False)]
            normal = np.cross(selected[1] - selected[0], selected[2] - selected[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            if abs(float(np.dot(normal, expected))) < 1.0 - tolerance:
                continue
            offset = -float(np.dot(normal, selected[0]))
            inliers = np.abs(sample @ normal + offset) <= threshold
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_mask = (normal, offset)
        if best_mask is None or best_count < minimum:
            return points
        normal, offset = best_mask
        ground = np.abs(points @ normal + offset) <= threshold
        return points[~ground]

    def remove_isolated(self, points):
        """Reject tiny monocular-depth speckles while retaining solid surfaces."""
        if not bool(self.get_parameter("remove_isolated_points").value) or len(points) < 3:
            return points
        size = max(0.05, float(self.get_parameter("isolation_voxel_size").value))
        minimum = max(1, int(self.get_parameter("isolation_min_points").value))
        keys = np.floor(points / size).astype(np.int32)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        return points[counts[inverse] >= minimum]

    def publish_empty_cloud(self, header):
        empty_cloud_msg = pc2.create_cloud_xyz32(header, [])
        self.pc_pub.publish(empty_cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloMaskingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
