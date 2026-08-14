"""Fuse the selected YOLO runner with AI depth into a body-frame vector."""

import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TargetVectorNode(Node):
    def __init__(self):
        super().__init__("target_vector_node")
        self.declare_parameter("runner_topic", "/perception/runner")
        self.declare_parameter("depth_topic", "/camera/depth_ai/depth_image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("target_topic", "/perception/runner_target")
        self.declare_parameter("runner_timeout_seconds", 4.0)
        self.declare_parameter("depth_window_radius", 6)
        self.declare_parameter("camera_upward_pitch_radians", 0.20)
        self.declare_parameter("mount_forward_metres", 0.0)

        self._bridge = CvBridge()
        self._runner = None
        self._runner_arrival = None
        self._intrinsics = None
        self._publisher = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("target_topic").value),
            1,
        )
        self.create_subscription(
            Detection2D,
            str(self.get_parameter("runner_topic").value),
            self._runner_callback,
            1,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._info_callback,
            SENSOR_QOS,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_callback,
            SENSOR_QOS,
        )
        self.get_logger().info("AI-depth runner target vector ready")

    def _runner_callback(self, message):
        self._runner = message
        self._runner_arrival = time.monotonic()

    def _info_callback(self, message):
        self._intrinsics = (
            float(message.k[0]), float(message.k[4]),
            float(message.k[2]), float(message.k[5]),
        )

    def _depth_callback(self, message):
        if self._runner is None or self._intrinsics is None:
            return
        timeout = float(self.get_parameter("runner_timeout_seconds").value)
        if time.monotonic() - self._runner_arrival > timeout:
            return
        depth = self._bridge.imgmsg_to_cv2(message, "32FC1")
        center = self._runner.bbox.center.position
        pixel_x, pixel_y = int(round(center.x)), int(round(center.y))
        radius = int(self.get_parameter("depth_window_radius").value)
        height, width = depth.shape
        x1, x2 = max(0, pixel_x - radius), min(width, pixel_x + radius + 1)
        y1, y2 = max(0, pixel_y - radius), min(height, pixel_y + radius + 1)
        values = depth[y1:y2, x1:x2]
        valid = values[np.isfinite(values) & (values > 0.2) & (values < 20.0)]
        if valid.size == 0:
            return
        z = float(np.median(valid))
        fx, fy, cx, cy = self._intrinsics
        optical_x = (float(center.x) - cx) * z / fx
        optical_y = (float(center.y) - cy) * z / fy
        pitch = float(self.get_parameter("camera_upward_pitch_radians").value)

        target = Vector3Stamped()
        target.header = message.header
        target.header.frame_id = "base_link"
        target.vector.x = (
            math.sin(pitch) * optical_y
            + math.cos(pitch) * z
            + float(self.get_parameter("mount_forward_metres").value)
        )
        target.vector.y = -optical_x
        target.vector.z = -math.cos(pitch) * optical_y + math.sin(pitch) * z
        self._publisher.publish(target)


def main(args=None):
    rclpy.init(args=args)
    node = TargetVectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
