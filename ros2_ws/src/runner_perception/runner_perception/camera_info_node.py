"""Publish calibrated intrinsics for the simulated monocular camera."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoNode(Node):
    def __init__(self):
        super().__init__("camera_info_node")
        self.declare_parameter("topic", "/camera/camera_info")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("horizontal_fov_radians", 1.3962634)
        self.declare_parameter("publish_rate", 2.0)

        self._publisher = self.create_publisher(
            CameraInfo, str(self.get_parameter("topic").value), 10
        )
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("Publishing calibrated camera intrinsics")

    def _publish(self):
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        horizontal_fov = float(
            self.get_parameter("horizontal_fov_radians").value
        )
        focal = width / (2.0 * math.tan(horizontal_fov / 2.0))
        cx = (width - 1.0) / 2.0
        cy = (height - 1.0) / 2.0

        message = CameraInfo()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.width = width
        message.height = height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            focal, 0.0, cx, 0.0,
            0.0, focal, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
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
