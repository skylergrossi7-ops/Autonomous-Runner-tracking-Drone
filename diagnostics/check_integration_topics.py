#!/usr/bin/env python3
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from vision_msgs.msg import Detection2DArray


class IntegrationTopicCheck(Node):
    def __init__(self):
        super().__init__("integration_topic_check")
        self.seen = {
            "/camera/image_raw": False,
            "/camera/camera_info": False,
            "/camera/depth_ai/depth_image": False,
            "/camera/depth_ai/points": False,
            "/camera/depth_ai/filtered_points": False,
            "/perception/detections": False,
            "/local_costmap/costmap": False,
        }
        subscriptions = (
            (Image, "/camera/image_raw", qos_profile_sensor_data),
            (CameraInfo, "/camera/camera_info", qos_profile_sensor_data),
            (Image, "/camera/depth_ai/depth_image", qos_profile_sensor_data),
            (PointCloud2, "/camera/depth_ai/points", qos_profile_sensor_data),
            (PointCloud2, "/camera/depth_ai/filtered_points", qos_profile_sensor_data),
            (Detection2DArray, "/perception/detections", 10),
            (OccupancyGrid, "/local_costmap/costmap", 10),
        )
        self._topic_subscriptions = []
        for message_type, topic, qos in subscriptions:
            if topic == "/perception/detections":
                callback = self._detections_callback
            else:
                callback = (
                    lambda _message, name=topic:
                    self.seen.__setitem__(name, True)
                )
            self._topic_subscriptions.append(
                self.create_subscription(message_type, topic, callback, qos)
            )

    def _detections_callback(self, message):
        if message.detections:
            self.seen["/perception/detections"] = True


def main():
    rclpy.init()
    node = IntegrationTopicCheck()
    deadline = time.monotonic() + 35.0
    while time.monotonic() < deadline and not all(node.seen.values()):
        rclpy.spin_once(node, timeout_sec=0.25)
    for topic, received in node.seen.items():
        print(f"{'PASS' if received else 'FAIL'} {topic}", flush=True)
    success = all(node.seen.values())
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
