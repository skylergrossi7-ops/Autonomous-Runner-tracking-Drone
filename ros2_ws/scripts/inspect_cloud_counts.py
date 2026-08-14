#!/usr/bin/env python3
import time
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class Counts(Node):
    def __init__(self):
        super().__init__("cloud_count_inspector")
        self.values = {}
        for topic in ("/camera/depth_ai/points", "/camera/depth_ai/filtered_points"):
            self.create_subscription(PointCloud2, topic,
                                     lambda msg, key=topic: self.cb(key, msg),
                                     qos_profile_sensor_data)

    def cb(self, topic, message):
        points = pc2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        if len(points):
            xyz = np.column_stack((points["x"], points["y"], points["z"]))
            sine, cosine = np.sin(0.20), np.cos(0.20)
            base_height = -cosine * xyz[:, 1] + sine * xyz[:, 2] + 0.255
            stats = np.percentile(base_height, [0, 25, 50, 75, 100]).round(3).tolist()
        else:
            stats = []
        self.values[topic] = (len(points), message.header.frame_id, stats)


rclpy.init()
node = Counts()
deadline = time.monotonic() + 25
while time.monotonic() < deadline and len(node.values) < 2:
    rclpy.spin_once(node, timeout_sec=0.25)
print(node.values)
node.destroy_node()
rclpy.shutdown()
