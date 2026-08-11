#!/usr/bin/env python3
import argparse
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageCapture(Node):
    def __init__(self, topic, output):
        super().__init__("one_shot_image_capture")
        self.output = output
        self.saved = False
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, topic, self.callback, qos_profile_sensor_data
        )

    def callback(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.saved = bool(cv2.imwrite(self.output, frame))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImageCapture(args.topic, args.output)
    deadline = time.monotonic() + args.timeout
    while not node.saved and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if node.saved else 1)


if __name__ == "__main__":
    main()
