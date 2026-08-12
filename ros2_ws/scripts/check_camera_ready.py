#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node("camera_preflight_check")
    received = False

    def image_callback(_message):
        nonlocal received
        received = True

    subscription = node.create_subscription(
        Image, "/camera/image_raw", image_callback, qos_profile_sensor_data
    )
    deadline = time.monotonic() + 30.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if received else 1)


if __name__ == "__main__":
    main()
