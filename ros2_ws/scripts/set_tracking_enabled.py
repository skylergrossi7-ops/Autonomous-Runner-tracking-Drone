#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    enabled = sys.argv[1].lower() in {"1", "true", "yes", "on"}
    rclpy.init()
    node = Node("tracking_activation_command")
    publisher = node.create_publisher(Bool, "/tracking/enabled", 10)
    message = Bool(data=enabled)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
