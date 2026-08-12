#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node("runner_target_preflight_check")
    received = False

    def target_callback(_message):
        nonlocal received
        received = True

    subscription = node.create_subscription(
        Vector3Stamped, "/perception/runner_target", target_callback, 1
    )
    deadline = time.monotonic() + 20.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if received else 1)


if __name__ == "__main__":
    main()
