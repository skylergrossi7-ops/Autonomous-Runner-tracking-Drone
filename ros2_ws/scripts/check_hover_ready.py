#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = Node("hover_preflight_check")
    altitude = None

    def pose_callback(message):
        nonlocal altitude
        altitude = message.pose.position.z

    subscription = node.create_subscription(
        PoseStamped,
        "/mavros/local_position/pose",
        pose_callback,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 30.0
    while (altitude is None or altitude < 1.0) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    if altitude is not None:
        print(f"Measured hover altitude: {altitude:.3f} m")
    raise SystemExit(0 if altitude is not None and altitude >= 1.0 else 1)


if __name__ == "__main__":
    main()
