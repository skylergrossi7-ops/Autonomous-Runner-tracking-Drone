import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarNode(Node):

    def __init__(self):
        super().__init__("lidar_node")

        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("LiDAR node started; listening to /scan")

    def scan_callback(self, msg):
        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance)
            and msg.range_min <= distance <= msg.range_max
        ]

        if not valid_ranges:
            self.get_logger().warning("No valid LiDAR measurements")
            return

        closest_distance = min(valid_ranges)
        closest_index = msg.ranges.index(closest_distance)

        closest_angle = (
            msg.angle_min
            + closest_index * msg.angle_increment
        )

        angle_degrees = math.degrees(closest_angle)

        self.get_logger().info(
            f"Closest object: {closest_distance:.2f} m "
            f"at {angle_degrees:.1f} degrees"
        )


def main(args=None):
    rclpy.init(args=args)

    node = LidarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
