#!/usr/bin/env python3
import time

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2


class MappingInspector(Node):
    def __init__(self):
        super().__init__("mapping_inspector")
        self.depth_stats = None
        self.obstacle_depths = None
        self.filtered_points = 0
        self.eligible_points = 0
        self.odom_height_stats = None
        self.odom_range_stats = None
        self.cost_values = set()
        self.create_subscription(
            Image, "/camera/depth_ai/depth_image", self.depth_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "/camera/depth_ai/filtered_points", self.cloud_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.costmap_callback, 10
        )

    def depth_callback(self, message):
        depth = np.frombuffer(message.data, dtype=np.float32)
        valid = depth[np.isfinite(depth) & (depth > 0)]
        if valid.size:
            self.depth_stats = tuple(
                float(value) for value in np.percentile(valid, [5, 50, 95])
            )
        if message.width == 640 and message.height == 480:
            image = depth.reshape((message.height, message.width))
            regions = {
                "left_crate": image[250:350, 150:260],
                "tall_post": image[240:335, 255:300],
                "right_barrel": image[255:340, 350:420],
            }
            self.obstacle_depths = {
                name: float(np.nanmedian(region))
                for name, region in regions.items()
            }

    def cloud_callback(self, message):
        points = pc2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        self.filtered_points = max(self.filtered_points, len(points))
        if len(points) == 0:
            return
        xyz = np.column_stack((points["x"], points["y"], points["z"]))
        # camera_optical -> base_link for a camera tilted 0.2 rad upward.
        sine = np.sin(0.2)
        cosine = np.cos(0.2)
        optical_to_base = np.array([
            [0.0, sine, cosine],
            [-1.0, 0.0, 0.0],
            [0.0, -cosine, sine],
        ])
        base = xyz @ optical_to_base.T + np.array([0.0, 0.0, 0.06])
        # base_link -> odom has +90 degree yaw and starts 0.195 m high.
        odom = np.column_stack((-base[:, 1], base[:, 0], base[:, 2] + 0.195))
        planar_range = np.linalg.norm(odom[:, :2], axis=1)
        self.odom_height_stats = tuple(
            float(value) for value in np.percentile(odom[:, 2], [1, 50, 99])
        )
        self.odom_range_stats = tuple(
            float(value) for value in np.percentile(planar_range, [1, 50, 99])
        )
        eligible = (
            (np.abs(odom[:, 0]) <= 15.0)
            & (np.abs(odom[:, 1]) <= 15.0)
            & (odom[:, 2] >= 0.0)
            & (odom[:, 2] <= 4.0)
            & (planar_range >= 0.3)
            & (planar_range <= 20.0)
        )
        self.eligible_points = max(self.eligible_points, int(np.count_nonzero(eligible)))

    def costmap_callback(self, message):
        self.cost_values.update(int(value) for value in message.data)


def main():
    rclpy.init()
    node = MappingInspector()
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    print(f"Depth percentiles (5/50/95 m): {node.depth_stats}")
    print(f"Obstacle-region median depths: {node.obstacle_depths}")
    print(f"Maximum filtered points: {node.filtered_points}")
    print(f"Points eligible for voxel marking: {node.eligible_points}")
    print(f"Transformed height percentiles (1/50/99 m): {node.odom_height_stats}")
    print(f"Planar range percentiles (1/50/99 m): {node.odom_range_stats}")
    print(f"Costmap values: {sorted(node.cost_values)}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
