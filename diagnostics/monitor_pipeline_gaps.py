#!/usr/bin/env python3
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PipelineGapMonitor(Node):
    def __init__(self, duration):
        super().__init__("pipeline_gap_monitor")
        self.duration = duration
        self.started = time.monotonic()
        self.arrivals = {
            "/camera/image_raw": [],
            "/camera/depth_ai/depth_image": [],
            "/camera/depth_ai/points": [],
            "/camera/depth_ai/filtered_points": [],
            "/perception/detections": [],
            "/local_costmap/costmap": [],
        }
        self.minimum_occupied = None
        self.maximum_occupied = 0
        specs = (
            (Image, "/camera/image_raw", SENSOR_QOS, self._record),
            (Image, "/camera/depth_ai/depth_image", SENSOR_QOS, self._record),
            (PointCloud2, "/camera/depth_ai/points", SENSOR_QOS, self._record),
            (PointCloud2, "/camera/depth_ai/filtered_points", SENSOR_QOS, self._record),
            (Detection2DArray, "/perception/detections", RELIABLE_QOS, self._record),
            (OccupancyGrid, "/local_costmap/costmap", RELIABLE_QOS, self._costmap),
        )
        self._topic_subscriptions = []
        for message_type, topic, qos, handler in specs:
            self._topic_subscriptions.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, name=topic, callback=handler: callback(name, message),
                    qos,
                )
            )

    def _record(self, topic, _message):
        self.arrivals[topic].append(time.monotonic())

    def _costmap(self, topic, message):
        self._record(topic, message)
        occupied = sum(1 for value in message.data if value >= 50)
        self.maximum_occupied = max(self.maximum_occupied, occupied)
        if self.minimum_occupied is None:
            self.minimum_occupied = occupied
        else:
            self.minimum_occupied = min(self.minimum_occupied, occupied)


def main():
    duration = 70.0
    rclpy.init()
    node = PipelineGapMonitor(duration)
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    passed = True
    for topic, arrivals in node.arrivals.items():
        if len(arrivals) > 1:
            gaps = [later - earlier for earlier, later in zip(arrivals, arrivals[1:])]
            maximum_gap = max(gaps)
            rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
        else:
            maximum_gap = float("inf")
            rate = 0.0
        gap_limit = 4.0 if "depth_ai" in topic else 4.0
        if topic == "/camera/image_raw":
            gap_limit = 4.0
        if topic == "/local_costmap/costmap":
            # Gazebo simulation time runs slower than wall time during CPU
            # inference; the transient-local map remains visible between sends.
            gap_limit = 8.0
        topic_passed = len(arrivals) > 1 and maximum_gap <= gap_limit
        passed &= topic_passed
        print(
            f"{'PASS' if topic_passed else 'FAIL'} {topic}: "
            f"count={len(arrivals)}, rate={rate:.2f} Hz, max_gap={maximum_gap:.2f} s"
        )

    occupied_persisted = (
        node.minimum_occupied is not None
        and node.maximum_occupied > 0
        and node.minimum_occupied > 0
    )
    passed &= occupied_persisted
    print(
        f"{'PASS' if occupied_persisted else 'FAIL'} occupied costmap continuity: "
        f"min={node.minimum_occupied}, max={node.maximum_occupied}"
    )
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
