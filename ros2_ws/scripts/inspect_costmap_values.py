#!/usr/bin/env python3
import time
from collections import Counter

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class Inspector(Node):
    def __init__(self):
        super().__init__("costmap_value_inspector")
        self.message = None
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 self.callback, qos)

    def callback(self, message):
        self.message = message


rclpy.init()
node = Inspector()
deadline = time.monotonic() + 12
while node.message is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.25)
if node.message is None:
    raise SystemExit("no costmap")
values = np.asarray(node.message.data, dtype=np.int16)
print("min/max", int(values.min()), int(values.max()))
print("counts", Counter(values.tolist()).most_common(20))
print("positive", int(np.count_nonzero(values > 0)), "over80", int(np.count_nonzero(values >= 80)))
node.destroy_node()
rclpy.shutdown()
