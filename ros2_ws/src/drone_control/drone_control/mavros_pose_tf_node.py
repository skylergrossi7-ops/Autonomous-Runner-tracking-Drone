"""Broadcast the live MAVROS local pose as odom -> base_link TF."""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class MavrosPoseTfNode(Node):
    def __init__(self):
        super().__init__("mavros_pose_tf_node")
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("broadcast_rate_hz", 20.0)
        self._broadcaster = TransformBroadcaster(self)
        self._latest_pose = None
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        rate = float(self.get_parameter("broadcast_rate_hz").value)
        self.create_timer(
            1.0 / rate,
            self._broadcast_latest,
            clock=self._steady_clock,
        )

    def _pose_callback(self, message):
        self._latest_pose = message.pose

    def _broadcast_latest(self):
        if self._latest_pose is None:
            return
        transform = TransformStamped()
        # Stamp against the same ROS / simulation clock as the point cloud.
        # MAVROS pose messages can be sparse under CPU load, but TF must remain
        # available at every cloud timestamp for the voxel observation buffer.
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("parent_frame").value)
        transform.child_frame_id = str(self.get_parameter("child_frame").value)
        transform.transform.translation.x = self._latest_pose.position.x
        transform.transform.translation.y = self._latest_pose.position.y
        transform.transform.translation.z = self._latest_pose.position.z
        transform.transform.rotation = self._latest_pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosPoseTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
