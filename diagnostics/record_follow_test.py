"""Record runner targets, drone pose, and commands during a follow test."""

import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float64


class FollowRecorder(Node):
    def __init__(self, output_path: str) -> None:
        super().__init__("follow_test_recorder")
        self._started = time.monotonic()
        self._pose = None
        self._command = None
        self._target = None
        self._target_received_at = None
        self._estimated_distance = math.nan
        self._tracking_enabled = False
        self._data_ready_reported = False
        self._file = open(output_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "elapsed_seconds",
                "offset_x",
                "offset_y",
                "box_area",
                "pose_x",
                "pose_y",
                "pose_z",
                "command_x",
                "command_yaw",
                "target_age_seconds",
                "estimated_distance_metres",
                "tracking_enabled",
            ]
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PositionTarget,
            "/mavros/setpoint_raw/local",
            self._command_callback,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            "/perception/runner_target",
            self._target_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/tracking/estimated_distance",
            self._distance_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/tracking/enabled",
            self._activation_callback,
            10,
        )
        self.create_timer(0.1, self._record_sample)
        print("RECORDER_READY", flush=True)

    def _pose_callback(self, message: PoseStamped) -> None:
        self._pose = message.pose

    def _command_callback(self, message: PositionTarget) -> None:
        self._command = message

    def _target_callback(self, message: Vector3Stamped) -> None:
        self._target = message.vector
        self._target_received_at = time.monotonic()

    def _distance_callback(self, message: Float64) -> None:
        self._estimated_distance = float(message.data)

    def _activation_callback(self, message: Bool) -> None:
        self._tracking_enabled = bool(message.data)

    def _record_sample(self) -> None:
        if self._pose is None or self._command is None:
            return
        if not self._data_ready_reported:
            print("RECORDER_DATA_READY", flush=True)
            self._data_ready_reported = True
        target = self._target
        target_age = (
            time.monotonic() - self._target_received_at
            if self._target_received_at is not None
            else math.nan
        )
        self._writer.writerow(
            [
                time.monotonic() - self._started,
                target.x if target is not None else math.nan,
                target.y if target is not None else math.nan,
                target.z if target is not None else math.nan,
                self._pose.position.x,
                self._pose.position.y,
                self._pose.position.z,
                self._command.velocity.x,
                self._command.yaw_rate,
                target_age,
                self._estimated_distance,
                self._tracking_enabled,
            ]
        )
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        return super().destroy_node()


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: record_follow_test.py OUTPUT.csv")
    rclpy.init()
    node = FollowRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
