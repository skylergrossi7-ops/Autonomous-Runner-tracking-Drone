"""ROS 2 runner follower with target timeout and LiDAR safety stop."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from mavros_msgs.msg import PositionTarget
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from .distance_controller import (
    DistanceTrackingController,
    MonocularDistanceEstimator,
)
from .gimbal_pitch_controller import GimbalPitchController
from .tracking_controller import TrackingCommand, TrackingController


class RunnerFollowerNode(Node):
    """Publish safe body-relative commands from runner perception."""

    def __init__(self) -> None:
        super().__init__("runner_follower")

        self.declare_parameter("enabled", False)
        self.declare_parameter("forward_commands_enabled", False)
        self.declare_parameter("target_topic", "/perception/runner_target")
        self.declare_parameter("scan_topic", "/lidar/scan")
        self.declare_parameter("command_topic", "/tracking/cmd_vel")
        self.declare_parameter(
            "activation_topic", "/tracking/enabled"
        )
        self.declare_parameter(
            "mavros_raw_topic",
            "/mavros/setpoint_raw/local",
        )
        self.declare_parameter("publish_to_mavros", False)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("target_timeout_seconds", 0.6)
        self.declare_parameter("scan_timeout_seconds", 0.6)
        self.declare_parameter("minimum_obstacle_distance", 2.0)
        self.declare_parameter("safety_angle_degrees", 35.0)
        self.declare_parameter("target_area", 0.12)
        self.declare_parameter("area_deadband", 0.02)
        self.declare_parameter("distance_control_enabled", True)
        self.declare_parameter("desired_follow_distance", 2.5)
        self.declare_parameter("distance_deadband", 0.25)
        self.declare_parameter("distance_gain", 0.5)
        self.declare_parameter("distance_reference_distance", 2.5)
        self.declare_parameter("distance_reference_area", 0.022)
        self.declare_parameter(
            "estimated_distance_topic", "/tracking/estimated_distance"
        )
        self.declare_parameter("horizontal_deadband", 0.08)
        self.declare_parameter("forward_gain", 3.0)
        self.declare_parameter("yaw_gain", 0.8)
        self.declare_parameter("maximum_forward_speed", 1.5)
        self.declare_parameter("maximum_reverse_speed", 0.5)
        self.declare_parameter("maximum_yaw_rate", 0.6)
        self.declare_parameter("forward_alignment_threshold", 0.15)
        self.declare_parameter("gimbal_pitch_enabled", True)
        self.declare_parameter(
            "gimbal_pitch_topic", "/tracking/gimbal_pitch"
        )
        self.declare_parameter("gimbal_initial_pitch", 0.0)
        self.declare_parameter("gimbal_minimum_pitch", -1.2)
        self.declare_parameter("gimbal_maximum_pitch", 0.6)
        self.declare_parameter("gimbal_vertical_deadband", 0.12)
        self.declare_parameter("gimbal_pitch_gain", 0.35)
        self.declare_parameter("gimbal_maximum_pitch_rate", 0.25)
        self.declare_parameter("gimbal_pitch_direction", 1.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._forward_enabled = bool(
            self.get_parameter("forward_commands_enabled").value
        )
        self._publish_to_mavros = bool(
            self.get_parameter("publish_to_mavros").value
        )
        self.add_on_set_parameters_callback(
            self._parameters_callback
        )
        self._target_timeout = float(
            self.get_parameter("target_timeout_seconds").value
        )
        self._scan_timeout = float(
            self.get_parameter("scan_timeout_seconds").value
        )
        self._minimum_obstacle_distance = float(
            self.get_parameter("minimum_obstacle_distance").value
        )
        self._safety_angle = math.radians(
            float(self.get_parameter("safety_angle_degrees").value)
        )

        self._controller = TrackingController(
            target_area=float(self.get_parameter("target_area").value),
            area_deadband=float(
                self.get_parameter("area_deadband").value
            ),
            horizontal_deadband=float(
                self.get_parameter("horizontal_deadband").value
            ),
            forward_gain=float(self.get_parameter("forward_gain").value),
            yaw_gain=float(self.get_parameter("yaw_gain").value),
            maximum_forward_speed=float(
                self.get_parameter("maximum_forward_speed").value
            ),
            maximum_reverse_speed=float(
                self.get_parameter("maximum_reverse_speed").value
            ),
            maximum_yaw_rate=float(
                self.get_parameter("maximum_yaw_rate").value
            ),
            forward_alignment_threshold=float(
                self.get_parameter(
                    "forward_alignment_threshold"
                ).value
            ),
        )
        self._distance_control_enabled = bool(
            self.get_parameter("distance_control_enabled").value
        )
        self._distance_estimator = MonocularDistanceEstimator(
            reference_distance=float(
                self.get_parameter("distance_reference_distance").value
            ),
            reference_area=float(
                self.get_parameter("distance_reference_area").value
            ),
        )
        self._distance_controller = DistanceTrackingController(
            desired_distance=float(
                self.get_parameter("desired_follow_distance").value
            ),
            distance_deadband=float(
                self.get_parameter("distance_deadband").value
            ),
            distance_gain=float(
                self.get_parameter("distance_gain").value
            ),
            horizontal_deadband=float(
                self.get_parameter("horizontal_deadband").value
            ),
            yaw_gain=float(self.get_parameter("yaw_gain").value),
            maximum_forward_speed=float(
                self.get_parameter("maximum_forward_speed").value
            ),
            maximum_reverse_speed=float(
                self.get_parameter("maximum_reverse_speed").value
            ),
            maximum_yaw_rate=float(
                self.get_parameter("maximum_yaw_rate").value
            ),
            forward_alignment_threshold=float(
                self.get_parameter(
                    "forward_alignment_threshold"
                ).value
            ),
        )
        self._gimbal_enabled = bool(
            self.get_parameter("gimbal_pitch_enabled").value
        )
        self._gimbal_controller = GimbalPitchController(
            initial_angle=float(
                self.get_parameter("gimbal_initial_pitch").value
            ),
            minimum_angle=float(
                self.get_parameter("gimbal_minimum_pitch").value
            ),
            maximum_angle=float(
                self.get_parameter("gimbal_maximum_pitch").value
            ),
            vertical_deadband=float(
                self.get_parameter("gimbal_vertical_deadband").value
            ),
            pitch_gain=float(
                self.get_parameter("gimbal_pitch_gain").value
            ),
            maximum_pitch_rate=float(
                self.get_parameter("gimbal_maximum_pitch_rate").value
            ),
            direction=float(
                self.get_parameter("gimbal_pitch_direction").value
            ),
        )

        self._latest_target: Optional[Vector3Stamped] = None
        self._target_received_at = None
        self._scan_received_at = None
        self._front_obstacle_distance = math.inf

        self._command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self._mavros_raw_publisher = self.create_publisher(
            PositionTarget,
            str(self.get_parameter("mavros_raw_topic").value),
            10,
        )
        self._gimbal_pitch_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("gimbal_pitch_topic").value),
            10,
        )
        self._distance_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("estimated_distance_topic").value),
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("target_topic").value),
            self._target_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("activation_topic").value),
            self._activation_callback,
            10,
        )

        rate = float(self.get_parameter("control_rate_hz").value)
        self._control_period = 1.0 / rate
        self.create_timer(1.0 / rate, self._control_callback)
        self.get_logger().info(
            "Runner follower ready; enabled=%s, forward=%s, mavros=%s"
            % (
                self._enabled,
                self._forward_enabled,
                self._publish_to_mavros,
            )
        )

    def _parameters_callback(self, parameters):
        """Apply the safety-critical enable switch without restarting ROS."""
        for parameter in parameters:
            if parameter.name == "enabled":
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False,
                        reason="enabled must be a boolean",
                    )
                self._enabled = bool(parameter.value)
                self.get_logger().info(
                    "Tracking motion %s"
                    % ("enabled" if self._enabled else "disabled")
                )
            elif parameter.name == "desired_follow_distance":
                if (
                    parameter.type_ != Parameter.Type.DOUBLE
                    or float(parameter.value) <= 0.0
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="desired_follow_distance must be positive",
                    )
                self._distance_controller.desired_distance = float(
                    parameter.value
                )
                self.get_logger().info(
                    "Desired trailing distance set to %.2f m"
                    % float(parameter.value)
                )
        return SetParametersResult(successful=True)

    def _activation_callback(self, message: Bool) -> None:
        """Apply a low-latency, externally observable motion gate."""
        self._enabled = bool(message.data)
        self.get_logger().info(
            "Tracking motion %s"
            % ("enabled" if self._enabled else "disabled")
        )

    def _target_callback(self, message: Vector3Stamped) -> None:
        self._latest_target = message
        self._target_received_at = self.get_clock().now()

    def _scan_callback(self, message: LaserScan) -> None:
        valid_ranges = []
        angle = message.angle_min
        for distance in message.ranges:
            if (
                abs(angle) <= self._safety_angle
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                valid_ranges.append(distance)
            angle += message.angle_increment

        self._front_obstacle_distance = (
            min(valid_ranges) if valid_ranges else math.inf
        )
        self._scan_received_at = self.get_clock().now()

    def _is_fresh(self, received_at, timeout: float) -> bool:
        if received_at is None:
            return False
        age = (self.get_clock().now() - received_at).nanoseconds / 1e9
        return age <= timeout

    def _safe_command(self) -> TrackingCommand:
        if (
            not self._enabled
            or self._latest_target is None
            or not self._is_fresh(
                self._target_received_at, self._target_timeout
            )
        ):
            return TrackingCommand()

        estimated_distance = self._distance_estimator.estimate(
            self._latest_target.vector.z
        )
        self._distance_publisher.publish(
            Float64(data=estimated_distance)
        )
        if self._distance_control_enabled:
            command = self._distance_controller.calculate(
                horizontal_offset=self._latest_target.vector.x,
                measured_distance=estimated_distance,
            )
        else:
            command = self._controller.calculate(
                horizontal_offset=self._latest_target.vector.x,
                box_area=self._latest_target.vector.z,
            )
        forward_speed = (
            command.forward_speed if self._forward_enabled else 0.0
        )

        scan_is_fresh = self._is_fresh(
            self._scan_received_at, self._scan_timeout
        )
        if not scan_is_fresh or (
            self._front_obstacle_distance
            < self._minimum_obstacle_distance
        ):
            forward_speed = min(0.0, forward_speed)

        return TrackingCommand(forward_speed, command.yaw_rate)

    def _control_callback(self) -> None:
        command = self._safe_command()
        twist = Twist()
        twist.linear.x = command.forward_speed
        twist.angular.z = command.yaw_rate
        self._command_publisher.publish(twist)

        if self._publish_to_mavros:
            self._mavros_raw_publisher.publish(
                self._to_body_setpoint(command)
            )

        if self._gimbal_enabled:
            self._publish_gimbal_pitch()

    def _publish_gimbal_pitch(self) -> None:
        """Keep a fresh runner vertically centered without moving the drone."""
        if (
            self._latest_target is not None
            and self._is_fresh(
                self._target_received_at,
                self._target_timeout,
            )
        ):
            pitch = self._gimbal_controller.calculate(
                vertical_offset=self._latest_target.vector.y,
                elapsed_seconds=self._control_period,
            ).angle
        else:
            pitch = self._gimbal_controller.angle
        self._gimbal_pitch_publisher.publish(Float64(data=pitch))

    def _to_body_setpoint(
        self, command: TrackingCommand
    ) -> PositionTarget:
        """Build the same body-NED velocity command as the original code."""
        setpoint = PositionTarget()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = "base_link"
        setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # Ignore position, acceleration / force, and absolute yaw. Use only
        # velocity and yaw rate, matching VELOCITY_AND_YAW_RATE_TYPE_MASK in
        # the original DroneKit implementation.
        setpoint.type_mask = 0b0000011111000111
        setpoint.velocity.x = command.forward_speed
        setpoint.velocity.y = 0.0
        setpoint.velocity.z = 0.0

        # With this ArduPilot / MAVROS raw-setpoint path, a positive value
        # produces the same counter-clockwise turn as ROS angular.z.
        # This sign was verified against the Gazebo vehicle heading.
        setpoint.yaw_rate = command.yaw_rate
        return setpoint


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RunnerFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
