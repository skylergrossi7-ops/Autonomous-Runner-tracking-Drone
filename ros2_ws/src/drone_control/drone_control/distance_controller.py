"""Distance estimation and bounded trailing control for one tracked person."""

import math

from .tracking_controller import TrackingCommand, clamp


class MonocularDistanceEstimator:
    """Estimate range from bounding-box area using an inverse-square model.

    The reference area is calibrated for the same camera, person model and
    flight altitude. This remains useful when the horizontal LiDAR beam is
    above a ground-level person's body.
    """

    def __init__(self, reference_distance: float, reference_area: float):
        if reference_distance <= 0.0 or reference_area <= 0.0:
            raise ValueError("distance and area references must be positive")
        self.reference_distance = reference_distance
        self.reference_area = reference_area

    def estimate(self, box_area: float) -> float:
        if not math.isfinite(box_area) or box_area <= 0.0:
            return math.inf
        return self.reference_distance * math.sqrt(
            self.reference_area / box_area
        )


class DistanceTrackingController:
    """Hold a chosen trailing distance while keeping the runner centered."""

    def __init__(
        self,
        desired_distance: float,
        distance_deadband: float,
        distance_gain: float,
        horizontal_deadband: float,
        yaw_gain: float,
        maximum_forward_speed: float,
        maximum_reverse_speed: float,
        maximum_yaw_rate: float,
        forward_alignment_threshold: float,
    ) -> None:
        if desired_distance <= 0.0:
            raise ValueError("desired distance must be positive")
        self.desired_distance = desired_distance
        self.distance_deadband = distance_deadband
        self.distance_gain = distance_gain
        self.horizontal_deadband = horizontal_deadband
        self.yaw_gain = yaw_gain
        self.maximum_forward_speed = maximum_forward_speed
        self.maximum_reverse_speed = maximum_reverse_speed
        self.maximum_yaw_rate = maximum_yaw_rate
        self.forward_alignment_threshold = forward_alignment_threshold

    def calculate(
        self, horizontal_offset: float, measured_distance: float
    ) -> TrackingCommand:
        yaw_rate = 0.0
        if abs(horizontal_offset) > self.horizontal_deadband:
            yaw_rate = clamp(
                -self.yaw_gain * horizontal_offset,
                -self.maximum_yaw_rate,
                self.maximum_yaw_rate,
            )

        forward_speed = 0.0
        distance_error = measured_distance - self.desired_distance
        if (
            math.isfinite(measured_distance)
            and abs(distance_error) > self.distance_deadband
        ):
            forward_speed = clamp(
                self.distance_gain * distance_error,
                -self.maximum_reverse_speed,
                self.maximum_forward_speed,
            )

        if abs(horizontal_offset) > self.forward_alignment_threshold:
            forward_speed = 0.0

        return TrackingCommand(forward_speed, yaw_rate)
