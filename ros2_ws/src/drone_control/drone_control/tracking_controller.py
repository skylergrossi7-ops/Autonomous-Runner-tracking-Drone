"""Pure control logic for following a detected runner."""

from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class TrackingCommand:
    forward_speed: float = 0.0
    yaw_rate: float = 0.0


class TrackingController:
    """Turn normalized camera measurements into bounded velocity commands."""

    def __init__(
        self,
        target_area: float,
        area_deadband: float,
        horizontal_deadband: float,
        forward_gain: float,
        yaw_gain: float,
        maximum_forward_speed: float,
        maximum_reverse_speed: float,
        maximum_yaw_rate: float,
        forward_alignment_threshold: float,
    ) -> None:
        self.target_area = target_area
        self.area_deadband = area_deadband
        self.horizontal_deadband = horizontal_deadband
        self.forward_gain = forward_gain
        self.yaw_gain = yaw_gain
        self.maximum_forward_speed = maximum_forward_speed
        self.maximum_reverse_speed = maximum_reverse_speed
        self.maximum_yaw_rate = maximum_yaw_rate
        self.forward_alignment_threshold = forward_alignment_threshold

    def calculate(
        self, horizontal_offset: float, box_area: float
    ) -> TrackingCommand:
        yaw_rate = 0.0
        if abs(horizontal_offset) > self.horizontal_deadband:
            yaw_rate = clamp(
                -self.yaw_gain * horizontal_offset,
                -self.maximum_yaw_rate,
                self.maximum_yaw_rate,
            )

        area_error = self.target_area - box_area
        forward_speed = 0.0
        if abs(area_error) > self.area_deadband:
            forward_speed = clamp(
                self.forward_gain * area_error,
                -self.maximum_reverse_speed,
                self.maximum_forward_speed,
            )

        # Do not close distance while the target is far from the optical
        # center. Turning first prevents a curved overshoot and target loss.
        if abs(horizontal_offset) > self.forward_alignment_threshold:
            forward_speed = 0.0

        return TrackingCommand(forward_speed, yaw_rate)
