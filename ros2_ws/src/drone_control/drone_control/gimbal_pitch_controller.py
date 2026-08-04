"""Bounded vertical image controller for the tracking-camera gimbal."""

from dataclasses import dataclass

from .tracking_controller import clamp


@dataclass(frozen=True)
class GimbalPitchCommand:
    angle: float
    rate: float


class GimbalPitchController:
    """Integrate vertical image error into an absolute gimbal angle."""

    def __init__(
        self,
        initial_angle: float,
        minimum_angle: float,
        maximum_angle: float,
        vertical_deadband: float,
        pitch_gain: float,
        maximum_pitch_rate: float,
        direction: float,
    ) -> None:
        self.minimum_angle = minimum_angle
        self.maximum_angle = maximum_angle
        self.vertical_deadband = vertical_deadband
        self.pitch_gain = pitch_gain
        self.maximum_pitch_rate = maximum_pitch_rate
        self.direction = direction
        self.angle = clamp(
            initial_angle,
            minimum_angle,
            maximum_angle,
        )

    def calculate(
        self,
        vertical_offset: float,
        elapsed_seconds: float,
    ) -> GimbalPitchCommand:
        rate = 0.0
        if abs(vertical_offset) > self.vertical_deadband:
            rate = clamp(
                self.direction * self.pitch_gain * vertical_offset,
                -self.maximum_pitch_rate,
                self.maximum_pitch_rate,
            )
            self.angle = clamp(
                self.angle + rate * max(0.0, elapsed_seconds),
                self.minimum_angle,
                self.maximum_angle,
            )
        return GimbalPitchCommand(self.angle, rate)
