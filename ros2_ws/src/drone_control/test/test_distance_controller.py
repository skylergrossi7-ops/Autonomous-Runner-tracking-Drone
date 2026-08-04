import math

import pytest

from drone_control.distance_controller import (
    DistanceTrackingController,
    MonocularDistanceEstimator,
)


def make_controller():
    return DistanceTrackingController(
        desired_distance=2.5,
        distance_deadband=0.25,
        distance_gain=0.5,
        horizontal_deadband=0.08,
        yaw_gain=0.3,
        maximum_forward_speed=0.6,
        maximum_reverse_speed=0.3,
        maximum_yaw_rate=0.15,
        forward_alignment_threshold=0.15,
    )


def test_reference_area_maps_to_reference_distance():
    estimator = MonocularDistanceEstimator(2.5, 0.022)
    assert estimator.estimate(0.022) == pytest.approx(2.5)


def test_smaller_box_is_farther_away():
    estimator = MonocularDistanceEstimator(2.5, 0.022)
    assert estimator.estimate(0.011) > 2.5


def test_invalid_box_area_does_not_create_motion_distance():
    estimator = MonocularDistanceEstimator(2.5, 0.022)
    assert math.isinf(estimator.estimate(0.0))


def test_far_runner_commands_forward_motion():
    assert make_controller().calculate(0.0, 3.5).forward_speed > 0.0


def test_close_runner_commands_reverse_motion():
    assert make_controller().calculate(0.0, 1.5).forward_speed < 0.0


def test_runner_inside_distance_deadband_holds_position():
    assert make_controller().calculate(0.0, 2.6).forward_speed == 0.0


def test_misaligned_runner_turns_before_changing_distance():
    command = make_controller().calculate(0.5, 4.0)
    assert command.forward_speed == 0.0
    assert command.yaw_rate < 0.0


def test_distance_speeds_are_bounded():
    controller = make_controller()
    assert controller.calculate(0.0, 100.0).forward_speed == 0.6
    assert controller.calculate(0.0, 0.1).forward_speed == -0.3
