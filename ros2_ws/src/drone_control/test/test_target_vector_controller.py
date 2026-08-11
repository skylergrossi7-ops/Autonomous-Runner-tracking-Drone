import math

from drone_control.distance_controller import TargetVectorController


def make_controller():
    return TargetVectorController(
        desired_distance=2.5,
        distance_deadband=0.25,
        distance_gain=0.5,
        heading_deadband=0.05,
        yaw_gain=0.8,
        maximum_forward_speed=0.6,
        maximum_reverse_speed=0.3,
        maximum_yaw_rate=0.4,
        forward_alignment_threshold=0.2,
    )


def test_far_centered_target_commands_forward():
    assert make_controller().calculate(4.0, 0.0).forward_speed > 0.0


def test_close_target_commands_reverse():
    assert make_controller().calculate(1.5, 0.0).forward_speed < 0.0


def test_left_target_commands_positive_yaw_and_turns_first():
    command = make_controller().calculate(3.0, 2.0)
    assert command.yaw_rate > 0.0
    assert command.forward_speed == 0.0


def test_invalid_target_stops():
    command = make_controller().calculate(math.nan, 0.0)
    assert command.forward_speed == 0.0
    assert command.yaw_rate == 0.0
