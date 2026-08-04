from drone_control.tracking_controller import TrackingController


def make_controller():
    return TrackingController(
        target_area=0.12,
        area_deadband=0.02,
        horizontal_deadband=0.08,
        forward_gain=3.0,
        yaw_gain=0.8,
        maximum_forward_speed=1.5,
        maximum_reverse_speed=0.5,
        maximum_yaw_rate=0.6,
        forward_alignment_threshold=0.15,
    )


def test_centered_distant_runner_moves_forward():
    command = make_controller().calculate(0.0, 0.04)
    assert command.forward_speed > 0.0
    assert command.yaw_rate == 0.0


def test_runner_on_right_commands_rightward_yaw():
    command = make_controller().calculate(0.5, 0.12)
    assert command.forward_speed == 0.0
    assert command.yaw_rate < 0.0


def test_misaligned_distant_runner_turns_before_moving_forward():
    command = make_controller().calculate(0.5, 0.04)
    assert command.forward_speed == 0.0
    assert command.yaw_rate < 0.0


def test_commands_are_limited():
    command = make_controller().calculate(-10.0, -10.0)
    assert command.forward_speed == 0.0
    assert command.yaw_rate == 0.6
