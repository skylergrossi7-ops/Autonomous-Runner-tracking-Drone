from drone_control.gimbal_pitch_controller import GimbalPitchController


def make_controller(direction=1.0):
    return GimbalPitchController(
        initial_angle=0.0,
        minimum_angle=-1.2,
        maximum_angle=0.6,
        vertical_deadband=0.12,
        pitch_gain=0.5,
        maximum_pitch_rate=0.25,
        direction=direction,
    )


def test_target_in_deadband_holds_pitch():
    command = make_controller().calculate(0.1, 1.0)
    assert command.angle == 0.0
    assert command.rate == 0.0


def test_target_below_center_changes_pitch_at_bounded_rate():
    command = make_controller().calculate(1.0, 1.0)
    assert command.rate == 0.25
    assert command.angle == 0.25


def test_pitch_angle_is_limited():
    controller = make_controller()
    for _ in range(20):
        command = controller.calculate(1.0, 1.0)
    assert command.angle == 0.6


def test_direction_can_be_reversed_for_simulator_joint_convention():
    command = make_controller(direction=-1.0).calculate(1.0, 1.0)
    assert command.rate == -0.25
    assert command.angle == -0.25
