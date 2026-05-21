from geometry_msgs.msg import Twist

from humanoid_navigation.g1_cmd_vel_adapter import (
    AdapterConfig,
    CommandConverter,
    CommandState,
)


def make_twist(x=0.0, y=0.0, yaw=0.0):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = yaw
    return msg


def test_converter_clips_and_inverts_unitree_axes():
    converter = CommandConverter(
        AdapterConfig(
            min_vel_x=-0.2,
            max_vel_x=0.5,
            max_vel_y=0.2,
            max_vel_theta=0.8,
            default_height=0.8,
            enable_lateral=True,
            invert_y=True,
            invert_yaw=True,
        )
    )

    command = converter.to_command(make_twist(x=1.0, y=0.1, yaw=2.0))

    assert command == [0.5, -0.1, -0.8, 0.8]


def test_converter_forces_zero_lateral_when_disabled():
    converter = CommandConverter(
        AdapterConfig(max_vel_y=0.2, enable_lateral=False, default_height=0.8)
    )

    command = converter.to_command(make_twist(y=0.2))

    assert command[1] == 0.0


def test_converter_zero_command_uses_default_height():
    converter = CommandConverter(AdapterConfig(default_height=0.8))

    assert converter.zero_command() == [0.0, 0.0, 0.0, 0.8]


def test_converter_formats_dds_payload_as_stringified_list():
    converter = CommandConverter(AdapterConfig(default_height=0.8))

    payload = converter.to_payload([0.1, 0.0, -0.2, 0.8])

    assert payload == "[0.1, 0.0, -0.2, 0.8]"


def test_command_state_returns_zero_after_timeout():
    converter = CommandConverter(AdapterConfig(default_height=0.8))
    state = CommandState(converter=converter, timeout_sec=0.25)
    state.update(make_twist(x=0.2), now_sec=10.0)

    command = state.command_at(now_sec=10.3)

    assert command == [0.0, 0.0, 0.0, 0.8]


def test_command_state_returns_latest_command_before_timeout():
    converter = CommandConverter(AdapterConfig(default_height=0.8))
    state = CommandState(converter=converter, timeout_sec=0.25)
    state.update(make_twist(x=0.2), now_sec=10.0)

    command = state.command_at(now_sec=10.1)

    assert command == [0.2, 0.0, 0.0, 0.8]
