import json
import socket
import sys
import types

from geometry_msgs.msg import Twist

from humanoid_navigation.g1_cmd_vel_adapter import (
    AdapterConfig,
    CommandConverter,
    CommandState,
    UdpCommandPublisher,
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
            policy_min_vel_x=-0.6,
            policy_max_vel_x=1.0,
            policy_max_vel_y=0.5,
            policy_max_vel_theta=1.57,
        )
    )

    command = converter.to_command(make_twist(x=1.0, y=0.1, yaw=2.0))

    assert command == [0.5, -0.1, -0.8, 0.8]


def test_converter_passes_nav2_velocity_without_rescaling():
    converter = CommandConverter(
        AdapterConfig(
            min_vel_x=-0.2,
            max_vel_x=0.5,
            default_height=0.8,
            policy_min_vel_x=-0.6,
            policy_max_vel_x=1.0,
        )
    )

    assert converter.to_command(make_twist(x=0.25)) == [0.25, 0.0, 0.0, 0.8]
    assert converter.to_command(make_twist(x=-0.1)) == [-0.1, 0.0, 0.0, 0.8]


def test_converter_uses_policy_range_as_safety_clamp_only():
    converter = CommandConverter(
        AdapterConfig(
            min_vel_x=-1.0,
            max_vel_x=1.0,
            max_vel_y=1.0,
            max_vel_theta=2.0,
            policy_min_vel_x=-0.3,
            policy_max_vel_x=0.4,
            policy_max_vel_y=0.2,
            policy_max_vel_theta=0.6,
            default_height=0.8,
            enable_lateral=True,
            invert_y=True,
            invert_yaw=True,
        )
    )

    assert converter.to_command(make_twist(x=1.0, y=1.0, yaw=2.0)) == [
        0.4,
        -0.2,
        -0.6,
        0.8,
    ]


def test_converter_forces_zero_lateral_when_disabled():
    converter = CommandConverter(
        AdapterConfig(max_vel_y=0.2, enable_lateral=False, default_height=0.8)
    )

    command = converter.to_command(make_twist(y=0.2))

    assert command[1] == 0.0


def test_converter_boosts_rotate_in_place_yaw_above_policy_deadband():
    converter = CommandConverter(
        AdapterConfig(
            max_vel_theta=1.0,
            policy_max_vel_theta=1.57,
            policy_min_abs_vel_theta=0.25,
            default_height=0.8,
            invert_yaw=False,
        )
    )

    assert converter.to_command(make_twist(yaw=0.102)) == [0.0, 0.0, 0.25, 0.8]
    assert converter.to_command(make_twist(yaw=-0.102)) == [
        0.0,
        0.0,
        -0.25,
        0.8,
    ]


def test_converter_boosts_yaw_while_following_path_forward():
    converter = CommandConverter(
        AdapterConfig(
            max_vel_theta=1.0,
            policy_max_vel_theta=1.57,
            policy_min_abs_vel_theta=0.25,
            default_height=0.8,
            invert_yaw=False,
        )
    )

    assert converter.to_command(make_twist(x=0.2, yaw=0.102)) == [
        0.2,
        0.0,
        0.25,
        0.8,
    ]


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


def test_udp_command_publisher_sends_json_payload_without_unitree_sdk():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(1.0)
    host, port = sock.getsockname()

    publisher = UdpCommandPublisher(host=host, port=port)
    publisher.publish("[0.1, 0.0, -0.2, 0.8]")

    data, _addr = sock.recvfrom(2048)
    sock.close()

    assert json.loads(data.decode("utf-8")) == {
        "payload": "[0.1, 0.0, -0.2, 0.8]"
    }


def test_main_ignores_external_shutdown_without_double_shutdown(monkeypatch):
    from humanoid_navigation.g1_cmd_vel_adapter import main

    class ExternalShutdownException(Exception):
        pass

    destroyed = []
    shutdown_calls = []

    class FakeNode:
        def __init__(self, _name):
            self._parameters = {}

        def declare_parameter(self, name, default_value):
            self._parameters[name] = default_value

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._parameters[name])

        def create_subscription(self, *_args):
            return None

        def create_timer(self, *_args):
            return None

        def destroy_node(self):
            destroyed.append(True)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.init = lambda args=None: None
    rclpy_module.spin = lambda _node: (_ for _ in ()).throw(
        ExternalShutdownException()
    )
    rclpy_module.ok = lambda: False
    rclpy_module.shutdown = lambda: shutdown_calls.append(True)

    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = FakeNode

    rclpy_executors_module = types.ModuleType("rclpy.executors")
    rclpy_executors_module.ExternalShutdownException = ExternalShutdownException

    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node_module)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy_executors_module)

    main()

    assert destroyed == [True]
    assert shutdown_calls == []
