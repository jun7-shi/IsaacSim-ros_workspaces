from __future__ import annotations

import json
import socket
from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class AdapterConfig:
    min_vel_x: float = -0.2
    max_vel_x: float = 0.5
    max_vel_y: float = 0.0
    max_vel_theta: float = 0.8
    policy_min_vel_x: float = -0.6
    policy_max_vel_x: float = 1.0
    policy_min_abs_vel_x: float = 0.0
    policy_max_vel_y: float = 0.5
    policy_max_vel_theta: float = 1.57
    policy_min_abs_vel_theta: float = 0.0
    default_height: float = 0.8
    enable_lateral: bool = False
    invert_y: bool = True
    invert_yaw: bool = True


class CommandConverter:
    def __init__(self, config: AdapterConfig):
        self._config = config

    def to_command(self, msg: Twist) -> list[float]:
        x = self._clip(
            self._clip(msg.linear.x, self._config.min_vel_x, self._config.max_vel_x),
            self._config.policy_min_vel_x,
            self._config.policy_max_vel_x,
        )
        x = self._boost_above_deadband(
            x,
            self._config.policy_min_abs_vel_x,
            self._config.policy_max_vel_x,
        )
        if self._config.enable_lateral:
            y = self._clip(
                msg.linear.y,
                -self._config.max_vel_y,
                self._config.max_vel_y,
            )
        else:
            y = 0.0
        yaw = self._clip(
            msg.angular.z,
            -self._config.max_vel_theta,
            self._config.max_vel_theta,
        )

        if self._config.invert_y:
            y = -y
        if self._config.invert_yaw:
            yaw = -yaw

        y = self._clip(
            y,
            -self._config.policy_max_vel_y,
            self._config.policy_max_vel_y,
        )
        yaw = self._clip(
            yaw,
            -self._config.policy_max_vel_theta,
            self._config.policy_max_vel_theta,
        )
        yaw = self._boost_above_deadband(
            yaw,
            self._config.policy_min_abs_vel_theta,
            self._config.policy_max_vel_theta,
        )

        return [
            self._round_command_value(x),
            self._round_command_value(y),
            self._round_command_value(yaw),
            self._config.default_height,
        ]

    def zero_command(self) -> list[float]:
        return [0.0, 0.0, 0.0, self._config.default_height]

    def to_payload(self, command: list[float]) -> str:
        return str([float(value) for value in command])

    @staticmethod
    def _clip(value: float, minimum: float, maximum: float) -> float:
        return min(max(value, minimum), maximum)

    @staticmethod
    def _round_command_value(value: float) -> float:
        rounded = round(value, 4)
        if abs(rounded) < 1e-9:
            return 0.0
        return rounded

    @staticmethod
    def _boost_above_deadband(
        value: float,
        min_abs_value: float,
        max_abs_value: float,
    ) -> float:
        minimum = min(abs(min_abs_value), abs(max_abs_value))
        if minimum == 0.0 or abs(value) < 1e-9 or abs(value) >= minimum:
            return value
        return minimum if value > 0.0 else -minimum


class CommandState:
    def __init__(self, converter: CommandConverter, timeout_sec: float):
        self._converter = converter
        self._timeout_sec = timeout_sec
        self._last_msg: Twist | None = None
        self._last_msg_time: float | None = None

    def update(self, msg: Twist, now_sec: float) -> None:
        self._last_msg = msg
        self._last_msg_time = now_sec

    def command_at(self, now_sec: float) -> list[float]:
        if self._last_msg is None or self._last_msg_time is None:
            return self._converter.zero_command()
        if now_sec - self._last_msg_time > self._timeout_sec:
            return self._converter.zero_command()
        return self._converter.to_command(self._last_msg)


class UnitreeDdsPublisher:
    def __init__(self, topic: str):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        ChannelFactoryInitialize(1)
        self._msg_type = String_
        self._publisher = ChannelPublisher(topic, String_)
        self._publisher.Init()

    def publish(self, payload: str) -> None:
        self._publisher.Write(self._msg_type(data=payload))


class UdpCommandPublisher:
    def __init__(self, host: str, port: int):
        self._address = (host, int(port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, payload: str) -> None:
        packet = json.dumps({"payload": payload}).encode("utf-8")
        self._socket.sendto(packet, self._address)


def create_command_publisher(
    transport: str,
    *,
    dds_topic: str,
    udp_host: str,
    udp_port: int,
):
    normalized_transport = transport.lower()
    if normalized_transport == "udp":
        return UdpCommandPublisher(host=udp_host, port=udp_port)
    if normalized_transport == "dds":
        return UnitreeDdsPublisher(topic=dds_topic)
    raise ValueError(f"unsupported G1 command transport: {transport}")


def create_g1_cmd_vel_adapter_class(node_base_cls):
    class G1CmdVelAdapter(node_base_cls):
        def __init__(self):
            super().__init__("g1_cmd_vel_adapter")
            self._declare_parameters()

            config = AdapterConfig(
                min_vel_x=self.get_parameter("min_vel_x").value,
                max_vel_x=self.get_parameter("max_vel_x").value,
                max_vel_y=self.get_parameter("max_vel_y").value,
                max_vel_theta=self.get_parameter("max_vel_theta").value,
                policy_min_vel_x=self.get_parameter("policy_min_vel_x").value,
                policy_max_vel_x=self.get_parameter("policy_max_vel_x").value,
                policy_min_abs_vel_x=self.get_parameter("policy_min_abs_vel_x").value,
                policy_max_vel_y=self.get_parameter("policy_max_vel_y").value,
                policy_max_vel_theta=self.get_parameter("policy_max_vel_theta").value,
                policy_min_abs_vel_theta=self.get_parameter(
                    "policy_min_abs_vel_theta"
                ).value,
                default_height=self.get_parameter("default_height").value,
                enable_lateral=self.get_parameter("enable_lateral").value,
                invert_y=self.get_parameter("invert_y").value,
                invert_yaw=self.get_parameter("invert_yaw").value,
            )
            self._converter = CommandConverter(config)
            self._state = CommandState(
                converter=self._converter,
                timeout_sec=self.get_parameter("cmd_timeout_sec").value,
            )
            self._publisher = create_command_publisher(
                self.get_parameter("transport").value,
                dds_topic=self.get_parameter("dds_topic").value,
                udp_host=self.get_parameter("udp_host").value,
                udp_port=self.get_parameter("udp_port").value,
            )

            self.create_subscription(
                Twist,
                self.get_parameter("input_cmd_vel_topic").value,
                self._on_cmd_vel,
                10,
            )
            self.create_timer(
                1.0 / self.get_parameter("publish_rate_hz").value,
                self._publish_tick,
            )

        def _declare_parameters(self) -> None:
            self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("transport", "udp")
            self.declare_parameter("dds_topic", "rt/run_command/cmd")
            self.declare_parameter("udp_host", "127.0.0.1")
            self.declare_parameter("udp_port", 18080)
            self.declare_parameter("publish_rate_hz", 50.0)
            self.declare_parameter("cmd_timeout_sec", 0.25)
            self.declare_parameter("default_height", 0.8)
            self.declare_parameter("enable_lateral", False)
            self.declare_parameter("invert_y", True)
            self.declare_parameter("invert_yaw", True)
            self.declare_parameter("min_vel_x", -0.2)
            self.declare_parameter("max_vel_x", 0.5)
            self.declare_parameter("max_vel_y", 0.0)
            self.declare_parameter("max_vel_theta", 0.8)
            self.declare_parameter("policy_min_vel_x", -0.6)
            self.declare_parameter("policy_max_vel_x", 1.0)
            self.declare_parameter("policy_min_abs_vel_x", 0.0)
            self.declare_parameter("policy_max_vel_y", 0.5)
            self.declare_parameter("policy_max_vel_theta", 1.57)
            self.declare_parameter("policy_min_abs_vel_theta", 0.0)

        def _on_cmd_vel(self, msg: Twist) -> None:
            self._state.update(msg, self._now_sec())

        def _publish_tick(self) -> None:
            command = self._state.command_at(self._now_sec())
            self._publisher.publish(self._converter.to_payload(command))

        def _now_sec(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

    return G1CmdVelAdapter


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init(args=args)
    node_cls = create_g1_cmd_vel_adapter_class(Node)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
