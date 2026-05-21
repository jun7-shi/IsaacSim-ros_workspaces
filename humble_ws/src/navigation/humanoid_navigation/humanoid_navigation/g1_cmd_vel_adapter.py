from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class AdapterConfig:
    min_vel_x: float = -0.2
    max_vel_x: float = 0.5
    max_vel_y: float = 0.0
    max_vel_theta: float = 0.8
    default_height: float = 0.8
    enable_lateral: bool = False
    invert_y: bool = True
    invert_yaw: bool = True


class CommandConverter:
    def __init__(self, config: AdapterConfig):
        self._config = config

    def to_command(self, msg: Twist) -> list[float]:
        x = self._clip(msg.linear.x, self._config.min_vel_x, self._config.max_vel_x)
        if self._config.enable_lateral:
            y = self._clip(msg.linear.y, -self._config.max_vel_y, self._config.max_vel_y)
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

        return [round(x, 4), round(y, 4), round(yaw, 4), self._config.default_height]

    def zero_command(self) -> list[float]:
        return [0.0, 0.0, 0.0, self._config.default_height]

    def to_payload(self, command: list[float]) -> str:
        return str([float(value) for value in command])

    @staticmethod
    def _clip(value: float, minimum: float, maximum: float) -> float:
        return min(max(value, minimum), maximum)


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
            self._dds = UnitreeDdsPublisher(self.get_parameter("dds_topic").value)

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
            self.declare_parameter("input_cmd_vel_topic", "/cmd_vel_smoothed")
            self.declare_parameter("dds_topic", "rt/run_command/cmd")
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

        def _on_cmd_vel(self, msg: Twist) -> None:
            self._state.update(msg, self._now_sec())

        def _publish_tick(self) -> None:
            command = self._state.command_at(self._now_sec())
            self._dds.publish(self._converter.to_payload(command))

        def _now_sec(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

    return G1CmdVelAdapter


def main(args=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node_cls = create_g1_cmd_vel_adapter_class(Node)
    node = node_cls()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
