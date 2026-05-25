from __future__ import annotations

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from humanoid_navigation.g1_cmd_vel_adapter import (
    AdapterConfig,
    CommandConverter,
    CommandState,
)


def policy_command_from_twist(msg: Twist, config: AdapterConfig) -> list[float]:
    return CommandConverter(config).to_command(msg)


def create_policy_command_message(command: list[float]) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [float(value) for value in command]
    return msg


def create_velocity_marker_array(
    *,
    nav_twist: Twist,
    policy_command: list[float],
    frame_id: str,
    scale: float = 1.0,
    stamp=None,
) -> MarkerArray:
    return MarkerArray(
        markers=[
            _arrow_marker(
                marker_id=0,
                namespace="nav2_cmd_vel",
                frame_id=frame_id,
                stamp=stamp,
                x=nav_twist.linear.x,
                y=nav_twist.linear.y,
                scale=scale,
                color=ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.9),
            ),
            _arrow_marker(
                marker_id=1,
                namespace="g1_policy_cmd",
                frame_id=frame_id,
                stamp=stamp,
                x=policy_command[0],
                y=policy_command[1],
                scale=scale,
                color=ColorRGBA(r=1.0, g=0.75, b=0.05, a=0.9),
            ),
            _text_marker(
                frame_id=frame_id,
                stamp=stamp,
                nav_twist=nav_twist,
                policy_command=policy_command,
            ),
        ]
    )


def _arrow_marker(
    *,
    marker_id: int,
    namespace: str,
    frame_id: str,
    stamp,
    x: float,
    y: float,
    scale: float,
    color: ColorRGBA,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.035
    marker.scale.y = 0.08
    marker.scale.z = 0.12
    marker.color = color
    marker.points = [
        Point(x=0.0, y=0.0, z=0.08),
        Point(x=float(x) * scale, y=float(y) * scale, z=0.08),
    ]
    return marker


def _text_marker(
    *,
    frame_id: str,
    stamp,
    nav_twist: Twist,
    policy_command: list[float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = "cmd_vel_debug_text"
    marker.id = 2
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position.z = 0.38
    marker.scale.z = 0.16
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
    marker.text = (
        f"nav2 vx={nav_twist.linear.x:.3f} vy={nav_twist.linear.y:.3f} "
        f"wz={nav_twist.angular.z:.3f}\n"
        f"policy vx={policy_command[0]:.3f} vy={policy_command[1]:.3f} "
        f"wz={policy_command[2]:.3f} h={policy_command[3]:.2f}"
    )
    return marker


def create_g1_cmd_vel_debug_visualizer_class(node_base_cls):
    class G1CmdVelDebugVisualizer(node_base_cls):
        def __init__(self):
            super().__init__("g1_cmd_vel_debug_visualizer")
            self._declare_parameters()

            config = AdapterConfig(
                min_vel_x=self.get_parameter("min_vel_x").value,
                max_vel_x=self.get_parameter("max_vel_x").value,
                max_vel_y=self.get_parameter("max_vel_y").value,
                max_vel_theta=self.get_parameter("max_vel_theta").value,
                policy_min_vel_x=self.get_parameter("policy_min_vel_x").value,
                policy_max_vel_x=self.get_parameter("policy_max_vel_x").value,
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
            self._last_msg: Twist | None = None
            self._last_msg_time: float | None = None

            self._policy_pub = self.create_publisher(
                Float32MultiArray,
                self.get_parameter("policy_command_debug_topic").value,
                10,
            )
            self._marker_pub = self.create_publisher(
                MarkerArray,
                self.get_parameter("debug_marker_topic").value,
                10,
            )
            self.create_subscription(
                Twist,
                self.get_parameter("input_cmd_vel_topic").value,
                self._on_cmd_vel,
                10,
            )
            self.create_timer(
                1.0 / self.get_parameter("debug_publish_rate_hz").value,
                self._publish_tick,
            )

        def _declare_parameters(self) -> None:
            self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
            self.declare_parameter("debug_marker_topic", "/g1/cmd_vel_debug_markers")
            self.declare_parameter("policy_command_debug_topic", "/g1/policy_cmd_debug")
            self.declare_parameter("debug_frame_id", "base_link")
            self.declare_parameter("debug_publish_rate_hz", 10.0)
            self.declare_parameter("marker_vector_scale", 1.0)
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
            self.declare_parameter("policy_max_vel_y", 0.5)
            self.declare_parameter("policy_max_vel_theta", 1.57)
            self.declare_parameter("policy_min_abs_vel_theta", 0.0)

        def _on_cmd_vel(self, msg: Twist) -> None:
            now_sec = self._now_sec()
            self._last_msg = msg
            self._last_msg_time = now_sec
            self._state.update(msg, now_sec)

        def _publish_tick(self) -> None:
            now_sec = self._now_sec()
            command = self._state.command_at(now_sec)
            self._policy_pub.publish(create_policy_command_message(command))
            self._marker_pub.publish(
                create_velocity_marker_array(
                    nav_twist=self._nav_twist_at(now_sec),
                    policy_command=command,
                    frame_id=self.get_parameter("debug_frame_id").value,
                    scale=self.get_parameter("marker_vector_scale").value,
                    stamp=self.get_clock().now().to_msg(),
                )
            )

        def _nav_twist_at(self, now_sec: float) -> Twist:
            if self._last_msg is None or self._last_msg_time is None:
                return Twist()
            timeout_sec = self.get_parameter("cmd_timeout_sec").value
            if now_sec - self._last_msg_time > timeout_sec:
                return Twist()
            return self._last_msg

        def _now_sec(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

    return G1CmdVelDebugVisualizer


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init(args=args)
    node_cls = create_g1_cmd_vel_debug_visualizer_class(Node)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
