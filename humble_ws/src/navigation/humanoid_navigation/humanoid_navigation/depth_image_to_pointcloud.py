import math
import struct
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class DepthProjectionConfig:
    stride: int = 8
    min_depth_m: float = 0.1
    max_depth_m: float = 4.0


def camera_info_intrinsics(
    width: int,
    height: int,
    k: Sequence[float],
) -> CameraIntrinsics:
    if len(k) != 9:
        raise ValueError("CameraInfo.k must contain exactly 9 values")
    return CameraIntrinsics(
        width=int(width),
        height=int(height),
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
    )


def project_depth_to_points(
    depth_m: Sequence[float],
    intrinsics: CameraIntrinsics,
    config: DepthProjectionConfig,
) -> list[tuple[float, float, float]]:
    if config.stride < 1:
        raise ValueError("stride must be >= 1")
    if len(depth_m) < intrinsics.width * intrinsics.height:
        raise ValueError("depth image is smaller than width * height")
    if intrinsics.fx == 0.0 or intrinsics.fy == 0.0:
        raise ValueError("camera focal lengths must be non-zero")

    points: list[tuple[float, float, float]] = []
    for v in range(0, intrinsics.height, config.stride):
        row_offset = v * intrinsics.width
        for u in range(0, intrinsics.width, config.stride):
            z = float(depth_m[row_offset + u])
            if (
                not math.isfinite(z)
                or z < config.min_depth_m
                or z > config.max_depth_m
            ):
                continue
            x = (u - intrinsics.cx) * z / intrinsics.fx
            y = (v - intrinsics.cy) * z / intrinsics.fy
            points.append((x, y, z))
    return points


def decode_depth_image_meters(
    data: bytes,
    width: int,
    height: int,
    encoding: str,
    is_bigendian: int,
    step: int,
    depth_scale: float = 0.001,
) -> list[float]:
    if encoding not in ("32FC1", "16UC1"):
        raise ValueError(f"unsupported depth image encoding: {encoding}")

    byte_order = ">" if is_bigendian else "<"
    if encoding == "32FC1":
        fmt = f"{byte_order}f"
        item_size = 4
        scale = 1.0
    else:
        fmt = f"{byte_order}H"
        item_size = 2
        scale = depth_scale

    depth: list[float] = []
    for v in range(height):
        row_offset = v * step
        for u in range(width):
            pixel_offset = row_offset + u * item_size
            depth.append(float(struct.unpack_from(fmt, data, pixel_offset)[0]) * scale)
    return depth


def pack_xyz_points(points: Iterable[tuple[float, float, float]]) -> bytes:
    payload = bytearray()
    for point in points:
        payload.extend(struct.pack("<fff", float(point[0]), float(point[1]), float(point[2])))
    return bytes(payload)


class DepthImageToPointCloudNode:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2

        class _Node(Node):
            pass

        self.node = _Node("depth_image_to_pointcloud")
        self._camera_info: CameraInfo | None = None
        self._pointcloud_type = PointCloud2
        self._declare_parameters()

        depth_topic = self.node.get_parameter("depth_topic").value
        camera_info_topic = self.node.get_parameter("camera_info_topic").value
        pointcloud_topic = self.node.get_parameter("pointcloud_topic").value

        self.publisher = self.node.create_publisher(PointCloud2, pointcloud_topic, 10)
        self.node.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Image,
            depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self.node.get_logger().info(
            f"depth image to PointCloud2 converter: {depth_topic} -> {pointcloud_topic}"
        )
        self._rclpy = rclpy

    def spin(self):
        self._rclpy.spin(self.node)

    def destroy_node(self):
        self.node.destroy_node()

    def _declare_parameters(self):
        self.node.declare_parameter("depth_topic", "/g1/head_rgbd/depth/image_raw")
        self.node.declare_parameter("camera_info_topic", "/g1/head_rgbd/camera_info")
        self.node.declare_parameter("pointcloud_topic", "/g1/head_rgbd/points")
        self.node.declare_parameter(
            "pointcloud_frame",
            "g1_front_rgbd_optical_frame",
        )
        self.node.declare_parameter("stride", 8)
        self.node.declare_parameter("min_depth_m", 0.1)
        self.node.declare_parameter("max_depth_m", 4.0)
        self.node.declare_parameter("depth_scale", 0.001)

    def _on_camera_info(self, msg):
        self._camera_info = msg

    def _on_depth_image(self, msg):
        if self._camera_info is None:
            self.node.get_logger().warn(
                "waiting for CameraInfo before publishing depth pointcloud",
                throttle_duration_sec=5.0,
            )
            return

        try:
            intrinsics = camera_info_intrinsics(
                self._camera_info.width,
                self._camera_info.height,
                self._camera_info.k,
            )
            depth_m = decode_depth_image_meters(
                bytes(msg.data),
                msg.width,
                msg.height,
                msg.encoding,
                msg.is_bigendian,
                msg.step,
                float(self.node.get_parameter("depth_scale").value),
            )
            points = project_depth_to_points(
                depth_m,
                intrinsics,
                DepthProjectionConfig(
                    stride=int(self.node.get_parameter("stride").value),
                    min_depth_m=float(self.node.get_parameter("min_depth_m").value),
                    max_depth_m=float(self.node.get_parameter("max_depth_m").value),
                ),
            )
        except Exception as exc:
            self.node.get_logger().warn(f"failed to convert depth image: {exc}")
            return

        self.publisher.publish(self._make_pointcloud(msg.header, points))

    def _make_pointcloud(self, header, points):
        from sensor_msgs.msg import PointCloud2, PointField

        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = str(self.node.get_parameter("pointcloud_frame").value)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = pack_xyz_points(points)
        cloud.is_dense = False
        return cloud


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = DepthImageToPointCloudNode()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
