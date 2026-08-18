from __future__ import annotations

import math
import struct
from typing import Optional

import mujoco
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .mujoco_diff_bridge import MujocoDiffBridge


class Mujoco3DLidarBridge(MujocoDiffBridge):
    """MuJoCo differential-drive bridge publishing a synthetic 3D point cloud."""

    def __init__(self) -> None:
        super().__init__()

    def _initialize_specialized_bridge(self) -> None:
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.points_topic = str(self.get_parameter('points_topic').value)
        self.horizontal_beams = int(self.get_parameter('horizontal_beams').value)
        self.vertical_beams = int(self.get_parameter('vertical_beams').value)
        self.horizontal_min_angle = float(self.get_parameter('horizontal_min_angle').value)
        self.horizontal_max_angle = float(self.get_parameter('horizontal_max_angle').value)
        self.vertical_min_angle = float(self.get_parameter('vertical_min_angle').value)
        self.vertical_max_angle = float(self.get_parameter('vertical_max_angle').value)
        self.points_pub = self.create_publisher(PointCloud2, self.points_topic, 10)

    def _declare_parameter_overrides(self) -> None:
        self.declare_parameter('lidar_frame', 'lidar3d_link')
        self.declare_parameter('points_topic', '/points')
        self.declare_parameter('horizontal_beams', 720)
        self.declare_parameter('vertical_beams', 16)
        self.declare_parameter('horizontal_min_angle', -math.pi)
        self.declare_parameter('horizontal_max_angle', math.pi)
        self.declare_parameter('vertical_min_angle', -math.pi / 12.0)
        self.declare_parameter('vertical_max_angle', math.pi / 12.0)

    def _publish_scan(self, stamp) -> None:
        points = self._point_cloud_points()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = PointCloud2(
            header=Header(stamp=stamp.to_msg(), frame_id=self.lidar_frame),
            height=1,
            width=len(points),
            fields=fields,
            is_bigendian=False,
            point_step=16,
            row_step=16 * len(points),
            is_dense=True,
        )
        msg.data = b''.join(struct.pack('<ffff', x, y, z, 1.0) for x, y, z in points)
        self.points_pub.publish(msg)

    def _point_cloud_points(self) -> list[tuple[float, float, float]]:
        origin = np.array(self.data.site_xpos[self.laser_site_id], dtype=np.float64)
        yaw = self.pose.yaw
        geomid = np.zeros(1, dtype=np.int32)
        points = []
        for vertical_index in range(self.vertical_beams):
            vertical = self.vertical_min_angle + vertical_index * (
                self.vertical_max_angle - self.vertical_min_angle
            ) / max(self.vertical_beams - 1, 1)
            cos_vertical = math.cos(vertical)
            sin_vertical = math.sin(vertical)
            for horizontal_index in range(self.horizontal_beams):
                horizontal = self.horizontal_min_angle + horizontal_index * (
                    self.horizontal_max_angle - self.horizontal_min_angle
                ) / max(self.horizontal_beams - 1, 1)
                ray_yaw = yaw + horizontal
                direction = np.array(
                    [
                        cos_vertical * math.cos(ray_yaw),
                        cos_vertical * math.sin(ray_yaw),
                        sin_vertical,
                    ],
                    dtype=np.float64,
                )
                try:
                    distance = mujoco.mj_ray(
                        self.model,
                        self.data,
                        origin,
                        direction,
                        self.ray_geom_group,
                        1,
                        self.base_body_id,
                        geomid,
                    )
                except Exception:
                    distance = -1.0
                if self.laser_min_range <= distance <= self.laser_max_range:
                    points.append(
                        (
                            float(distance * cos_vertical * math.cos(horizontal)),
                            float(distance * cos_vertical * math.sin(horizontal)),
                            float(distance * sin_vertical),
                        )
                    )
        return points


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Mujoco3DLidarBridge] = None
    try:
        node = Mujoco3DLidarBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
            if node.viewer is not None:
                node.viewer.close()
        rclpy.shutdown()