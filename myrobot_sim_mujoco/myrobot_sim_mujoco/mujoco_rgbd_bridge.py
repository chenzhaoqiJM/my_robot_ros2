from __future__ import annotations

import math
from typing import Optional

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

from .mujoco_diff_bridge import MujocoDiffBridge


class MujocoRGBDBridge(MujocoDiffBridge):
    """MuJoCo differential-drive bridge publishing RGB-D images and TF."""

    def _declare_parameter_overrides(self) -> None:
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('optical_frame', 'camera_optical_frame')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fov', math.radians(60.0))
        self.declare_parameter('camera_near', 0.02)
        self.declare_parameter('camera_far', 8.0)
        self.declare_parameter('camera_noise_stddev', 0.007)
        self.declare_parameter('camera_update_rate', 20.0)
        self.declare_parameter('camera_offset_x', 0.12)
        self.declare_parameter('camera_offset_y', 0.0)
        self.declare_parameter('camera_offset_z', 0.02)

    def _initialize_specialized_bridge(self) -> None:
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.optical_frame = str(self.get_parameter('optical_frame').value)
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.camera_width = int(self.get_parameter('camera_width').value)
        self.camera_height = int(self.get_parameter('camera_height').value)
        self.camera_fov = float(self.get_parameter('camera_fov').value)
        self.camera_near = float(self.get_parameter('camera_near').value)
        self.camera_far = float(self.get_parameter('camera_far').value)
        self.camera_noise_stddev = float(self.get_parameter('camera_noise_stddev').value)
        self.camera_update_dt = 1.0 / float(self.get_parameter('camera_update_rate').value)
        self.camera_offset = [float(self.get_parameter(name).value) for name in (
            'camera_offset_x', 'camera_offset_y', 'camera_offset_z')]
        self.last_camera_pub_time = 0.0
        self.camera_pub = self.create_publisher(Image, self.camera_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.renderer = None

    def _step(self) -> None:
        super()._step()
        # Rendering is intentionally rate-limited independently of physics.
        if self.sim_time - self.last_camera_pub_time >= self.camera_update_dt:
            self._publish_camera(self.sim_time)
            self.last_camera_pub_time = self.sim_time

    def _publish_scan(self, stamp) -> None:
        return

    def _publish_camera(self, sim_time: float) -> None:
        stamp = self._sim_stamp(sim_time)
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, self.camera_height, self.camera_width)
            self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        depth = np.asarray(self.renderer.render(), dtype=np.float32)
        self.renderer.disable_depth_rendering()
        rgb = np.asarray(self.renderer.render(), dtype=np.uint8)
        self.renderer.enable_depth_rendering()
        if self.camera_noise_stddev > 0.0:
            depth += np.random.normal(0.0, self.camera_noise_stddev, depth.shape).astype(np.float32)
        depth = np.clip(depth, self.camera_near, self.camera_far)
        depth_msg = Image(header=self._header(stamp, self.optical_frame), height=self.camera_height,
                          width=self.camera_width, encoding='32FC1', is_bigendian=0,
                          step=self.camera_width * 4, data=depth.tobytes())
        image_msg = Image(header=self._header(stamp, self.optical_frame), height=self.camera_height,
                          width=self.camera_width, encoding='rgb8', is_bigendian=0,
                          step=self.camera_width * 3, data=rgb.tobytes())
        self.depth_pub.publish(depth_msg)
        self.camera_pub.publish(image_msg)
        self.camera_info_pub.publish(self._camera_info(stamp))
        self._publish_camera_tf(stamp)

    @staticmethod
    def _sim_stamp(sim_time: float):
        from builtin_interfaces.msg import Time as TimeMsg
        stamp = TimeMsg()
        stamp.sec = int(sim_time)
        stamp.nanosec = int((sim_time - stamp.sec) * 1e9)
        return stamp

    def _camera_info(self, stamp) -> CameraInfo:
        focal = self.camera_width / (2.0 * math.tan(self.camera_fov / 2.0))
        info = CameraInfo(header=self._header(stamp, self.optical_frame), width=self.camera_width,
                          height=self.camera_height, distortion_model='plumb_bob')
        info.k = [focal, 0.0, self.camera_width / 2.0, 0.0, focal,
                  self.camera_height / 2.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, self.camera_width / 2.0, 0.0, 0.0, focal,
                  self.camera_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_camera_tf(self, stamp) -> None:
        self._send_tf(stamp, 'base_footprint', self.base_link_frame, (0.0, 0.0, 0.059), (0.0, 0.0, 0.0, 1.0))
        self._send_tf(stamp, self.base_link_frame, self.camera_frame, tuple(self.camera_offset), (0.0, 0.0, 0.0, 1.0))
        self._send_tf(stamp, self.base_link_frame, self.imu_frame, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        # Matches rgbd_gazebo.xacro: rpy = (-pi/2, 0, -pi/2).
        q = self._quat_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        self._send_tf(stamp, self.camera_frame, self.optical_frame, (0.0, 0.0, 0.0), q)

    def _send_tf(self, stamp, parent, child, translation, rotation) -> None:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = rotation
        self.tf_broadcaster.sendTransform(msg)

    @staticmethod
    def _header(stamp, frame_id):
        from std_msgs.msg import Header
        return Header(stamp=stamp, frame_id=frame_id)

    @staticmethod
    def _quat_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoRGBDBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            if node.renderer is not None:
                node.renderer.close()
            node.destroy_node()
            if node.viewer is not None:
                node.viewer.close()
        rclpy.shutdown()