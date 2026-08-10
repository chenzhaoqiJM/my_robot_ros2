from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import TransformBroadcaster

try:
    import mujoco
except ImportError:  # pragma: no cover - handled at runtime
    mujoco = None

try:
    import mujoco.viewer
except ImportError:  # pragma: no cover - handled at runtime
    mujoco_viewer = None
else:
    mujoco_viewer = mujoco.viewer


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class MujocoDiffBridge(Node):
    def __init__(self) -> None:
        super().__init__('mujoco_diff_bridge')

        self.declare_parameter('model_path', '')
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('publish_clock', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_separation', 0.225)
        self.declare_parameter('laser_min_angle', -math.pi)
        self.declare_parameter('laser_max_angle', math.pi)
        self.declare_parameter('laser_num_beams', 360)
        self.declare_parameter('laser_min_range', 0.08)
        self.declare_parameter('laser_max_range', 12.0)
        self.declare_parameter('laser_frame_id', 'laser_link')
        self.declare_parameter('laser_site', 'laser_site')
        self.declare_parameter('enable_viewer', False)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.5)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not self.model_path:
            raise RuntimeError('parameter model_path is required')
        if mujoco is None:
            raise RuntimeError('mujoco python package is not installed in the current environment')
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_clock = bool(self.get_parameter('publish_clock').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.base_link_frame = str(self.get_parameter('base_link_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.laser_min_angle = float(self.get_parameter('laser_min_angle').value)
        self.laser_max_angle = float(self.get_parameter('laser_max_angle').value)
        self.laser_num_beams = int(self.get_parameter('laser_num_beams').value)
        self.laser_min_range = float(self.get_parameter('laser_min_range').value)
        self.laser_max_range = float(self.get_parameter('laser_max_range').value)
        self.laser_site = str(self.get_parameter('laser_site').value)
        self.enable_viewer = bool(self.get_parameter('enable_viewer').value)
        self.max_linear_velocity = float(self.get_parameter('max_linear_velocity').value)
        self.max_angular_velocity = float(self.get_parameter('max_angular_velocity').value)

        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        self.laser_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.laser_site)
        if self.laser_site_id < 0:
            raise RuntimeError(f'MuJoCo site not found: {self.laser_site}')
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        self.ray_geom_group = np.ones(6, dtype=np.uint8)
        self.viewer = None
        if self.enable_viewer:
            if mujoco_viewer is None:
                raise RuntimeError('mujoco.viewer is not available in the current environment')
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)

        self.pose = Pose2D()
        self.last_left = 0.0
        self.last_right = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.sim_time = 0.0
        self.last_odom_pub_time = 0.0
        self.last_scan_pub_time = 0.0
        self.last_imu_pub_time = 0.0

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 1.0 / self.update_rate
        self.publish_dt = 1.0 / self.publish_rate
        self._running = True
        self.step_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.step_thread.start()
        self.get_logger().info(f'Loaded MuJoCo model: {self.model_path}')

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_v = self._clamp(float(msg.linear.x), -self.max_linear_velocity, self.max_linear_velocity)
        self.cmd_w = self._clamp(float(msg.angular.z), -self.max_angular_velocity, self.max_angular_velocity)

    def _run_loop(self) -> None:
        next_time = time.monotonic()
        while self._running and rclpy.ok():
            self._step()
            next_time += self.dt
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)

    def _step(self) -> None:
        left = (self.cmd_v - self.cmd_w * self.wheel_separation / 2.0) / self.wheel_radius
        right = (self.cmd_v + self.cmd_w * self.wheel_separation / 2.0) / self.wheel_radius

        self.last_left = left
        self.last_right = right
        self._integrate_planar_motion()
        mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
        self.sim_time += self.dt

        now = Time(seconds=self.sim_time)
        if self.publish_clock:
            clock_msg = Clock()
            clock_msg.clock.sec = int(self.sim_time)
            clock_msg.clock.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
            self.clock_pub.publish(clock_msg)

        if self.sim_time - self.last_odom_pub_time >= self.publish_dt:
            self._publish_odom(now)
            self.last_odom_pub_time = self.sim_time

        if self.sim_time - self.last_scan_pub_time >= self.publish_dt:
            self._publish_scan(now)
            self.last_scan_pub_time = self.sim_time

        if self.sim_time - self.last_imu_pub_time >= self.publish_dt:
            self._publish_imu(now)
            self.last_imu_pub_time = self.sim_time

    def _apply_wheel_velocity(self, left: float, right: float) -> None:
        self.data.ctrl[:] = 0.0
        if self.model.nu >= 2:
            self.data.ctrl[0] = left
            self.data.ctrl[1] = right
        elif self.model.nu == 1:
            self.data.ctrl[0] = 0.5 * (left + right)

    def _integrate_planar_motion(self) -> None:
        self.pose.x += self.cmd_v * math.cos(self.pose.yaw) * self.dt
        self.pose.y += self.cmd_v * math.sin(self.pose.yaw) * self.dt
        self.pose.yaw = self._wrap(self.pose.yaw + self.cmd_w * self.dt)

        if self.model.nq >= 7:
            self.data.qpos[0] = self.pose.x
            self.data.qpos[1] = self.pose.y
            self.data.qpos[2] = 0.08
            self.data.qpos[3:7] = [math.cos(self.pose.yaw * 0.5), 0.0, 0.0, math.sin(self.pose.yaw * 0.5)]
        if self.model.nq >= 9:
            self.data.qpos[7] += self.last_left * self.dt
            self.data.qpos[8] += self.last_right * self.dt
        if self.model.nv >= 6:
            self.data.qvel[0] = self.cmd_v * math.cos(self.pose.yaw)
            self.data.qvel[1] = self.cmd_v * math.sin(self.pose.yaw)
            self.data.qvel[2] = 0.0
            self.data.qvel[3] = 0.0
            self.data.qvel[4] = 0.0
            self.data.qvel[5] = self.cmd_w
        if self.model.nv >= 8:
            self.data.qvel[6] = self.last_left
            self.data.qvel[7] = self.last_right

    def _publish_odom(self, stamp) -> None:
        x, y, yaw = self._pose_from_data()
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        q = self._quat_from_yaw(yaw)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.cmd_v
        odom.twist.twist.angular.z = self.cmd_w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_scan(self, stamp) -> None:
        scan = LaserScan()
        scan.header.stamp = stamp.to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = self.laser_min_angle
        scan.angle_max = self.laser_max_angle
        scan.angle_increment = (self.laser_max_angle - self.laser_min_angle) / max(self.laser_num_beams - 1, 1)
        scan.time_increment = 0.0
        scan.scan_time = self.publish_dt
        scan.range_min = self.laser_min_range
        scan.range_max = self.laser_max_range
        scan.ranges = self._laser_ranges()
        scan.intensities = [0.0] * self.laser_num_beams
        self.scan_pub.publish(scan)

    def _laser_ranges(self) -> list[float]:
        origin = np.array(self.data.site_xpos[self.laser_site_id], dtype=np.float64)
        _, _, yaw = self._pose_from_data()
        geomid = np.zeros(1, dtype=np.int32)
        ranges = []
        for i in range(self.laser_num_beams):
            angle = self.laser_min_angle + i * (self.laser_max_angle - self.laser_min_angle) / max(self.laser_num_beams - 1, 1)
            ray_yaw = yaw + angle
            direction = np.array([math.cos(ray_yaw), math.sin(ray_yaw), 0.0], dtype=np.float64)
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
            if distance < self.laser_min_range or distance > self.laser_max_range:
                ranges.append(self.laser_max_range)
            else:
                ranges.append(float(distance))
        return ranges

    def _publish_imu(self, stamp) -> None:
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = self.imu_frame
        imu.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        imu.angular_velocity = Vector3(x=0.0, y=0.0, z=self.cmd_w)
        imu.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = 1e-3
        imu.linear_acceleration_covariance[0] = 1e-2
        self.imu_pub.publish(imu)

    def _pose_from_data(self) -> tuple[float, float, float]:
        return self.pose.x, self.pose.y, self.pose.yaw

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _quat_from_yaw(yaw: float) -> Quaternion:
        half = yaw * 0.5
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    @staticmethod
    def _yaw_from_quat(qw: float, qx: float, qy: float, qz: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self) -> bool:
        self._running = False
        if hasattr(self, 'step_thread') and self.step_thread.is_alive():
            self.step_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[MujocoDiffBridge] = None
    try:
        node = MujocoDiffBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
            if node.viewer is not None:
                node.viewer.close()
        rclpy.shutdown()
