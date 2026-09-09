"""Adapt Gazebo differential-drive data for SCAN-Planner."""

from copy import deepcopy
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class ScanPlannerAdapter(Node):
    """Publish SCAN-Planner body/sensor poses and a floor-filtered cloud."""

    def __init__(self):
        super().__init__('scan_planner_adapter')

        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        cloud_topic = self.declare_parameter('cloud_topic', '/points').value
        self.body_z = float(self.declare_parameter('body_z', 0.25).value)
        self.sensor_x = float(self.declare_parameter('sensor_x', 0.07).value)
        self.sensor_y = float(self.declare_parameter('sensor_y', 0.0).value)
        self.sensor_z = float(self.declare_parameter('sensor_z', 0.144).value)
        self.min_sensor_z = float(
            self.declare_parameter('min_sensor_z', -0.10).value)
        self.output_frame = self.declare_parameter(
            'output_frame', 'world').value

        self.body_pub = self.create_publisher(
            Odometry, 'body_pose', qos_profile_sensor_data)
        self.sensor_pub = self.create_publisher(
            Odometry, 'sensor_pose', qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'cloud', qos_profile_sensor_data)

        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, cloud_topic, self.cloud_callback,
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Adapting {odom_topic} and {cloud_topic}; '
            f'body_z={self.body_z:.3f}, sensor offset='
            f'({self.sensor_x:.3f}, {self.sensor_y:.3f}, {self.sensor_z:.3f})')

    def odom_callback(self, message):
        body = deepcopy(message)
        body.header.frame_id = self.output_frame
        body.pose.pose.position.z = self.body_z
        self.body_pub.publish(body)

        sensor = deepcopy(message)
        sensor.header.frame_id = self.output_frame
        sensor.child_frame_id = 'lidar3d_link'
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        sensor.pose.pose.position.x += (
            cos_yaw * self.sensor_x - sin_yaw * self.sensor_y)
        sensor.pose.pose.position.y += (
            sin_yaw * self.sensor_x + cos_yaw * self.sensor_y)
        sensor.pose.pose.position.z = self.sensor_z
        self.sensor_pub.publish(sensor)

    def cloud_callback(self, message):
        points = point_cloud2.read_points(
            message, field_names=['x', 'y', 'z'], skip_nans=True)
        if points.size == 0:
            return
        keep = points['z'] >= self.min_sensor_z
        xyz = np.column_stack(
            (points['x'][keep], points['y'][keep], points['z'][keep]))
        filtered = point_cloud2.create_cloud_xyz32(message.header, xyz)
        self.cloud_pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlannerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
