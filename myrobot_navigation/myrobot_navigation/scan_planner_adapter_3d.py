"""Adapt full 3D FAST-LIO state and lidar data for SCAN-Planner."""

from copy import deepcopy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def normalized_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def rotate_vector(q, vector):
    """Rotate vector by quaternion q without adding a tf dependency."""
    qx, qy, qz, qw = q
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class ScanPlannerAdapter3D(Node):
    """Publish full body/lidar poses and an unmodified 3D lidar cloud."""

    def __init__(self):
        super().__init__('scan_planner_adapter_3d')
        odom_topic = self.declare_parameter('odom_topic', '/odom_3d').value
        cloud_topic = self.declare_parameter('cloud_topic', '/points').value
        self.output_frame = self.declare_parameter(
            'output_frame', 'world').value
        self.sensor_frame = self.declare_parameter(
            'sensor_frame', 'lidar3d_link').value
        self.sensor_offset = (
            float(self.declare_parameter('sensor_x', 0.07).value),
            float(self.declare_parameter('sensor_y', 0.0).value),
            float(self.declare_parameter('sensor_z', 0.085).value),
        )

        self.body_publisher = self.create_publisher(
            Odometry, 'body_pose', qos_profile_sensor_data)
        self.sensor_publisher = self.create_publisher(
            Odometry, 'sensor_pose', qos_profile_sensor_data)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, 'cloud', qos_profile_sensor_data)
        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'3D adapter: odom={odom_topic}, cloud={cloud_topic}, '
            f'lidar offset={self.sensor_offset}')

    def odom_callback(self, message):
        body = deepcopy(message)
        body.header.frame_id = self.output_frame
        self.body_publisher.publish(body)

        sensor = deepcopy(message)
        sensor.header.frame_id = self.output_frame
        sensor.child_frame_id = self.sensor_frame
        quaternion = normalized_quaternion(message.pose.pose.orientation)
        dx, dy, dz = rotate_vector(quaternion, self.sensor_offset)
        sensor.pose.pose.position.x += dx
        sensor.pose.pose.position.y += dy
        sensor.pose.pose.position.z += dz
        self.sensor_publisher.publish(sensor)

    def cloud_callback(self, message):
        # Keep slopes, stairs and vertical motion by avoiding z clipping.
        self.cloud_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlannerAdapter3D()
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
