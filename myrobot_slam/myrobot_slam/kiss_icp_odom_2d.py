"""Convert KISS-ICP's 6-DoF pose output into planar robot odometry."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    """Return the z and w components of a yaw-only quaternion."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def yaw_from_quaternion(quaternion):
    """Extract yaw from a geometry_msgs quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class KissIcpOdom2D(Node):
    """Publish planar odometry, velocity and TF from KISS-ICP poses."""

    def __init__(self):
        super().__init__('kiss_icp_odom_2d')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('velocity_smoothing', 0.5)

        self.odom_frame_id = self.get_parameter(
            'odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter(
            'base_frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter(
            'publish_tf').get_parameter_value().bool_value
        self.velocity_smoothing = self.get_parameter(
            'velocity_smoothing').get_parameter_value().double_value
        self.velocity_smoothing = min(max(self.velocity_smoothing, 0.0), 1.0)

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, 'kiss/odometry_raw', self.odometry_callback, 10)

        self.previous_stamp_ns = None
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

    def odometry_callback(self, raw_msg):
        """Project pose to SE(2), derive velocity, then publish odom and TF."""
        stamp_ns = (
            raw_msg.header.stamp.sec * 1_000_000_000
            + raw_msg.header.stamp.nanosec)
        x = raw_msg.pose.pose.position.x
        y = raw_msg.pose.pose.position.y
        yaw = yaw_from_quaternion(raw_msg.pose.pose.orientation)

        if self.previous_stamp_ns is not None:
            dt = (stamp_ns - self.previous_stamp_ns) / 1_000_000_000.0
            if 0.0 < dt <= 1.0:
                dx = x - self.previous_x
                dy = y - self.previous_y
                measured_linear_x = (
                    math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
                measured_linear_y = (
                    -math.sin(yaw) * dx + math.cos(yaw) * dy) / dt
                measured_angular_z = normalize_angle(
                    yaw - self.previous_yaw) / dt
                alpha = self.velocity_smoothing
                self.linear_x = (
                    alpha * measured_linear_x + (1.0 - alpha) * self.linear_x)
                self.linear_y = (
                    alpha * measured_linear_y + (1.0 - alpha) * self.linear_y)
                self.angular_z = (
                    alpha * measured_angular_z
                    + (1.0 - alpha) * self.angular_z)
            else:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0

        self.previous_stamp_ns = stamp_ns
        self.previous_x = x
        self.previous_y = y
        self.previous_yaw = yaw

        odom = Odometry()
        odom.header.stamp = raw_msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = (
            quaternion_from_yaw(yaw))
        odom.pose.covariance = raw_msg.pose.covariance
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.angular.z = self.angular_z
        odom.twist.covariance = raw_msg.pose.covariance
        self.odom_publisher.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = KissIcpOdom2D()
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
