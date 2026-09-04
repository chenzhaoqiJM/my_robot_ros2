"""Publish nav_msgs/Odometry from an existing TF transform."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


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


class TfToOdom(Node):
    """Convert a TF pose stream to planar odometry with derived velocity."""

    def __init__(self):
        super().__init__('tf_to_odom')

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('position_covariance', 0.05)
        self.declare_parameter('orientation_covariance', 0.05)
        self.declare_parameter('velocity_smoothing', 0.5)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        odom_topic = self.get_parameter('odom_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.position_covariance = self.get_parameter(
            'position_covariance').value
        self.orientation_covariance = self.get_parameter(
            'orientation_covariance').value
        self.velocity_smoothing = min(max(
            self.get_parameter('velocity_smoothing').value, 0.0), 1.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_odometry)

        self.previous_stamp_ns = None
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_yaw = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.warned_missing_transform = False

        self.get_logger().info(
            f'Publishing {odom_topic} from TF '
            f'{self.odom_frame_id} -> {self.base_frame_id}')

    def publish_odometry(self):
        """Look up the latest transform and publish it as planar odometry."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame_id, self.base_frame_id, Time())
        except TransformException as error:
            if not self.warned_missing_transform:
                self.get_logger().warning(
                    f'Waiting for TF {self.odom_frame_id} -> '
                    f'{self.base_frame_id}: {error}')
                self.warned_missing_transform = True
            return

        self.warned_missing_transform = False
        stamp_ns = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec)
        if self.previous_stamp_ns is not None and stamp_ns <= self.previous_stamp_ns:
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)

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
        odom.header = transform.header
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = transform.transform.rotation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
        odom.twist.twist.linear.x = self.linear_x
        odom.twist.twist.linear.y = self.linear_y
        odom.twist.twist.angular.z = self.angular_z

        for index in (0, 7):
            odom.pose.covariance[index] = self.position_covariance
            odom.twist.covariance[index] = self.position_covariance
        for index in (14, 21, 28):
            odom.pose.covariance[index] = 1e6
            odom.twist.covariance[index] = 1e6
        odom.pose.covariance[35] = self.orientation_covariance
        odom.twist.covariance[35] = self.orientation_covariance

        self.odom_publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = TfToOdom()
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
