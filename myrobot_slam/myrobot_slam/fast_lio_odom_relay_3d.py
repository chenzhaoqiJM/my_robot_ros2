"""Relay FAST-LIO odometry without discarding its 3D motion."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q.x / norm, q.y / norm, q.z / norm, q.w / norm)


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def world_angular_velocity(previous, current, dt):
    """Convert the shortest quaternion delta to a world rotation vector."""
    px, py, pz, pw = previous
    delta = quaternion_multiply(current, (-px, -py, -pz, pw))
    if delta[3] < 0.0:
        delta = tuple(-component for component in delta)

    vx, vy, vz, w = delta
    vector_norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    if vector_norm < 1.0e-12:
        return (0.0, 0.0, 0.0)

    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, w)))
    scale = angle / (vector_norm * dt)
    return (vx * scale, vy * scale, vz * scale)


class FastLioOdomRelay3D(Node):
    """Preserve FAST-LIO xyz/quaternion and derive a smooth 6-DoF twist."""

    def __init__(self):
        super().__init__('fast_lio_odom_relay_3d')
        input_topic = self.declare_parameter('input_topic', '/Odometry').value
        output_topic = self.declare_parameter('output_topic', '/odom_3d').value
        self.frame_id = self.declare_parameter('frame_id', 'world').value
        self.child_frame_id = self.declare_parameter(
            'child_frame_id', 'body').value
        self.velocity_filter_alpha = float(
            self.declare_parameter('velocity_filter_alpha', 0.35).value)
        self.max_delta_time = float(self.declare_parameter(
            'max_delta_time', 0.5).value)

        self.previous_time = None
        self.previous_position = None
        self.previous_orientation = None
        self.filtered_linear = (0.0, 0.0, 0.0)
        self.filtered_angular = (0.0, 0.0, 0.0)

        self.publisher = self.create_publisher(Odometry, output_topic, 20)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.odom_callback, 20)
        self.get_logger().info(
            f'Relaying full 3D odometry: {input_topic} -> {output_topic} '
            f'({self.frame_id} -> {self.child_frame_id})')

    def filtered(self, previous, measurement):
        alpha = min(1.0, max(0.0, self.velocity_filter_alpha))
        return tuple(
            alpha * value + (1.0 - alpha) * old
            for old, value in zip(previous, measurement)
        )

    def odom_callback(self, message):
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.frame_id
        output.child_frame_id = self.child_frame_id
        output.pose = message.pose
        output.twist.covariance = message.twist.covariance

        stamp = rclpy.time.Time.from_msg(message.header.stamp)
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        orientation = normalize_quaternion(message.pose.pose.orientation)

        if self.previous_time is not None:
            dt = (stamp - self.previous_time).nanoseconds * 1.0e-9
            if 1.0e-6 < dt <= self.max_delta_time:
                linear = tuple(
                    (value - old) / dt
                    for value, old in zip(position, self.previous_position)
                )
                angular = world_angular_velocity(
                    self.previous_orientation, orientation, dt)
                self.filtered_linear = self.filtered(
                    self.filtered_linear, linear)
                self.filtered_angular = self.filtered(
                    self.filtered_angular, angular)

        output.twist.twist.linear.x = self.filtered_linear[0]
        output.twist.twist.linear.y = self.filtered_linear[1]
        output.twist.twist.linear.z = self.filtered_linear[2]
        output.twist.twist.angular.x = self.filtered_angular[0]
        output.twist.twist.angular.y = self.filtered_angular[1]
        output.twist.twist.angular.z = self.filtered_angular[2]

        self.previous_time = stamp
        self.previous_position = position
        self.previous_orientation = orientation
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdomRelay3D()
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
