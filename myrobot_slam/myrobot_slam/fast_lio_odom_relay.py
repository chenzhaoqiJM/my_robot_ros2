import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FastLioOdomRelay(Node):
    def __init__(self):
        super().__init__('fast_lio_odom_relay')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.last_time = None
        self.last_x = None
        self.last_y = None
        self.last_yaw = None

        self.pub = self.create_publisher(Odometry, output_topic, 20)
        self.sub = self.create_subscription(Odometry, input_topic, self.on_odom, 20)
        self.get_logger().info(f'Relaying {input_topic} to {output_topic}')

    def on_odom(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose = msg.pose
        out.pose.pose.position.z = 0.0

        now = rclpy.time.Time.from_msg(msg.header.stamp)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = (x - self.last_x) / dt
                dy = (y - self.last_y) / dt
                out.twist.twist.linear.x = math.cos(yaw) * dx + math.sin(yaw) * dy
                out.twist.twist.linear.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                out.twist.twist.angular.z = wrap_angle(yaw - self.last_yaw) / dt

        out.twist.covariance = msg.twist.covariance
        self.last_time = now
        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FastLioOdomRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
