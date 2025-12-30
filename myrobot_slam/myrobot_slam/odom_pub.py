#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # 写死一个固定位置
        self.x = 1.0
        self.y = 1.0
        self.z = 0.0

        # 固定朝向 yaw = 30度
        yaw = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        self.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    def timer_callback(self):
        # 时间戳
        now = self.get_clock().now().to_msg()

        # --- 发布 Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation = self.orientation

        self.odom_pub.publish(odom)

        # --- 发布 TF (可选) ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation = self.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
