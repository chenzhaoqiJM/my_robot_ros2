#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraColorRelay(Node):
    def __init__(self):
        super().__init__('camera_color_relay')

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        camera_info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            camera_info_qos
        )
        self.image_raw_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            image_qos
        )
        self.depth_image_rect_pub = self.create_publisher(
            Image,
            '/camera/depth/image_rect_raw',
            image_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            camera_info_qos
        )
        self.image_raw_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_raw_callback,
            image_qos
        )
        self.depth_image_raw_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_raw_callback,
            image_qos
        )

        self.get_logger().info(
            'Relaying /camera/camera_info -> /camera/color/camera_info, '
            '/camera/image_raw -> /camera/color/image_raw, '
            '/camera/depth/image_raw -> /camera/depth/image_rect_raw'
        )

    def camera_info_callback(self, msg):
        self.camera_info_pub.publish(msg)

    def image_raw_callback(self, msg):
        self.image_raw_pub.publish(msg)

    def depth_image_raw_callback(self, msg):
        self.depth_image_rect_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraColorRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
