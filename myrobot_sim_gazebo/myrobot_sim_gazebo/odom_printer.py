import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomPositionPrinter(Node):
    def __init__(self):
        super().__init__('odom_position_printer')
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            '/lidar_icp_odom',
            self.lidar_icp_odom_callback,
            10,
        )
        self.get_logger().info(
            'Listening to /odom and /lidar_icp_odom and printing xyz position...'
        )

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.get_logger().info(
            f'/odom -> x: {position.x:.3f}, y: {position.y:.3f}, z: {position.z:.3f}'
        )

    def lidar_icp_odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.get_logger().info(
            f'/lidar_icp_odom -> x: {position.x:.3f}, y: {position.y:.3f}, z: {position.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdomPositionPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
