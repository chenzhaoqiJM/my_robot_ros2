from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([


        # Node(package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_odom_footprint',
        #     arguments=['1', '1', '0.0', '0', '0.0', '0', '/odom', '/base_footprint'],
        #     ),

        Node(package='myrobot_slam',
            executable='odom_pub_node',
            name='odom_pub_node1',
            ),

        Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_link_footprint',
            arguments=['0', '0', '0.05', '0', '0.0', '0', '/base_footprint', '/base_link'],
            ),

        Node(package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser22',
            arguments=['0.07', '0', '0.12', '-1.57', '0.0', '-1.57', '/base_link', '/ascamera_hp60c_camera_link_0'],
            ),

        # Node(package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_pub_laser33',
        #     arguments=['0.0', '0', '0.0001', '0', '0.0', '0', '/ascamera_hp60c_ascamera_0', '/ascamera_hp60c_color_0'],
        #     )

    ])
