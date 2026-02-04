from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_path = get_package_share_path('myrobot_visualization')
    default_rviz_config_path = os.path.join(package_path, 'config', 'lidar.rviz')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # ================== 静态 TF：base_footprint -> laser_link ==================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '0.1', '0', '0',        # x y z (米)
            '3.14159', '0', '0',        # roll pitch yaw (弧度)
            'base_footprint',
            'laser_link'
        ]
    )

    return LaunchDescription([
        rviz_arg,
        static_tf_node,
        rviz_node
    ])
