import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_config_file = os.path.join(
        get_package_share_directory('myrobot_slam'),
        'config',
        'fast_lio_myrobot_3d_lidar.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='FAST-LIO parameter file for the Gazebo 3D lidar robot.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo /clock.',
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fast_lio_camera_init_to_odom',
            arguments=[
                '--frame-id', 'camera_init',
                '--child-frame-id', 'odom',
            ],
            output='screen',
        ),
    ])
