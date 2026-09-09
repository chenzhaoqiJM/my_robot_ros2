"""Start FAST-LIO with a non-planar odometry output for Gazebo."""

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
            description='FAST-LIO parameters for the Gazebo 3D lidar.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping_3d',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='myrobot_slam',
            executable='fast_lio_odom_relay_3d',
            name='fast_lio_odom_relay_3d',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/Odometry',
                'output_topic': '/odom_3d',
                'frame_id': 'world',
                'child_frame_id': 'body',
                'velocity_filter_alpha': 0.35,
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fast_lio_world_to_camera_init_3d',
            arguments=[
                '--frame-id', 'world',
                '--child-frame-id', 'camera_init',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fast_lio_body_to_base_footprint_3d',
            arguments=[
                '--z', '-0.059',
                '--frame-id', 'body',
                '--child-frame-id', 'base_footprint',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
