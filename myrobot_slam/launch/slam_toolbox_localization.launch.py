# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    pkg_share_dir = get_package_share_directory("myrobot_slam")
    default_params_file = (
        pkg_share_dir + '/config/mapper_params_localization.yaml'
    )

    map_file_name = LaunchConfiguration('map_file_name')
    params_file = LaunchConfiguration(
        'params_file', default=default_params_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_name',
            description='Path to the .posegraph map file (without extension)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the SLAM Toolbox params yaml'),

        launch_ros.actions.Node(
            parameters=[
                params_file,
                {'map_file_name': map_file_name},
            ],
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
        )
    ])
