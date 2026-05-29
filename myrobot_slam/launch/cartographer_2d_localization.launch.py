# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('myrobot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(pkg_share_dir, 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='lds_2d_localization.lua')
    load_state_filename = LaunchConfiguration('load_state_filename')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-load_state_filename', load_state_filename,
        ])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution,
                   '-publish_period_sec', publish_period_sec])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'load_state_filename',
        description='Path to the .pbstream map file for localization'))
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld
