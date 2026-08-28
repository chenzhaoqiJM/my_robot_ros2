import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'myrobot_sim_mujoco'
    pkg_path = get_package_share_directory(package_name)
    model_path = os.path.join(pkg_path, 'models', 'myrobot_rgbd.xml')
    params_path = os.path.join(pkg_path, 'config', 'mujoco_rgbd.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=model_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_viewer', default_value='false'),
        Node(package=package_name, executable='mujoco_rgbd_bridge', name='mujoco_rgbd_bridge', output='screen',
               additional_env={'MUJOCO_GL': 'egl'},
             parameters=[params_path, {'model_path': LaunchConfiguration('model_path'),
                                       'use_sim_time': LaunchConfiguration('use_sim_time'),
                                       'enable_viewer': LaunchConfiguration('enable_viewer')}]),
    ])