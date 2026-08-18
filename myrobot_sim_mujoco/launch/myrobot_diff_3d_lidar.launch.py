import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'myrobot_sim_mujoco'
    pkg_path = get_package_share_directory(package_name)
    model_path = os.path.join(pkg_path, 'models', 'myrobot_diff_3d_lidar.xml')
    urdf_path = os.path.join(pkg_path, 'models', 'myrobot_3d_lidar.urdf')
    params_path = os.path.join(pkg_path, 'config', 'mujoco_3d_lidar.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_viewer = LaunchConfiguration('enable_viewer')
    with open(urdf_path, encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )
    bridge = Node(
        package=package_name,
        executable='mujoco_3d_lidar_bridge',
        name='mujoco_3d_lidar_bridge',
        output='screen',
        parameters=[
            params_path,
            {
                'model_path': LaunchConfiguration('model_path'),
                'use_sim_time': use_sim_time,
                'enable_viewer': enable_viewer,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=model_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_viewer', default_value='false'),
        robot_state_publisher,
        bridge,
    ])