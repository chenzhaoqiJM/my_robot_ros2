import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    myrobot_nav_dir = get_package_share_directory('myrobot_navigation')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    default_params_file = os.path.join(
        myrobot_nav_dir, 'config', 'myrobot_diff_mpc.yaml')

    params_file = LaunchConfiguration('params_file')

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to Nav2 params file'),
        nav2_navigation_launch,
    ])
