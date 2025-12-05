import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # for navigation
    rdk_nav_dir = get_package_share_directory('rdk_navigation')
    rdk_nav_launchr = os.path.join(rdk_nav_dir, 'launch')

    param_file_name = 'rdk_diff.yaml'
    param_dir = os.path.join(rdk_nav_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(param_dir, param_file_name))

    # for slam
    slam_bringup_dir = get_package_share_directory('rdk_localization')
    # slam_launch_file = os.path.join(slam_bringup_dir, 'launch', 'slam_gmapping_sim.launch.py')
    slam_launch_file = os.path.join(slam_bringup_dir, 'launch', 'online_async_sim.launch.py')


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [rdk_nav_launchr, '/bringup_launch_for_slam.py']),
            launch_arguments={
                'map': '',
                'slam': 'True',
                'slam_launch_file': slam_launch_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
