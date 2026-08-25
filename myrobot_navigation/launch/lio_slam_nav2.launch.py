import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    start_slam = LaunchConfiguration('start_slam')
    start_pointcloud_to_scan = LaunchConfiguration('start_pointcloud_to_scan')

    nav_dir = get_package_share_directory('myrobot_navigation')
    slam_dir = get_package_share_directory('myrobot_slam')
    default_params = os.path.join(nav_dir, 'config', 'lio_cartographer_nav2_params.yaml')

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py',
    )
    slam_launch = os.path.join(
        slam_dir,
        'launch',
        'online_async_sim.launch.py',
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        condition=IfCondition(start_pointcloud_to_scan),
        remappings=[
            ('cloud_in', '/points'),
            ('scan', '/scan'),
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'target_frame': 'lidar3d_link',
                'transform_tolerance': 0.2,
                'min_height': 0.02,
                'max_height': 0.35,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00873,
                'scan_time': 0.1,
                'range_min': 0.10,
                'range_max': 12.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            },
        ],
    )

    slam = GroupAction(
        condition=IfCondition(start_slam),
        actions=[
            SetParameter(name='use_sim_time', value=use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
            ),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically bring up Nav2 lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters for FAST-LIO odom with online SLAM map.',
        ),
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
            description='Start slam_toolbox online async SLAM to publish /map and map->odom.',
        ),
        DeclareLaunchArgument(
            'start_pointcloud_to_scan',
            default_value='true',
            description='Project /points to /scan for 2D SLAM.',
        ),
        pointcloud_to_scan,
        slam,
        nav2,
    ])
