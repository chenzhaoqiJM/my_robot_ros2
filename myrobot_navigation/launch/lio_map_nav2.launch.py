import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    map_to_odom_x = LaunchConfiguration('map_to_odom_x')
    map_to_odom_y = LaunchConfiguration('map_to_odom_y')
    map_to_odom_yaw = LaunchConfiguration('map_to_odom_yaw')

    nav_dir = get_package_share_directory('myrobot_navigation')
    default_params = os.path.join(nav_dir, 'config', 'lio_map_nav2_params.yaml')
    default_map = os.path.join(nav_dir, 'map', 'my_map.yaml')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py',
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=[
            '--x', map_to_odom_x,
            '--y', map_to_odom_y,
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', map_to_odom_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            },
        ],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            },
        ],
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
            description='Automatically bring up lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the occupancy grid map yaml.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 parameters for map-fixed FAST-LIO navigation.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_x',
            default_value='0.0',
            description='Static map->odom x offset for map alignment.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_y',
            default_value='0.0',
            description='Static map->odom y offset for map alignment.',
        ),
        DeclareLaunchArgument(
            'map_to_odom_yaw',
            default_value='0.0',
            description='Static map->odom yaw offset for map alignment.',
        ),
        map_to_odom_tf,
        map_server,
        map_lifecycle_manager,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
            }.items(),
        ),
    ])
