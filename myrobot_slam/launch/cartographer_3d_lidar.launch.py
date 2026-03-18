import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    myrobot_slam_dir = get_package_share_directory('myrobot_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    points_topic = LaunchConfiguration('points_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_resolution = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution of a grid cell for the published occupancy grid'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Occupancy grid publishing period'
    )

    declare_configuration_directory = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(myrobot_slam_dir, 'config'),
        description='Full path to Cartographer config directory'
    )

    declare_configuration_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='points_3d.lua',
        description='Cartographer lua configuration file name'
    )

    declare_points_topic = DeclareLaunchArgument(
        'points_topic',
        default_value='/points',
        description='Input PointCloud2 topic'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='Input IMU topic'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('points2', points_topic),
            ('imu', imu_topic),
        ]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_resolution,
        declare_publish_period_sec,
        declare_configuration_directory,
        declare_configuration_basename,
        declare_points_topic,
        declare_imu_topic,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
