"""Run SCAN-Planner with the myrobot Gazebo Classic simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')
    start_rviz = LaunchConfiguration('start_rviz')
    navi_mode = LaunchConfiguration('navi_mode')

    gazebo_launch = os.path.join(
        get_package_share_directory('myrobot_sim_gazebo'),
        'launch',
        'myrobot_diff_3d_lidar.launch.py',
    )
    scan_share = get_package_share_directory('scan_planner')
    planner_yaml = os.path.join(scan_share, 'config', 'planner.yaml')
    controllers_yaml = os.path.join(scan_share, 'config', 'controllers.yaml')
    rviz_launch = os.path.join(scan_share, 'launch', 'rviz.launch.py')

    adapter = Node(
        package='myrobot_navigation',
        executable='scan_planner_adapter',
        name='scan_planner_adapter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'cloud_topic': '/points',
            'body_z': 0.25,
            'sensor_x': 0.07,
            'sensor_y': 0.0,
            'sensor_z': 0.144,
            'min_sensor_z': -0.10,
            'output_frame': 'world',
        }],
        remappings=[
            ('body_pose', '/scan_planner/body_pose'),
            ('sensor_pose', '/scan_planner/sensor_pose'),
            ('cloud', '/scan_planner/cloud'),
        ],
    )

    planner = Node(
        package='scan_planner',
        executable='scan_planner_node',
        name='scan_planner_node',
        output='screen',
        parameters=[planner_yaml, {
            'use_sim_time': use_sim_time,
            'fsm.navi_mode': navi_mode,
            'fsm.planning_horizon': 3.5,
            'fsm.max_replan_fail_count': 50,
            'grid_map.sensor_type': 'lidar',
            'grid_map.cloud_is_world': False,
            'grid_map.need_extrinsic': False,
            'grid_map.frame_id': 'world',
            'grid_map.ground_height': 0.0,
            'grid_map.double_cylinder_radius': 0.16,
            'grid_map.double_cylinder_offset': 0.0,
            'grid_map.body_height': 0.18,
            'grid_map.obstacles_inflation_z_up': 0.05,
            'grid_map.obstacles_inflation_z_down': 0.05,
            'manager.max_vel': 0.35,
            'manager.max_acc': 0.5,
            'manager.planning_horizon': 3.5,
            'optimization.max_vel': 0.35,
            'optimization.max_acc': 0.5,
        }],
        remappings=[
            ('body_pose', '/scan_planner/body_pose'),
            ('sensor_pose', '/scan_planner/sensor_pose'),
            ('cloud', '/scan_planner/cloud'),
            ('move_base_simple/goal', '/move_base_simple/goal'),
        ],
    )

    controller = Node(
        package='scan_planner',
        executable='closed_loop_controller',
        name='closed_loop_controller',
        output='screen',
        parameters=[controllers_yaml, {
            'use_sim_time': use_sim_time,
            'max_vx': 0.35,
            'max_vy': 0.0,
            'max_vyaw': 1.0,
        }],
        remappings=[
            ('body_pose', '/scan_planner/body_pose'),
            ('cmd_vel', '/cmd_vel'),
            ('planning/bspline', '/planning/bspline'),
            (
                'planning/go2_execution_frozen',
                '/planning/go2_execution_frozen',
            ),
        ],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('navi_mode', default_value='1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(start_gazebo),
        ),
        adapter,
        planner,
        controller,
        world_to_odom,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(start_rviz),
        ),
    ])
