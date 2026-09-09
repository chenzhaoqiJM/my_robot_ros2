"""Run the isolated FAST-LIO + SCAN-Planner 3D Gazebo pipeline."""

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
    start_fast_lio = LaunchConfiguration('start_fast_lio')
    start_rviz = LaunchConfiguration('start_rviz')
    navi_mode = LaunchConfiguration('navi_mode')

    gazebo_launch = os.path.join(
        get_package_share_directory('myrobot_sim_gazebo'),
        'launch',
        'myrobot_diff_3d_lidar_lio.launch.py',
    )
    fast_lio_launch = os.path.join(
        get_package_share_directory('myrobot_slam'),
        'launch',
        'fast_lio_sim_3d.launch.py',
    )
    scan_share = get_package_share_directory('scan_planner')
    planner_yaml = os.path.join(scan_share, 'config', 'planner.yaml')
    controllers_yaml = os.path.join(scan_share, 'config', 'controllers.yaml')
    rviz_launch = os.path.join(scan_share, 'launch', 'rviz.launch.py')

    adapter = Node(
        package='myrobot_navigation',
        executable='scan_planner_adapter_3d',
        name='scan_planner_adapter_3d',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom_3d',
            'cloud_topic': '/points',
            # This is the same lidar-to-IMU translation used by FAST-LIO.
            'sensor_x': 0.07,
            'sensor_y': 0.0,
            'sensor_z': 0.085,
            'output_frame': 'world',
            'sensor_frame': 'lidar3d_link',
        }],
        remappings=[
            ('body_pose', '/scan_planner_3d/body_pose'),
            ('sensor_pose', '/scan_planner_3d/sensor_pose'),
            ('cloud', '/scan_planner_3d/cloud'),
        ],
    )

    planner = Node(
        package='scan_planner',
        executable='scan_planner_node',
        # Keep this name: planner.yaml is scoped to scan_planner_node.
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
            # FAST-LIO starts at the IMU pose, so the floor is below zero.
            'grid_map.ground_height': -1.0,
            'grid_map.sliding_map_size_z': 5.0,
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
            ('body_pose', '/scan_planner_3d/body_pose'),
            ('sensor_pose', '/scan_planner_3d/sensor_pose'),
            ('cloud', '/scan_planner_3d/cloud'),
            ('move_base_simple/goal', '/move_base_simple/goal'),
        ],
    )

    # The simulated robot is differential drive, so control remains planar.
    # The planner and estimator above still retain all 3D state.
    controller = Node(
        package='scan_planner',
        executable='closed_loop_controller',
        # controllers.yaml is scoped to this exact node name.
        name='closed_loop_controller',
        output='screen',
        parameters=[controllers_yaml, {
            'use_sim_time': use_sim_time,
            'max_vx': 0.35,
            'max_vy': 0.0,
            'max_vyaw': 1.0,
        }],
        remappings=[
            ('body_pose', '/scan_planner_3d/body_pose'),
            ('cmd_vel', '/cmd_vel'),
            ('planning/bspline', '/planning/bspline'),
            ('planning/go2_execution_frozen',
             '/planning/go2_execution_frozen'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_gazebo', default_value='true'),
        DeclareLaunchArgument('start_fast_lio', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('navi_mode', default_value='1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(start_gazebo),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(start_fast_lio),
        ),
        adapter,
        planner,
        controller,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(start_rviz),
        ),
    ])
