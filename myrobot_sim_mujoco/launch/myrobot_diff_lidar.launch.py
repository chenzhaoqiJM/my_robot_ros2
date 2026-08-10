import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = 'myrobot_sim_mujoco'
    gazebo_package_name = 'myrobot_sim_gazebo'

    pkg_path = get_package_share_directory(package_name)
    gazebo_pkg_path = get_package_share_directory(gazebo_package_name)

    default_model_path = os.path.join(pkg_path, 'models', 'myrobot_diff_lidar.xml')
    default_params_path = os.path.join(pkg_path, 'config', 'mujoco_diff.yaml')
    xacro_file = os.path.join(gazebo_pkg_path, 'xacro', 'myrobot_lidar.xacro')

    model_path = LaunchConfiguration('model_path')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_viewer = LaunchConfiguration('enable_viewer')

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    mujoco_bridge = Node(
        package=package_name,
        executable='mujoco_diff_bridge',
        name='mujoco_diff_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'model_path': model_path,
                'use_sim_time': use_sim_time,
                'enable_viewer': enable_viewer,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Full path to the MuJoCo XML model file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to the MuJoCo bridge parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'enable_viewer',
            default_value='false',
            description='Open the MuJoCo passive viewer window',
        ),
        robot_state_publisher,
        mujoco_bridge,
    ])
