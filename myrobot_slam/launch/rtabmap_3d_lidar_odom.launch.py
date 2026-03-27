import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = True

    lidar_icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='lidar_icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'lidar_odom',
            'guess_frame_id': 'odom',
            'guess_min_translation': 0.02,
            'guess_min_rotation': 0.02,
            'publish_tf': False,
            'wait_for_transform': 0.2,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/MaxTranslation': '2.0',
            'Icp/MaxRotation': '1.0',
            'Odom/Strategy': '0',
        }],
        remappings=[
            ('scan_cloud', '/points'),
            ('odom', '/lidar_icp_odom'),
        ],
    )

    return LaunchDescription([
        lidar_icp_odometry
    ])
