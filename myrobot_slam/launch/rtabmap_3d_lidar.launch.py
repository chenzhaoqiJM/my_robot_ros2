import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    parameters = [{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': False,
            'approx_sync': False,
            'Reg/Strategy': '1',
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Grid/FromDepth': 'false',
            'Grid/3D': 'true',
            'Grid/RangeMax': '20.0',
            'Grid/CellSize': '0.1',
            'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
            'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
            'RGBD/NeighborLinkRefining': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Rtabmap/DetectionRate': '10.0',
            'Optimizer/GravitySigma': '0',
            'publish_tf': True,
        }]

    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=[
            ('odom', '/odom'),
            ('scan_cloud', '/points'),
        ],
        arguments=['-d'],
    )

    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[parameters[0], {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True',
        }],
        remappings=[
            ('odom', '/odom'),
            ('scan_cloud', '/points'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
        }],
        remappings=[
            ('odom', '/odom'),
            ('scan_cloud', '/points'),
        ],
    )

    obstacles_detection = Node(
        package='rtabmap_util',
        executable='obstacles_detection',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'Grid/3D': 'true',
            'Grid/NormalsSegmentation': 'true',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '2.0',
        }],
        remappings=[
            ('cloud', '/points'),
            ('obstacles', '/points/obstacles'),
            ('ground', '/points/ground'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode.',
        ),
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        obstacles_detection,
    ])
