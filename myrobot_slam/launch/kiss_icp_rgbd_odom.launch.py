"""Launch planar KISS-ICP odometry using an RGB-D depth image."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame_id = LaunchConfiguration('base_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    odom_topic = LaunchConfiguration('odom_topic')
    raw_odom_topic = LaunchConfiguration('raw_odom_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    cloud_topic = LaunchConfiguration('cloud_topic')
    max_depth = LaunchConfiguration('max_depth')
    voxel_size = LaunchConfiguration('voxel_size')
    publish_debug_clouds = LaunchConfiguration('publish_debug_clouds')

    depth_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='kiss_icp_depth_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 2,
            'max_depth': ParameterValue(max_depth, value_type=float),
            'voxel_size': 0.02,
            'approx_sync': False,
        }],
        remappings=[
            ('depth/image', depth_topic),
            ('depth/camera_info', camera_info_topic),
            ('cloud', cloud_topic),
        ],
    )

    kiss_icp = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_rgbd',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': base_frame_id,
            'lidar_odom_frame': odom_frame_id,
            'publish_odom_tf': False,
            'invert_odom_tf': False,
            'publish_debug_clouds': publish_debug_clouds,
            'position_covariance': 0.05,
            'orientation_covariance': 0.05,
            # A depth camera has no per-point acquisition timestamps.
            'data.deskew': False,
            'data.max_range': ParameterValue(max_depth, value_type=float),
            'data.min_range': 0.2,
            'mapping.voxel_size': ParameterValue(voxel_size, value_type=float),
            'mapping.max_points_per_voxel': 20,
            'adaptive_threshold.initial_threshold': 0.5,
            'adaptive_threshold.min_motion_th': 0.02,
            'registration.max_num_iterations': 100,
            'registration.convergence_criterion': 0.0001,
        }],
        remappings=[
            ('pointcloud_topic', cloud_topic),
            ('kiss/odometry', raw_odom_topic),
        ],
    )

    planar_odometry = Node(
        package='myrobot_slam',
        executable='kiss_icp_odom_2d',
        name='kiss_icp_odom_2d',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            'publish_tf': True,
            'velocity_smoothing': 0.5,
        }],
        remappings=[
            ('kiss/odometry_raw', raw_odom_topic),
            ('odom', odom_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'base_frame_id', default_value='base_footprint',
            description='Robot base frame.'),
        DeclareLaunchArgument(
            'odom_frame_id', default_value='odom',
            description='Odometry frame.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/odom',
            description='Planar odometry output topic.'),
        DeclareLaunchArgument(
            'raw_odom_topic', default_value='/kiss/odometry_raw',
            description='Internal unprojected KISS-ICP odometry topic.'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth/image_raw',
            description='Depth image topic.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info',
            description='Depth camera calibration topic.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/kiss_icp_cloud',
            description='Generated RGB-D point cloud topic.'),
        DeclareLaunchArgument(
            'max_depth', default_value='4.0',
            description='Maximum depth used by point cloud registration.'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='KISS-ICP voxel size in meters.'),
        DeclareLaunchArgument(
            'publish_debug_clouds', default_value='false',
            description='Publish KISS-ICP frame, keypoint and map clouds.'),
        depth_cloud,
        kiss_icp,
        planar_odometry,
    ])
