# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 导航功能包的路径
    pkg_share_dir = get_package_share_directory('myrobot_slam')
    # 是否使用仿真时间，这里使用Gazebo，所以配置为true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 参数配置文件在功能包中的文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share_dir, 'config') )
    # 参数配置文件的名称
    configuration_basename = LaunchConfiguration('configuration_basename', default='provider_odom.lua')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename],
        )

    tf_to_odom_node = Node(
        package='myrobot_slam',
        executable='tf_to_odom',
        name='cartographer_tf_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'odom_topic': '/odom',
            'publish_rate': 20.0,
        }])


    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(tf_to_odom_node)

    return ld
