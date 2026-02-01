# Copyright (C) 2025 atinfinity
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    rviz_config_dir = os.path.join(toio_ros2_dir, 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'toio.rviz')
    toio_description_dir = get_package_share_directory('toio_description')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(toio_ros2_dir, 'params', 'toio_a4_play_mat_params.yaml'),
        description='Full path to the ROS2 parameters file to use toio_ros2 node')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Use RViz2 if true'),

    toio_ros2_node = Node(
        package='toio_ros2',
        executable='toio_ros2_node',
        name='toio_ros2_node',
        parameters=[params_file],
        output='screen')
    
    toio_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(toio_description_dir, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
                {
                }
        ],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(toio_ros2_node)
    ld.add_action(toio_description_node)
    ld.add_action(rviz2_node)
    return ld
