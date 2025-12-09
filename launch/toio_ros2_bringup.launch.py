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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    rviz_config_dir = os.path.join(toio_ros2_dir, 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'toio.rviz')

    toio_description_dir = get_package_share_directory('toio_description')

    toio_ros2_node = Node(
        package='toio_ros2',
        executable='toio_ros2_node',
        name='toio_ros2_node',
        parameters=[
            {
            }
        ],
        output='screen')
    
    toio_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(toio_description_dir, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    rviz2_node = Node(
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
    ld.add_action(toio_ros2_node)
    ld.add_action(toio_description_node)
    ld.add_action(rviz2_node)
    return ld
