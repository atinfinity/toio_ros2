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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Bring up two toio cubes in the namespaces toio1 and toio2.

    Cube identification is mandatory in a multi-cube setup: without
    cube1_id / cube2_id both nodes would race for the same cube.
    """
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    bringup_launch_file = os.path.join(
        toio_ros2_dir, 'launch', 'toio_ros2_bringup.launch.py')
    rviz_config_file = os.path.join(toio_ros2_dir, 'rviz', 'toio_multi.rviz')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    cube1_id = LaunchConfiguration('cube1_id')
    cube2_id = LaunchConfiguration('cube2_id')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(toio_ros2_dir, 'params', 'toio_a4_play_mat_params.yaml'),
        description='Full path to the ROS2 parameters file shared by both cubes')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Use RViz2 if true')
    declare_cube1_id_cmd = DeclareLaunchArgument(
        name='cube1_id',
        description='cube_id of the first cube (a substring of its BLE local name)')
    declare_cube2_id_cmd = DeclareLaunchArgument(
        name='cube2_id',
        description='cube_id of the second cube (a substring of its BLE local name)')

    toio1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            'namespace': 'toio1',
            'cube_id': cube1_id,
            'frame_prefix': 'toio1/',
            'params_file': params_file,
            'use_rviz': 'false',
        }.items()
    )

    toio2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            'namespace': 'toio2',
            'cube_id': cube2_id,
            'frame_prefix': 'toio2/',
            'params_file': params_file,
            'use_rviz': 'false',
        }.items()
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_cube1_id_cmd)
    ld.add_action(declare_cube2_id_cmd)

    ld.add_action(toio1_bringup)
    ld.add_action(toio2_bringup)
    ld.add_action(rviz2_node)
    return ld
