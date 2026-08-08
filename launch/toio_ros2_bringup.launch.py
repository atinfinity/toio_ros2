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
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    rviz_config_dir = os.path.join(toio_ros2_dir, 'rviz')
    rviz_config_file = os.path.join(rviz_config_dir, 'toio.rviz')
    toio_description_dir = get_package_share_directory('toio_description')

    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
    cube_id = LaunchConfiguration('cube_id')
    frame_prefix = LaunchConfiguration('frame_prefix')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(toio_ros2_dir, 'params', 'toio_a4_play_mat_params.yaml'),
        description='Full path to the ROS2 parameters file to use toio_ros2 node')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Use RViz2 if true')
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace of the toio nodes and topics (e.g. toio1)')
    declare_cube_id_cmd = DeclareLaunchArgument(
        name='cube_id',
        default_value='',
        description='Connect only to the cube whose BLE local name contains cube_id')
    declare_frame_prefix_cmd = DeclareLaunchArgument(
        name='frame_prefix',
        default_value='',
        description='TF frame prefix of this cube (e.g. "toio1/")')

    toio_ros2_node = Node(
        package='toio_ros2',
        executable='toio_ros2_node',
        name='toio_ros2_node',
        namespace=namespace,
        parameters=[
            params_file,
            {'cube_id': cube_id, 'frame_prefix': frame_prefix}],
        output='screen')

    toio_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(toio_description_dir, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'frame_prefix': frame_prefix,
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
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_cube_id_cmd)
    ld.add_action(declare_frame_prefix_cmd)

    ld.add_action(toio_ros2_node)
    ld.add_action(toio_description_node)
    ld.add_action(rviz2_node)
    return ld
