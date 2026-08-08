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
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bringup_robots(context):
    """
    Create one bringup include per robot listed in the robots argument.

    Cube identification is mandatory in a multi-cube setup: without a
    cube_id per robot all nodes would race for the same cube. cube_ids
    holds a comma-separated list matching the robots list; the legacy
    cube1_id / cube2_id arguments fill in for the first two robots when
    cube_ids is empty.
    """
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    bringup_launch_file = os.path.join(
        toio_ros2_dir, 'launch', 'toio_ros2_bringup.launch.py')

    robots = [r.strip() for r in
              context.launch_configurations['robots'].split(',') if r.strip()]
    cube_ids_str = context.launch_configurations['cube_ids']
    cube_ids = [c.strip() for c in cube_ids_str.split(',')] \
        if cube_ids_str else []
    if not cube_ids:
        cube_ids = [context.launch_configurations['cube1_id'],
                    context.launch_configurations['cube2_id']]
    # Pad so that robots beyond the given ids get an empty cube_id
    cube_ids += [''] * (len(robots) - len(cube_ids))

    actions = []
    for robot, cube_id in zip(robots, cube_ids):
        # Each include is wrapped in a scoped GroupAction because the
        # launch_arguments of IncludeLaunchDescription overwrite the
        # parent's launch configurations: without the scope,
        # 'use_rviz': 'false' below leaks out and the rviz2_node condition
        # always evaluates to false.
        actions.append(GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bringup_launch_file),
                    launch_arguments={
                        'namespace': robot,
                        'cube_id': cube_id,
                        'frame_prefix': f'{robot}/',
                        'params_file':
                            context.launch_configurations['params_file'],
                        'use_rviz': 'false',
                    }.items()
                ),
            ]))
    return actions


def generate_launch_description():
    """Bring up N toio cubes, one namespace per robot (default: toio1,toio2)."""
    toio_ros2_dir = get_package_share_directory('toio_ros2')
    rviz_config_file = os.path.join(toio_ros2_dir, 'rviz', 'toio_multi.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(toio_ros2_dir, 'params', 'toio_a4_play_mat_params.yaml'),
        description='Full path to the ROS2 parameters file shared by all cubes')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Use RViz2 if true')
    declare_robots_cmd = DeclareLaunchArgument(
        name='robots',
        default_value='toio1,toio2',
        description='Comma-separated list of robot namespaces')
    declare_cube_ids_cmd = DeclareLaunchArgument(
        name='cube_ids',
        default_value='',
        description='Comma-separated list of cube_ids matching the robots '
                    'list (takes precedence over cube1_id/cube2_id)')
    declare_cube1_id_cmd = DeclareLaunchArgument(
        name='cube1_id',
        default_value='',
        description='cube_id of the first cube (a substring of its BLE local '
                    'name); used when cube_ids is not set')
    declare_cube2_id_cmd = DeclareLaunchArgument(
        name='cube2_id',
        default_value='',
        description='cube_id of the second cube (a substring of its BLE '
                    'local name); used when cube_ids is not set')

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
    ld.add_action(declare_robots_cmd)
    ld.add_action(declare_cube_ids_cmd)
    ld.add_action(declare_cube1_id_cmd)
    ld.add_action(declare_cube2_id_cmd)

    ld.add_action(OpaqueFunction(function=bringup_robots))
    ld.add_action(rviz2_node)
    return ld
