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
from launch_ros.parameter_descriptions import ParameterValue


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
    enable_goal_pose_motion = LaunchConfiguration('enable_goal_pose_motion')
    publish_odom = LaunchConfiguration('publish_odom')
    stop_on_position_id_missed = LaunchConfiguration('stop_on_position_id_missed')
    stop_on_button = LaunchConfiguration('stop_on_button')

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
    # Node parameters that a deployment switches per launch (issue #55).
    # An undeclared launch argument is silently ignored by launch, so the
    # ones worth flipping from the command line are declared here with the
    # node's own defaults. They are passed to the node after params_file and
    # therefore override a value set in that file.
    declare_enable_goal_pose_motion_cmd = DeclareLaunchArgument(
        name='enable_goal_pose_motion',
        default_value='true',
        description='Subscribe goal_pose for the cube built-in target motion. '
                    'Set to false when Nav2 / Open-RMF owns the motion plan '
                    '(RViz "2D Goal Pose" publishes goal_pose too)')
    declare_publish_odom_cmd = DeclareLaunchArgument(
        name='publish_odom',
        default_value='true',
        description='Publish /odom and the map -> odom -> center TF tree from the '
                    'wheel odometry; false publishes map -> center directly')
    declare_stop_on_position_id_missed_cmd = DeclareLaunchArgument(
        name='stop_on_position_id_missed',
        default_value='true',
        description='Stop the motor instead of following cmd_vel while the '
                    'Position ID is missed')
    declare_stop_on_button_cmd = DeclareLaunchArgument(
        name='stop_on_button',
        default_value='false',
        description='Use the cube button as a hold-to-stop for cmd_vel')

    toio_ros2_node = Node(
        package='toio_ros2',
        executable='toio_ros2_node',
        name='toio_ros2_node',
        namespace=namespace,
        parameters=[
            params_file,
            {'cube_id': cube_id,
             'frame_prefix': frame_prefix,
             # launch configurations are strings; the node declares these
             # as bool, so convert or the parameter type check fails
             'enable_goal_pose_motion':
                 ParameterValue(enable_goal_pose_motion, value_type=bool),
             'publish_odom': ParameterValue(publish_odom, value_type=bool),
             'stop_on_position_id_missed':
                 ParameterValue(stop_on_position_id_missed, value_type=bool),
             'stop_on_button': ParameterValue(stop_on_button, value_type=bool)}],
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
    ld.add_action(declare_enable_goal_pose_motion_cmd)
    ld.add_action(declare_publish_odom_cmd)
    ld.add_action(declare_stop_on_position_id_missed_cmd)
    ld.add_action(declare_stop_on_button_cmd)

    ld.add_action(toio_ros2_node)
    ld.add_action(toio_description_node)
    ld.add_action(rviz2_node)
    return ld
