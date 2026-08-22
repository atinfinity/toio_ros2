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

"""The bringup launch files declare the node parameters worth switching per launch (issue #55)."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
import pytest

LAUNCH_DIR = os.path.join(os.path.dirname(__file__), '..', 'launch')

# name -> default, matching the defaults declared by toio_ros2_node
NODE_PARAM_ARGS = {
    'enable_goal_pose_motion': 'false',
    'publish_odom': 'true',
    'stop_on_position_id_missed': 'true',
    'stop_on_button': 'false',
}


def declared_arguments(launch_file):
    spec = importlib.util.spec_from_file_location(
        launch_file, os.path.join(LAUNCH_DIR, launch_file))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    return {a.name: a.default_value[0].text
            for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


@pytest.mark.parametrize('launch_file', [
    'toio_ros2_bringup.launch.py', 'toio_multi_bringup.launch.py'])
def test_node_parameter_arguments_are_declared(launch_file):
    args = declared_arguments(launch_file)
    for name, default in NODE_PARAM_ARGS.items():
        assert args.get(name) == default, name


def test_launch_defaults_match_the_node_defaults(monkeypatch):
    # keep the launch defaults honest: they must not silently flip a node
    # default, since the launch value overrides params_file
    import rclpy
    from toio_ros2.toio_ros2_node import ToioNode

    async def _noop(self):
        return None

    monkeypatch.setattr(ToioNode, 'connect_toio', _noop)
    monkeypatch.setattr(ToioNode, 'motor_command_loop', _noop)
    rclpy.init()
    try:
        node = ToioNode()
        for name, default in NODE_PARAM_ARGS.items():
            assert node.get_parameter(name).value is (default == 'true'), name
        node.destroy_node()
    finally:
        rclpy.shutdown()
