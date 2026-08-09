# Copyright (C) 2026 atinfinity
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

from pathlib import Path

import pytest
import yaml

PARAMS_DIR = Path(__file__).resolve().parents[1] / 'params'
PARAMS_FILES = sorted(PARAMS_DIR.glob('*.yaml'))


@pytest.mark.parametrize(
    'params_file', PARAMS_FILES, ids=[p.name for p in PARAMS_FILES])
def test_node_keys_use_namespace_wildcard(params_file):
    # A bare 'toio_ros2_node:' key only matches the root namespace and is
    # silently ignored by the namespaced nodes that
    # toio_multi_bringup.launch.py starts (issue #24). '/**/' matches any
    # namespace including the root one.
    data = yaml.safe_load(params_file.read_text())
    assert list(data) == ['/**/toio_ros2_node'], (
        f'{params_file.name}: top-level keys {list(data)} must be '
        "['/**/toio_ros2_node'] so the file also applies to namespaced nodes")
    assert 'ros__parameters' in data['/**/toio_ros2_node']


def test_mat_params_expose_the_same_parameter_set():
    param_names = [
        set(yaml.safe_load(p.read_text())['/**/toio_ros2_node']
            ['ros__parameters'])
        for p in PARAMS_FILES
    ]
    assert all(names == param_names[0] for names in param_names)
