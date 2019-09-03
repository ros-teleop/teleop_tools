# Copyright 2019 Canonical Ltd.
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

import glob
import os
import subprocess

import ament_index_python

import pytest


testdata = glob.glob(
    os.path.join(
        ament_index_python.get_package_share_directory('joy_teleop'),
        'test',
        '*.test.py'
    )
)


# This test will automatically run for any *.test.py file in the examples folder and expect
# it to pass
@pytest.mark.parametrize('example_path', testdata, ids=[os.path.basename(d) for d in testdata])
def test_all_joy_teleop(example_path):

    proc = ['test', example_path]

    assert 0 == subprocess.run(args=proc).returncode
