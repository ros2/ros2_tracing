# Copyright 2020 Christophe Bedard
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

import unittest

import launch
import launch_ros
import launch_testing


def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    status_tool = launch_ros.actions.Node(
        package='tracetools',
        executable='status',
        env=proc_env,
        output='screen',
    )

    return launch.LaunchDescription([
        status_tool,
        launch_testing.actions.ReadyToTest(),
    ]), {'status_tool': status_tool}


class TestStatus(unittest.TestCase):

    def test_output(self, proc_output):
        # This should be printed no matter what
        proc_output.assertWaitFor('Tracing', timeout=10, stream='stdout')
