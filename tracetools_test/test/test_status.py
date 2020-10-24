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

import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest


def generate_test_description():
    launch_description = LaunchDescription()
    status_node = Node(package='tracetools', executable='status', output='screen')
    launch_description.add_action(status_node)
    launch_description.add_action(ReadyToTest())
    return launch_description, {'status_node': status_node}


class TestStatus(unittest.TestCase):

    def test_sanity_check(self, proc_output):
        proc_output.assertWaitFor('Tracing ', timeout=10, stream='stdout')
