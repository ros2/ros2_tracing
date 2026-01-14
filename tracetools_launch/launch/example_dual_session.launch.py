# Copyright 2025 Deva Shravan
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

"""
Example launch file to pre-configure a dual tracing session.

Use the ros2 trace commands with the --dual-session option and the
same session name at runtime to start the tracing.
"""

import launch
from launch_ros.actions import Node
from tracetools_launch.action import Trace


def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='my-dual-session',
            dual_session=True,
        ),
        Node(
            package='test_tracetools',
            executable='test_ping',
            output='screen',
        ),
        Node(
            package='test_tracetools',
            executable='test_pong',
            output='screen',
        ),
    ])
