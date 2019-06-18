# Copyright 2019 Robert Bosch GmbH
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

"""Utils for tracetools_test."""

import shutil
import time

import babeltrace
from launch import LaunchDescription
from launch import LaunchService
from launch_ros import get_default_launch_description
import launch_ros.actions
from tracetools_trace.tools.lttng import (
    lttng_destroy,
    lttng_setup,
    lttng_start,
    lttng_stop,
)


def run_and_trace(
        base_path,
        session_name_prefix,
        ros_events,
        kernel_events,
        package_name,
        node_names):
    """
    Run a node while tracing.

    :param base_path (str): the base path where to put the trace directory
    :param session_name_prefix (str): the session name prefix for the trace directory
    :param ros_events (list(str)): the list of ROS UST events to enable
    :param kernel_events (list(str)): the list of kernel events to enable
    :param package_name (str): the name of the package to use
    :param node_names (list(str)): the names of the nodes to execute
    """
    session_name = f'{session_name_prefix}-{time.strftime("%Y%m%d%H%M%S")}'
    full_path = f'{base_path}/{session_name}'
    print(f'trace directory: {full_path}')

    lttng_setup(session_name, full_path, ros_events=ros_events, kernel_events=kernel_events)
    lttng_start(session_name)

    nodes = []
    for node_name in node_names:
        n = launch_ros.actions.Node(
            package=package_name,
            node_executable=node_name,
            output='screen')
        nodes.append(n)
    ld = LaunchDescription(nodes)
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)

    exit_code = ls.run()

    lttng_stop(session_name)
    lttng_destroy(session_name)

    return exit_code, full_path


def cleanup_trace(full_path):
    """
    Cleanup trace data.

    :param full_path (str): the full path to the main trace directory
    """
    shutil.rmtree(full_path)


def get_trace_event_names(trace_directory):
    """
    Get a set of event names in a trace.

    :param trace_directory (str): the path to the main/top trace directory
    :return: event names (set(str))
    """
    tc = babeltrace.TraceCollection()
    tc.add_traces_recursive(trace_directory, 'ctf')

    event_names = set()

    for event in tc.events:
        event_names.add(event.name)

    return event_names
