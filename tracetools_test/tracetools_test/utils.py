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

"""Utils for tracetools_test that are not strictly test-related."""

import os
import shutil
import time
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple

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
        base_path: str,
        session_name_prefix: str,
        ros_events: List[str],
        kernel_events: List[str],
        package_name: str,
        node_names: List[str]
) -> Tuple[int, str]:
    """
    Run a node while tracing.

    :param base_path: the base path where to put the trace directory
    :param session_name_prefix: the session name prefix for the trace directory
    :param ros_events: the list of ROS UST events to enable
    :param kernel_events: the list of kernel events to enable
    :param package_name: the name of the package to use
    :param node_names: the names of the nodes to execute
    :return: exit code, full generated path
    """
    session_name = f'{session_name_prefix}-{time.strftime("%Y%m%d%H%M%S")}'
    full_path = os.path.join(base_path, session_name)

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


def cleanup_trace(full_path: str) -> None:
    """
    Cleanup trace data.

    :param full_path: the full path to the main trace directory
    """
    shutil.rmtree(full_path)

DictEvent = Dict[str, Any]

def get_trace_events(trace_directory: str) -> List[DictEvent]:
    """
    Get the events of a trace.

    :param trace_directory: the path to the main/top trace directory
    :return: events
    """
    tc = babeltrace.TraceCollection()
    tc.add_traces_recursive(trace_directory, 'ctf')

    return [_event_to_dict(event) for event in tc.events]


# List of ignored CTF fields
_IGNORED_FIELDS = [
    'content_size', 'cpu_id', 'events_discarded', 'id', 'packet_size', 'packet_seq_num',
    'stream_id', 'stream_instance_id', 'timestamp_end', 'timestamp_begin', 'magic', 'uuid', 'v'
]
_DISCARD = 'events_discarded'


def _event_to_dict(event: babeltrace.babeltrace.Event) -> DictEvent:
    """
    Convert name, timestamp, and all other keys except those in IGNORED_FIELDS into a dictionary.

    :param event: the event to convert
    :return: the event as a dictionary
    """
    d = {'_name': event.name, '_timestamp': event.timestamp}
    if hasattr(event, _DISCARD) and event[_DISCARD] > 0:
        print(event[_DISCARD])
    for key in [key for key in event.keys() if key not in _IGNORED_FIELDS]:
        d[key] = event[key]
    return d


def get_event_names(events: List[DictEvent]) -> List[str]:
    """
    Get a list of names of the events in the trace.

    :param events: the events of the trace
    :return: the list of event names
    """
    return [get_event_name(e) for e in events]


def get_field(event: DictEvent, field_name: str, default=None, raise_if_not_found=True) -> Any:
    field_value = event.get(field_name, default)
    # If enabled, raise exception as soon as possible to avoid headaches
    if raise_if_not_found and field_value is None:
        raise AttributeError(f'event field "{field_name}" not found!')
    return field_value


def get_event_name(event: DictEvent) -> str:
    return event['_name']


def get_event_timestamp(event: DictEvent) -> int:
    return event['_timestamp']


def get_procname(event: DictEvent) -> str:
    return event['procname']
