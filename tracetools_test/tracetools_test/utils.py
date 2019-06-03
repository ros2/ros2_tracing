# Utils for tracetools_test

import time
import shutil
import subprocess
import babeltrace
from launch import LaunchDescription
from launch import LaunchService
from launch_ros import get_default_launch_description
import launch_ros.actions
from tracetools_trace.tools.lttng import (
    lttng_setup,
    lttng_start,
    lttng_stop,
    lttng_destroy,
)

def run_and_trace(base_path, session_name_prefix, ros_events, kernel_events, package_name, node_executable):
    session_name = f'{session_name_prefix}-{time.strftime("%Y%m%d%H%M%S")}'
    full_path = f'{base_path}/{session_name}'
    print(f'trace directory: {full_path}')

    lttng_setup(session_name, full_path, ros_events=ros_events, kernel_events=kernel_events)
    lttng_start(session_name)

    ld = LaunchDescription([
        launch_ros.actions.Node(
            package=package_name, node_executable=node_executable, output='screen'),
    ])
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)

    exit_code = ls.run()

    lttng_stop(session_name)
    lttng_destroy(session_name)

    return exit_code, full_path


def cleanup_trace(full_path):
    shutil.rmtree(full_path)


def get_trace_event_names(trace_directory):
    """
    Get a set of event names in a trace
    :param trace_directory (str): the path to the main/top trace directory
    :return: event names (set(str))
    """
    tc = babeltrace.TraceCollection()
    tc.add_traces_recursive(trace_directory, 'ctf')

    event_names = set()

    for event in tc.events:
        event_names.add(event.name)

    return event_names
