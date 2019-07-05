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

"""API functions for the ROS 2 trace command."""

from tracetools_trace.tools import args
from tracetools_trace.tools import lttng
from tracetools_trace.tools import path


def add_trace_arguments(parser):
    args.add_arguments(parser)


def init(args):
    """
    Init and start tracing.

    :param args: the parsed arguments object containing the right fields
    """
    session_name = args.session_name
    base_path = args.path
    ros_events = args.events_ust
    kernel_events = args.events_kernel

    ust_enabled = len(ros_events) > 0
    kernel_enabled = len(kernel_events) > 0
    if ust_enabled:
        print(f'UST tracing enabled ({len(ros_events)} events)')
        if args.list:
            print(f'\tevents: {ros_events}')
    else:
        print('UST tracing disabled')
    if kernel_enabled:
        print(f'kernel tracing enabled ({len(kernel_events)} events)')
        if args.list:
            print(f'\tevents: {kernel_events}')
    else:
        print('kernel tracing disabled')

    full_session_path = path.get_full_session_path(session_name, base_path)
    print(f'writing tracing session to: {full_session_path}')
    input('press enter to start...')
    lttng.lttng_init(
        session_name,
        base_path=base_path,
        ros_events=ros_events,
        kernel_events=kernel_events)


def fini(args):
    """
    Stop and finalize tracing.

    :param args: the parsed arguments object containing the right fields
    """
    session_name = args.session_name
    input('press enter to stop...')
    print('stopping & destroying tracing session')
    lttng.lttng_fini(session_name)
