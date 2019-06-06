#!/usr/bin/env python3
# Entrypoint/script to setup and start an LTTng tracing session

import sys
import time
import argparse
from tracetools_trace.tools.lttng import (
    lttng_setup,
    lttng_start,
    lttng_stop,
    lttng_destroy,
)
from tracetools_trace.tools.names import (
    DEFAULT_EVENTS_ROS,
    DEFAULT_EVENTS_KERNEL,
)

def main():
    parser = argparse.ArgumentParser(description='Setup and launch an LTTng tracing session.')
    parser.add_argument('--session-name', '-s', dest='session_name',
                        default=f'session-{time.strftime("%Y%m%d%H%M%S")}',
                        help='the name of the tracing session (default: session-YYYYMMDDHHMMSS)')
    parser.add_argument('--path', '-p', dest='path',
                        default='/tmp',
                        help='path of the base directory for trace data (default: %(default)s)')
    parser.add_argument('--ust', '-u', nargs='*', dest='events_ust', default=DEFAULT_EVENTS_ROS,
                        help='the ROS UST events to enable (default: all events) [to disable all UST events, provide this flag without any event name]')
    parser.add_argument('--kernel', '-k', nargs='*', dest='events_kernel', default=DEFAULT_EVENTS_KERNEL,
                        help='the kernel events to enable (default: all events) [to disable all UST events, provide this flag without any event name]')
    parser.add_argument('--list', '-l', dest='list', action='store_true',
                        help='display lists of enabled events (default: %(default)s)')
    args = parser.parse_args()

    session_name = args.session_name
    base_path = args.path
    path = base_path + '/' + session_name
    ros_events = args.events_ust
    kernel_events = args.events_kernel

    ust_enabled = len(ros_events) > 0
    kernel_enabled = len(kernel_events) > 0
    print(f'UST tracing {f"enabled ({len(ros_events)} events)" if ust_enabled else "disabled"}')
    if args.list and ust_enabled:
        print(f'\tevents: {ros_events}')
    print(f'kernel tracing {f"enabled ({len(kernel_events)} events)" if kernel_enabled else "disabled"}')
    if args.list and kernel_enabled:
        print(f'\tevents: {kernel_events}')

    lttng_setup(session_name, path, ros_events=ros_events, kernel_events=kernel_events)
    print(f'writting tracing session to: {path}')

    input('press enter to start...')
    lttng_start(session_name)
    input('press enter to stop...')

    print('stopping & destroying tracing session')
    lttng_stop(session_name)
    lttng_destroy(session_name)
