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

def main():
    parser = argparse.ArgumentParser(description='Setup and launch an LTTng tracing session.')
    parser.add_argument('--session-name', '-s', dest='session_name',
                        default=f'session-{time.strftime("%Y%m%d%H%M%S")}',
                        help='the name of the tracing session (default: session-YYYYMMDDHHMMSS)')
    parser.add_argument('--path', '-p', dest='path',
                        default='/tmp',
                        help='path of the base directory for trace data (default: %(default)s)')
    args = parser.parse_args()

    session_name = args.session_name
    base_path = args.path
    path = base_path + '/' + session_name

    lttng_setup(session_name, path)
    print(f'writting tracing session to: {path}')

    input('press enter to start...')
    lttng_start(session_name)

    # TODO integrate this with launch + ROS shutdown
    input('press enter to stop...')

    print('stopping & destroying tracing session')
    lttng_stop(session_name)
    lttng_destroy(session_name)
