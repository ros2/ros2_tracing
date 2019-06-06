#!/usr/bin/env python3
# Entrypoint/script to setup and start an LTTng tracing session
# TODO

import sys
import time
from tracetools_trace.tools.lttng import (
    lttng_setup,
    lttng_start,
    lttng_stop,
    lttng_destroy,
)

def main(argv=sys.argv):
    if len(argv) != 3:
        print("usage: session-name /path")
        exit(1)

    session_name = argv[1]
    path = argv[2] + '/' + session_name
    lttng_setup(session_name, path)
    lttng_start(session_name)
    print(f'tracing session started: {path}')

    # TODO integrate this with launch + ROS shutdown
    input('press enter to stop...')

    print('stopping & destroying tracing session')
    lttng_stop(session_name)
    lttng_destroy(session_name)
