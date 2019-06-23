import argparse
import time

from . import names


def parse_args():
    """
    Parse args for tracing.
    """
    parser = argparse.ArgumentParser(description='Setup and launch an LTTng tracing session.')
    add_arguments(parser)
    return parser.parse_args()


def add_arguments(parser):
    parser.add_argument(
        '--session-name', '-s', dest='session_name',
        default=f'session-{time.strftime("%Y%m%d%H%M%S")}',
        help='the name of the tracing session (default: session-YYYYMMDDHHMMSS)')
    parser.add_argument(
        '--path', '-p', dest='path',
        default='/tmp',
        help='path of the base directory for trace data (default: %(default)s)')
    parser.add_argument(
        '--ust', '-u', nargs='*', dest='events_ust', default=names.DEFAULT_EVENTS_ROS,
        help='the ROS UST events to enable (default: all events) '
            '[to disable all UST events, '
            'provide this flag without any event name]')
    parser.add_argument(
        '--kernel', '-k', nargs='*', dest='events_kernel',
        default=names.DEFAULT_EVENTS_KERNEL,
        help='the kernel events to enable (default: all events) '
            '[to disable all UST events, '
            'provide this flag without any event name]')
    parser.add_argument(
        '--list', '-l', dest='list', action='store_true',
        help='display lists of enabled events (default: %(default)s)')
