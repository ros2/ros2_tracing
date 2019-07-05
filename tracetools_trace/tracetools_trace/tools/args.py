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

"""Module containing parsing functions for tracing commands."""

import argparse
import time

from . import names
from . import path


class DefaultArgValueCompleter:
    """Callable returning an arg's default value."""

    def __init__(self, arg):
        default = arg.default
        self.list = default if isinstance(default, list) else [default]

    def __call__(self, **kwargs):
        return self.list


def parse_args():
    """Parse args for tracing."""
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
        default=path.DEFAULT_BASE_PATH,
        help='path of the base directory for trace data (default: %(default)s)')
    arg = parser.add_argument(
        '--ust', '-u', nargs='*', dest='events_ust', default=names.DEFAULT_EVENTS_ROS,
        help='the ROS UST events to enable (default: all events) '
             '[to disable all UST events, '
             'provide this flag without any event name]')
    arg.completer = DefaultArgValueCompleter(arg)
    arg = parser.add_argument(
        '--kernel', '-k', nargs='*', dest='events_kernel',
        default=names.DEFAULT_EVENTS_KERNEL,
        help='the kernel events to enable (default: all events) '
             '[to disable all kernel events, '
             'provide this flag without any event name]')
    arg.completer = DefaultArgValueCompleter(arg)
    parser.add_argument(
        '--list', '-l', dest='list', action='store_true',
        help='display lists of enabled events (default: %(default)s)')
