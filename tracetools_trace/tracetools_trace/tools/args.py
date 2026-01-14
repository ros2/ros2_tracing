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

from . import names
from . import path


class ArgCompleter:
    """Callable return given value."""

    def __init__(self, value):
        self.value = value

    def __call__(self, **kwargs):
        return self.value


class DefaultArgValueCompleter(ArgCompleter):
    """Callable returning an arg's default value."""

    def __init__(self, arg):
        super().__init__(arg.default if isinstance(arg.default, list) else [arg.default])


def parse_args() -> argparse.Namespace:
    """Parse arguments for interactive tracing session configuration."""
    parser = argparse.ArgumentParser(
        description='Trace ROS 2 nodes to get information on their execution')
    add_arguments(parser)
    return parser.parse_args()


def _add_arguments_configure(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '-p', '--path', dest='path',
        help='path of the base directory for trace data (default: '
             '$ROS_TRACE_DIR if ROS_TRACE_DIR is set and not empty, or '
             '$ROS_HOME/tracing, using ~/.ros for ROS_HOME if not set or if empty)')
    events_ust_arg = parser.add_argument(
        '-u', '--ust', nargs='*', dest='events_ust', metavar='EVENT',
        default=names.DEFAULT_EVENTS_UST,
        help='the userspace events to enable (default: see tracetools_trace.tools.names) '
             '[to disable all UST events, '
             'provide this flag without any event name]')
    events_ust_arg.completer = DefaultArgValueCompleter(events_ust_arg)  # type: ignore
    events_kernel_arg = parser.add_argument(
        '-k', '--kernel', nargs='*', dest='events_kernel', metavar='EVENT',
        default=[],
        help='the kernel events to enable (default: no kernel events)')
    events_kernel_arg.completer = ArgCompleter(names.EVENTS_KERNEL)  # type: ignore
    parser.add_argument(
        '--syscall', nargs='*', dest='syscalls', metavar='SYSCALL',
        default=[],
        help='the syscalls to enable (default: no syscalls)')
    context_arg = parser.add_argument(
        '-c', '--context', nargs='*', dest='context_fields', metavar='CONTEXT',
        default=names.DEFAULT_CONTEXT,
        help='the context fields to enable (default: see tracetools_trace.tools.names) '
             '[to disable all context fields, '
             'provide this flag without any field names]')
    context_arg.completer = DefaultArgValueCompleter(context_arg)  # type: ignore
    parser.add_argument(
        '-l', '--list', dest='list', action='store_true',
        help='display lists of enabled events and context names (default: %(default)s)')
    parser.add_argument(
        '-a', '--append-trace', dest='append_trace', action='store_true',
        help='append to trace if it already exists, otherwise error out (default: %(default)s)')


def _add_arguments_default_session_name(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '-s', '--session-name', dest='session_name',
        default=path.append_timestamp('session'),
        help='the name of the tracing session (default: session-YYYYMMDDHHMMSS)')


def _add_arguments_dual_session(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '-d', '--dual-session', dest='dual_session', action='store_true',
        help='use this in interactive mode to record snapshot of the pre-configured snapshot '
             'session with the same name and start normal runtime tracing session; use this in '
             'non-interactive mode along with: start/resume to record snapshot of pre-configured '
             'snapshot session and start/resume normal runtime tracing session, stop/pause to '
             'stop/pause normal runtime tracing session (default: %(default)s)')


def _add_arguments_session_mode(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        '--snapshot-mode', dest='snapshot_mode', action='store_true',
        help='use this to create tracing session in snapshot mode in interactive mode or with '
             'the start verb in non-interactive mode (default: %(default)s)')


def add_arguments(parser: argparse.ArgumentParser) -> None:
    """Add arguments to parser for interactive tracing session configuration."""
    _add_arguments_default_session_name(parser)
    _add_arguments_session_mode(parser)
    _add_arguments_dual_session(parser)
    _add_arguments_configure(parser)


def add_arguments_noninteractive_configure(parser: argparse.ArgumentParser) -> None:
    """Add arguments to parser for non-interactive tracing session configuration."""
    _add_arguments_configure(parser)
    _add_arguments_session_mode(parser)
    add_arguments_noninteractive_control(parser)


def add_arguments_noninteractive_control(parser: argparse.ArgumentParser) -> None:
    """Add arguments to parser for non-interactive tracing session control."""
    parser.add_argument('session_name', help='the name of the tracing session')
    _add_arguments_dual_session(parser)
