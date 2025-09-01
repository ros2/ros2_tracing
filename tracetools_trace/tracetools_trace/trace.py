#!/usr/bin/env python3
# Copyright 2019 Robert Bosch GmbH
# Copyright 2021 Christophe Bedard
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

"""Entrypoint/script to setup and start an LTTng tracing session."""

import argparse
import os
import sys
from typing import Callable
from typing import List
from typing import Optional
from typing import Tuple

from tracetools_trace.tools import args
from tracetools_trace.tools import lttng
from tracetools_trace.tools import path
from tracetools_trace.tools import print_names_list
from tracetools_trace.tools import signals


def _assert_lttng_installed() -> None:
    if not lttng.is_lttng_installed():
        sys.exit(2)


def _display_info(
    *,
    ros_events: List[str],
    kernel_events: List[str],
    syscalls: List[str],
    context_fields: List[str],
    display_list: bool,
) -> None:
    if ros_events:
        event_str = 'events' if len(ros_events) > 1 else 'event'
        print(f'userspace tracing enabled ({len(ros_events)} {event_str})')
        if display_list:
            print_names_list(ros_events)
    else:
        print('userspace tracing disabled')
    if kernel_events:
        event_str = 'events' if len(kernel_events) > 1 else 'event'
        print(f'kernel tracing enabled ({len(kernel_events)} {event_str})')
        if display_list:
            print_names_list(kernel_events)
    else:
        print('kernel tracing disabled')
    if syscalls:
        syscall_str = 'syscalls' if len(syscalls) > 1 else 'syscall'
        print(f'syscall tracing enabled ({len(syscalls)} {syscall_str})')
        if display_list:
            print_names_list(syscalls)
    else:
        print('syscalls tracing disabled')
    if len(context_fields) > 0:
        field_str = 'fields' if len(context_fields) > 1 else 'field'
        print(f'context ({len(context_fields)} {field_str})')
        if display_list:
            print_names_list(context_fields)


def _resolve_session_path(
    *,
    session_name: str,
    base_path: Optional[str],
) -> Tuple[str, str]:
    if not base_path:
        base_path = path.get_tracing_directory()
    full_session_path = os.path.join(base_path, session_name)
    print(f'writing tracing session to: {full_session_path}')
    return base_path, full_session_path


def _resolve_runtime_session_path(
    *,
    session_name: str,
    base_path: Optional[str],
) -> Tuple[str, str]:
    if not base_path:
        base_path = path.get_tracing_directory()
    full_session_path = os.path.join(base_path, session_name, 'runtime')
    print(f'writing runtime tracing session to: {full_session_path}')
    return base_path, full_session_path


def _display_snapshot_session_path(
    *,
    session_name: str,
    base_path: Optional[str],
) -> None:
    if not base_path:
        base_path = path.get_tracing_directory()
    full_session_path = os.path.join(base_path, session_name, 'snapshot')
    print(f'writing snapshot tracing session to: {full_session_path}')


def init(
    *,
    session_name: str,
    snapshot_mode: bool,
    dual_session: bool,
    base_path: Optional[str],
    append_trace: bool,
    ros_events: List[str],
    kernel_events: List[str],
    syscalls: List[str],
    context_fields: List[str],
    display_list: bool,
    interactive: bool,
) -> bool:
    """
    Init and start tracing.

    Can be interactive by requiring user interaction to start tracing. If non-interactive, tracing
    starts right away.

    Raises RuntimeError on failure, in which case the tracing session might still exist.

    :param session_name: the name of the session
    :param snapshot_mode: whether this is a snapshot session
    :param dual_session: whether this is part of a dual session
    :param base_path: the path to the directory in which to create the tracing session directory,
        or `None` for default
    :param append_trace: whether to append to the trace directory if it already exists, otherwise
        an error is reported
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param syscalls: list of syscalls to enable
    :param context_fields: list of context fields to enable
    :param display_list: whether to display list(s) of enabled events and context names
    :param interactive: whether to require user interaction to start tracing
    :return: True if successful, False otherwise
    """
    _display_info(
        ros_events=ros_events,
        kernel_events=kernel_events,
        syscalls=syscalls,
        context_fields=context_fields,
        display_list=display_list,
    )

    if dual_session:
        _display_snapshot_session_path(session_name=session_name, base_path=base_path)
        base_path, full_runtime_session_path = _resolve_runtime_session_path(
            session_name=session_name,
            base_path=base_path,
        )

        snapshot_session_name = session_name + path.SNAPSHOT_SESSION_SUFFIX
        runtime_session_name = session_name + path.RUNTIME_SESSION_SUFFIX

        if interactive:
            input('press enter to start tracing...')
        lttng.lttng_record_snapshot(session_name=snapshot_session_name)

        # For runtime sessions, use the runtime session name
        # and path so lttng_init() can be called uniformly
        full_session_path = full_runtime_session_path
        session_name = runtime_session_name
    else:
        base_path, full_session_path = _resolve_session_path(
            session_name=session_name,
            base_path=base_path,
        )
        if interactive:
            input('press enter to start...')

    trace_directory = lttng.lttng_init(
        session_name=session_name,
        snapshot_mode=snapshot_mode,
        dual_session=dual_session,
        base_path=base_path,
        append_trace=append_trace,
        ros_events=ros_events,
        kernel_events=kernel_events,
        syscalls=syscalls,
        context_fields=context_fields,
    )
    if trace_directory is None:
        return False
    # Simple sanity check
    assert trace_directory == full_session_path
    return True


def fini(
    *,
    session_name: str,
    dual_session: bool = False,
) -> None:
    """
    Stop and finalize tracing.

    Needs user interaction to stop tracing. Stops tracing automatically on SIGINT.

    :param session_name: the name of the session
    """
    def _run() -> None:
        input('press enter to stop...')

    def _fini() -> None:
        print('stopping & destroying tracing session')
        lttng.lttng_fini(
            session_name=session_name + (path.RUNTIME_SESSION_SUFFIX if dual_session else '')
        )

    signals.execute_and_handle_sigint(_run, _fini)


def cleanup(
    *,
    session_name: str,
    dual_session: bool = False,
) -> None:
    """
    Clean up and remove tracing session if it exists.

    :param session_name: the name of the session
    """
    if dual_session:
        session_name += path.RUNTIME_SESSION_SUFFIX
    lttng.lttng_fini(session_name=session_name, ignore_error=True)


def _do_work_and_report_error(
    work: Callable[[], int],
    session_name: str,
    dual_session: bool = False,
    *,
    do_cleanup: bool,
) -> int:
    """
    Perform some work, reporting any error and cleaning up.

    This will call the work function and catch `RuntimeError`, in which case the error will be
    printed, and the session will be cleaned up if needed.

    :param work: the work function to be called which may raise `RuntimeError`
    :param session_name: the session name
    :param do_cleanup: whether to clean the session up on error
    :return: the return code of the work function, or 1 if an error was reported
    """
    _assert_lttng_installed()
    try:
        return work()
    except RuntimeError as e:
        print(f'error: {str(e)}', file=sys.stderr)
        if do_cleanup:
            cleanup(session_name=session_name, dual_session=dual_session)
        return 1


def trace(args: argparse.Namespace) -> int:
    """
    Trace.

    Needs user interaction to start tracing and then stop tracing.

    On failure, the tracing session will not exist.

    :param args: the arguments parsed using `tracetools_trace.tools.args.add_arguments`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        if not init(
            session_name=args.session_name,
            snapshot_mode=args.snapshot_mode,
            dual_session=args.dual_session,
            base_path=args.path,
            append_trace=args.append_trace,
            ros_events=args.events_ust,
            kernel_events=args.events_kernel,
            syscalls=args.syscalls,
            context_fields=args.context_fields,
            display_list=args.list,
            interactive=True,
        ):
            return 1
        fini(session_name=args.session_name, dual_session=args.dual_session)
        return 0
    return _do_work_and_report_error(
        work,
        args.session_name,
        args.dual_session,
        do_cleanup=True,
    )


def start(args: argparse.Namespace) -> int:
    """
    Configure tracing session and start tracing.

    On failure, the tracing session will not exist.

    :param args: the arguments parsed using
        `tracetools_trace.tools.args.add_arguments_noninteractive_configure`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        return int(
            not init(
                session_name=args.session_name,
                snapshot_mode=args.snapshot_mode,
                dual_session=args.dual_session,
                base_path=args.path,
                append_trace=args.append_trace,
                ros_events=args.events_ust,
                kernel_events=args.events_kernel,
                syscalls=args.syscalls,
                context_fields=args.context_fields,
                display_list=args.list,
                interactive=False,
            )
        )
    return _do_work_and_report_error(
        work,
        args.session_name,
        args.dual_session,
        do_cleanup=True,
    )


def stop(args: argparse.Namespace) -> int:
    """
    Stop tracing.

    On failure, the tracing session might still exist.

    :param args: the arguments parsed using
        `tracetools_trace.tools.args.add_arguments_noninteractive_control`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        session_name = args.session_name
        if args.dual_session:
            session_name += path.RUNTIME_SESSION_SUFFIX
        lttng.lttng_fini(session_name=session_name)
        return 0
    return _do_work_and_report_error(work, args.session_name, do_cleanup=False)


def pause(args: argparse.Namespace) -> int:
    """
    Pause tracing after starting or resuming.

    On failure, the tracing session might still exist.

    :param args: the arguments parsed using
        `tracetools_trace.tools.args.add_arguments_noninteractive_control`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        session_name = args.session_name
        if args.dual_session:
            session_name += path.RUNTIME_SESSION_SUFFIX
        lttng.lttng_stop(session_name=session_name)
        return 0
    return _do_work_and_report_error(work, args.session_name, do_cleanup=False)


def resume(args: argparse.Namespace) -> int:
    """
    Resume tracing after pausing.

    On failure, the tracing session might still exist.

    :param args: the arguments parsed using
        `tracetools_trace.tools.args.add_arguments_noninteractive_control`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        session_name = args.session_name
        if args.dual_session:
            # Trigger snapshot to record any init events missed while paused
            snapshot_session_name = session_name + path.SNAPSHOT_SESSION_SUFFIX
            lttng.lttng_record_snapshot(session_name=snapshot_session_name)
            session_name += path.RUNTIME_SESSION_SUFFIX
        lttng.lttng_start(session_name=session_name)
        return 0
    return _do_work_and_report_error(work, args.session_name, do_cleanup=False)


def record_snapshot(args: argparse.Namespace) -> int:
    """
    Record snapshot of a tracing session (created in snapshot mode).

    On failure, the tracing session might still exist.

    :param args: the arguments parsed using
        `tracetools_trace.tools.args.add_arguments_noninteractive_control`
    :return: the return code (0 if successful, 1 otherwise)
    """
    def work() -> int:
        session_name = args.session_name
        if args.dual_session:
            session_name += path.SNAPSHOT_SESSION_SUFFIX
        lttng.lttng_record_snapshot(session_name=session_name)
        return 0
    return _do_work_and_report_error(work, args.session_name, do_cleanup=False)


def main() -> int:
    return trace(args.parse_args())
