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

# LTTng tracing interface

import sys

# Temporary workaround
sys.path = ['/usr/local/lib/python3.6/site-packages'] + sys.path
from typing import List

import lttng  # noqa: E402

from .names import (  # noqa: E402
    DEFAULT_CONTEXT,
    DEFAULT_EVENTS_KERNEL,
    DEFAULT_EVENTS_ROS,
)


def lttng_init(
    session_name: str,
    full_path: str,
    ros_events: List[str] = DEFAULT_EVENTS_ROS,
    kernel_events: List[str] = DEFAULT_EVENTS_KERNEL,
    context_names: List[str] = DEFAULT_CONTEXT
) -> None:
    """
    Set up and start LTTng session.

    :param session_name: the name of the session
    :param full_path: the full path to the main directory to write trace data to
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_names: list of context elements to enable
    """
    _lttng_setup(session_name, full_path, ros_events, kernel_events, context_names)
    _lttng_start(session_name)


def lttng_fini(session_name: str) -> None:
    """
    Stop and destroy LTTng session.

    :param session_name: the name of the session
    """
    _lttng_stop(session_name)
    _lttng_destroy(session_name)


def _lttng_setup(
        session_name: str,
        full_path: str,
        ros_events: List[str] = DEFAULT_EVENTS_ROS,
        kernel_events: List[str] = DEFAULT_EVENTS_KERNEL,
        context_names: List[str] = DEFAULT_CONTEXT
    ) -> None:
    """
    Set up LTTng session, with events and context.

    :param session_name: the name of the session
    :param full_path: the full path to the main directory to write trace data to
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_names: list of context elements to enable
    """
    ust_enabled = ros_events is not None and len(ros_events) > 0
    kernel_enabled = kernel_events is not None and len(kernel_events) > 0

    # Domains
    if ust_enabled:
        domain_ust = lttng.Domain()
        domain_ust.type = lttng.DOMAIN_UST
        domain_ust.buf_type = lttng.BUFFER_PER_UID
        channel_ust = lttng.Channel()
        channel_ust.name = 'ros2'
        channel_ust.attr.overwrite = 0
        channel_ust.attr.subbuf_size = 2 * 4096
        channel_ust.attr.num_subbuf = 8
        channel_ust.attr.switch_timer_interval = 0
        channel_ust.attr.read_timer_interval = 200
        channel_ust.attr.output = lttng.EVENT_MMAP
        events_list_ust = _create_events(ros_events)
    if kernel_enabled:
        domain_kernel = lttng.Domain()
        domain_kernel.type = lttng.DOMAIN_KERNEL
        domain_kernel.buf_type = lttng.BUFFER_GLOBAL
        channel_kernel = lttng.Channel()
        channel_kernel.name = 'kchan'
        channel_kernel.attr.overwrite = 0
        channel_kernel.attr.subbuf_size = 8 * 4096
        channel_kernel.attr.num_subbuf = 8
        channel_kernel.attr.switch_timer_interval = 0
        channel_kernel.attr.read_timer_interval = 200
        channel_kernel.attr.output = lttng.EVENT_MMAP
        events_list_kernel = _create_events(kernel_events)

    # Session
    _create_session(session_name, full_path)

    # Handles, channels, events
    handle_ust = None
    if ust_enabled:
        handle_ust = _create_handle(session_name, domain_ust)
        _enable_channel(handle_ust, channel_ust)
        _enable_events(handle_ust, events_list_ust, channel_ust.name)
    handle_kernel = None
    if kernel_enabled:
        handle_kernel = _create_handle(session_name, domain_kernel)
        _enable_channel(handle_kernel, channel_kernel)
        _enable_events(handle_kernel, events_list_kernel, channel_kernel.name)

    # Context
    context_list = _create_context_list(context_names)
    enabled_handles = [h for h in [handle_ust, handle_kernel] if h is not None]
    _add_context(enabled_handles, context_list)


def _lttng_start(session_name: str) -> None:
    """
    Start LTTng session, and check for errors.

    :param session_name: the name of the session
    """
    result = lttng.start(session_name)
    if result < 0:
        raise RuntimeError(f'failed to start tracing: {lttng.strerror(result)}')


def _lttng_stop(session_name: str) -> None:
    """
    Stop LTTng session, and check for errors.

    :param session_name: the name of the session
    """
    result = lttng.stop(session_name)
    if result < 0:
        raise RuntimeError(f'failed to stop tracing: {lttng.strerror(result)}')


def _lttng_destroy(session_name: str) -> None:
    """
    Destroy LTTng session, and check for errors.

    :param session_name: the name of the session
    """
    result = lttng.destroy(session_name)
    if result < 0:
        raise RuntimeError(f'failed to destroy tracing session: {lttng.strerror(result)}')


def _create_events(event_names_list: List[str]) -> List[lttng.Event]:
    """
    Create events list from names.

    :param event_names_list: a list of names to create events for
    :return: the list of events
    """
    events_list = []
    for event_name in event_names_list:
        e = lttng.Event()
        e.name = event_name
        e.type = lttng.EVENT_TRACEPOINT
        e.loglevel_type = lttng.EVENT_LOGLEVEL_ALL
        events_list.append(e)
    return events_list


def _create_session(session_name: str, full_path: str) -> None:
    """
    Create session from name and full directory path, and check for errors.

    :param session_name: the name of the session
    :param full_path: the full path to the main directory to write trace data to
    """
    result = lttng.create(session_name, full_path)
    LTTNG_ERR_EXIST_SESS = 28
    if result == -LTTNG_ERR_EXIST_SESS:
        # Sessions seem to persist, so if it already exists,
        # just destroy it and try again
        lttng_destroy(session_name)
        result = lttng.create(session_name, full_path)
    if result < 0:
        raise RuntimeError(f'session creation failed: {lttng.strerror(result)}')


def _create_handle(session_name: str, domain: lttng.Domain) -> lttng.Handle:
    """
    Create a handle for a given session name and a domain, and check for errors.

    :param session_name: the name of the session
    :param domain: the domain to be used
    :return: the handle
    """
    handle = None
    handle = lttng.Handle(session_name, domain)
    if handle is None:
        raise RuntimeError('handle creation failed')
    return handle


def _enable_channel(handle: lttng.Handle, channel: lttng.Channel) -> None:
    """
    Enable channel for a handle, and check for errors.

    :param handle: the handle to be used
    :param channel: the channel to enable
    """
    result = lttng.enable_channel(handle, channel)
    if result < 0:
        raise RuntimeError(f'channel enabling failed: {lttng.strerror(result)}')


def _enable_events(
    handle: lttng.Handle,
    events_list: List[lttng.Event],
    channel_name: str,
) -> None:
    """
    Enable events list for a given handle and channel name, and check for errors.

    :param handle: the handle to be used
    :param events_list: the list of events to enable
    :param channel_name: the name of the channel to associate
    """
    for event in events_list:
        result = lttng.enable_event(handle, event, channel_name)
        if result < 0:
            raise RuntimeError(f'event enabling failed: {lttng.strerror(result)}')


context_map = {
    'procname': lttng.EVENT_CONTEXT_PROCNAME,
    'pid': lttng.EVENT_CONTEXT_PID,
    'vpid': lttng.EVENT_CONTEXT_VPID,
    'vtid': lttng.EVENT_CONTEXT_VTID,
}


def _context_name_to_type(context_name: str) -> int:
    """
    Convert from context name to LTTng enum/constant type.

    :param context_name: the generic name for the context
    :return: the associated type
    """
    return context_map.get(context_name)


def _create_context_list(context_names_list: List[str]) -> List[lttng.EventContext]:
    """
    Create context list from names, and check for errors.

    :param context_names_list: the list of context names
    :return: the event context list
    """
    context_list = []
    for c in context_names_list:
        ec = lttng.EventContext()
        context_type = _context_name_to_type(c)
        if context_type is not None:
            ec.ctx = context_type
            context_list.append(ec)
    return context_list


def _add_context(
    handles: List[lttng.Handle],
    context_list: List[lttng.EventContext],
) -> None:
    """
    Add context list to given handles, and check for errors.

    :param handles: the list of handles for which to add context
    :param context_list: the list of event contexts to add to the handles
    """
    for handle in handles:
        for contex in context_list:
            result = lttng.add_context(handle, contex, None, None)
            if result < 0:
                raise RuntimeError(f'failed to add context: {lttng.strerror(result)}')
