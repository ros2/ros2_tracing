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

"""Implementation of the interface for tracing with LTTng."""

from typing import List

import lttng

from .names import DEFAULT_CONTEXT
from .names import DEFAULT_EVENTS_KERNEL
from .names import DEFAULT_EVENTS_ROS
from .path import DEFAULT_BASE_PATH
from .path import get_full_session_path


def setup(
    session_name: str,
    base_path: str = DEFAULT_BASE_PATH,
    ros_events: List[str] = DEFAULT_EVENTS_ROS,
    kernel_events: List[str] = DEFAULT_EVENTS_KERNEL,
    context_names: List[str] = DEFAULT_CONTEXT,
    channel_name_ust: str = 'ros2',
    channel_name_kernel: str = 'kchan',
) -> None:
    """
    Set up LTTng session, with events and context.

    See: https://lttng.org/docs/#doc-core-concepts

    :param session_name: the name of the session
    :param base_path: the path to the directory in which to create the tracing session directory
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_names: list of context elements to enable
    :param channel_name_ust: the UST channel name
    :param channel_name_kernel: the kernel channel name
    """
    # Resolve full tracing directory path
    full_path = get_full_session_path(session_name, base_path=base_path)

    ust_enabled = ros_events is not None and len(ros_events) > 0
    kernel_enabled = kernel_events is not None and len(kernel_events) > 0

    # Domains
    if ust_enabled:
        domain_ust = lttng.Domain()
        domain_ust.type = lttng.DOMAIN_UST
        # Per-user buffer
        domain_ust.buf_type = lttng.BUFFER_PER_UID
        channel_ust = lttng.Channel()
        channel_ust.name = channel_name_ust
        # Discard, do not overwrite
        channel_ust.attr.overwrite = 0
        # 8 sub-buffers of 2 times the usual page size
        channel_ust.attr.subbuf_size = 2 * 4096
        channel_ust.attr.num_subbuf = 8
        # Ignore switch timer interval and use read timer instead
        channel_ust.attr.switch_timer_interval = 0
        channel_ust.attr.read_timer_interval = 200
        # mmap channel output instead of splice
        channel_ust.attr.output = lttng.EVENT_MMAP
        events_list_ust = _create_events(ros_events)
    if kernel_enabled:
        domain_kernel = lttng.Domain()
        domain_kernel.type = lttng.DOMAIN_KERNEL
        # Global buffer (only option for kernel domain)
        domain_kernel.buf_type = lttng.BUFFER_GLOBAL
        channel_kernel = lttng.Channel()
        channel_kernel.name = channel_name_kernel
        # Discard, do not overwrite
        channel_kernel.attr.overwrite = 0
        # 8 sub-buffers of 8 times the usual page size, since
        # there can be way more kernel events than UST events
        channel_kernel.attr.subbuf_size = 8 * 4096
        channel_kernel.attr.num_subbuf = 8
        # Ignore switch timer interval and use read timer instead
        channel_kernel.attr.switch_timer_interval = 0
        channel_kernel.attr.read_timer_interval = 200
        # mmap channel output instead of splice
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


def start(session_name: str) -> None:
    """
    Start LTTng session, and check for errors.

    :param session_name: the name of the session
    """
    result = lttng.start(session_name)
    if result < 0:
        raise RuntimeError(f'failed to start tracing: {lttng.strerror(result)}')


def stop(session_name: str) -> None:
    """
    Stop LTTng session, and check for errors.

    :param session_name: the name of the session
    """
    result = lttng.stop(session_name)
    if result < 0:
        raise RuntimeError(f'failed to stop tracing: {lttng.strerror(result)}')


def destroy(session_name: str) -> None:
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
        destroy(session_name)
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
