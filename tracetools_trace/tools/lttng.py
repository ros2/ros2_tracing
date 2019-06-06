# LTTng tracing interface

# Temporary workaround
import sys
sys.path = ['/usr/local/lib/python3.6/site-packages'] + sys.path

from lttng import *
from .names import DEFAULT_EVENTS_ROS, DEFAULT_EVENTS_KERNEL, DEFAULT_CONTEXT

def lttng_setup(session_name, directory, ros_events=DEFAULT_EVENTS_ROS, kernel_events=DEFAULT_EVENTS_KERNEL, context_names=DEFAULT_CONTEXT):
    """
    Setup LTTng session, with events and context
    :param session_name (str): the name of the session
    :param directory (str): the path of the main directory to write trace data to
    :param ros_events (list(str)): list of ROS events to enable
    :param kernel_events (list(str)): list of kernel events to enable
    :param context_names (list(str)): list of context elements to enable
    """
    ust_enabled = ros_events is not None and len(ros_events) > 0
    kernel_enabled = kernel_events is not None and len(kernel_events) > 0
    print(f'UST tracing {f"enabled ({len(ros_events)} events)" if ust_enabled else "disabled"}')
    print(f'kernel tracing {f"enabled ({len(kernel_events)} events)" if kernel_enabled else "disabled"}')

    # Domains
    if ust_enabled:
        domain_ust = Domain()
        domain_ust.type = DOMAIN_UST
        domain_ust.buf_type = BUFFER_PER_UID
        channel_ust = Channel()
        channel_ust.name = 'ros2'
        channel_ust.attr.overwrite = 0
        channel_ust.attr.subbuf_size = 2*4096
        channel_ust.attr.num_subbuf = 8
        channel_ust.attr.switch_timer_interval = 0
        channel_ust.attr.read_timer_interval = 200
        channel_ust.attr.output = EVENT_MMAP
        events_list_ust = _create_events(ros_events)
    if kernel_enabled:
        domain_kernel = Domain()
        domain_kernel.type = DOMAIN_KERNEL
        domain_kernel.buf_type = BUFFER_GLOBAL
        channel_kernel = Channel()
        channel_kernel.name = 'kchan'
        channel_kernel.attr.overwrite = 0
        channel_kernel.attr.subbuf_size = 8*4096
        channel_kernel.attr.num_subbuf = 8
        channel_kernel.attr.switch_timer_interval = 0
        channel_kernel.attr.read_timer_interval = 200
        channel_kernel.attr.output = EVENT_MMAP
        events_list_kernel = _create_events(kernel_events)

    # Session
    _create_session(session_name, directory)

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

def lttng_start(session_name):
    """
    Start LTTng session, and check for errors
    """
    result = start(session_name)
    if result < 0:
        raise RuntimeError(f'failed to start tracing: {strerror(result)}')

def lttng_stop(session_name):
    """
    Stop LTTng session, and check for errors
    """
    result = stop(session_name)
    if result < 0:
        raise RuntimeError(f'failed to stop tracing: {strerror(result)}')

def lttng_destroy(session_name):
    """
    Destroy LTTng session, and check for errors
    """
    result = destroy(session_name)
    if result < 0:
        raise RuntimeError(f'failed to destroy tracing session: {strerror(result)}')

def _create_events(event_names_list):
    """
    Create events list from names
    """
    events_list = []
    for event_name in event_names_list:
        e = Event()
        e.name = event_name
        e.type = EVENT_TRACEPOINT
        e.loglevel_type = EVENT_LOGLEVEL_ALL
        events_list.append(e)
    return events_list

def _create_session(session_name, directory):
    """
    Create session from name and directory path, and check for errors
    """
    result = create(session_name, directory)
    LTTNG_ERR_EXIST_SESS = 28
    if result == -LTTNG_ERR_EXIST_SESS:
        # Sessions seem to persist, so if it already exists,
        # just destroy it and try again
        lttng_destroy(session_name)
        result = create(session_name, directory)
    if result < 0:
        raise RuntimeError(f'session creation failed: {strerror(result)}')

def _create_handle(session_name, domain):
    """
    Create a handle for a given session name and a domain, and check for errors
    """
    handle = None
    handle = Handle(session_name, domain)
    if handle is None:
        raise RuntimeError('handle creation failed')
    return handle

def _enable_channel(handle, channel):
    """
    Enable channel for a handle, and check for errors
    """
    result = enable_channel(handle, channel)
    if result < 0:
        raise RuntimeError(f'channel enabling failed: {strerror(result)}')

def _enable_events(handle, events_list, channel_name):
    """
    Enable events list for a given handle and channel name, and check for errors
    """
    for event in events_list:
        result = enable_event(handle, event, channel_name)
        if result < 0:
            raise RuntimeError(f'event enabling failed: {strerror(result)}')

context_map = {
    'procname': EVENT_CONTEXT_PROCNAME,
    'pid': EVENT_CONTEXT_PID,
    'vpid': EVENT_CONTEXT_VPID,
    'vtid': EVENT_CONTEXT_VTID,
}
def _context_name_to_type(context_name):
    """
    Convert from context name to LTTng enum/constant type
    """
    return context_map.get(context_name)

def _create_context_list(context_names_list):
    """
    Create context list from names, and check for errors
    """
    context_list = []
    for c in context_names_list:
        ec = EventContext()
        context_type = _context_name_to_type(c)
        if context_type is not None:
            ec.ctx = context_type
            context_list.append(ec)
    return context_list

def _add_context(handles, context_list):
    """
    Add context list to given handles, and check for errors
    """
    for handle in handles:
        for contex in context_list:
            result = add_context(handle, contex, None, None)
            if result < 0:
                raise RuntimeError(f'failed to add context: {strerror(result)}')
