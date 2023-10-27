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

"""Implementation of the interface for tracing with LTTng."""

import os
import shlex
import subprocess
from typing import Dict
from typing import List
from typing import Optional
from typing import Set
from typing import Union

from lttngpy import impl as lttngpy
from packaging.version import Version

from .names import DEFAULT_CONTEXT
from .names import DEFAULT_EVENTS_ROS
from .names import DOMAIN_TYPE_KERNEL
from .names import DOMAIN_TYPE_USERSPACE


def get_version() -> Optional[Version]:
    """
    Get version of lttng-ctl.

    :return: the version as a Version object, or `None` if it cannot be extracted
    """
    if not lttngpy.is_available():
        return None
    return Version(lttngpy.LTTNG_CTL_VERSION)


def is_kernel_tracer_available() -> bool:
    """
    Check if the kernel tracer is available.

    This must not be called if `lttngpy.is_available()` is `False`.

    :return: `True` if available or `False` if not
    """
    return not isinstance(lttngpy.get_tracepoints(domain_type=lttngpy.LTTNG_DOMAIN_KERNEL), int)


def get_lttng_home() -> Optional[str]:
    """
    Get the LTTng home value.

    $LTTNG_HOME, or $HOME if unset: https://lttng.org/man/1/lttng/v2.13/#doc-_files

    :return: the LTTng home value
    """
    return os.environ.get('LTTNG_HOME') or os.environ.get('HOME')


def get_session_daemon_pid() -> Optional[int]:
    """
    Get the non-root session daemon PID, if there is one.

    This does not apply to root session daemons.

    :return: the non-root session daemon PID, or `None` if there is none
    """
    # When a non-root session daemon is started, its PID is written to .lttng/lttng-sessiond.pid
    # under the LTTng home
    lttng_home = get_lttng_home()
    # If we can't find the home, then we can just assume that there is no session daemon
    if not lttng_home:
        return None
    # If the file doesn't exist, there is no session daemon
    lttng_sessiond_pid = os.path.join(lttng_home, '.lttng', 'lttng-sessiond.pid')
    if not os.path.isfile(lttng_sessiond_pid):
        return None
    with open(lttng_sessiond_pid, 'r') as f:
        pid = f.read().strip()
        if not pid.isdigit():
            return None
        return int(pid)


def is_session_daemon_unreachable() -> bool:
    """
    Check if the session daemon appears to exist while being unreachable.

    This tries to detect cases of this LTTng issue: https://bugs.lttng.org/issues/1371
    If this issue happens, LTTng will think that the session daemon exists and will happily trace,
    but it will silently not record any trace data, since there is no actual session daemon.
    Therefore, if this returns `True`, then tracing will silently not work.

    TODO(christophebedard) remove this once Rolling uses a version of LTTng with a fix for this bug

    :return: `True` if the session daemon is unreachable, `False` otherwise
    """
    pid = get_session_daemon_pid()
    # If we can't find the PID, then the session daemon really doesn't exist and we can just create
    # one, so it's fine
    if pid is None:
        return False
    # Otherwise, try to look up the process with that PID; if we can't find it, or if it is not
    # lttng-sessiond, then it means that the session daemon is unreachable
    process = subprocess.run(
        shlex.split(f'ps -o comm= {pid}'),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding='utf-8',
    )
    return 1 == process.returncode or 'lttng-sessiond' != process.stdout.strip()


def is_session_daemon_not_alive() -> bool:
    """
    Check if the session daemon isn't alive.

    This must not be called if `lttngpy.is_available()` is `False`.

    :return: `True` if the session daemon is not alive, or `False` if it is alive
    """
    return not lttngpy.is_lttng_session_daemon_alive()


def spawn_session_daemon() -> None:
    """
    Try to spawn a session daemon.

    Raises RuntimeError if lttng-sessiond is not found.
    """
    try:
        subprocess.run(['lttng-sessiond', '--daemonize'])
    except FileNotFoundError:
        raise RuntimeError(
            'cannot find lttng-sessiond: on Ubuntu, install lttng-tools and liblttng-ust-dev')


def setup(
    *,
    session_name: str,
    base_path: str,
    append_trace: bool = False,
    ros_events: Union[List[str], Set[str]] = DEFAULT_EVENTS_ROS,
    kernel_events: Union[List[str], Set[str]] = [],
    context_fields: Union[List[str], Set[str], Dict[str, List[str]]] = DEFAULT_CONTEXT,
    channel_name_ust: str = 'ros2',
    channel_name_kernel: str = 'kchan',
    subbuffer_size_ust: int = 8 * 4096,
    subbuffer_size_kernel: int = 32 * 4096,
) -> Optional[str]:
    """
    Set up LTTng session, with events and context.

    See: https://lttng.org/docs/#doc-core-concepts

    Initialization will fail if the list of kernel events to be
    enabled is not empty and if the kernel tracer is not installed.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure, in which case the tracing session might still exist.

    :param session_name: the name of the session
    :param base_path: the path to the directory in which to create the tracing session directory,
        which will be created if needed
    :param append_trace: whether to append to the trace directory if it already exists, otherwise
        an error is reported
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_fields: the names of context fields to enable
        if it's a list or a set, the context fields are enabled for both kernel and userspace;
        if it's a dictionary: { domain type string -> context fields list }
            with the domain type string being either `names.DOMAIN_TYPE_KERNEL` or
            `names.DOMAIN_TYPE_USERSPACE`
    :param channel_name_ust: the UST channel name
    :param channel_name_kernel: the kernel channel name
    :param subbuffer_size_ust: the size of the subbuffers for userspace events (defaults to 8 times
        the usual page size)
    :param subbuffer_size_kernel: the size of the subbuffers for kernel events (defaults to 32
        times the usual page size, since there can be way more kernel events than UST events)
    :return: the full path to the trace directory, or `None` if initialization failed
    """
    # Validate parameters
    if not session_name:
        raise RuntimeError('empty session name')
    # Resolve full tracing directory path
    full_path = os.path.join(base_path, session_name)
    if os.path.isdir(full_path) and not append_trace:
        raise RuntimeError(
            f'trace directory already exists, use the append option to append to it: {full_path}')

    # If there is no session daemon running, try to spawn one
    if is_session_daemon_not_alive():
        spawn_session_daemon()
    # Error out if it looks like there is a session daemon that we can't actually reach
    if is_session_daemon_unreachable():
        raise RuntimeError(
            'lttng-sessiond seems to exist, but is unreachable. '
            'If using two containers with the same HOME directory, set the LTTNG_HOME environment '
            'variable to the path to a unique directory for each container and make sure that the '
            'directory exists. See: https://bugs.lttng.org/issues/1371'
        )
    # Error out if there is still no session daemon
    if is_session_daemon_not_alive():
        raise RuntimeError('failed to start lttng session daemon')

    # Make sure the kernel tracer is available if there are kernel events
    # Do this after spawning a session daemon, otherwise we can't detect the kernel tracer
    if 0 < len(kernel_events) and not is_kernel_tracer_available():
        raise RuntimeError(
            'kernel tracer is not available:\n'
            '  cannot use kernel events:\n'
            "    'ros2 trace' command: cannot use '-k' option\n"
            "    'Trace' action: cannot set 'events_kernel'/'events-kernel' list\n"
            '  install the kernel tracer, e.g., on Ubuntu, install lttng-modules-dkms\n'
            '  see: https://github.com/ros2/ros2_tracing#building'
        )

    # Convert lists to sets
    if not isinstance(ros_events, set):
        ros_events = set(ros_events)
    if not isinstance(kernel_events, set):
        kernel_events = set(kernel_events)
    if isinstance(context_fields, list):
        context_fields = set(context_fields)

    ust_enabled = ros_events is not None and len(ros_events) > 0
    kernel_enabled = kernel_events is not None and len(kernel_events) > 0
    if not (ust_enabled or kernel_enabled):
        raise RuntimeError('no events enabled')

    # Create session
    # LTTng will create the parent directories if needed
    _create_session(
        session_name=session_name,
        full_path=full_path,
    )

    # Enable channel, events, and contexts for each domain
    contexts_dict = _normalize_contexts_dict(context_fields)
    if ust_enabled:
        domain = DOMAIN_TYPE_USERSPACE
        domain_type = lttngpy.LTTNG_DOMAIN_UST
        channel_name = channel_name_ust
        _enable_channel(
            session_name=session_name,
            domain_type=domain_type,
            # Per-user buffer
            buffer_type=lttngpy.LTTNG_BUFFER_PER_UID,
            channel_name=channel_name,
            # Discard, do not overwrite
            overwrite=0,
            # We use 2 sub-buffers because the number of sub-buffers is pointless in discard mode,
            # and switching between sub-buffers introduces noticeable CPU overhead
            subbuf_size=subbuffer_size_ust,
            num_subbuf=2,
            # Ignore switch timer interval and use read timer instead
            switch_timer_interval=0,
            read_timer_interval=200,
            # mmap channel output (only option for UST)
            output=lttngpy.LTTNG_EVENT_MMAP,
        )
        _enable_events(
            session_name=session_name,
            domain_type=domain_type,
            channel_name=channel_name,
            events=ros_events,
        )
        _add_contexts(
            session_name=session_name,
            domain_type=domain_type,
            channel_name=channel_name,
            context_fields=contexts_dict.get(domain),
        )
    if kernel_enabled:
        domain = DOMAIN_TYPE_KERNEL
        domain_type = lttngpy.LTTNG_DOMAIN_KERNEL
        channel_name = channel_name_kernel
        _enable_channel(
            session_name=session_name,
            domain_type=domain_type,
            # Global buffer (only option for kernel domain)
            buffer_type=lttngpy.LTTNG_BUFFER_GLOBAL,
            channel_name=channel_name,
            # Discard, do not overwrite
            overwrite=0,
            # We use 2 sub-buffers because the number of sub-buffers is pointless in discard mode,
            # and switching between sub-buffers introduces noticeable CPU overhead
            subbuf_size=subbuffer_size_kernel,
            num_subbuf=2,
            # Ignore switch timer interval and use read timer instead
            switch_timer_interval=0,
            read_timer_interval=200,
            # mmap channel output instead of splice
            output=lttngpy.LTTNG_EVENT_MMAP,
        )
        _enable_events(
            session_name=session_name,
            domain_type=domain_type,
            channel_name=channel_name,
            events=kernel_events,
        )
        _add_contexts(
            session_name=session_name,
            domain_type=domain_type,
            channel_name=channel_name,
            context_fields=contexts_dict.get(domain),
        )

    return full_path


def start(
    *,
    session_name: str,
    **kwargs,
) -> None:
    """
    Start LTTng session, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure to start.

    :param session_name: the name of the session
    """
    result = lttngpy.lttng_start_tracing(session_name=session_name)
    if result < 0:
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(f"failed to start tracing session '{session_name}': {error}")


def stop(
    *,
    session_name: str,
    ignore_error: bool = False,
    **kwargs,
) -> None:
    """
    Stop LTTng session, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure to stop, unless ignored.

    :param session_name: the name of the session
    :param ignore_error: whether to ignore any error when stopping
    """
    result = lttngpy.lttng_stop_tracing(session_name=session_name)
    if result < 0 and not ignore_error:
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(f"failed to stop tracing session '{session_name}': {error}")


def destroy(
    *,
    session_name: str,
    ignore_error: bool = False,
    **kwargs,
) -> None:
    """
    Destroy LTTng session, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure to destroy, unless ignored.

    :param session_name: the name of the session
    :param ignore_error: whether to ignore any error when destroying
    """
    result = lttngpy.lttng_destroy_session(session_name=session_name)
    if result < 0 and not ignore_error:
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(f"failed to destroy tracing session '{session_name}': {error}")


def _create_session(
    *,
    session_name: str,
    full_path: str,
) -> None:
    """
    Create session from name and full directory path, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure.

    :param session_name: the name of the session
    :param full_path: the full path to the main directory to write trace data to
    """
    result = lttngpy.lttng_create_session(
        session_name=session_name,
        url=full_path,
    )
    if -lttngpy.LTTNG_ERR_EXIST_SESS.value == result:
        # Sessions may persist if there was an error previously, so if it already exists, just
        # destroy it and try again
        destroy(session_name=session_name)
        result = lttngpy.lttng_create_session(
            session_name=session_name,
            url=full_path,
        )
    if result < 0:
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(f"failed to create tracing session '{session_name}': {error}")


def _enable_channel(**kwargs) -> None:
    """
    Enable channel, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure.

    See `lttngpy.enable_channel` for kwargs.
    """
    result = lttngpy.enable_channel(**kwargs)
    if result < 0:
        session_name = kwargs['session_name']
        channel_name = kwargs['channel_name']
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(
            f"failed to enable channel '{channel_name}' "
            f"in tracing session '{session_name}': {error}"
        )


def _enable_events(**kwargs) -> None:
    """
    Enable events for a given channel name, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure.

    See `lttngpy.enable_events` for kwargs.
    """
    result = lttngpy.enable_events(**kwargs)
    if result < 0:
        session_name = kwargs['session_name']
        channel_name = kwargs['channel_name']
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(
            f"failed to enable event for channel '{channel_name}' "
            f"in tracing session '{session_name}': {error}"
        )


def _normalize_contexts_dict(
    context_fields: Union[Set[str], Dict[str, List[str]]],
) -> Dict[str, Set[str]]:
    """
    Normalize context set/dict to dict.

    :param context_fields: the names of context fields to enable
        if it's a set, the context fields are enabled for both kernel and userspace;
        if it's a dictionary: { domain type string -> context fields list }
            with the domain type string being either `names.DOMAIN_TYPE_KERNEL` or
            `names.DOMAIN_TYPE_USERSPACE`
    :return: a dictionary of domain type name to list of context field names
    """
    DOMAIN_TYPES = {DOMAIN_TYPE_USERSPACE, DOMAIN_TYPE_KERNEL}
    if isinstance(context_fields, dict):
        unknown_domain_types = context_fields.keys() - DOMAIN_TYPES
        if unknown_domain_types:
            raise RuntimeError(f'unknown context domain type(s): {unknown_domain_types}')
        return {domain: set(field_names) for domain, field_names in context_fields.items()}
    assert isinstance(context_fields, set)
    return {domain_type: context_fields for domain_type in DOMAIN_TYPES}


def _add_contexts(**kwargs) -> None:
    """
    Add context lists to given channel, and check for errors.

    This must not be called if `lttngpy.is_available()` is `False`.
    Raises RuntimeError on failure.

    See `lttngpy.add_contexts` for kwargs.
    """
    result = lttngpy.add_contexts(**kwargs)
    if result < 0:
        session_name = kwargs['session_name']
        channel_name = kwargs['channel_name']
        domain_type = kwargs['domain_type']
        error = lttngpy.lttng_strerror(result)
        raise RuntimeError(
            f"failed to add context fields for channel '{channel_name}' "
            f"and domain '{domain_type}' in tracing session '{session_name}': {error}"
        )
