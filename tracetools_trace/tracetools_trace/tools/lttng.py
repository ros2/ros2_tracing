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

"""Interface for tracing with LTTng."""

from typing import List

try:
    import lttng

    from .lttng_impl import _lttng_destroy
    from .lttng_impl import _lttng_setup
    from .lttng_impl import _lttng_start
    from .lttng_impl import _lttng_stop
except ImportError:
    lttng = None

    def _lttng_destroy(*args, **kwargs) -> None:
        pass

    def _lttng_setup(*args, **kwargs) -> None:
        pass

    def _lttng_start(*args, **kwargs) -> None:
        pass

    def _lttng_stop(*args, **kwargs) -> None:
        pass

from .names import DEFAULT_CONTEXT
from .names import DEFAULT_EVENTS_KERNEL
from .names import DEFAULT_EVENTS_ROS
from .path import DEFAULT_BASE_PATH


def lttng_init(
    session_name: str,
    base_path: str = DEFAULT_BASE_PATH,
    ros_events: List[str] = DEFAULT_EVENTS_ROS,
    kernel_events: List[str] = DEFAULT_EVENTS_KERNEL,
    context_names: List[str] = DEFAULT_CONTEXT
) -> None:
    """
    Set up and start LTTng session.

    :param session_name: the name of the session
    :param base_path: the path to the directory in which to create the tracing session directory
    :param ros_events: list of ROS events to enable
    :param kernel_events: list of kernel events to enable
    :param context_names: list of context elements to enable
    """
    if lttng is None:
        return None
    _lttng_setup(session_name, base_path, ros_events, kernel_events, context_names)
    _lttng_start(session_name)


def lttng_fini(session_name: str) -> None:
    """
    Stop and destroy LTTng session.

    :param session_name: the name of the session
    """
    if lttng is None:
        return None
    _lttng_stop(session_name)
    _lttng_destroy(session_name)
