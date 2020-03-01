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

"""Module for the Trace action."""

import re
from typing import List
from typing import Optional

from launch import logging
from launch.action import Action
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from tracetools_trace.tools import lttng
from tracetools_trace.tools import names
from tracetools_trace.tools import path

from .actions.ld_preload import LdPreload


class Trace(Action):
    """
    Tracing action for launch.

    Sets up and enables tracing through a launch file description.
    """

    LIB_PROFILE_NORMAL = 'liblttng-ust-cyg-profile.so'
    LIB_PROFILE_FAST = 'liblttng-ust-cyg-profile-fast.so'
    LIB_MEMORY_UST = 'liblttng-ust-libc-wrapper.so'

    PROFILE_EVENT_PATTERN = '^lttng_ust_cyg_profile.*:func_.*'
    MEMORY_UST_EVENT_PATTERN = '^lttng_ust_libc:.*'

    def __init__(
        self,
        *,
        session_name: str,
        append_timestamp: bool = False,
        base_path: str = path.DEFAULT_BASE_PATH,
        events_ust: List[str] = names.DEFAULT_EVENTS_ROS,
        events_kernel: List[str] = names.DEFAULT_EVENTS_KERNEL,
        context_names: List[str] = names.DEFAULT_CONTEXT,
        profile_fast: bool = True,
        **kwargs,
    ) -> None:
        """
        Create a Trace.

        :param session_name: the name of the tracing session
        :param append_timestamp: whether to append timestamp to the session name
        :param base_path: the path to the base directory in which to create the session directory
        :param events_ust: the list of ROS UST events to enable
        :param events_kernel: the list of kernel events to enable
        :param context_names: the list of context names to enable
        :param profile_fast: `True` to use fast profiling, `False` for normal (only if necessary)
        """
        super().__init__(**kwargs)
        if append_timestamp:
            session_name = path.append_timestamp(session_name)
        self.__session_name = session_name
        self.__base_path = base_path
        self.__events_ust = events_ust
        self.__events_kernel = events_kernel
        self.__context_names = context_names
        self.__profile_fast = profile_fast
        self.__logger = logging.get_logger(__name__)
        self.__ld_preload_actions: List[LdPreload] = []
        # Add LD_PRELOAD actions if corresponding events are enabled
        if self.has_profiling_events(self.__events_ust):
            self.__ld_preload_actions.append(
                LdPreload(self.LIB_PROFILE_FAST if profile_fast else self.LIB_PROFILE_NORMAL)
            )
        if self.has_ust_memory_events(self.__events_ust):
            self.__ld_preload_actions.append(
                LdPreload(self.LIB_MEMORY_UST)
            )

    @staticmethod
    def any_events_match(
        name_pattern: str,
        events: List[str],
    ) -> bool:
        """
        Check if any event name in the list matches the given pattern.

        :param name_pattern: the pattern to use for event names
        :param events: the list of event names
        :return true if there is a match, false otherwise
        """
        return any(re.match(name_pattern, event_name) for event_name in events)

    @classmethod
    def has_profiling_events(
        cls,
        events_ust: List[str],
    ) -> bool:
        """Check if the UST events list contains at least one profiling event."""
        return cls.any_events_match(cls.PROFILE_EVENT_PATTERN, events_ust)

    @classmethod
    def has_ust_memory_events(
        cls,
        events_ust: List[str],
    ) -> bool:
        """Check if the UST events list contains at least one userspace memory event."""
        return cls.any_events_match(cls.MEMORY_UST_EVENT_PATTERN, events_ust)

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        # TODO make sure this is done as late as possible
        context.register_event_handler(OnShutdown(on_shutdown=self._destroy))
        # TODO make sure this is done as early as possible
        self._setup()
        return self.__ld_preload_actions

    def _setup(self) -> None:
        trace_directory = lttng.lttng_init(
            self.__session_name,
            self.__base_path,
            ros_events=self.__events_ust,
            kernel_events=self.__events_kernel,
            context_names=self.__context_names,
        )
        self.__logger.info(f'Writing tracing session to: {trace_directory}')

    def _destroy(self, event: Event, context: LaunchContext) -> None:
        self.__logger.debug(f'Finalizing tracing session: {self.__session_name}')
        lttng.lttng_fini(self.__session_name)

    def __repr__(self):
        return (
            'Trace('
            f'session_name={self.__session_name}, '
            f'base_path={self.__base_path}, '
            f'num_events_ust={len(self.__events_ust)}, '
            f'num_events_kernel={len(self.__events_kernel)}, '
            f'context_names={self.__context_names}, '
            f'profile_fast={self.__profile_fast}, '
            f'ld_preload_actions={self.__ld_preload_actions})'
        )
