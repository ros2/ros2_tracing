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
import subprocess
from typing import List
from typing import Optional
from typing import Union

from launch.action import Action
from launch.actions import SetEnvironmentVariable
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from tracetools_trace.tools import lttng
from tracetools_trace.tools import names
from tracetools_trace.tools import path
from tracetools_trace.tools import tracing_supported


class Trace(Action):
    """
    Tracing action for launch.

    Sets up and enables tracing through a launch file description.
    """

    PROFILE_LIB_NORMAL = 'liblttng-ust-cyg-profile.so'
    PROFILE_LIB_FAST = 'liblttng-ust-cyg-profile-fast.so'
    PROFILE_EVENT_PATTERN = '^lttng_ust_cyg_profile.*:func_.*'

    def __init__(
        self,
        *,
        session_name: str,
        append_timestamp: bool = False,
        base_path: str = path.DEFAULT_BASE_PATH,
        events_ust: List[str] = names.DEFAULT_EVENTS_ROS,
        events_kernel: List[str] = names.DEFAULT_EVENTS_KERNEL,
        profile_fast: bool = True,
        **kwargs,
    ) -> None:
        """
        Constructor.

        :param session_name: the name of the tracing session
        :param append_timestamp: whether to append timestamp to the session name
        :param base_path: the path to the base directory in which to create the session directory
        :param events_ust: the list of ROS UST events to enable
        :param events_kernel: the list of kernel events to enable
        :param profile_fast: `True` to use fast profiling, `False` for normal (only if necessary)
        """
        super().__init__(**kwargs)
        if append_timestamp:
            session_name = path.append_timestamp(session_name)
        self.__session_name = session_name
        self.__base_path = base_path
        self.__events_ust = events_ust
        self.__events_kernel = events_kernel
        self.__profile_fast = profile_fast
        self.__ld_preload_action = None
        if self.has_profiling_events(events_ust):
            profile_lib_name = self.PROFILE_LIB_FAST if profile_fast else self.PROFILE_LIB_NORMAL
            profile_lib_path = self.get_shared_lib_path(profile_lib_name)
            if profile_lib_path is not None:
                self.__ld_preload_action = SetEnvironmentVariable(
                    'LD_PRELOAD',
                    profile_lib_path,
                )

    @classmethod
    def has_profiling_events(cls, events_ust: List[str]) -> bool:
        """Check if the UST events list contains at least one profiling event."""
        return any(re.match(cls.PROFILE_EVENT_PATTERN, event_name) for event_name in events_ust)

    @staticmethod
    def get_shared_lib_path(lib_name: str) -> Union[str, None]:
        """
        Get the full path to a given shared lib, if possible.

        :param lib_name: the name of the shared library
        :return: the full path if found, `None` otherwise
        """
        if not tracing_supported():
            return None
        (exit_code, output) = subprocess.getstatusoutput(f'whereis -b {lib_name}')
        if exit_code != 0:
            return None
        # Output of whereis is
        # <input_lib_name>: <full path, if found>
        # Filter out empty strings, in case lib is not found
        output_split = [split_part for split_part in output.split(':') if len(split_part) > 0]
        if len(output_split) != 2:
            return None
        return output_split[1].strip()

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        # TODO make sure this is done as late as possible
        context.register_event_handler(OnShutdown(on_shutdown=self._destroy))
        # TODO make sure this is done as early as possible
        self._setup()
        if self.__ld_preload_action is not None:
            return [self.__ld_preload_action]
        return None

    def _setup(self) -> None:
        lttng.lttng_init(
            self.__session_name,
            self.__base_path,
            ros_events=self.__events_ust,
            kernel_events=self.__events_kernel)

    def _destroy(self, event: Event, context: LaunchContext) -> None:
        lttng.lttng_fini(self.__session_name)

    def __repr__(self):
        return (
            'Trace('
            f'session_name={self.__session_name}, '
            f'base_path={self.__base_path}, '
            f'num_events_ust={len(self.__events_ust)}, '
            f'num_events_kernel={len(self.__events_kernel)}), '
            f'profiling={self.__ld_preload_action is not None}, '
            f'profile_fast={self.__profile_fast}'
        )
