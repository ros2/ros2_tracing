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

import os
from typing import List
from typing import Optional

from launch.action import Action
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from tracetools_trace.tools import lttng
from tracetools_trace.tools import names
from tracetools_trace.tools import path


class Trace(Action):
    """
    Tracing action for launch.

    Sets up and enables tracing through a launch file description.
    """

    def __init__(
        self,
        *,
        session_name: str,
        append_timestamp: bool = False,
        base_path: str = path.DEFAULT_BASE_PATH,
        events_ust: List[str] = names.DEFAULT_EVENTS_ROS,
        events_kernel: List[str] = names.DEFAULT_EVENTS_KERNEL,
        **kwargs,
    ) -> None:
        """
        Constructor.

        :param session_name: the name of the tracing session
        :param append_timestamp: whether to append timestamp to the session name
        :param base_path: the path to the base directory in which to create the tracing session directory
        :param events_ust: the list of ROS UST events to enable
        :param events_kernel: the list of kernel events to enable
        """
        super().__init__(**kwargs)
        if append_timestamp:
            session_name = path.append_timestamp(session_name)
        self.__session_name = session_name
        self.__base_path = base_path
        self.__events_ust = events_ust
        self.__events_kernel = events_kernel

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        # TODO make sure this is done as late as possible
        context.register_event_handler(OnShutdown(on_shutdown=self._destroy))
        # TODO make sure this is done as early as possible
        self._setup()

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
            "Trace("
            f"session_name='{self.__session_name}', "
            f"base_path='{self.__base_path}', "
            f"num_events_ust={len(self.__events_ust)}, "
            f"num_events_kernel={len(self.__events_kernel)})"
        )
