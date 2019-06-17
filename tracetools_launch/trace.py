
import os
from typing import List
from typing import Optional
from typing import Text

from launch.action import Action
from launch.event import Event
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from tracetools_trace.tools import names


class Trace(Action):
    """
    Tracing action for launch.

    Sets up and enables tracing through a launch file description.
    """

    def __init__(
        self, *,
        session_name: str,
        base_path: str = '/tmp',
        events_ust: List[str] = names.DEFAULT_EVENTS_ROS,
        events_kernel: List[str] = names.DEFAULT_EVENTS_KERNEL,
        **kwargs) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        self.__session_name = session_name
        self.__path = os.path.join(base_path, session_name)
        self.__events_ust = events_ust
        self.__events_kernel = events_kernel

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        # TODO make sure this is done as late as possible
        context.register_event_handler(OnShutdown(
            on_shutdown=self._destroy,
        ))
        # TODO make sure this is done as early as possible
        self._setup()

    def _setup(self):
        # TODO
        print('setting up tracing!')

    def _destroy(self, event: Event, context: LaunchContext):
        # TODO
        print('destroying tracing session!')
