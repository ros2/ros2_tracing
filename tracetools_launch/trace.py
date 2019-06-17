
from typing import List
from typing import Optional
from typing import Text

from launch.action import Action
from launch_context import LaunchContext
from launch_description_entity import LaunchDescriptionEntity


class Trace(Action):
    """
    Tracing action for launch.

    Sets up and enables tracing through a launch file description.
    """

    def __init__(self, *,
        session_name: str,
        base_path: str,
        events_ust: List[str],
        events_kernel: List[str],
        **kwargs) -> None:
        """Constructor."""
        super().__init__(**kwargs)

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        # TODO
        pass
