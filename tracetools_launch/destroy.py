
from typing import List
from typing import Optional
from typing import Text

from launch.action import Action
from launch_context import LaunchContext
from launch_description_entity import LaunchDescriptionEntity


class Destroy(Action):
    """
    Action for launch that destroys a tracing session.

    It finalizes and destroys a tracing session that was setup with the Trace action.
    """

    def __init__(self, session_name: str, **kwargs) -> None:
        """Constructor."""
        super().__init__(**kwargs)
        self.__session_name = session_name

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        pass