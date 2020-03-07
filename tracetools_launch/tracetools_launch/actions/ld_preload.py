# Copyright 2019 Apex.AI, Inc.
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

"""Module for the LdPreload action."""

import subprocess
from typing import List
from typing import Optional
from typing import Union

from launch import logging
from launch.action import Action
from launch.actions import SetEnvironmentVariable
from launch.launch_context import LaunchContext
from tracetools_trace.tools import tracing_supported


class LdPreload(Action):
    """Action that adds a SetEnvironmentVariable action to preload a library."""

    ENV_VAR_LD_PRELOAD = 'LD_PRELOAD'

    def __init__(
        self,
        lib_name: str,
        **kwargs,
    ) -> None:
        """
        Create an LdPreload action.

        :param lib_name: the name of the library (e.g. 'lib.so')
        """
        super().__init__(**kwargs)
        self.__lib_name = lib_name
        self.__set_env_action = None
        self.__logger = logging.get_logger(__name__)
        # Try to find lib
        self.__lib_path = self.get_shared_lib_path(self.__lib_name)
        # And create action if found
        if self.__lib_path is not None:
            self.__logger.debug(f'Shared library found at: {self.__lib_path}')
            self.__set_env_action = SetEnvironmentVariable(
                self.ENV_VAR_LD_PRELOAD,
                self.__lib_path,
            )
        else:
            self.__logger.warn(f'Could not find shared library: {self.__lib_name}')

    def lib_found(self) -> bool:
        return self.__set_env_action is not None

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        if self.lib_found():
            return [self.__set_env_action]
        return None

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

    def __repr__(self):
        return (
            'LdPreload('
            f'lib_name={self.__lib_name}, '
            f'lib found={self.lib_found()}, '
            f'lib_path={self.__lib_path})'
        )
