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

import os
import time


DEFAULT_BASE_PATH = '~/.ros/tracing/'


def append_timestamp(
    session_name_base: str,
) -> str:
    """
    Append timestamp to base session name.

    :param session_name_base: the base name of the tracing session
    :return: the session name with timestamp
    """
    return session_name_base + '-' + time.strftime('%Y%m%d%H%M%S')


def get_full_session_path(
    session_name: str,
    base_path: str = DEFAULT_BASE_PATH
) -> str:
    """
    Get the full path to the trace directory of a given session.

    :param session_name: the name of the tracing session
    :param base_path: the path to the directory containing the trace directory
    :return: the full path to the tracing session directory
    """
    if base_path is None:
        base_path = DEFAULT_BASE_PATH
    return os.path.expanduser(os.path.join(base_path, session_name))
