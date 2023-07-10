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

"""Interface for tracing with LTTng."""

import platform
import sys
from typing import Optional

from packaging.version import Version

try:
    from . import lttng_impl as _lttng
except ImportError:
    # Fall back on stub functions so that this still passes linter checks
    # This will happen if lttngpy isn't found, in which case importing lttng_impl will fail
    from . import lttng_stub as _lttng  # type: ignore


def get_lttng_version() -> Optional[Version]:
    """
    Get version of lttng-ctl.

    :return: the version of lttng-ctl, or `None` if it is not available
    """
    if not hasattr(_lttng, 'get_version') or not callable(_lttng.get_version):
        return None
    return _lttng.get_version()


def lttng_init(**kwargs) -> Optional[str]:
    """
    Set up and start LTTng session.

    For the full list of kwargs, see `lttng_impl.setup()`.

    Raises RuntimeError on failure, in which case the tracing session might still exist.

    :return: the full path to the trace directory, or `None` if initialization failed
    """
    if not is_lttng_installed():
        return None

    trace_directory = _lttng.setup(**kwargs)
    if trace_directory is None:
        return None
    _lttng.start(**kwargs)
    return trace_directory


def lttng_fini(**kwargs) -> None:
    """
    Stop and destroy LTTng session.

    Raises RuntimeError on failure.

    :param session_name: the name of the session
    """
    _lttng.stop(**kwargs)
    _lttng.destroy(**kwargs)


def lttng_start(**kwargs) -> None:
    """
    Start tracing.

    Raises RuntimeError on failure.

    :param session_name: the name of the session
    """
    _lttng.start(**kwargs)


def lttng_stop(**kwargs) -> None:
    """
    Stop tracing.

    Raises RuntimeError on failure.

    :param session_name: the name of the session
    """
    _lttng.stop(**kwargs)


def is_lttng_installed(
    *,
    minimum_version: Optional[str] = None,
) -> bool:
    """
    Check if LTTng is installed.

    It first checks if the OS can support LTTng.
    If so, it then checks if lttng-ctl is installed.

    Optionally, a minimum version can also be specified for lttng-ctl.

    :param minimum_version: the minimum required lttng-ctl version
    :return: True if lttng-ctl is installed, and optionally if the version of lttng-ctl is
        sufficient, False otherwise
    """
    # Check system
    message_doc = (
        'Cannot trace. See documentation at: '
        'https://github.com/ros2/ros2_tracing'
    )
    system = platform.system()
    if 'Linux' != system:
        print(f"System '{system}' does not support LTTng.\n{message_doc}", file=sys.stderr)
        return False
    # Check if lttng-ctl is installed
    lttng_version = get_lttng_version()
    if not lttng_version:
        print(
            f'lttng-ctl (liblttng-ctl-dev) not installed\n{message_doc}',
            file=sys.stderr,
        )
        return False
    # Check if lttng-ctl version is sufficient
    if minimum_version and lttng_version < Version(minimum_version):
        print(
            (
                f'lttng-ctl (liblttng-ctl-dev) version >={minimum_version} required, '
                f'found {str(lttng_version)}'
            ),
            file=sys.stderr,
        )
        return False
    return True
