# Copyright 2026 Sylvester Kaczmarek
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

from contextlib import contextmanager
import os
import shutil
import signal
import tempfile
import time
from typing import Iterator
from typing import Optional


_SESSION_DAEMON_STOP_TIMEOUT = 5.0
_SESSION_DAEMON_STOP_POLL_INTERVAL = 0.05


def _get_session_daemon_pid(lttng_home: str) -> Optional[int]:
    pid_path = os.path.join(lttng_home, '.lttng', 'lttng-sessiond.pid')
    if not os.path.isfile(pid_path):
        return None
    try:
        with open(pid_path, 'r') as pid_file:
            pid = pid_file.read().strip()
    except OSError:
        return None
    if not pid.isdigit():
        return None
    return int(pid)


def _process_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _stop_session_daemon(lttng_home: str) -> None:
    pid = _get_session_daemon_pid(lttng_home)
    if pid is None:
        return

    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return

    deadline = time.monotonic() + _SESSION_DAEMON_STOP_TIMEOUT
    process_exists = _process_exists(pid)
    while process_exists and time.monotonic() < deadline:
        time.sleep(_SESSION_DAEMON_STOP_POLL_INTERVAL)
        process_exists = _process_exists(pid)

    if process_exists:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


@contextmanager
def isolated_lttng_home(*, prefix: str = 'tracetools-test-') -> Iterator[str]:
    """Use a private LTTng home and clean up its session daemon afterwards."""
    previous_lttng_home = os.environ.get('LTTNG_HOME')
    lttng_home = tempfile.mkdtemp(prefix=prefix)
    os.environ['LTTNG_HOME'] = lttng_home
    try:
        yield lttng_home
    finally:
        try:
            _stop_session_daemon(lttng_home)
        finally:
            if previous_lttng_home is None:
                os.environ.pop('LTTNG_HOME', None)
            else:
                os.environ['LTTNG_HOME'] = previous_lttng_home
            shutil.rmtree(lttng_home, ignore_errors=True)
