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

import os
import tempfile
import unittest
from unittest import mock

from packaging.version import Version
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(), 'LTTng is required')
class TestLttngTracing(unittest.TestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def test_is_lttng_installed(self):
        # Different OS
        with mock.patch('platform.system', return_value='Windows'):
            self.assertFalse(is_lttng_installed())

        # lttng-ctl or version not found
        with mock.patch('tracetools_trace.tools.lttng.get_lttng_version', return_value=None):
            self.assertFalse(is_lttng_installed())

        # Minimum version requirement
        with mock.patch(
            'tracetools_trace.tools.lttng.get_lttng_version',
            return_value=Version('1.2.3'),
        ):
            self.assertFalse(is_lttng_installed(minimum_version='1.2.4'))
            self.assertTrue(is_lttng_installed(minimum_version='1.2.3'))
            self.assertTrue(is_lttng_installed())

    def test_lttng_not_installed(self):
        from tracetools_trace.tools.lttng import lttng_init
        with mock.patch('tracetools_trace.tools.lttng.is_lttng_installed', return_value=False):
            self.assertIsNone(lttng_init(session_name='test-session', base_path='/tmp'))

    def test_no_kernel_tracer(self):
        from tracetools_trace.tools.lttng_impl import setup
        with (
            mock.patch(
                'tracetools_trace.tools.lttng_impl.is_session_daemon_not_alive',
                return_value=False,
            ),
            mock.patch(
                'tracetools_trace.tools.lttng_impl.is_session_daemon_unreachable',
                return_value=False,
            ),
            mock.patch(
                'tracetools_trace.tools.lttng_impl.is_kernel_tracer_available',
                return_value=False,
            ),
        ):
            with self.assertRaises(RuntimeError):
                setup(
                    session_name='test-session',
                    base_path='/tmp',
                    kernel_events=['sched_switch'],
                )

    def test_get_lttng_home(self):
        from tracetools_trace.tools.lttng_impl import get_lttng_home
        # Uses $LTTNG_HOME if set
        environ = {'LTTNG_HOME': 'the_lttng_home', 'HOME': 'the_home'}
        with mock.patch.dict(os.environ, environ, clear=True):
            self.assertEqual('the_lttng_home', get_lttng_home())
        # Defaults to $HOME if LTTNG_HOME is unset
        environ = {'HOME': 'the_home'}
        with mock.patch.dict(os.environ, environ, clear=True):
            self.assertEqual('the_home', get_lttng_home())
        # Returns `None` otherwise
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(get_lttng_home())

    def test_get_session_daemon_pid(self):
        from tracetools_trace.tools.lttng_impl import get_session_daemon_pid
        # No PID if there is no LTTng home
        with mock.patch('tracetools_trace.tools.lttng_impl.get_lttng_home', return_value=None):
            self.assertIsNone(get_session_daemon_pid())
        # No PID if the PID file doesn't exist
        with mock.patch(
            'tracetools_trace.tools.lttng_impl.get_lttng_home',
            return_value=os.path.join(tempfile.gettempdir(), 'doesnt_exist'),
        ):
            self.assertIsNone(get_session_daemon_pid())
        # PID file exists...
        with (
            mock.patch(
                'tracetools_trace.tools.lttng_impl.get_lttng_home',
                return_value='some_non-None_value',
            ),
            mock.patch('os.path.isfile', return_value=True),
        ):
            # ...but is not a valid int
            with mock.patch('builtins.open', mock.mock_open(read_data='')):
                self.assertIsNone(get_session_daemon_pid())
            with mock.patch('builtins.open', mock.mock_open(read_data='abc')):
                self.assertIsNone(get_session_daemon_pid())
            # ...and has a valid int when stripped
            with mock.patch('builtins.open', mock.mock_open(read_data='123\n')):
                self.assertEqual(123, get_session_daemon_pid())

    def test_is_session_daemon_unreachable(self):
        from tracetools_trace.tools.lttng_impl import is_session_daemon_unreachable
        # All good if we can't get the session daemon PID
        with mock.patch(
            'tracetools_trace.tools.lttng_impl.get_session_daemon_pid',
            return_value=None,
        ):
            self.assertFalse(is_session_daemon_unreachable())
        # If we can get the session daemon PID...
        with mock.patch(
            'tracetools_trace.tools.lttng_impl.get_session_daemon_pid',
            return_value=123,
        ):
            # Unreachable if we can't find the process with the PID
            with mock.patch('subprocess.run') as patched_subprocess_run:
                patched_subprocess_run.return_value.returncode = 1
                self.assertTrue(is_session_daemon_unreachable())
            # Unreachable if we can find the process with the PID, but it is not a session daemon
            with mock.patch('subprocess.run') as patched_subprocess_run:
                patched_subprocess_run.return_value.returncode = 0
                patched_subprocess_run.return_value.stdout = 'some-random-command\n'
                self.assertTrue(is_session_daemon_unreachable())
            # All good if we can find the process with the PID and it is a session daemon
            with mock.patch('subprocess.run') as patched_subprocess_run:
                patched_subprocess_run.return_value.returncode = 0
                patched_subprocess_run.return_value.stdout = 'lttng-sessiond\n'
                self.assertFalse(is_session_daemon_unreachable())

    def test_unreachable_session_daemon(self):
        from tracetools_trace.tools.lttng_impl import setup
        with (
            mock.patch(
                'tracetools_trace.tools.lttng_impl.is_session_daemon_not_alive',
                return_value=False,
            ),
            mock.patch(
                'tracetools_trace.tools.lttng_impl.is_session_daemon_unreachable',
                return_value=True,
            ),
        ):
            with self.assertRaises(RuntimeError):
                setup(session_name='test-session', base_path='/tmp')
