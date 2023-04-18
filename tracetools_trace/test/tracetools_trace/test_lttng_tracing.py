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

import subprocess
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

        # LTTng command not found
        class PopenFileNotFound:

            def __init__(self, *args, **kwargs):
                raise FileNotFoundError('file not found')

        with mock.patch.object(subprocess, 'Popen', PopenFileNotFound):
            self.assertFalse(is_lttng_installed())

        # Other error when running LTTng command
        class PopenReturnCodeError:

            def __init__(self, *args, **kwargs):
                pass

            def communicate(self):
                return 'stdout'.encode(), 'stderr'.encode()

            @property
            def returncode(self):
                return 1

        with mock.patch.object(subprocess, 'Popen', PopenReturnCodeError):
            self.assertFalse(is_lttng_installed())

        # lttng Python package or version not found
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
                'tracetools_trace.tools.lttng_impl.is_kernel_tracer_available',
                return_value=(False, 'some error message'),
            ),
            mock.patch('lttng.session_daemon_alive', return_value=1),
        ):
            with self.assertRaises(RuntimeError):
                setup(
                    session_name='test-session',
                    base_path='/tmp',
                    kernel_events=['sched_switch'],
                )
