# Copyright 2023 Apex.AI, Inc.
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

import shutil
import tempfile
import unittest

from lttngpy import impl as lttngpy


@unittest.skipIf(not lttngpy.is_lttng_session_daemon_alive(), 'session daemon required')
class TestSession(unittest.TestCase):

    def create_test_tmpdir(self, test_name: str) -> str:
        prefix = self.__class__.__name__ + '__' + test_name
        return tempfile.mkdtemp(prefix=prefix)

    def test_session_list_create_start_stop_destroy(self):
        session_name = 'test_session_list_create_start_stop_destroy'
        tmpdir = self.create_test_tmpdir(session_name)

        self.assertSetEqual(set(), lttngpy.get_session_names())
        self.assertEqual(0, lttngpy.lttng_create_session(session_name=session_name, url=tmpdir))
        self.assertSetEqual({session_name}, lttngpy.get_session_names())
        self.assertEqual(
            0,
            lttngpy.enable_channel(
                session_name=session_name,
                domain_type=lttngpy.LTTNG_DOMAIN_UST,
                buffer_type=lttngpy.LTTNG_BUFFER_PER_UID,
                channel_name='dummy_channel',
                overwrite=None,
                subbuf_size=None,
                num_subbuf=None,
                switch_timer_interval=None,
                read_timer_interval=None,
                output=None,
            ),
        )
        self.assertEqual(0, lttngpy.lttng_start_tracing(session_name=session_name))
        self.assertEqual(0, lttngpy.lttng_stop_tracing(session_name=session_name))
        self.assertEqual(0, lttngpy.lttng_destroy_session(session_name=session_name))
        self.assertSetEqual(set(), lttngpy.get_session_names())

        self.assertEqual(0, lttngpy.lttng_create_session(session_name=session_name, url=tmpdir))
        self.assertEqual(0, lttngpy.destroy_all_sessions())
        self.assertSetEqual(set(), lttngpy.get_session_names())

        shutil.rmtree(tmpdir)

    def test_error(self):
        session_name = 'test_error'
        self.assertSetEqual(set(), lttngpy.get_session_names())
        self.assertNotEqual(0, lttngpy.lttng_start_tracing(session_name=session_name))
        self.assertNotEqual(0, lttngpy.lttng_stop_tracing(session_name=session_name))
        self.assertNotEqual(0, lttngpy.lttng_destroy_session(session_name=session_name))
        self.assertSetEqual(set(), lttngpy.get_session_names())
