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

import unittest

from tracetools_test.utils import (
    cleanup_trace,
    get_trace_event_names,
    run_and_trace,
)

BASE_PATH = '/tmp'
PKG = 'tracetools_test'
node_creation_events = [
    'ros2:rcl_init',
    'ros2:rcl_node_init',
]


class TestNode(unittest.TestCase):

    def test_creation(self):
        session_name_prefix = 'session-test-node-creation'
        test_node = ['test_publisher']

        exit_code, full_path = run_and_trace(
            BASE_PATH,
            session_name_prefix,
            node_creation_events,
            None,
            PKG,
            test_node)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(node_creation_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
