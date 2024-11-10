# Copyright 2024 Apex.AI, Inc.
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

from tracetools_test.case import TraceTestCase
from tracetools_trace.tools import tracepoints as tp
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestClient(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-client',
            events_ros=[
                tp.rcl_node_init,
                tp.rcl_client_init,
            ],
            package='test_tracetools',
            nodes=['test_service_ping', 'test_service_pong'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        client_init_events = self.get_events_with_name(tp.rcl_client_init)

        for event in client_init_events:
            self.assertValidHandle(event, ['client_handle', 'node_handle', 'rmw_client_handle'])
            self.assertStringFieldNotEmpty(event, 'service_name')

        # Find node event
        node_init_events = self.get_events_with_name(tp.rcl_node_init)
        # The test_service_ping node is the one that has the client
        test_node_init_event = self.get_event_with_field_value_and_assert(
            'node_name',
            'test_service_ping',
            node_init_events,
            allow_multiple=False,
        )
        node_handle = self.get_field(test_node_init_event, 'node_handle')

        # Find client init event and check that the node handle matches
        test_client_init_event = self.get_event_with_field_value_and_assert(
            'service_name',
            '/ping',
            client_init_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_client_init_event, 'node_handle', node_handle)

        # Check events order
        self.assertEventOrder([
            test_node_init_event,
            test_client_init_event,
        ])


if __name__ == '__main__':
    unittest.main()
