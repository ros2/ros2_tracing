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
                tp.rmw_client_init,
                tp.rcl_client_init,
                tp.rmw_send_request,
                tp.rmw_take_response,
            ],
            package='test_tracetools',
            nodes=['test_service_ping', 'test_service_pong'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        rmw_client_init_events = self.get_events_with_name(tp.rmw_client_init)
        rcl_client_init_events = self.get_events_with_name(tp.rcl_client_init)
        rmw_send_request_events = self.get_events_with_name(tp.rmw_send_request)
        rmw_take_response_events = self.get_events_with_name(tp.rmw_take_response)

        for event in rmw_client_init_events:
            self.assertValidHandle(event, 'rmw_client_handle')
            self.assertValidStaticArray(event, 'gid', int, 16)
        for event in rcl_client_init_events:
            self.assertValidHandle(event, ['client_handle', 'node_handle', 'rmw_client_handle'])
            self.assertStringFieldNotEmpty(event, 'service_name')
        for event in rmw_send_request_events:
            self.assertValidHandle(event, 'rmw_client_handle')
            self.assertValidPointer(event, 'request')
            self.assertFieldType(event, 'sequence_number', int)
        for event in rmw_take_response_events:
            self.assertValidHandle(event, 'rmw_client_handle')
            self.assertValidPointer(event, 'response')
            self.assertFieldType(event, ['sequence_number', 'source_timestamp', 'taken'], int)

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

        # Find client init events and check that the handles match
        test_rcl_client_init_event = self.get_event_with_field_value_and_assert(
            'service_name',
            '/ping',
            rcl_client_init_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_rcl_client_init_event, 'node_handle', node_handle)
        rmw_client_handle = self.get_field(test_rcl_client_init_event, 'rmw_client_handle')
        test_rmw_client_init_event = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_client_init_events,
            allow_multiple=False,
        )

        # Find rmw_send_request and rmw_take_response events
        test_rmw_send_request_event = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_send_request_events,
            allow_multiple=False,
        )
        test_rmw_take_response_event = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_take_response_events,
            allow_multiple=False,
        )

        # Check events order
        self.assertEventOrder([
            test_node_init_event,
            test_rmw_client_init_event,
            test_rcl_client_init_event,
            test_rmw_send_request_event,
            test_rmw_take_response_event,
        ])


if __name__ == '__main__':
    unittest.main()
