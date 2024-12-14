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

from tracetools_test.case import TraceTestCase
from tracetools_trace.tools import tracepoints as tp
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestService(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-service',
            events_ros=[
                tp.rcl_node_init,
                tp.rcl_service_init,
                tp.rclcpp_service_callback_added,
                tp.rclcpp_callback_register,
                tp.rmw_take_request,
                tp.callback_start,
                tp.rmw_send_response,
                tp.callback_end,
            ],
            package='test_tracetools',
            nodes=['test_service_ping', 'test_service_pong'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        rcl_srv_init_events = self.get_events_with_name(tp.rcl_service_init)
        callback_added_events = self.get_events_with_name(tp.rclcpp_service_callback_added)
        callback_register_events = self.get_events_with_name(tp.rclcpp_callback_register)
        rmw_take_request_events = self.get_events_with_name(tp.rmw_take_request)
        callback_start_events = self.get_events_with_name(tp.callback_start)
        rmw_send_response_events = self.get_events_with_name(tp.rmw_send_response)
        callback_end_events = self.get_events_with_name(tp.callback_end)

        for event in rcl_srv_init_events:
            self.assertValidHandle(event, ['service_handle', 'node_handle', 'rmw_service_handle'])
            self.assertStringFieldNotEmpty(event, 'service_name')
        for event in callback_added_events:
            self.assertValidHandle(event, ['service_handle', 'callback'])
        for event in callback_register_events:
            self.assertValidPointer(event, 'callback')
            self.assertStringFieldNotEmpty(event, 'symbol')
        for event in rmw_take_request_events:
            self.assertValidHandle(event, 'rmw_service_handle')
            self.assertValidPointer(event, 'request')
            self.assertValidStaticArray(event, 'client_gid', int, 16)
            self.assertFieldType(event, ['sequence_number', 'taken'], int)
        for event in callback_start_events:
            self.assertValidHandle(event, 'callback')
            # Should not be 1 for services (yet)
            self.assertFieldEquals(event, 'is_intra_process', 0)
        for event in rmw_send_response_events:
            self.assertValidHandle(event, 'rmw_service_handle')
            self.assertValidPointer(event, 'response')
            self.assertValidStaticArray(event, 'client_gid', int, 16)
            self.assertFieldType(event, ['sequence_number', 'timestamp'], int)
        for event in callback_end_events:
            self.assertValidHandle(event, 'callback')

        # Find node event
        rcl_node_init_events = self.get_events_with_name(tp.rcl_node_init)
        # The test_service_pong node is the one that has the service (server)
        test_rcl_node_init_event = self.get_event_with_field_value_and_assert(
            'node_name',
            'test_service_pong',
            rcl_node_init_events,
            allow_multiple=False,
        )
        node_handle = self.get_field(test_rcl_node_init_event, 'node_handle')

        # Find service init event and check that the node handle matches
        test_rcl_srv_init_event = self.get_event_with_field_value_and_assert(
            'service_name',
            '/ping',
            rcl_srv_init_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_rcl_srv_init_event, 'node_handle', node_handle)
        rmw_service_handle = self.get_field(test_rcl_srv_init_event, 'rmw_service_handle')

        # Check that there is a matching rclcpp_service_callback_added event
        service_handle = self.get_field(test_rcl_srv_init_event, 'service_handle')
        test_srv_callback_added_event = self.get_event_with_field_value_and_assert(
            'service_handle',
            service_handle,
            callback_added_events,
            allow_multiple=False,
        )

        # Check that there are matching rclcpp_callback_register events
        callback_ref = self.get_field(test_srv_callback_added_event, 'callback')
        test_callback_register_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_ref,
            callback_register_events,
            allow_multiple=False,
        )

        # Check that there are corresponding callback_start/stop pairs
        test_callback_start_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_ref,
            callback_start_events,
            allow_multiple=False,
        )
        test_callback_end_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_ref,
            callback_end_events,
            allow_multiple=False,
        )

        # Find rmw_take_request and rmw_send_response events
        test_rmw_take_request_event = self.get_event_with_field_value_and_assert(
            'rmw_service_handle',
            rmw_service_handle,
            rmw_take_request_events,
            allow_multiple=False,
        )
        test_rmw_send_response_event = self.get_event_with_field_value_and_assert(
            'rmw_service_handle',
            rmw_service_handle,
            rmw_send_response_events,
            allow_multiple=False,
        )

        # Check events order
        self.assertEventOrder([
            test_rcl_node_init_event,
            test_rcl_srv_init_event,
            test_srv_callback_added_event,
            test_callback_register_event,
            test_rmw_take_request_event,
            test_callback_start_event,
            test_rmw_send_response_event,
            test_callback_end_event,
        ])


if __name__ == '__main__':
    unittest.main()
