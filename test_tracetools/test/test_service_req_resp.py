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
class TestServiceReqResp(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-service-req-resp',
            events_ros=[
                tp.rmw_client_init,
                tp.rcl_client_init,
                tp.rcl_service_init,
                tp.rclcpp_service_callback_added,
                tp.rmw_send_request,
                tp.rmw_take_request,
                tp.callback_start,
                tp.rmw_send_response,
                tp.callback_end,
                tp.rmw_take_response,
            ],
            package='test_tracetools',
            nodes=['test_service_ping', 'test_service_pong'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        rmw_client_init_events = self.get_events_with_name(tp.rmw_client_init)
        rcl_client_init_events = self.get_events_with_name(tp.rcl_client_init)
        rcl_service_init_events = self.get_events_with_name(tp.rcl_service_init)
        service_callback_added_events = self.get_events_with_name(tp.rclcpp_service_callback_added)
        rmw_send_request_events = self.get_events_with_name(tp.rmw_send_request)
        rmw_take_request_events = self.get_events_with_name(tp.rmw_take_request)
        callback_start_events = self.get_events_with_name(tp.callback_start)
        rmw_send_response_events = self.get_events_with_name(tp.rmw_send_response)
        callback_end_events = self.get_events_with_name(tp.callback_end)
        rmw_take_response_events = self.get_events_with_name(tp.rmw_take_response)

        # Get client
        test_client_init_event = self.get_event_with_field_value_and_assert(
            'service_name',
            '/ping',
            rcl_client_init_events,
            allow_multiple=False,
        )
        rmw_client_handle = self.get_field(test_client_init_event, 'rmw_client_handle')
        test_rmw_client_init = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_client_init_events,
            allow_multiple=False,
        )
        client_gid = self.get_field(test_rmw_client_init, 'gid')

        # Get service
        test_service_init_event = self.get_event_with_field_value_and_assert(
            'service_name',
            '/ping',
            rcl_service_init_events,
            allow_multiple=False,
        )
        service_handle = self.get_field(test_service_init_event, 'service_handle')
        rmw_service_handle = self.get_field(test_service_init_event, 'rmw_service_handle')
        test_service_callback_added_event = self.get_event_with_field_value_and_assert(
            'service_handle',
            service_handle,
            service_callback_added_events,
            allow_multiple=False,
        )
        callback_ref = self.get_field(test_service_callback_added_event, 'callback')

        # Get send request
        test_rmw_send_request_event = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_send_request_events,
            allow_multiple=False,
        )
        sequence_number = self.get_field(test_rmw_send_request_event, 'sequence_number')

        # Get take request
        test_rmw_take_request_event = self.get_event_with_field_value_and_assert(
            'rmw_service_handle',
            rmw_service_handle,
            rmw_take_request_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_rmw_take_request_event, 'client_gid', client_gid)
        self.assertFieldEquals(test_rmw_take_request_event, 'sequence_number', sequence_number)
        self.assertFieldEquals(test_rmw_take_request_event, 'taken', 1)

        # Get request callback start
        test_request_callback_start_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_ref,
            callback_start_events,
            allow_multiple=False,
        )

        # Get send response
        test_rmw_send_response_event = self.get_event_with_field_value_and_assert(
            'rmw_service_handle',
            rmw_service_handle,
            rmw_send_response_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_rmw_send_response_event, 'client_gid', client_gid)
        self.assertFieldEquals(test_rmw_send_response_event, 'sequence_number', sequence_number)
        source_timestamp = self.get_field(test_rmw_send_response_event, 'timestamp')

        # Get request callback end
        test_request_callback_end_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_ref,
            callback_end_events,
            allow_multiple=False,
        )

        # Get take response
        test_rmw_take_response_event = self.get_event_with_field_value_and_assert(
            'rmw_client_handle',
            rmw_client_handle,
            rmw_take_response_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(test_rmw_take_response_event, 'sequence_number', sequence_number)
        self.assertFieldEquals(test_rmw_take_response_event, 'source_timestamp', source_timestamp)
        self.assertFieldEquals(test_rmw_take_response_event, 'taken', 1)

        # Assert request/response order
        # We shouldn't necessarily expect the response to be taken after the request callback end
        self.assertEventOrder([
            test_rmw_send_request_event,
            test_rmw_take_request_event,
            test_request_callback_start_event,
            test_rmw_send_response_event,
            test_request_callback_end_event,
        ])
        self.assertEventOrder([
            test_rmw_send_response_event,
            test_rmw_take_response_event,
        ])
