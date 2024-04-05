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

import unittest

from tracetools_test.case import TraceTestCase
from tracetools_trace.tools import tracepoints as tp
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestTimer(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-timer-all',
            events_ros=[
                tp.rcl_node_init,
                tp.rcl_timer_init,
                tp.rclcpp_timer_callback_added,
                tp.rclcpp_callback_register,
                tp.rclcpp_timer_link_node,
                tp.rclcpp_executor_execute,
                tp.callback_start,
                tp.callback_end,
            ],
            package='test_tracetools',
            nodes=['test_timer'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        timer_init_events = self.get_events_with_name(tp.rcl_timer_init)
        for event in timer_init_events:
            self.assertValidHandle(event, 'timer_handle')
            period_value = self.get_field(event, 'period')
            self.assertIsInstance(period_value, int)
            self.assertGreaterEqual(period_value, 0, f'invalid period value: {period_value}')

        callback_added_events = self.get_events_with_name(tp.rclcpp_timer_callback_added)
        for event in callback_added_events:
            self.assertValidHandle(event, ['timer_handle', 'callback'])

        callback_register_events = self.get_events_with_name(tp.rclcpp_callback_register)
        for event in callback_register_events:
            self.assertValidPointer(event, 'callback')
            self.assertStringFieldNotEmpty(event, 'symbol')

        link_node_events = self.get_events_with_name(tp.rclcpp_timer_link_node)
        for event in link_node_events:
            self.assertValidHandle(event, ['timer_handle', 'node_handle'])

        executor_execute_events = self.get_events_with_name(tp.rclcpp_executor_execute)
        for event in executor_execute_events:
            self.assertValidHandle(event, ['handle'])

        start_events = self.get_events_with_name(tp.callback_start)
        for event in start_events:
            self.assertValidHandle(event, 'callback')
            # Should not be 1 for timer
            self.assertFieldEquals(event, 'is_intra_process', 0)

        end_events = self.get_events_with_name(tp.callback_end)
        for event in end_events:
            self.assertValidHandle(event, 'callback')

        # Find and check given timer period
        self.assertNumEventsEqual(timer_init_events, 1)
        test_timer_init_event = timer_init_events[0]
        self.assertFieldEquals(test_timer_init_event, 'period', 1000000)
        timer_handle = self.get_field(test_timer_init_event, 'timer_handle')

        # Check that the timer_init:callback_added pair exists and has a common timer handle
        test_callback_added_event = self.get_event_with_field_value_and_assert(
            'timer_handle',
            timer_handle,
            callback_added_events,
            allow_multiple=False,
        )

        # Check that callback pointer matches between callback_added and callback_register
        callback_handle = self.get_field(test_callback_added_event, 'callback')
        test_callback_register_event = self.get_event_with_field_value_and_assert(
            'callback',
            callback_handle,
            callback_register_events,
            allow_multiple=False,
        )

        # Check that there is a link_node event for the timer
        test_link_node_event = self.get_event_with_field_value_and_assert(
            'timer_handle',
            timer_handle,
            link_node_events,
            allow_multiple=False,
        )

        # And that the node from that node handle exists
        node_handle = self.get_field(test_link_node_event, 'node_handle')
        node_init_events = self.get_events_with_name(tp.rcl_node_init)
        test_node_inits_event = self.get_event_with_field_value_and_assert(
            'node_handle',
            node_handle,
            node_init_events,
            allow_multiple=False,
        )

        # Check timer creation events order
        self.assertEventOrder([
            test_node_inits_event,
            test_timer_init_event,
            test_callback_added_event,
            test_callback_register_event,
            test_link_node_event,
        ])

        # Check that there are 2 executor execute events for the timer
        timer_execute_events = self.get_events_with_field_value(
            'handle',
            timer_handle,
            executor_execute_events,
        )
        self.assertNumEventsEqual(timer_execute_events, 2)

        # Check that the callback events correspond to the registered timer callback
        self.assertMatchingField(
            test_callback_added_event,
            'callback',
            None,
            start_events,
        )
        self.assertMatchingField(
            test_callback_added_event,
            'callback',
            None,
            end_events,
        )

        # Check that a callback start:end pair has a common callback handle
        for start_event in start_events:
            self.assertMatchingField(
                start_event,
                'callback',
                None,
                end_events,
            )


if __name__ == '__main__':
    unittest.main()
