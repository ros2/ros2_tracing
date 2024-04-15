# Copyright 2023 Research Institute of Systems Planning, Inc.
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
class TestBuffer(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-buffer-creation',
            events_ros=[
                tp.rclcpp_construct_ring_buffer,
                tp.rclcpp_ipb_to_subscription,
                tp.rclcpp_buffer_to_ipb,
            ],
            package='test_tracetools',
            nodes=['test_intra'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        construct_buffer_events = self.get_events_with_name(tp.rclcpp_construct_ring_buffer)
        for event in construct_buffer_events:
            self.assertValidPointer(event, 'buffer')
            self.assertFieldType(event, 'capacity', int)

        ipb_to_subscription_events = self.get_events_with_name(tp.rclcpp_ipb_to_subscription)
        for event in ipb_to_subscription_events:
            self.assertValidPointer(event, ['ipb', 'subscription'])

        buffer_to_ipb_events = self.get_events_with_name(tp.rclcpp_buffer_to_ipb)
        for event in buffer_to_ipb_events:
            self.assertValidPointer(event, ['buffer', 'ipb'])

        # Check corresponding events for construct_buffer_event
        for construct_event in construct_buffer_events:
            target_buffer = self.get_field(construct_event, 'buffer')
            target_buffer_to_ipb_event = self.get_event_with_field_value_and_assert(
                'buffer',
                target_buffer,
                buffer_to_ipb_events,
                allow_multiple=False,
            )

            target_ipb = self.get_field(target_buffer_to_ipb_event, 'ipb')
            target_ipb_to_subscription_event = self.get_event_with_field_value_and_assert(
                'ipb',
                target_ipb,
                ipb_to_subscription_events,
                allow_multiple=False,
            )

            # Check subscription init order
            #   * rclcpp_construct_ring_buffer
            #   * rclcpp_buffer_to_ipb
            #   * rclcpp_ipb_to_subscription
            self.assertEventOrder([
                construct_event,
                target_buffer_to_ipb_event,
                target_ipb_to_subscription_event,
            ])


if __name__ == '__main__':
    unittest.main()
