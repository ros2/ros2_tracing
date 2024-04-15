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
class TestIntraPubSub(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-intra-pub-sub',
            events_ros=[
                tp.rclcpp_subscription_init,
                tp.rclcpp_subscription_callback_added,
                tp.rclcpp_ipb_to_subscription,
                tp.rclcpp_buffer_to_ipb,
                tp.rclcpp_construct_ring_buffer,
                tp.rclcpp_intra_publish,
                tp.rclcpp_ring_buffer_enqueue,
                tp.rclcpp_ring_buffer_dequeue,
                tp.callback_start,
                tp.callback_end,
            ],
            package='test_tracetools',
            nodes=['test_intra'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # init phase
        rclcpp_subscription_callback_added_events = \
            self.get_events_with_name(tp.rclcpp_subscription_callback_added)
        ipb_to_subscription_events = self.get_events_with_name(tp.rclcpp_ipb_to_subscription)
        buffer_to_ipb_events = self.get_events_with_name(tp.rclcpp_buffer_to_ipb)
        construct_ring_buffer_events = self.get_events_with_name(tp.rclcpp_construct_ring_buffer)

        # runtime phase
        rclcpp_intra_publish_events = self.get_events_with_name(tp.rclcpp_intra_publish)
        ring_buffer_enqueue_events = self.get_events_with_name(tp.rclcpp_ring_buffer_enqueue)
        ring_buffer_dequeue_events = self.get_events_with_name(tp.rclcpp_ring_buffer_dequeue)
        callback_start_events = self.get_events_with_name(tp.callback_start)
        callback_end_events = self.get_events_with_name(tp.callback_end)

        # Check the init events
        buffers = [self.get_field(e, 'buffer') for e in construct_ring_buffer_events]
        buffer_to_callback = {}
        for buffer in buffers:
            ipb_event = self.get_event_with_field_value_and_assert(
                'buffer',
                buffer,
                buffer_to_ipb_events,
                allow_multiple=False,
            )

            subscription_event = self.get_event_with_field_value_and_assert(
                'ipb',
                self.get_field(ipb_event, 'ipb'),
                ipb_to_subscription_events,
                allow_multiple=False,
            )

            subscription_callback_added_event = self.get_event_with_field_value_and_assert(
                'subscription',
                self.get_field(subscription_event, 'subscription'),
                rclcpp_subscription_callback_added_events,
                allow_multiple=False,
            )
            buffer_to_callback[buffer] = \
                self.get_field(subscription_callback_added_event, 'callback')

        # Check that intra-publish events can be linked.
        for i, intra_publish_event in enumerate(rclcpp_intra_publish_events):
            # Find corresponding intra-publish/enqueue event.
            enqueue_event_cand = self.get_events_with_field_value(
                'vtid',
                self.get_tid(intra_publish_event),
                ring_buffer_enqueue_events,
            )
            target_enqueue_event = self.get_corresponding_event(
                self.get_field(intra_publish_event, '_timestamp'),
                enqueue_event_cand,
            )

            # Find corresponding enqueue/dequeue event
            target_index = self.get_field(target_enqueue_event, 'index')
            target_buffer = self.get_field(target_enqueue_event, 'buffer')
            filtered_dequeue_events = self.get_filtered_event(
                {'buffer': target_buffer, 'index': target_index},
                ring_buffer_dequeue_events,
            )
            target_dequeue_event = self.get_corresponding_event(
                self.get_field(target_enqueue_event, '_timestamp'),
                filtered_dequeue_events,
            )

            callback_ = buffer_to_callback[target_buffer]

            filterd_callback_start = self.get_events_with_field_value(
                'callback',
                callback_,
                callback_start_events,
            )

            # Find corresponding callback_start/callback_end event
            target_callback_start = self.get_corresponding_event(
                self.get_field(target_dequeue_event, '_timestamp'),
                filterd_callback_start,
            )

            callback_end_cand = self.get_events_with_field_value(
                'callback',
                callback_,
                callback_end_events,
            )
            target_callback_end = self.get_corresponding_event(
                self.get_field(target_callback_start, '_timestamp'),
                callback_end_cand,
            )

            event_sequence = [
                intra_publish_event,
                target_enqueue_event,
                target_dequeue_event,
                target_callback_start,
                target_callback_end,
            ]

            # Check for the existence of events corresponding to rclcpp_intra_publish
            for e in event_sequence:
                self.assertTrue(
                    e is not None,
                    'cannot find corresponding event for intra_publish',
                )

            # Check the order of events.
            self.assertEventOrder(event_sequence)

            # Check the number of events callback_start/callback_end
            self.assertEqual(len(filterd_callback_start), len(callback_end_cand))

    def get_corresponding_event(self, timestamp, events):
        corresponding_event = None
        diff = 2e64
        for e in events:
            tmp_diff = self.get_field(e, '_timestamp') - timestamp
            if diff > tmp_diff and tmp_diff > 0:
                diff = tmp_diff
                corresponding_event = e
        return corresponding_event

    def get_filtered_event(self, keys, events):
        import copy
        filterd_events = copy.deepcopy(events)
        for k, v in keys.items():
            filterd_events = self.get_events_with_field_value(k, v, filterd_events)
        return filterd_events


if __name__ == '__main__':
    unittest.main()
