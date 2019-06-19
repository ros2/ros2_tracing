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


class TestIntra(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-intra',
            events_ros=[
                'ros2:rcl_subscription_init',
                'ros2:rclcpp_subscription_callback_added',
                'ros2:callback_start',
                'ros2:callback_end',
            ],
            nodes=['test_intra']
        )

    def test_all(self):
        # Check events order as set (e.g. node_init before pub_init)
        self.assertEventsOrderSet(self._events_ros)

        # Check sub_init for normal and intraprocess events
        sub_init_events = self.get_events_with_name('ros2:rcl_subscription_init')
        sub_init_normal_events = self.get_events_with_field_value(
            'topic_name',
            '/the_topic',
            sub_init_events)
        sub_init_intra_events = self.get_events_with_field_value(
            'topic_name',
            '/the_topic/_intra',
            sub_init_events)
        self.assertEqual(
            len(sub_init_normal_events),
            1,
            'none or more than 1 sub init event for normal sub')
        self.assertEqual(
            len(sub_init_intra_events),
            1,
            'none or more than 1 sub init event for intra sub')

        # Get subscription handles for normal & intra subscriptions
        sub_init_normal_event = sub_init_normal_events[0]
        sub_init_intra_event = sub_init_intra_events[0]
        sub_handle_normal = self.get_field(sub_init_normal_event, 'subscription_handle')
        sub_handle_intra = self.get_field(sub_init_intra_event, 'subscription_handle')

        # Get corresponding callback handles
        callback_added_events = self.get_events_with_name(
            'ros2:rclcpp_subscription_callback_added')
        callback_added_events_normal = self.get_events_with_field_value(
            'subscription_handle',
            sub_handle_normal,
            callback_added_events)
        callback_added_events_intra = self.get_events_with_field_value(
            'subscription_handle',
            sub_handle_intra,
            callback_added_events)
        self.assertEqual(
            len(callback_added_events_normal),
            1,
            'none or more than 1 callback added event for normal sub')
        self.assertEqual(
            len(callback_added_events_intra),
            1,
            'none or more than 1 callback added event for intra sub')
        callback_added_event_normal = callback_added_events_normal[0]
        callback_added_event_intra = callback_added_events_intra[0]
        callback_handle_normal = self.get_field(callback_added_event_normal, 'callback')
        callback_handle_intra = self.get_field(callback_added_event_intra, 'callback')

        # Get corresponding callback start/end pairs
        start_events = self.get_events_with_name('ros2:callback_start')
        end_events = self.get_events_with_name('ros2:callback_end')
        self.assertEqual(len(start_events), 2, 'does not have 2 callback start events')
        self.assertEqual(len(end_events), 2, 'does not have 2 callback end events')
        start_events_normal = self.get_events_with_field_value(
            'callback',
            callback_handle_normal,
            start_events)
        start_events_intra = self.get_events_with_field_value(
            'callback',
            callback_handle_intra,
            start_events)
        end_events_normal = self.get_events_with_field_value(
            'callback',
            callback_handle_normal,
            end_events)
        end_events_intra = self.get_events_with_field_value(
            'callback',
            callback_handle_intra,
            end_events)
        self.assertEqual(
            len(start_events_normal),
            1,
            'none or more than one normal start event')
        self.assertEqual(
            len(start_events_intra),
            1,
            'none or more than one intra start event')
        self.assertEqual(
            len(end_events_normal),
            1,
            'none or more than one normal end event')
        self.assertEqual(
            len(end_events_intra),
            1,
            'none or more than one intra end event')
        start_event_normal = start_events_normal[0]
        start_event_intra = start_events_intra[0]
        is_intra_value_normal = self.get_field(start_event_normal, 'is_intra_process')
        is_intra_value_intra = self.get_field(start_event_intra, 'is_intra_process')
        self.assertEqual(
            is_intra_value_normal,
            0,
            'is_intra_process field value not valid for normal sub')
        self.assertEqual(
            is_intra_value_intra,
            1,
            'is_intra_process field value not valid for intra sub')


if __name__ == '__main__':
    unittest.main()
