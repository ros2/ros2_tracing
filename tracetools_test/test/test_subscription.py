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


class TestSubscription(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-subscription-creation',
            events_ros=[
                'ros2:rcl_node_init',
                'ros2:rcl_subscription_init',
                'ros2:rclcpp_subscription_callback_added',
            ],
            nodes=['test_subscription']
        )

    def test_all(self):
        # Check events order as set (e.g. sub_init before callback_added)
        self.assertEventsOrderSet(self._events_ros)

        # Check fields
        sub_init_events = self.get_events_with_name('ros2:rcl_subscription_init')
        for event in sub_init_events:
            self.assertValidHandle(
                event,
                ['subscription_handle', 'node_handle', 'rmw_subscription_handle'])
            self.assertValidQueueDepth(event, 'queue_depth')
            self.assertStringFieldNotEmpty(event, 'topic_name')

        callback_added_events = self.get_events_with_name(
            'ros2:rclcpp_subscription_callback_added')
        for event in callback_added_events:
            self.assertValidHandle(event, ['subscription_handle', 'callback'])

        # Check that the test topic name exists
        test_sub_init_events = self.get_events_with_field_value(
            'topic_name',
            '/the_topic',
            sub_init_events)
        self.assertNumEventsEqual(test_sub_init_events, 1, 'cannot find test topic name')
        test_sub_init_event = test_sub_init_events[0]

        # Check queue_depth value
        self.assertFieldEquals(
            test_sub_init_event,
            'queue_depth',
            10,
            'sub_init event does not have expected queue depth value')

        # Check that the node handle matches the node_init event
        node_init_events = self.get_events_with_name('ros2:rcl_node_init')
        test_sub_node_init_events = self.get_events_with_procname(
            'test_subscription',
            node_init_events)
        self.assertNumEventsEqual(
            test_sub_node_init_events,
            1,
            'none or more than 1 node_init event')
        test_sub_node_init_event = test_sub_node_init_events[0]
        self.assertMatchingField(
            test_sub_node_init_event,
            'node_handle',
            'ros2:rcl_subscription_init',
            sub_init_events)

        # Check that subscription handle matches with callback_added event
        self.assertMatchingField(
            test_sub_init_event,
            'subscription_handle',
            None,
            callback_added_events)


if __name__ == '__main__':
    unittest.main()
