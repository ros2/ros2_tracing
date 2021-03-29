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


class TestPublisher(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-publisher',
            events_ros=[
                'ros2:rcl_node_init',
                'ros2:rcl_publisher_init',
                'ros2:rcl_publish',
                'ros2:rclcpp_publish',
            ],
            nodes=['test_publisher'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        pub_init_events = self.get_events_with_name('ros2:rcl_publisher_init')
        for event in pub_init_events:
            self.assertValidHandle(
                event,
                ['publisher_handle', 'node_handle', 'rmw_publisher_handle'],
            )
            self.assertValidQueueDepth(event, 'queue_depth')
            self.assertStringFieldNotEmpty(event, 'topic_name')
        rcl_publish_events = self.get_events_with_name('ros2:rcl_publish')
        for event in rcl_publish_events:
            # Message is a pointer (aka a handle)
            self.assertValidHandle(
                event,
                ['publisher_handle', 'message'],
            )
        rclcpp_publish_events = self.get_events_with_name('ros2:rclcpp_publish')
        for event in rcl_publish_events:
            # Message is a pointer (aka a handle)
            self.assertValidHandle(
                event,
                ['publisher_handle', 'message'],
            )

        # Check that the test topic name exists
        test_pub_init_events = self.get_events_with_procname('test_publisher', pub_init_events)
        test_pub_init_topic_events = self.get_events_with_field_value(
            'topic_name',
            '/the_topic',
            test_pub_init_events,
        )
        self.assertNumEventsEqual(
            test_pub_init_topic_events,
            1,
            'none or more than 1 rcl_pub_init even for test topic',
        )

        # Check queue_depth value
        test_pub_init_topic_event = test_pub_init_topic_events[0]
        self.assertFieldEquals(
            test_pub_init_topic_event,
            'queue_depth',
            10,
            'pub_init event does not have expected queue depth value',
        )

        # Check that the node handle matches with the node_init event
        node_init_events = self.get_events_with_name('ros2:rcl_node_init')
        test_pub_node_init_events = self.get_events_with_procname(
            'test_publisher',
            node_init_events,
        )
        self.assertNumEventsEqual(
            test_pub_node_init_events,
            1,
            'none or more than 1 node_init event',
        )
        test_pub_node_init_event = test_pub_node_init_events[0]
        self.assertMatchingField(
            test_pub_node_init_event,
            'node_handle',
            None,
            test_pub_init_events,
        )

        # Check publish events
        # Get publisher_handle of publisher and find related rcl/rclcpp_publish events
        publisher_handle = self.get_field(test_pub_init_topic_event, 'publisher_handle')
        rclcpp_publish_topic_events = self.get_events_with_field_value(
            'publisher_handle',
            publisher_handle,
            rclcpp_publish_events,
        )
        rcl_publish_topic_events = self.get_events_with_field_value(
            'publisher_handle',
            publisher_handle,
            rcl_publish_events,
        )
        self.assertNumEventsEqual(
            rclcpp_publish_topic_events,
            1,
            'none or more than 1 rclcpp_publish event for test topic',
        )
        self.assertNumEventsEqual(
            rcl_publish_topic_events,
            1,
            'none or more than 1 rcl_publish event for test topic',
        )
        rclcpp_publish_topic_event = rclcpp_publish_topic_events[0]
        rcl_publish_topic_event = rcl_publish_topic_events[0]
        # Make sure the message field matches and check the order
        rclcpp_publish_topic_msg = self.get_field(rclcpp_publish_topic_event, 'message')
        rcl_publish_topic_msg = self.get_field(rcl_publish_topic_event, 'message')
        self.assertEqual(rclcpp_publish_topic_msg, rcl_publish_topic_msg)
        self.assertEventOrder([rclcpp_publish_topic_event, rcl_publish_topic_event])


if __name__ == '__main__':
    unittest.main()
