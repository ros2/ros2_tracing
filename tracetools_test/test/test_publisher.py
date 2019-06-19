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


class TestPublisher(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-publisher-creation',
            events_ros=[
                'ros2:rcl_node_init',
                'ros2:rcl_publisher_init',
            ],
            nodes=['test_publisher']
        )

    def test_all(self):
        # Check events order as set (e.g. node_init before pub_init)
        self.assertEventsOrderSet(self._events_ros)

        # Check fields
        pub_init_events = self.get_events_with_name('ros2:rcl_publisher_init')
        for event in pub_init_events:
            self.assertValidHandle(
                event,
                ['publisher_handle', 'node_handle', 'rmw_publisher_handle'])
            self.assertValidQueueDepth(event, 'queue_depth')
            self.assertStringFieldNotEmpty(event, 'topic_name')

        # Check that the test topic name exists
        test_pub_init_events = self.get_events_with_procname('test_publisher', pub_init_events)
        event_topic_names = [self.get_field(e, 'topic_name') for e in test_pub_init_events]
        self.assertTrue('/the_topic' in event_topic_names, 'cannot find test topic name')

        # Check that the node handle matches with the node_init event
        node_init_events = self.get_events_with_name('ros2:rcl_node_init')
        test_pub_node_init_events = self.get_events_with_procname(
            'test_publisher',
            node_init_events)
        self.assertEqual(len(test_pub_node_init_events), 1, 'none or more than 1 node_init event')
        test_pub_node_init_event = test_pub_node_init_events[0]
        self.assertMatchingField(
            test_pub_node_init_event,
            'node_handle',
            None,
            test_pub_init_events)


if __name__ == '__main__':
    unittest.main()
