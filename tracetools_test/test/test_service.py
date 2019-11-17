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


class TestService(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-service-creation',
            events_ros=[
                'ros2:rcl_node_init',
                'ros2:rcl_service_init',
                'ros2:rclcpp_service_callback_added',
            ],
            nodes=['test_service'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Check fields
        srv_init_events = self.get_events_with_name('ros2:rcl_service_init')
        callback_added_events = self.get_events_with_name('ros2:rclcpp_service_callback_added')

        for event in srv_init_events:
            self.assertValidHandle(event, ['service_handle', 'node_handle', 'rmw_service_handle'])
            self.assertStringFieldNotEmpty(event, 'service_name')
        for event in callback_added_events:
            self.assertValidHandle(event, ['service_handle', 'callback'])

        # Check that the test service name exists
        test_srv_init_events = self.get_events_with_procname('test_service', srv_init_events)
        event_service_names = self.get_events_with_field_value(
            'service_name',
            '/the_service',
            test_srv_init_events,
        )
        self.assertGreaterEqual(
            len(event_service_names),
            1,
            'cannot find test service name',
        )

        # Check that the node handle matches the node_init event
        node_init_events = self.get_events_with_name('ros2:rcl_node_init')
        test_srv_node_init_events = self.get_events_with_procname(
            'test_service',
            node_init_events,
        )
        self.assertNumEventsEqual(
            test_srv_node_init_events,
            1,
            'none or more than 1 rcl_node_init event',
        )
        test_srv_node_init_event = test_srv_node_init_events[0]
        self.assertMatchingField(
            test_srv_node_init_event,
            'node_handle',
            'ros2:rcl_service_init',
            test_srv_init_events,
        )

        # Check that the service handles match
        test_event_srv_init = event_service_names[0]
        self.assertMatchingField(
            test_event_srv_init,
            'service_handle',
            None,
            callback_added_events,
        )


if __name__ == '__main__':
    unittest.main()
