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


class TestServiceCallback(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-service-callback',
            events_ros=[
                'ros2:callback_start',
                'ros2:callback_end',
            ],
            nodes=['test_service_ping', 'test_service_pong']
        )

    def test_all(self):
        # Check events order as set (e.g. start before end)
        self.assertEventsOrderSet(self._events_ros)

        # Check fields
        start_events = self.get_events_with_name('ros2:callback_start')
        for event in start_events:
            self.assertValidHandle(event, 'callback')
            # Should not be 1 for services (yet)
            self.assertFieldEquals(
                event,
                'is_intra_process',
                0,
                'invalid value for is_intra_process')

        end_events = self.get_events_with_name('ros2:callback_end')
        for event in end_events:
            self.assertValidHandle(event, 'callback')

        # Check that there is at least a start/end pair for each node
        for node in self._nodes:
            test_start_events = self.get_events_with_procname(node, start_events)
            test_end_events = self.get_events_with_procname(node, end_events)
            self.assertGreater(
                len(test_start_events),
                0,
                f'no start_callback events for node: {node}')
            self.assertGreater(
                len(test_end_events),
                0,
                f'no end_callback events for node: {node}')


if __name__ == '__main__':
    unittest.main()
