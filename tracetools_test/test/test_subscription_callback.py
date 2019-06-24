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


class TestSubscriptionCallback(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-subscription-callback',
            events_ros=[
                'ros2:callback_start',
                'ros2:callback_end',
            ],
            nodes=['test_ping', 'test_pong']
        )

    def test_all(self):
        # Check events order as set (e.g. start before end)
        self.assertEventsOrderSet(self._events_ros)

        # Check fields
        start_events = self.get_events_with_name('ros2:callback_start')
        for event in start_events:
            self.assertValidHandle(event, 'callback')
            is_intra_process_value = self.get_field(event, 'is_intra_process')
            self.assertIsInstance(is_intra_process_value, int, 'is_intra_process not int')
            self.assertTrue(
                is_intra_process_value in [0, 1],
                f'invalid value for is_intra_process: {is_intra_process_value}')

        end_events = self.get_events_with_name('ros2:callback_end')
        for event in end_events:
            self.assertValidHandle(event, 'callback')

        # Check that a start:end pair has a common callback handle
        # Note: might be unstable if tracing is disabled too early
        ping_events = self.get_events_with_procname('test_ping')
        pong_events = self.get_events_with_procname('test_pong')
        ping_events_start = self.get_events_with_name('ros2:callback_start', ping_events)
        pong_events_start = self.get_events_with_name('ros2:callback_start', pong_events)
        for ping_start in ping_events_start:
            self.assertMatchingField(
                ping_start,
                'callback',
                'ros2:callback_end',
                ping_events)
        for pong_start in pong_events_start:
            self.assertMatchingField(
                pong_start,
                'callback',
                'ros2:callback_end',
                pong_events)


if __name__ == '__main__':
    unittest.main()
