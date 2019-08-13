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

from tracetools_launch.action import Trace


class TestTraceAction(unittest.TestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def test_has_profiling_events(self) -> None:
        events_lists_match = [
            [
                'lttng_ust_cyg_profile_fast:func_entry',
                'hashtag:yopo',
            ],
            [
                'lttng_ust_cyg_profile:func_entry',
                'some_other_event',
                'lttng_ust_cyg_profile:func_exit',
            ],
        ]
        events_lists_no_match = [
            [
                'lttng_ust_statedump:bin_info',
                'ros2:event',
            ],
            []
        ]
        for events in events_lists_match:
            self.assertTrue(Trace.has_profiling_events(events))
        for events in events_lists_no_match:
            self.assertFalse(Trace.has_profiling_events(events))

    def test_get_shared_lib_path(self) -> None:
        # Only test not finding a lib for now
        self.assertIsNone(Trace.get_shared_lib_path('random_lib_that_does_not_exist_I_hope.so'))


if __name__ == '__main__':
    unittest.main()
