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

"""Module for a tracing-specific unittest.TestCase extension."""

import time
from typing import Any
from typing import Dict
from typing import List
import unittest

# from .utils import cleanup_trace
from .utils import DictEvent
from .utils import get_event_name
from .utils import get_event_names
from .utils import get_event_timestamp
from .utils import get_field
from .utils import get_trace_events
from .utils import run_and_trace


class TraceTestCase(unittest.TestCase):
    """
    TestCase extension for tests on a trace.

    Sets up tracing, traces given nodes, and provides
    the resulting events for an extending class to test on.
    It also does some basic checks on the resulting trace.
    """

    def __init__(
        self,
        *args,
        session_name_prefix: str,
        events_ros: List[str],
        nodes: List[str],
        base_path: str = '/tmp',
        events_kernel: List[str] = None,
        package: str = 'tracetools_test'
    ) -> None:
        """Constructor."""
        print(f'methodName={args[0]}')
        super().__init__(methodName=args[0])
        self._base_path = base_path
        self._session_name_prefix = session_name_prefix
        self._events_ros = events_ros
        self._events_kernel = events_kernel
        self._package = package
        self._nodes = nodes

    def setUp(self):
        # Get timestamp before trace (ns)
        timestamp_before = int(time.time() * 1000000000.0)

        exit_code, full_path = run_and_trace(
            self._base_path,
            self._session_name_prefix,
            self._events_ros,
            self._events_kernel,
            self._package,
            self._nodes)

        print(f'TRACE DIRECTORY: {full_path}')
        self._exit_code = exit_code
        self._full_path = full_path

        # Check that setUp() ran fine
        self.assertEqual(self._exit_code, 0)

        # Read events once
        self._events = get_trace_events(self._full_path)
        self._event_names = get_event_names(self._events)
        self.assertGreater(len(self._events), 0, 'no events found in trace')

        # Check the timestamp of the first event
        self.assertEventAfterTimestamp(self._events[0], timestamp_before)

        # Check that the enabled events are present
        ros = set(self._events_ros) if self._events_ros is not None else set()
        kernel = set(self._events_kernel) if self._events_kernel is not None else set()
        all_event_names = ros | kernel
        self.assertSetEqual(all_event_names, set(self._event_names))

        # Check that the launched nodes are present as processes
        self.assertProcessNamesExist(self._nodes)

    def tearDown(self):
        pass
        # cleanup_trace(self._full_path)

    def assertEventsOrderSet(self, event_names: List[str]):
        """
        Compare given event names to trace events names as sets.

        :param event_names: the list of event names to compare to (as a set)
        """
        self.assertSetEqual(set(self._event_names), set(event_names), 'wrong events order')

    def assertProcessNamesExist(self, names: List[str]):
        """
        Check that the given processes exist.

        :param names: the node names to look for
        """
        procnames = [e['procname'] for e in self._events]
        for name in names:
            # Procnames have a max length of 15
            name_trimmed = name[:15]
            self.assertTrue(name_trimmed in procnames, 'node name not found in tracepoints')

    def assertValidHandle(self, event: DictEvent, handle_field_name: str):
        """
        Check that the handle associated to a field name is valid.

        :param event: the event which has a handle field
        :param handle_field_name: the field name of the handle to check
        """
        handle_field = self.get_field(event, handle_field_name)
        self.assertGreater(handle_field, 0, f'invalid handle: {handle_field_name}')

    def assertStringFieldNotEmpty(self, event: DictEvent, string_field_name: str):
        """
        Check that a string field is not empty.

        :param event: the event which has a string field
        :param string_field_name: the field name of the string field
        """
        string_field = self.get_field(event, string_field_name)
        self.assertGreater(len(string_field), 0, 'empty string')

    def assertEventAfterTimestamp(self, event: DictEvent, timestamp: int):
        self.assertGreater(get_event_timestamp(event), timestamp, 'event not after timestamp')

    def get_field(self, event: DictEvent, field_name: str) -> Any:
        """
        Get field value; will fail test if not found.

        :param event: the event from which to get the value
        :param field_name: the field name
        :return: the value
        """
        try:
            value = get_field(event, field_name, default=None, raise_if_not_found=True)
        except AttributeError as e:
            # Explicitly failing here
            self.fail(str(e))
        else:
            return value

    def get_events_with_name(self, event_name: str) -> List[DictEvent]:
        """
        Get all events with the given name.

        :param event_name: the event name
        :return: the list of events with the given name
        """
        return [e for e in self._events if get_event_name(e) == event_name]
