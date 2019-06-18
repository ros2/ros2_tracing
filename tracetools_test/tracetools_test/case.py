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

from typing import List
from typing import Set
import unittest

from .utils import cleanup_trace
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
        exit_code, full_path = run_and_trace(
            self._base_path,
            self._session_name_prefix,
            self._events_ros,
            self._events_kernel,
            self._package,
            self._nodes)

        print(f'trace directory: {full_path}')
        self._exit_code = exit_code
        self._full_path = full_path

        # Check that setUp() ran fine
        self.assertEqual(self._exit_code, 0)

        # Read events once
        self._events = get_trace_events(self._full_path)
        self._event_names = self._get_event_names()

        # Check that the enabled events are present
        ros = set(self._events_ros) if self._events_ros is not None else set()
        kernel = set(self._events_kernel) if self._events_kernel is not None else set()
        all_event_names = ros | kernel
        self.assertSetEqual(all_event_names, self._event_names)

    def tearDown(self):
        cleanup_trace(self._full_path)

    def _get_event_names(self) -> Set[str]:
        """Get a set of names of the events in the trace."""
        events_names = set()
        for event in self._events:
            events_names.add(event.name)
        return events_names
