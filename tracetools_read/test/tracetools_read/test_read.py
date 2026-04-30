# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import os
import platform
import shutil
import tempfile
from typing import Any
from typing import Dict
from typing import List
from typing import Tuple
from typing import TypeAlias
import unittest

if 'Linux' != platform.system():
    raise unittest.SkipTest('Linux-specific test')

from tracetools_read import get_event_name
from tracetools_read import get_event_pid
from tracetools_read import get_events_with_field_value
from tracetools_read import get_events_with_name
from tracetools_read import get_field
from tracetools_read import get_procname
from tracetools_read import get_tid
from tracetools_read.trace import get_trace_events
from tracetools_read.trace import is_trace_directory


class TestRead(unittest.TestCase):

    TraceEvent: TypeAlias = Tuple[str, Dict[str, Any]]
    TraceEvents: TypeAlias = List[TraceEvent]

    trace_dir: str

    # Debug env var. If set, don't cleanup test trace directory.
    ENV_VAR_DEBUG = 'TRACETOOLS_READ_DEBUG'

    # Events to write into the trace, in order. Used to verify what is read back.
    expected_events: TraceEvents = [
        ('event_a', {'value': 1, 'label': 'one'}),
        ('event_b', {'value': 2}),
        ('event_a', {'value': 3, 'label': 'three'}),
        ('event_a', {'value': 1, 'label': 'duplicate-value'}),
    ]
    expected_procname = 'test_proc'
    expected_vpid = 1234
    expected_vtid = 5678
    expected_cpu_id = 27

    @classmethod
    def setUpClass(cls) -> None:
        cls.trace_dir = cls.write_test_trace(
            cls.expected_events,
            cls.expected_procname,
            cls.expected_vpid,
            cls.expected_vtid,
            cls.expected_cpu_id,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        if not os.environ.get(cls.ENV_VAR_DEBUG, None):
            shutil.rmtree(cls.trace_dir)

    @staticmethod
    def write_test_trace(
        events: TraceEvents,
        procname: str,
        vpid: int,
        vtid: int,
        cpu_id: int,
    ) -> str:
        """
        Write a test trace with the given events.

        This uses the babeltrace CTF writer API to write a trace. We could also just keep a real
        trace around.

        :param events: The events to write.
        :return: The path to the trace directory.
        """
        import babeltrace

        trace_dir = tempfile.mkdtemp(prefix='tracetools_read__test_trace_')

        writer = babeltrace.CTFWriter.Writer(trace_dir)

        clock = babeltrace.CTFWriter.Clock('test_clock')
        writer.add_clock(clock)

        stream_class = babeltrace.CTFWriter.StreamClass('test_stream')
        stream_class.clock = clock

        # Build one event class per unique event name, inferring field
        # declarations from the first payload seen for that name.
        event_classes: Dict[str, Any] = {}
        for name, payload in events:
            if name in event_classes:
                continue
            event_class = babeltrace.CTFWriter.EventClass(name)
            for field_name, field_value in payload.items():
                if isinstance(field_value, str):
                    decl = babeltrace.CTFWriter.StringFieldDeclaration()
                elif isinstance(field_value, int):
                    decl = babeltrace.CTFWriter.IntegerFieldDeclaration(32)
                    decl.signed = False
                else:
                    raise TypeError(
                        f'unsupported field type for {field_name!r}: '
                        f'{type(field_value).__name__}'
                    )
                event_class.add_field(decl, field_name)
            # Fake context fields, populated on every event below.
            event_class.add_field(
                babeltrace.CTFWriter.StringFieldDeclaration(), 'procname')
            for ctx_name in ('vpid', 'vtid', 'cpu_id'):
                ctx_decl = babeltrace.CTFWriter.IntegerFieldDeclaration(32)
                ctx_decl.signed = False
                event_class.add_field(ctx_decl, ctx_name)
            stream_class.add_event_class(event_class)
            event_classes[name] = event_class

        stream = writer.create_stream(stream_class)

        for i, (name, payload) in enumerate(events):
            clock.time = (i + 1) * 1000
            event = babeltrace.CTFWriter.Event(event_classes[name])
            for field_name, field_value in payload.items():
                event.payload(field_name).value = field_value
            # The babeltrace Python API doesn't seem to support providing context fields directly,
            # but our reading code kind of treats them the same, so it doesn't really matter
            event.payload('procname').value = procname
            event.payload('vpid').value = vpid
            event.payload('vtid').value = vtid
            event.payload('cpu_id').value = cpu_id
            stream.append_event(event)

        stream.flush()
        print(f'Trace directory: {trace_dir}')
        return trace_dir

    def test_is_trace_directory_with_non_trace(self) -> None:
        # Directory exists but is not a trace directory
        non_trace_dir = tempfile.mkdtemp(prefix='tracetools_read__test_nontrace_')
        self.assertFalse(is_trace_directory(non_trace_dir))
        shutil.rmtree(non_trace_dir)

    def test_is_trace_directory_with_trace(self) -> None:
        self.assertTrue(is_trace_directory(self.trace_dir))

    def test_get_trace_events(self) -> None:
        events = get_trace_events(self.trace_dir)
        self.assertEqual(len(events), len(self.expected_events))
        for actual, (name, payload) in zip(events, self.expected_events):
            self.assertEqual(get_event_name(actual), name)
            for field_name, field_value in payload.items():
                self.assertEqual(actual[field_name], field_value)

    def test_get_field(self) -> None:
        events = get_trace_events(self.trace_dir)
        first = events[0]
        # Field present
        self.assertEqual(get_field(first, 'value'), 1)
        # Field absent, custom default returned (no raise since result is non-None)
        self.assertEqual(
            get_field(first, 'missing', default='fallback'),
            'fallback',
        )
        # Field absent, raise suppressed -> None
        self.assertIsNone(get_field(first, 'missing', raise_if_not_found=False))
        # Field absent, default behavior raises
        with self.assertRaises(AttributeError):
            get_field(first, 'missing')

    def test_get_events_with_name(self) -> None:
        events = get_trace_events(self.trace_dir)
        a_events = get_events_with_name('event_a', events)
        b_events = get_events_with_name('event_b', events)
        self.assertEqual(len(a_events), 3)
        self.assertEqual(len(b_events), 1)
        for event in a_events:
            self.assertEqual(get_event_name(event), 'event_a')
        self.assertEqual(get_events_with_name('event_missing', events), [])

    def test_get_event_pid_procname_tid(self) -> None:
        events = get_trace_events(self.trace_dir)
        for event in events:
            self.assertEqual(get_procname(event), self.expected_procname)
            self.assertEqual(get_event_pid(event), self.expected_vpid)
            self.assertEqual(get_tid(event), self.expected_vtid)

    def test_get_events_with_field_value(self) -> None:
        events = get_trace_events(self.trace_dir)
        # Single value: two events have value=1
        single = get_events_with_field_value('value', 1, events)
        self.assertEqual(len(single), 2)
        for event in single:
            self.assertEqual(get_field(event, 'value'), 1)
        # List of values
        multi = get_events_with_field_value('value', [1, 2], events)
        self.assertEqual(len(multi), 3)
        # No matches
        self.assertEqual(get_events_with_field_value('value', 999, events), [])


if __name__ == '__main__':
    unittest.main()
