# Copyright 2024 Apex.AI, Inc.
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
from typing import List

from tracetools_read import DictEvent
from tracetools_read import get_event_pid
from tracetools_read import get_events_with_field_value
from tracetools_read import get_events_with_name
from tracetools_read import get_field


# Name of environment variable used by a trace test to communicate its trace test ID
TRACE_TEST_ID_ENV_VAR = 'TRACETOOLS_TEST_TRACE_TEST_ID'
# Name of trace event used to register a trace test ID for a given process
TRACE_TEST_ID_TP_NAME = 'lttng_ust_tracef:event'
# Field name of trace event used to register a trace test ID for a given process
TRACE_TEST_ID_TP_FIELD_NAME = 'msg'


def get_trace_test_id(id_prefix: str) -> str:
    """
    Get a trace test ID value for this test.

    The environment for the processes spawned by the current test (e.g., test application process)
    should include the environment variable with name `$TRACE_TEST_ID_ENV_VAR` and with this value.

    The string ID prefix should uniquely-enough represent the test.

    :param id_prefix: a string ID prefix
    :return: trace test ID for this test
    """
    # Append the current PID to make the ID unique
    return f'{id_prefix}#{os.getpid()}'


def _get_trace_test_id_event_value(trace_test_id: str) -> str:
    """
    Get value for trace test ID event field (`$TRACE_TEST_ID_TP_NAME`).

    :param trace_test_id: the trace test ID
    :return: the trace test ID marker event field value
    """
    return f'{TRACE_TEST_ID_ENV_VAR}={trace_test_id}'


def get_corresponding_trace_test_events(
    events: List[DictEvent],
    trace_test_id: str,
) -> List[DictEvent]:
    """
    Get trace events corresponding to the current trace test.

    :param events: all of the trace events
    :param trace_test_id: the trace test ID
    :return: the relevant trace events corresponding to the given test
    """
    expected_event_msg = _get_trace_test_id_event_value(trace_test_id)
    events_id = get_events_with_name(TRACE_TEST_ID_TP_NAME, events)
    pids = {
        get_event_pid(event_id)
        for event_id in events_id
        if get_field(event_id, TRACE_TEST_ID_TP_FIELD_NAME) == expected_event_msg
    }
    return get_events_with_field_value('vpid', pids, events)
