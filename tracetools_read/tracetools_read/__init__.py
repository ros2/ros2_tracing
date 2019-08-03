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

"""Module with functions for reading traces."""

from typing import Any
from typing import Dict
from typing import Iterable
from typing import List

import babeltrace


DictEvent = Dict[str, Any]


def _get_trace_ctf_events(trace_directory: str) -> Iterable[babeltrace.babeltrace.Event]:
    """
    Get the events of a trace.

    :param trace_directory: the path to the main/top trace directory
    :return: events iterable
    """
    tc = babeltrace.TraceCollection()
    tc.add_traces_recursive(trace_directory, 'ctf')
    return tc.events


def get_trace_events(trace_directory: str) -> List[DictEvent]:
    """
    Get the events of a trace.

    :param trace_directory: the path to the main/top trace directory
    :return: events
    """
    return [event_to_dict(event) for event in _get_trace_ctf_events(trace_directory)]


# List of ignored CTF fields
_IGNORED_FIELDS = [
    'content_size',
    'events_discarded',
    'id',
    'packet_size',
    'packet_seq_num',
    'stream_id',
    'stream_instance_id',
    'timestamp_end',
    'timestamp_begin',
    'magic',
    'uuid',
    'v',
]
_DISCARD = 'events_discarded'


def event_to_dict(event: babeltrace.babeltrace.Event) -> DictEvent:
    """
    Convert name, timestamp, and all other keys except those in IGNORED_FIELDS into a dictionary.

    :param event: the event to convert
    :return: the event as a dictionary
    """
    d = {'_name': event.name, '_timestamp': event.timestamp}
    if hasattr(event, _DISCARD) and event[_DISCARD] > 0:
        print(event[_DISCARD])
    for key in [key for key in event.keys() if key not in _IGNORED_FIELDS]:
        d[key] = event[key]
    return d


def get_field(event: DictEvent, field_name: str, default=None, raise_if_not_found=True) -> Any:
    field_value = event.get(field_name, default)
    # If enabled, raise exception as soon as possible to avoid headaches
    if raise_if_not_found and field_value is None:
        raise AttributeError(f"event field '{field_name}' not found for event: {event}")
    return field_value


def get_event_name(event: DictEvent) -> str:
    return event['_name']


def get_event_timestamp(event: DictEvent) -> int:
    return event['_timestamp']


def get_procname(event: DictEvent) -> str:
    return event['procname']
