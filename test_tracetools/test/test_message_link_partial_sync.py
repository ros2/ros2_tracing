# Copyright 2026 Raphael van Kempen
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
from tracetools_trace.tools import tracepoints as tp
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestPubSub(TraceTestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-message-link-partial-sync',
            events_ros=[
                tp.rcl_publisher_init,
                tp.rcl_subscription_init,
                tp.message_link_partial_sync,
            ],
            package='test_tracetools',
            nodes=['test_ping', 'test_pong', 'test_message_link_partial_sync'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # The test_message_link_partial_sync node subscribes to /ping and /pong and publishes
        # to /combined. While the relationship between /pong and /combined can be inferred
        # directly, the link to /ping is annotated with message_link_partial_sync.

        # Get subscription handles of /ping topic and publisher handle of /combined topic
        rcl_subscription_init_events = self.get_events_with_name(
            tp.rcl_subscription_init
        )
        ping_rcl_subscription_init_events = self.get_events_with_procname(
            'test_message_link_partial_sync',
            rcl_subscription_init_events,
        )
        ping_rcl_subscription_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/ping',
            ping_rcl_subscription_init_events,
            allow_multiple=False,
        )
        ping_sub_handle = self.get_field(
            ping_rcl_subscription_init_event, 'subscription_handle'
        )
        publisher_init_events = self.get_events_with_name(tp.rcl_publisher_init)
        combined_pub_init_event = (
            self.get_event_with_field_value_and_assert(
                'topic_name',
                '/combined',
                publisher_init_events,
                allow_multiple=False,
            )
        )
        pub_handle = self.get_field(
            combined_pub_init_event, 'publisher_handle'
        )

        # Get the message_link_partial_sync event
        message_link_events = self.get_events_with_name(tp.message_link_partial_sync)
        self.assertEqual(
            len(message_link_events),
            1,
            'Expected exactly one message_link_partial_sync event',
        )
        message_link_event = message_link_events[0]

        # Verify the event contains the correct subscription and publisher handles
        # The event should have the /ping subscription linked to the /combined publisher
        link_subs = self.get_field(message_link_event, 'subs')
        link_pubs = self.get_field(message_link_event, 'pubs')

        self.assertEqual(len(link_subs), 1, 'Expected 1 subscription handle')
        self.assertIn(
            ping_sub_handle,
            link_subs,
            'Expected /ping subscription handle in message link',
        )
        self.assertEqual(len(link_pubs), 1, 'Expected 1 publisher handle')
        self.assertIn(
            pub_handle,
            link_pubs,
            'Expected /combined publisher handle in message link',
        )


if __name__ == '__main__':
    unittest.main()
