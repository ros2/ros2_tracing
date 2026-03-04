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
            session_name_prefix='session-test-message-link-periodic-async',
            events_ros=[
                tp.rcl_publisher_init,
                tp.rcl_subscription_init,
                tp.message_link_periodic_async,
            ],
            package='test_tracetools',
            nodes=['test_ping', 'test_pong', 'test_message_link_periodic_async'],
        )

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # The test_message_link_periodic_async node subscribes to /ping and /pong,
        # and publishes to /combined periodically. It annotates this relationship  between
        # both subscriptions and the publisher with message_link_periodic_async.

        # Get subscription handles of input topics and publisher handle of output topic
        rcl_subscription_init_events = self.get_events_with_name(
            tp.rcl_subscription_init
        )
        message_link_rcl_subscription_init_events = self.get_events_with_procname(
            'test_message_link_periodic_async',
            rcl_subscription_init_events,
        )
        ping_rcl_subscription_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/ping',
            message_link_rcl_subscription_init_events,
            allow_multiple=False,
        )
        ping_sub_handle = self.get_field(
            ping_rcl_subscription_init_event, 'subscription_handle'
        )
        pong_rcl_subscription_init_event = (
            self.get_event_with_field_value_and_assert(
                'topic_name',
                '/pong',
                message_link_rcl_subscription_init_events,
                allow_multiple=False,
            )
        )
        pong_sub_handle = self.get_field(
            pong_rcl_subscription_init_event,
            'subscription_handle',
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

        # Get the message_link_periodic_async event
        message_link_events = self.get_events_with_name(tp.message_link_periodic_async)
        self.assertEqual(
            len(message_link_events),
            1,
            'Expected exactly one message_link_periodic_async event',
        )
        message_link_event = message_link_events[0]

        # Verify the event contains the correct subscription and publisher handles
        # The event should have both subscriptions linked to the combined publisher
        link_subs = self.get_field(message_link_event, 'subs')
        link_pubs = self.get_field(message_link_event, 'pubs')

        self.assertEqual(len(link_subs), 2, 'Expected 2 subscription handles')
        self.assertIn(
            ping_sub_handle,
            link_subs,
            'Expected /ping subscription handle in message link',
        )
        self.assertIn(
            pong_sub_handle,
            link_subs,
            'Expected /pong subscription handle in message link',
        )
        self.assertEqual(len(link_pubs), 1, 'Expected 1 publisher handle')
        self.assertIn(
            pub_handle,
            link_pubs,
            'Expected /combined publisher handle in message link',
        )


if __name__ == '__main__':
    unittest.main()
