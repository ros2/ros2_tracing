# Copyright 2026 Christophe Bedard
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
import tempfile
import unittest
from pathlib import Path

from tracetools_test.case import TraceTestCase
from tracetools_trace.tools import tracepoints as tp
from tracetools_trace.tools.lttng import is_lttng_installed


# Enables Iceoryx shared memory in Cyclone DDS so dds_is_loan_available() can be true and
# rmw_take_loaned_message is used (see rmw_cyclonedds/shared_memory_support.md).
_CYCLO_DDS_SHM_XML = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/iceoryx/etc/cyclonedds.xsd">
    <Domain id="any">
        <SharedMemory>
            <Enable>true</Enable>
            <LogLevel>warn</LogLevel>
        </SharedMemory>
    </Domain>
</CycloneDDS>
"""


def _ensure_cyclonedds_shared_memory_enabled_for_loaning() -> None:
    """If unset, point CYCLONEDDS_URI at a minimal config with SharedMemory enabled."""
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        return
    if os.environ.get('CYCLONEDDS_URI'):
        return
    fd, path = tempfile.mkstemp(prefix='tracetools_pub_sub_loaning_', suffix='.xml')
    with os.fdopen(fd, 'w') as f:
        f.write(_CYCLO_DDS_SHM_XML)
    os.environ['CYCLONEDDS_URI'] = Path(path).as_uri()


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestPubSubWithMessageLoaning(TraceTestCase):
    """
    Tracing for pub/sub when subscriptions use rclcpp's loaned (unique_ptr) callback path.

    Registered in CMake only for rmw_cyclonedds_cpp, rmw_fastrtps_cpp, and
    rmw_fastrtps_dynamic_cpp. Uses std_msgs/msg/UInt32 so middleware can enable
    subscription loaning (ros2_tracing#240).

    For rmw_cyclonedds_cpp, shared memory must be enabled (CYCLONEDDS_URI) so that
    subscription loaning and rmw_take_loaned_message are available; Iceoryx RouDi
    (``iox-roudi``) should be running when SHM is enabled.
    """

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
            session_name_prefix='session-test-pub-sub-loaned',
            events_ros=[
                tp.rmw_publisher_init,
                tp.rcl_publisher_init,
                tp.rmw_publish,
                tp.rcl_publish,
                tp.rclcpp_publish,
                tp.rmw_subscription_init,
                tp.rcl_subscription_init,
                tp.rclcpp_subscription_init,
                tp.rclcpp_subscription_callback_added,
                tp.rmw_take,
                tp.callback_start,
                tp.callback_end,
            ],
            package='test_tracetools',
            nodes=['test_ping_loaned', 'test_pong_loaned'],
        )
        # Loaned message take on subscriptions is disabled by default in Rolling.
        # This test must explicitly enable it, otherwise the executor falls back to non-loaned take
        # and the test becomes unable to validate rmw_take trace coverage for loaned-take paths.
        os.environ['ROS_DISABLE_LOANED_MESSAGES'] = '0'
        _ensure_cyclonedds_shared_memory_enabled_for_loaning()

    def test_all(self):
        # Check events as set
        self.assertEventsSet(self._events_ros)

        # Get publisher init events & publisher handles of test topics
        rmw_pub_init_events = self.get_events_with_name(tp.rmw_publisher_init)
        rmw_sub_init_events = self.get_events_with_name(tp.rmw_subscription_init)
        publisher_init_events = self.get_events_with_name(tp.rcl_publisher_init)
        ping_publisher_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/ping',
            publisher_init_events,
            allow_multiple=False,
        )
        pong_publisher_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/pong',
            publisher_init_events,
            allow_multiple=False,
        )
        ping_pub_handle = self.get_field(ping_publisher_init_event, 'publisher_handle')
        ping_rmw_pub_handle = self.get_field(ping_publisher_init_event, 'rmw_publisher_handle')
        pong_pub_handle = self.get_field(pong_publisher_init_event, 'publisher_handle')
        pong_rmw_pub_handle = self.get_field(pong_publisher_init_event, 'rmw_publisher_handle')

        # Find corresponding rmw_pub_init events
        ping_rmw_pub_init_event = self.get_event_with_field_value_and_assert(
            'rmw_publisher_handle',
            ping_rmw_pub_handle,
            rmw_pub_init_events,
            allow_multiple=False,
        )
        pong_rmw_pub_init_event = self.get_event_with_field_value_and_assert(
            'rmw_publisher_handle',
            pong_rmw_pub_handle,
            rmw_pub_init_events,
            allow_multiple=False,
        )

        # Check publisher init order (rmw then rcl)
        self.assertEventOrder([
            ping_rmw_pub_init_event,
            ping_publisher_init_event,
        ])
        self.assertEventOrder([
            pong_rmw_pub_init_event,
            pong_publisher_init_event,
        ])

        # Get corresponding rmw/rcl/rclcpp publish events for ping & pong
        rmw_publish_events = self.get_events_with_name(tp.rmw_publish)
        ping_rmw_pub_event = self.get_event_with_field_value_and_assert(
            'rmw_publisher_handle',
            ping_rmw_pub_handle,
            rmw_publish_events,
            allow_multiple=False,
        )
        pong_rmw_pub_event = self.get_event_with_field_value_and_assert(
            'rmw_publisher_handle',
            pong_rmw_pub_handle,
            rmw_publish_events,
            allow_multiple=False,
        )
        ping_pub_message = self.get_field(ping_rmw_pub_event, 'message')
        pong_pub_message = self.get_field(pong_rmw_pub_event, 'message')

        rclcpp_publish_events = self.get_events_with_name(tp.rclcpp_publish)
        rcl_publish_events = self.get_events_with_name(tp.rcl_publish)
        ping_rclcpp_pub_event = self.get_event_with_field_value_and_assert(
            'message',
            ping_pub_message,
            rclcpp_publish_events,
            allow_multiple=False,
        )
        pong_rclcpp_pub_event = self.get_event_with_field_value_and_assert(
            'message',
            pong_pub_message,
            rclcpp_publish_events,
            allow_multiple=False,
        )
        ping_rcl_pub_event = self.get_event_with_field_value_and_assert(
            'message',
            ping_pub_message,
            rcl_publish_events,
            allow_multiple=False,
        )
        pong_rcl_pub_event = self.get_event_with_field_value_and_assert(
            'message',
            pong_pub_message,
            rcl_publish_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(ping_rcl_pub_event, 'publisher_handle', ping_pub_handle)
        self.assertFieldEquals(pong_rcl_pub_event, 'publisher_handle', pong_pub_handle)

        # Get subscription init events & subscription handles of test topics
        rcl_subscription_init_events = self.get_events_with_name(tp.rcl_subscription_init)
        ping_rcl_subscription_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/ping',
            rcl_subscription_init_events,
            allow_multiple=False,
        )
        pong_rcl_subscription_init_event = self.get_event_with_field_value_and_assert(
            'topic_name',
            '/pong',
            rcl_subscription_init_events,
            allow_multiple=False,
        )
        ping_sub_handle = self.get_field(ping_rcl_subscription_init_event, 'subscription_handle')
        ping_rmw_sub_handle = \
            self.get_field(ping_rcl_subscription_init_event, 'rmw_subscription_handle')
        pong_sub_handle = self.get_field(pong_rcl_subscription_init_event, 'subscription_handle')
        pong_rmw_sub_handle = \
            self.get_field(pong_rcl_subscription_init_event, 'rmw_subscription_handle')

        # Find corresponding rmw_sub_init events
        ping_rmw_sub_init_event = self.get_event_with_field_value_and_assert(
            'rmw_subscription_handle',
            ping_rmw_sub_handle,
            rmw_sub_init_events,
            allow_multiple=False,
        )
        pong_rmw_sub_init_event = self.get_event_with_field_value_and_assert(
            'rmw_subscription_handle',
            pong_rmw_sub_handle,
            rmw_sub_init_events,
            allow_multiple=False,
        )

        # Get corresponding subscription objects
        rclcpp_subscription_init_events = self.get_events_with_name(
            tp.rclcpp_subscription_init,
        )
        ping_rclcpp_subscription_init_event = self.get_event_with_field_value_and_assert(
            'subscription_handle',
            ping_sub_handle,
            rclcpp_subscription_init_events,
            allow_multiple=False,
        )
        pong_rclcpp_subscription_init_event = self.get_event_with_field_value_and_assert(
            'subscription_handle',
            pong_sub_handle,
            rclcpp_subscription_init_events,
            allow_multiple=False,
        )
        ping_sub_object = self.get_field(ping_rclcpp_subscription_init_event, 'subscription')
        pong_sub_object = self.get_field(pong_rclcpp_subscription_init_event, 'subscription')

        # Get corresponding subscription callback objects
        rclcpp_subscription_callback_events = self.get_events_with_name(
            tp.rclcpp_subscription_callback_added,
        )
        ping_rclcpp_subscription_callback_event = self.get_event_with_field_value_and_assert(
            'subscription',
            ping_sub_object,
            rclcpp_subscription_callback_events,
            allow_multiple=False,
        )
        pong_rclcpp_subscription_callback_event = self.get_event_with_field_value_and_assert(
            'subscription',
            pong_sub_object,
            rclcpp_subscription_callback_events,
            allow_multiple=False,
        )
        ping_callback_object = self.get_field(ping_rclcpp_subscription_callback_event, 'callback')
        pong_callback_object = self.get_field(pong_rclcpp_subscription_callback_event, 'callback')

        # Loaned receive path must emit rmw_take with taken=1 (see ros2_tracing#240)
        rmw_take_events = self.get_events_with_name(tp.rmw_take)
        rmw_take_taken_events = [
            e for e in rmw_take_events
            if self.get_field(e, 'taken') == 1
        ]
        pong_rmw_take_event = self.get_event_with_field_value_and_assert(
            'rmw_subscription_handle',
            pong_rmw_sub_handle,
            rmw_take_taken_events,
            allow_multiple=False,
        )
        ping_rmw_take_event = self.get_event_with_field_value_and_assert(
            'rmw_subscription_handle',
            ping_rmw_sub_handle,
            rmw_take_taken_events,
            allow_multiple=False,
        )
        self.assertFieldEquals(ping_rmw_take_event, 'taken', 1, 'ping subscription rmw_take')
        self.assertFieldEquals(pong_rmw_take_event, 'taken', 1, 'pong subscription rmw_take')

        # Check that pub->sub timestamps match
        ping_timestamp = self.get_field(ping_rmw_pub_event, 'timestamp')
        self.assertFieldEquals(ping_rmw_take_event, 'source_timestamp', ping_timestamp)
        pong_timestamp = self.get_field(pong_rmw_pub_event, 'timestamp')
        self.assertFieldEquals(pong_rmw_take_event, 'source_timestamp', pong_timestamp)

        # Check subscription init order
        self.assertEventOrder([
            ping_rmw_sub_init_event,
            ping_rcl_subscription_init_event,
            ping_rclcpp_subscription_init_event,
            ping_rclcpp_subscription_callback_event,
        ])
        self.assertEventOrder([
            pong_rmw_sub_init_event,
            pong_rcl_subscription_init_event,
            pong_rclcpp_subscription_init_event,
            pong_rclcpp_subscription_callback_event,
        ])

        # Get corresponding callback start/end events
        callback_start_events = self.get_events_with_name(tp.callback_start)
        callback_end_events = self.get_events_with_name(tp.callback_end)
        ping_callback_start_event = self.get_event_with_field_value_and_assert(
            'callback',
            ping_callback_object,
            callback_start_events,
            allow_multiple=False,
        )
        pong_callback_start_event = self.get_event_with_field_value_and_assert(
            'callback',
            pong_callback_object,
            callback_start_events,
            allow_multiple=False,
        )
        ping_callback_end_event = self.get_event_with_field_value_and_assert(
            'callback',
            ping_callback_object,
            callback_end_events,
            allow_multiple=False,
        )
        pong_callback_end_event = self.get_event_with_field_value_and_assert(
            'callback',
            pong_callback_object,
            callback_end_events,
            allow_multiple=False,
        )

        self.assertEventOrder([
            ping_rclcpp_pub_event,
            ping_rcl_pub_event,
            ping_rmw_pub_event,
            ping_rmw_take_event,
            ping_callback_start_event,
            pong_rclcpp_pub_event,
            pong_rcl_pub_event,
            pong_rmw_pub_event,
            ping_callback_end_event,
        ])
        self.assertEventOrder([
            pong_rclcpp_pub_event,
            pong_rcl_pub_event,
            pong_rmw_pub_event,
            pong_rmw_take_event,
            pong_callback_start_event,
            pong_callback_end_event,
        ])


if __name__ == '__main__':
    unittest.main()
