import unittest
from tracetools_test.utils import (
    get_trace_event_names,
    run_and_trace,
    cleanup_trace,
)

PKG = 'tracetools_test'

subscription_callback_events = [
    'ros2:rclcpp_subscription_callback_start',
    'ros2:rclcpp_subscription_callback_end',
]

class TestSubscriptionCallback(unittest.TestCase):

    def test_callback(self):
        session_name_prefix = 'session-test-subscription-callback'
        base_path = '/tmp'
        test_nodes = ['test_ping', 'test_pong']

        exit_code, full_path = run_and_trace(base_path, session_name_prefix, subscription_callback_events, None, PKG, test_nodes)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(subscription_callback_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
