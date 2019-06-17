import unittest

from tracetools_test.utils import (
    cleanup_trace,
    get_trace_event_names,
    run_and_trace,
)

BASE_PATH = '/tmp'
PKG = 'tracetools_test'
service_creation_events = [
    'ros2:rcl_service_init',
    'ros2:rclcpp_service_callback_added',
]


class TestService(unittest.TestCase):

    def test_creation(self):
        session_name_prefix = 'session-test-service-creation'
        test_nodes = ['test_service']

        exit_code, full_path = run_and_trace(
            BASE_PATH,
            session_name_prefix,
            service_creation_events,
            None,
            PKG,
            test_nodes)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(service_creation_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
