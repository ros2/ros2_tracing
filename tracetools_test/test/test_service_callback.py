import unittest

from tracetools_test.utils import (
    cleanup_trace,
    get_trace_event_names,
    run_and_trace,
)

BASE_PATH = '/tmp'
PKG = 'tracetools_test'
service_callback_events = [
    'ros2:callback_start',
    'ros2:callback_end',
]


class TestServiceCallback(unittest.TestCase):

    def test_callback(self):
        session_name_prefix = 'session-test-service-callback'
        test_nodes = ['test_service_ping', 'test_service_pong']

        exit_code, full_path = run_and_trace(
            BASE_PATH,
            session_name_prefix,
            service_callback_events,
            None,
            PKG,
            test_nodes)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(service_callback_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
