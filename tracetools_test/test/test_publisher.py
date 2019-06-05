import unittest

from tracetools_test.utils import (
    cleanup_trace,
    get_trace_event_names,
    run_and_trace,
)

BASE_PATH = '/tmp'
PKG = 'tracetools_test'
publisher_creation_events = [
    'ros2:rcl_publisher_init',
]


class TestPublisher(unittest.TestCase):

    def test_creation(self):
        session_name_prefix = 'session-test-publisher-creation'
        test_node = ['test_publisher']

        exit_code, full_path = run_and_trace(BASE_PATH,
                                             session_name_prefix,
                                             publisher_creation_events,
                                             None,
                                             PKG,
                                             test_node)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(publisher_creation_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
