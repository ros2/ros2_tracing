import unittest
from tracetools_test.utils import (
    get_trace_event_names,
    run_and_trace,
    cleanup_trace,
)

PKG = 'tracetools_test'

node_creation_events = [
    'ros2:rcl_init',
    'ros2:rcl_node_init',
]

class TestNode(unittest.TestCase):

    def test_creation(self):
        session_name_prefix = 'session-test-node-creation'
        base_path = '/tmp'
        test_node = ['test_publisher']

        exit_code, full_path = run_and_trace(base_path, session_name_prefix, node_creation_events, None, PKG, test_node)
        self.assertEqual(exit_code, 0)

        trace_events = get_trace_event_names(full_path)
        print(f'trace_events: {trace_events}')
        self.assertSetEqual(set(node_creation_events), trace_events)

        cleanup_trace(full_path)


if __name__ == '__main__':
    unittest.main()
