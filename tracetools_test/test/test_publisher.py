import time
import shutil
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros import get_default_launch_description
import launch_ros.actions
import unittest
from tracetools_analysis.test.utils import get_trace_event_names
from tracetools_trace.tools.lttng import (
    lttng_setup,
    lttng_start,
    lttng_stop,
    lttng_destroy,
)

PKG = 'tracetools_test'

publisher_creation_events = [
    'ros2:rcl_publisher_init',
]

class TestPublisher(unittest.TestCase):

    def test_creation(self):
        session_name = f'session-test-publisher-creation-{time.strftime("%Y%m%d%H%M%S")}'
        path = '/tmp/' + session_name
        print(f'trace directory: {path}')

        lttng_setup(session_name, path, ros_events=publisher_creation_events, kernel_events=None)
        lttng_start(session_name)

        ld = LaunchDescription([
            launch_ros.actions.Node(
                package=PKG, node_executable='test_publisher', output='screen'),
        ])
        ls = LaunchService()
        ls.include_launch_description(get_default_launch_description())
        ls.include_launch_description(ld)

        exit_code = ls.run()
        self.assertEqual(exit_code, 0)

        lttng_stop(session_name)
        lttng_destroy(session_name)

        trace_events = get_trace_event_names(path)
        print(f'trace_events: {trace_events}')
        self.assertListEqual(publisher_creation_events, list(trace_events))

        shutil.rmtree(path)

if __name__ == '__main__':
    unittest.main()
