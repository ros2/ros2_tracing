# Example launch file for the Trace action

from launch import LaunchDescription
from launch_ros.actions import Node
from tracetools_launch.trace import Trace


def generate_launch_description():
    return LaunchDescription([
        Trace(
            session_name='my-tracing-session',
            base_path='/tmp'),
        Node(
            package='examples_rclcpp_minimal_publisher',
            node_executable='publisher_member_function',
            output='screen'),
        Node(
            package='examples_rclcpp_minimal_subscriber',
            node_executable='subscriber_member_function',
            output='screen'),
    ])
