from ros2cli.command import CommandExtension
from ros2trace.api import add_trace_arguments
from ros2trace.api import init
from ros2trace.api import fini


class TraceCommand(CommandExtension):
    """Trace ROS."""

    def add_arguments(self, parser, cli_name):
        add_trace_arguments(parser)

    def main(self, *, parser, args):
        init(args)
        fini(args)
        return 0
