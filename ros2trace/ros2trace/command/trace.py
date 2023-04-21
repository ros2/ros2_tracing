# Copyright 2019 Robert Bosch GmbH
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

"""Module for trace command extension implementation."""

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension
from tracetools_trace.tools import args
from tracetools_trace.trace import trace


class TraceCommand(CommandExtension):
    """Trace ROS 2 nodes to get information on their execution. The main 'trace' command requires user interaction; to trace non-interactively, use the 'start'/'stop'/'pause'/'resume' sub-commands."""  # noqa: E501

    def add_arguments(self, parser, cli_name) -> None:
        self._subparser = parser
        args.add_arguments(parser)

        # Add arguments and sub-commands of verbs
        add_subparsers_on_demand(parser, cli_name, '_verb', 'ros2trace.verb', required=False)

    def main(self, *, parser, args) -> int:
        if not hasattr(args, '_verb'):
            # In case no verb was passed, do interactive tracing
            return trace(args)

        extension = getattr(args, '_verb')

        # Call the verb's main method
        return extension.main(args=args)
