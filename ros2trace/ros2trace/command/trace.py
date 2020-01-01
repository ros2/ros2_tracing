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

from ros2cli.command import CommandExtension
from tracetools_trace.tools import args
from tracetools_trace.trace import fini
from tracetools_trace.trace import init


class TraceCommand(CommandExtension):
    """Trace ROS nodes to get information on their execution."""

    def add_arguments(self, parser, cli_name):
        args.add_arguments(parser)

    def main(self, *, parser, args):
        init(
            args.session_name,
            args.path,
            args.events_ust,
            args.events_kernel,
            args.context_names,
            args.list,
        )
        fini(
            args.session_name,
        )
        return 0
