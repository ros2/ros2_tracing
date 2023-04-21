# Copyright 2023 Apex.AI, Inc.
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

from ros2trace.verb import VerbExtension
from tracetools_trace.tools import args
from tracetools_trace.trace import start


class StartVerb(VerbExtension):
    """Configure tracing session and start tracing."""

    def add_arguments(self, parser, cli_name) -> None:
        args.add_arguments_noninteractive(parser)

    def main(self, *, args) -> int:
        return start(args)
