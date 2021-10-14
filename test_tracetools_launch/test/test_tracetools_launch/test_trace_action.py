# Copyright 2019 Robert Bosch GmbH
# Copyright 2021 Christophe Bedard
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

import io
import pathlib
import shutil
import tempfile
import textwrap
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch.frontend import Parser

from tracetools_launch.action import Trace
from tracetools_trace.tools.lttng import is_lttng_installed


class TestTraceAction(unittest.TestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def _assert_launch_no_errors(self, actions) -> None:
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def _assert_launch_frontend_no_errors(self, file) -> Trace:
        root_entity, parser = Parser.load(file)
        ld = parser.parse_description(root_entity)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 == ls.run()
        trace_action = ld.describe_sub_entities()[0]
        return trace_action

    def _check_trace_action(self, action, tmpdir) -> None:
        self.assertEqual('my-session-name', action.session_name)
        self.assertEqual(tmpdir, action.base_path)
        self.assertTrue(action.trace_directory.startswith(tmpdir))
        self.assertEqual([], action.events_kernel)
        self.assertEqual(['ros2:*', '*'], action.events_ust)
        self.assertTrue(pathlib.Path(tmpdir).exists())

    @unittest.skipIf(not is_lttng_installed(), 'LTTng is required')
    def test_action(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action')

        # Disable kernel events just to not require kernel tracing for the test
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir)

        shutil.rmtree(tmpdir)

    @unittest.skipIf(not is_lttng_installed(), 'LTTng is required')
    def test_action_frontend_xml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_xml')

        xml_file = textwrap.dedent(
            r"""
            <launch>
                <trace
                    session-name="my-session-name"
                    base-path="{}"
                    events-kernel=""
                    events-ust="ros2:* *"
                />
            </launch>
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(xml_file) as f:
            trace_action = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, tmpdir)

        shutil.rmtree(tmpdir)

    @unittest.skipIf(not is_lttng_installed(), 'LTTng is required')
    def test_action_frontend_yaml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_yaml')

        yaml_file = textwrap.dedent(
            r"""
            launch:
            - trace:
                session-name: my-session-name
                base-path: {}
                events-kernel: ""
                events-ust: ros2:* *
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(yaml_file) as f:
            trace_action = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, tmpdir)

        shutil.rmtree(tmpdir)


if __name__ == '__main__':
    unittest.main()
