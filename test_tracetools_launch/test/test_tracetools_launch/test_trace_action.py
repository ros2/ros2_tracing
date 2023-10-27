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
import os
import pathlib
import shutil
import tempfile
import textwrap
from typing import List
from typing import Optional
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.frontend import Parser
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

from tracetools_launch.action import Trace
from tracetools_trace.tools.lttng import is_lttng_installed


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestTraceAction(unittest.TestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def setUp(self) -> None:
        self.assertIsNone(os.environ.get('LD_PRELOAD'))

    def tearDown(self) -> None:
        if 'LD_PRELOAD' in os.environ:
            del os.environ['LD_PRELOAD']

    def _assert_launch(self, actions) -> int:
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        return ls.run()

    def _assert_launch_no_errors(self, actions) -> None:
        self.assertEqual(0, self._assert_launch(actions), 'expected no errors')

    def _assert_launch_errors(self, actions) -> None:
        self.assertEqual(1, self._assert_launch(actions), 'expected errors')

    def _assert_launch_frontend_no_errors(self, file) -> Trace:
        root_entity, parser = Parser.load(file)
        ld = parser.parse_description(root_entity)
        ls = LaunchService()
        ls.include_launch_description(ld)
        self.assertEqual(0, ls.run(), 'expected no errors')
        trace_action = ld.describe_sub_entities()[0]
        return trace_action

    def _check_trace_action(
        self,
        action,
        tmpdir,
        *,
        session_name: Optional[str] = 'my-session-name',
        append_trace: bool = False,
        events_ust: List[str] = ['ros2:*', '*'],
        subbuffer_size_ust: int = 524288,
        subbuffer_size_kernel: int = 1048576,
    ) -> None:
        if session_name is not None:
            self.assertEqual(session_name, action.session_name)
        self.assertEqual(tmpdir, action.base_path)
        self.assertTrue(action.trace_directory.startswith(tmpdir))
        self.assertEqual(append_trace, action.append_trace)
        self.assertEqual([], action.events_kernel)
        self.assertEqual(events_ust, action.events_ust)
        self.assertTrue(pathlib.Path(tmpdir).exists())
        self.assertEqual(subbuffer_size_ust, action.subbuffer_size_ust)
        self.assertEqual(subbuffer_size_kernel, action.subbuffer_size_kernel)

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
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir)

        shutil.rmtree(tmpdir)

    def test_action_frontend_xml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_xml')

        xml_file = textwrap.dedent(
            r"""
            <launch>
                <trace
                    session-name="my-session-name"
                    append-timestamp="false"
                    base-path="{}"
                    append-trace="true"
                    events-kernel=""
                    events-ust="ros2:* *"
                    subbuffer-size-ust="524288"
                    subbuffer-size-kernel="1048576"
                />
            </launch>
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(xml_file) as f:
            trace_action = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_frontend_yaml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_yaml')

        yaml_file = textwrap.dedent(
            r"""
            launch:
            - trace:
                session-name: my-session-name
                append-timestamp: false
                base-path: {}
                append-trace: true
                events-kernel: ""
                events-ust: ros2:* *
                subbuffer-size-ust: 524288
                subbuffer-size-kernel: 1048576
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(yaml_file) as f:
            trace_action = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_context_per_domain(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_context_per_domain')

        # Invalid context domain
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            context_fields={
                'some_unknown_domain_type': [],
                'userspace': ['vpid', 'vtid'],
            },
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_errors([action])

        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            context_fields={
                'kernel': [],
                'userspace': ['vpid', 'vtid'],
            },
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir)

        self.assertDictEqual(
            action.context_fields,
            {
                'kernel': [],
                'userspace': ['vpid', 'vtid'],
            },
        )

        shutil.rmtree(tmpdir)

    def test_action_substitutions(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_substitutions')

        self.assertIsNone(os.environ.get('TestTraceAction__event_ust', None))
        os.environ['TestTraceAction__event_ust'] = 'ros2:*'
        self.assertIsNone(os.environ.get('TestTraceAction__context_field', None))
        os.environ['TestTraceAction__context_field'] = 'vpid'

        session_name_arg = DeclareLaunchArgument(
            'session-name',
            default_value='my-session-name',
            description='the session name',
        )
        action = Trace(
            session_name=LaunchConfiguration(session_name_arg.name),
            base_path=TextSubstitution(text=tmpdir),
            events_kernel=[],
            events_ust=[
                EnvironmentVariable(name='TestTraceAction__event_ust'),
                TextSubstitution(text='*'),
            ],
            context_fields={
                'kernel': [],
                'userspace': [
                    EnvironmentVariable(name='TestTraceAction__context_field'),
                    TextSubstitution(text='vtid'),
                ],
            },
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([session_name_arg, action])
        self._check_trace_action(action, tmpdir)

        self.assertDictEqual(
            action.context_fields,
            {
                'kernel': [],
                'userspace': ['vpid', 'vtid'],
            },
        )

        shutil.rmtree(tmpdir)
        del os.environ['TestTraceAction__event_ust']
        del os.environ['TestTraceAction__context_field']

    def test_action_ld_preload(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_ld_preload')

        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            events_ust=[
                'lttng_ust_cyg_profile_fast:*',
                'lttng_ust_libc:*',
                'ros2:*',
                'lttng_ust_pthread:*',
                'lttng_ust_dl:*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        node_ping_action = Node(
            package='test_tracetools',
            executable='test_ping',
            output='screen',
        )
        node_pong_action = Node(
            package='test_tracetools',
            executable='test_pong',
            output='screen',
        )
        self._assert_launch_no_errors([action, node_ping_action, node_pong_action])
        self._check_trace_action(
            action,
            tmpdir,
            events_ust=[
                'lttng_ust_cyg_profile_fast:*',
                'lttng_ust_libc:*',
                'ros2:*',
                'lttng_ust_pthread:*',
                'lttng_ust_dl:*',
            ],
        )

        # Check that LD_PRELOAD was set accordingly
        ld_preload = os.environ.get('LD_PRELOAD')
        assert ld_preload is not None
        paths = ld_preload.split(':')
        self.assertEqual(4, len(paths))
        self.assertTrue(any(p.endswith('liblttng-ust-cyg-profile-fast.so') for p in paths))
        self.assertTrue(any(p.endswith('liblttng-ust-libc-wrapper.so') for p in paths))
        self.assertTrue(any(p.endswith('liblttng-ust-pthread-wrapper.so') for p in paths))
        self.assertTrue(any(p.endswith('liblttng-ust-dl.so') for p in paths))

        shutil.rmtree(tmpdir)

    def test_append_timestamp(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_append_timestamp')

        action = Trace(
            session_name='my-session-name',
            append_timestamp=True,
            base_path=tmpdir,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir, session_name=None)
        # Session name should start with the given prefix and end with the timestamp, but don't
        # bother validating the timestamp here
        self.assertTrue(action.session_name.startswith('my-session-name-'))
        self.assertNotEqual('my-session-name-', action.session_name)

        shutil.rmtree(tmpdir)

    def test_append_trace(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_append_trace')

        # Generate a normal trace
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            append_trace=False,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir, append_trace=False)

        # Generating another trace with the same path should error out
        self._assert_launch_errors([action])

        # But it should work if we use the append_trace option
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            append_trace=True,
            events_kernel=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self._assert_launch_no_errors([action])
        self._check_trace_action(action, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)


if __name__ == '__main__':
    unittest.main()
