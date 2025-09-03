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
from typing import TextIO
from typing import Tuple
import unittest

from launch import Action
from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.frontend import Parser
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch.utilities.type_utils import perform_typed_substitution
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

    def _assert_launch(self, actions: List[Action]) -> Tuple[int, LaunchContext]:
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        return ls.run(), ls.context

    def _assert_launch_no_errors(self, actions: List[Action]) -> LaunchContext:
        ret, context = self._assert_launch(actions)
        self.assertEqual(0, ret, 'expected no errors')
        return context

    def _assert_launch_errors(self, actions: List[Action]) -> LaunchContext:
        ret, context = self._assert_launch(actions)
        self.assertEqual(1, ret, 'expected errors')
        return context

    def _assert_launch_frontend_no_errors(self, file: TextIO) -> Tuple[Trace, LaunchContext]:
        root_entity, parser = Parser.load(file)
        ld = parser.parse_description(root_entity)
        ls = LaunchService()
        ls.include_launch_description(ld)
        self.assertEqual(0, ls.run(), 'expected no errors')
        trace_action = next(
            (action for action in ld.entities if isinstance(action, Trace)),
            None
        )
        assert trace_action is not None, 'did not find Trace action'
        return trace_action, ls.context

    def _check_trace_action(
        self,
        action: Trace,
        context: LaunchContext,
        tmpdir: Optional[str] = None,
        *,
        session_name: Optional[str] = 'my-session-name',
        snapshot_mode: bool = False,
        append_trace: bool = False,
        events_ust: List[str] = ['ros2:*', '*'],
        subbuffer_size_ust: int = 524288,
        subbuffer_size_kernel: int = 1048576,
    ) -> None:
        if session_name is not None:
            self.assertEqual(session_name, perform_substitutions(context, action.session_name))
        if tmpdir is not None:
            self.assertEqual(tmpdir, perform_substitutions(context, action.base_path))
            assert action.trace_directory
            self.assertTrue(action.trace_directory.startswith(tmpdir))
            self.assertTrue(pathlib.Path(tmpdir).exists())
        self.assertEqual(
            snapshot_mode,
            perform_typed_substitution(context, action.snapshot_mode, bool)
        )
        self.assertEqual(
            append_trace,
            perform_typed_substitution(context, action.append_trace, bool)
        )
        self.assertEqual(0, len(action.events_kernel))
        self.assertEqual(
            events_ust, [perform_substitutions(context, x) for x in action.events_ust])
        self.assertEqual(
            subbuffer_size_ust,
            perform_typed_substitution(context, action.subbuffer_size_ust, int)
        )
        self.assertEqual(
            subbuffer_size_kernel,
            perform_typed_substitution(context, action.subbuffer_size_kernel, int)
        )

    def test_action(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action')

        # Disable kernel events just to not require kernel tracing for the test
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir)

        shutil.rmtree(tmpdir)

    def test_action_snapshot_mode(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_snapshot_mode')

        action = Trace(
            session_name='my-session-name',
            snapshot_mode=True,
            base_path=tmpdir,
            events_kernel=[],
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir, snapshot_mode=True)

        shutil.rmtree(tmpdir)

    def test_action_frontend_xml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_xml')

        xml_file = textwrap.dedent(
            r"""
            <launch>
                <trace
                    session-name="my-session-name"
                    snapshot-mode="false"
                    append-timestamp="false"
                    base-path="{}"
                    append-trace="true"
                    events-kernel=""
                    syscalls=""
                    events-ust="ros2:* *"
                    subbuffer-size-ust="524288"
                    subbuffer-size-kernel="1048576"
                />
            </launch>
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(xml_file) as f:
            trace_action, context = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, context, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_frontend_yaml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_frontend_yaml')

        yaml_file = textwrap.dedent(
            r"""
            launch:
            - trace:
                session-name: my-session-name
                snapshot-mode: false
                append-timestamp: false
                base-path: {}
                append-trace: true
                events-kernel: ""
                syscalls: ""
                events-ust: ros2:* *
                subbuffer-size-ust: 524288
                subbuffer-size-kernel: 1048576
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(yaml_file) as f:
            trace_action, context = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, context, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_context_per_domain(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_context_per_domain')

        # Invalid context domain
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            syscalls=[],
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
            syscalls=[],
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
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir)

        assert isinstance(action.context_fields, dict)
        self.assertDictEqual(
            {
                domain: [perform_substitutions(context, field) for field in fields]
                for domain, fields in action.context_fields.items()
            },
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
        snapshot_mode_arg = DeclareLaunchArgument(
            'snapshot-mode',
            default_value='False',
            description='whether to take a snapshot of the session',
        )
        append_timestamp_arg = DeclareLaunchArgument(
            'append-timestamp',
            default_value='False',
            description='whether to append a timestamp to the session name',
        )
        append_trace_arg = DeclareLaunchArgument(
            'append-trace',
            default_value='False',
            description='whether to append to an existing trace',
        )
        subbuffer_size_ust_arg = DeclareLaunchArgument(
            'subbuffer-size-ust',
            default_value='524288',
            description='the subbuffer size for userspace traces',
        )
        subbuffer_size_kernel_arg = DeclareLaunchArgument(
            'subbuffer-size-kernel',
            default_value='1048576',
            description='the subbuffer size for kernel traces',
        )
        action = Trace(
            session_name=LaunchConfiguration(session_name_arg.name),
            snapshot_mode=LaunchConfiguration(snapshot_mode_arg.name),
            append_timestamp=LaunchConfiguration(append_timestamp_arg.name),
            base_path=TextSubstitution(text=tmpdir),
            append_trace=LaunchConfiguration(append_trace_arg.name),
            events_kernel=[],
            syscalls=[],
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
            subbuffer_size_ust=LaunchConfiguration(subbuffer_size_ust_arg.name),
            subbuffer_size_kernel=LaunchConfiguration(subbuffer_size_kernel_arg.name)
        )
        context = self._assert_launch_no_errors([
            session_name_arg,
            snapshot_mode_arg,
            append_timestamp_arg,
            append_trace_arg,
            subbuffer_size_ust_arg,
            subbuffer_size_kernel_arg,
            action
        ])

        self._check_trace_action(action, context, tmpdir)

        assert isinstance(action.context_fields, dict)
        self.assertDictEqual(
            {
                domain: [perform_substitutions(context, field) for field in fields]
                for domain, fields in action.context_fields.items()
            },
            {
                'kernel': [],
                'userspace': ['vpid', 'vtid'],
            },
        )

        shutil.rmtree(tmpdir)
        del os.environ['TestTraceAction__event_ust']
        del os.environ['TestTraceAction__context_field']

    def test_action_substitutions_frontend_xml(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_substitutions_frontend_xml')

        xml_file = textwrap.dedent(
            r"""
            <launch>
                <arg name="session-name" default="my-session-name" />
                <arg name="snapshot-mode" default="false" />
                <arg name="append-timestamp" default="false" />
                <arg name="base-path" default="{}" />
                <arg name="append-trace" default="true" />
                <arg name="events-ust-1" default="ros2:*" />
                <arg name="events-ust-2" default="*" />
                <arg name="subbuffer-size-ust" default="524288" />
                <arg name="subbuffer-size-kernel" default="1048576" />
                <trace
                    session-name="$(var session-name)"
                    append-timestamp="$(var append-timestamp)"
                    base-path="$(var base-path)"
                    append-trace="$(var append-trace)"
                    events-kernel=""
                    syscalls=""
                    events-ust="$(var events-ust-1) $(var events-ust-2)"
                    subbuffer-size-ust="$(var subbuffer-size-ust)"
                    subbuffer-size-kernel="$(var subbuffer-size-kernel)"
                />
            </launch>
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(xml_file) as f:
            trace_action, context = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, context, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_substitutions_frontend_yaml(self) -> None:
        tmpdir = tempfile.mkdtemp(
            prefix='TestTraceAction__test_action_substitutions_frontend_yaml')

        yaml_file = textwrap.dedent(
            r"""
            launch:
            - arg:
                name: session-name
                default: my-session-name
            - arg:
                name: snapshot-mode
                default: "false"
            - arg:
                name: append-timestamp
                default: "false"
            - arg:
                name: base-path
                default: "{}"
            - arg:
                name: append-trace
                default: "true"
            - arg:
                name: events-ust-1
                default: "ros2:*"
            - arg:
                name: events-ust-2
                default: "*"
            - arg:
                name: subbuffer-size-ust
                default: "524288"
            - arg:
                name: subbuffer-size-kernel
                default: "1048576"
            - trace:
                session-name: "$(var session-name)"
                append-timestamp: "$(var append-timestamp)"
                base-path: "$(var base-path)"
                append-trace: "$(var append-trace)"
                events-kernel: ""
                syscalls: ""
                events-ust: "$(var events-ust-1) $(var events-ust-2)"
                subbuffer-size-ust: "$(var subbuffer-size-ust)"
                subbuffer-size-kernel: "$(var subbuffer-size-kernel)"
            """.format(tmpdir)
        )

        trace_action = None
        with io.StringIO(yaml_file) as f:
            trace_action, context = self._assert_launch_frontend_no_errors(f)

        self._check_trace_action(trace_action, context, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)

    def test_action_ld_preload(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_action_ld_preload')

        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            events_kernel=[],
            syscalls=[],
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
        context = self._assert_launch_no_errors([action, node_ping_action, node_pong_action])
        self._check_trace_action(
            action,
            context,
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
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir, session_name=None)
        # Session name should start with the given prefix and end with the timestamp
        session_name = perform_substitutions(context, action.session_name)
        self.assertTrue(session_name.startswith('my-session-name-'))
        session_name_timestamp = session_name[len('my-session-name-'):]
        self.assertNotEqual(0, len(session_name_timestamp))
        self.assertTrue(all(c.isdigit() for c in session_name_timestamp))

        shutil.rmtree(tmpdir)

    def test_base_path_trace_directory(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_base_path_trace_directory')

        # Let it rely on the ROS_HOME env var for the base path
        os.environ.pop('ROS_HOME', None)
        tmpdir_home = os.path.join(tmpdir, 'home')
        set_ros_home = SetEnvironmentVariable('ROS_HOME', tmpdir_home)
        action = Trace(
            session_name='my-session-name',
            base_path=None,
            events_kernel=[],
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        self.assertIsNone(action.trace_directory)
        context = self._assert_launch_no_errors([set_ros_home, action])
        self._check_trace_action(action, context, tmpdir=None, session_name=None)
        base_path = os.path.join(tmpdir_home, 'tracing')
        self.assertEqual(base_path, perform_substitutions(context, action.base_path))
        assert action.trace_directory
        self.assertTrue(action.trace_directory.startswith(base_path + os.path.sep))
        self.assertTrue(pathlib.Path(base_path).exists())
        os.environ.pop('ROS_HOME', None)

        shutil.rmtree(tmpdir)

    def test_append_trace(self) -> None:
        tmpdir = tempfile.mkdtemp(prefix='TestTraceAction__test_append_trace')

        # Generate a normal trace
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            append_trace=False,
            events_kernel=[],
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir, append_trace=False)

        # Generating another trace with the same path should error out
        self._assert_launch_errors([action])

        # But it should work if we use the append_trace option
        action = Trace(
            session_name='my-session-name',
            base_path=tmpdir,
            append_trace=True,
            events_kernel=[],
            syscalls=[],
            events_ust=[
                'ros2:*',
                '*',
            ],
            subbuffer_size_ust=524288,
            subbuffer_size_kernel=1048576,
        )
        context = self._assert_launch_no_errors([action])
        self._check_trace_action(action, context, tmpdir, append_trace=True)

        shutil.rmtree(tmpdir)


if __name__ == '__main__':
    unittest.main()
