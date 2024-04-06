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

import os
import shutil
import subprocess
import tempfile
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from lttngpy import impl as lttngpy
from tracetools_test.mark_process import get_corresponding_trace_test_events
from tracetools_test.mark_process import get_trace_test_id
from tracetools_test.mark_process import TRACE_TEST_ID_ENV_VAR
from tracetools_test.mark_process import TRACE_TEST_ID_TP_NAME
from tracetools_trace.tools import tracepoints
from tracetools_trace.tools.lttng import is_lttng_installed


def are_tracepoints_included() -> bool:
    """
    Check if tracing instrumentation is enabled and if tracepoints are included.

    :return: True if tracepoints are included, False otherwise
    """
    if not is_lttng_installed():
        return False
    process = subprocess.run(
        ['ros2', 'run', 'tracetools', 'status'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding='utf-8',
    )
    return 0 == process.returncode


@unittest.skipIf(not is_lttng_installed(minimum_version='2.9.0'), 'LTTng is required')
class TestROS2TraceCLI(unittest.TestCase):

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def setUp(self) -> None:
        self.trace_test_id = get_trace_test_id(self._testMethodName)

    def tearDown(self) -> None:
        del self.trace_test_id

    def assertTracingSessionExist(self, session_name: str) -> None:
        self.assertTrue(
            lttngpy.is_lttng_session_daemon_alive(),
            f"tracing session '{session_name}' does not exist because there is no daemon",
        )
        session_names = lttngpy.get_session_names()
        self.assertIn(
            session_name,
            session_names,
            f"tracing session '{session_name}' does not exist",
        )

    def assertTracingSessionNotExist(self, session_name: str) -> None:
        # If there is no session daemon, then there are no tracing sessions
        if not lttngpy.is_lttng_session_daemon_alive():
            return
        session_names = lttngpy.get_session_names()
        self.assertNotIn(session_name, session_names, f"tracing session '{session_name}' exists")

    def assertTraceExist(self, trace_dir: str) -> None:
        self.assertTrue(os.path.isdir(trace_dir), f'trace directory does not exist: {trace_dir}')

    def assertTraceNotExist(self, trace_dir: str) -> None:
        self.assertFalse(os.path.isdir(trace_dir), f'trace directory exists: {trace_dir}')

    def assertTraceContains(
        self,
        trace_dir: str,
        *,
        expected_field_value: List[Tuple[str, str]] = [],
        expected_field: List[str] = [],
    ) -> int:
        self.assertTraceExist(trace_dir)
        from tracetools_read.trace import get_trace_events
        events_all = get_trace_events(trace_dir)
        self.assertGreater(len(events_all), 0, 'no events found in trace')
        assert self.trace_test_id
        events = get_corresponding_trace_test_events(events_all, self.trace_test_id)
        self.assertGreater(
            len(events),
            0,
            f'no matching trace test events found in trace from events: {events_all}',
        )
        for field_value in expected_field_value:
            self.assertTrue(
                any(field_value in event.items() for event in events),
                f'{field_value} not found in events: {events}',
            )
        for field in expected_field:
            self.assertTrue(
                any(field in event.keys() for event in events),
                f'{field} not found in events: {events}',
            )
        return len(events)

    def create_test_tmpdir(self, test_name: str) -> str:
        prefix = self.__class__.__name__ + '__' + test_name
        return tempfile.mkdtemp(prefix=prefix)

    def get_subdirectories(self, directory: str) -> List[str]:
        return [f.name for f in os.scandir(directory) if f.is_dir()]

    def run_command(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
    ) -> subprocess.Popen:
        print('=>running:', args)
        process_env = os.environ.copy()
        process_env['PYTHONUNBUFFERED'] = '1'
        if env:
            process_env.update(env)
        return subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding='utf-8',
            env=process_env,
        )

    def wait_and_print_command_output(
        self,
        process: subprocess.Popen,
    ) -> int:
        stdout, stderr = process.communicate()
        stdout = stdout.strip(' \r\n\t')
        stderr = stderr.strip(' \r\n\t')
        print('=>stdout:\n' + stdout)
        print('=>stderr:\n' + stderr)
        return process.wait()

    def run_trace_command_start(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
        wait_for_start: bool = False,
    ) -> subprocess.Popen:
        process = self.run_command(['ros2', 'trace', *args], env=env)
        # Write <enter> to stdin to start tracing
        assert process.stdin
        process.stdin.write('\n')
        process.stdin.flush()
        # If needed, wait until tracing has started by waiting until 'ros2 trace' is ready to stop
        if wait_for_start:
            assert process.stdout
            stdout = ''
            while 'press enter to stop...' not in stdout:
                stdout += process.stdout.read(1)
        return process

    def run_trace_command_stop(
        self,
        process: subprocess.Popen,
    ) -> int:
        # Write <enter> to stdin to stop tracing
        assert process.stdin
        process.stdin.write('\n')
        process.stdin.flush()
        return self.wait_and_print_command_output(process)

    def run_trace_command(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
    ) -> int:
        process = self.run_trace_command_start(args, env=env)
        return self.run_trace_command_stop(process)

    def run_trace_subcommand(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
    ) -> int:
        process = self.run_command(['ros2', 'trace', *args], env=env)
        return self.wait_and_print_command_output(process)

    def run_nodes(self) -> None:
        # Set trace test ID env var for spawned processes
        env = os.environ.copy()
        assert self.trace_test_id
        env[TRACE_TEST_ID_ENV_VAR] = self.trace_test_id
        nodes = [
            Node(
                package='test_tracetools',
                executable='test_ping',
                output='screen',
                env=env,
            ),
            Node(
                package='test_tracetools',
                executable='test_pong',
                output='screen',
                env=env,
            ),
        ]
        ld = LaunchDescription(nodes)
        ls = LaunchService()
        ls.include_launch_description(ld)
        exit_code = ls.run()
        self.assertEqual(0, exit_code)

    def test_default(self) -> None:
        tmpdir = self.create_test_tmpdir('test_default')

        # Test with the default session name
        ret = self.run_trace_command(['--path', tmpdir])
        self.assertEqual(0, ret)
        # Check that the trace directory was created
        subdirs = self.get_subdirectories(tmpdir)
        self.assertEqual(1, len(subdirs))
        trace_dir_name = subdirs[0]
        self.assertTrue(trace_dir_name.startswith('session-'))
        self.assertTracingSessionNotExist(trace_dir_name)

        # Test with a provided session name
        session_name = 'test_default'
        ret = self.run_trace_command(['--path', tmpdir, '--session-name', session_name])
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    @unittest.skipIf(not are_tracepoints_included(), 'tracepoints are required')
    def test_default_tracing(self) -> None:
        tmpdir = self.create_test_tmpdir('test_default_tracing')

        # Test with the default session name
        process = self.run_trace_command_start(
            [
                '--path', tmpdir,
                '--ust', tracepoints.rcl_subscription_init, TRACE_TEST_ID_TP_NAME,
            ],
            wait_for_start=True,
        )
        self.run_nodes()
        ret = self.run_trace_command_stop(process)
        self.assertEqual(0, ret)
        # Check that the trace directory was created
        subdirs = self.get_subdirectories(tmpdir)
        self.assertEqual(1, len(subdirs))
        trace_dir_name = subdirs[0]
        self.assertTrue(trace_dir_name.startswith('session-'))
        trace_dir = os.path.join(tmpdir, trace_dir_name)
        # Check that the trace contains at least the publishers/subscriptions we expect
        self.assertTraceContains(
            trace_dir,
            expected_field_value=[
                ('topic_name', '/ping'),
                ('topic_name', '/pong'),
            ],
        )
        self.assertTracingSessionNotExist(trace_dir_name)

        # Test with a provided session name
        session_name = 'test_default_tracing'
        process = self.run_trace_command_start(
            [
                '--path', tmpdir,
                '--ust', tracepoints.rcl_subscription_init, TRACE_TEST_ID_TP_NAME,
                '--session-name', session_name,
            ],
            wait_for_start=True,
        )
        self.assertTracingSessionExist(session_name)
        self.run_nodes()
        ret = self.run_trace_command_stop(process)
        self.assertEqual(0, ret)
        self.assertTraceContains(
            os.path.join(tmpdir, session_name),
            expected_field_value=[
                ('topic_name', '/ping'),
                ('topic_name', '/pong'),
            ],
        )
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    def test_env_var_ros_trace_dir(self) -> None:
        tmpdir = self.create_test_tmpdir('test_env_var_ros_trace_dir')
        session_name = 'test_env_var_ros_trace_dir'

        # Env var only
        ret = self.run_trace_command(
            ['--session-name', session_name],
            env={'ROS_TRACE_DIR': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        # Env var and argument should use argument
        tmpdir_path = self.create_test_tmpdir('test_env_var_ros_trace_dir__arg')
        session_name = 'test_env_var_ros_trace_dir2'
        ret = self.run_trace_command(
            ['--path', tmpdir_path, '--session-name', session_name],
            env={'ROS_TRACE_DIR': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir_path, session_name))
        self.assertTraceNotExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir_path)
        shutil.rmtree(tmpdir)

    def test_env_var_ros_home(self) -> None:
        tmpdir = self.create_test_tmpdir('test_env_var_ros_home')
        session_name = 'test_env_var_ros_home'

        ret = self.run_trace_command(
            ['--session-name', session_name],
            env={'ROS_HOME': tmpdir},
        )
        self.assertEqual(0, ret)
        # Under the 'tracing' directory
        self.assertTraceExist(os.path.join(tmpdir, 'tracing', session_name))
        self.assertTracingSessionNotExist(session_name)

        # Env var and argument should use argument
        tmpdir_path = self.create_test_tmpdir('test_env_var_ros_home__arg')
        session_name = 'test_env_var_ros_home2'
        ret = self.run_trace_command(
            ['--path', tmpdir_path, '--session-name', session_name],
            env={'ROS_HOME': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir_path, session_name))
        self.assertTraceNotExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir_path)
        shutil.rmtree(tmpdir)

    def test_empty_session_name(self) -> None:
        tmpdir = self.create_test_tmpdir('test_empty_session_name')

        # Empty session name should result in an error
        ret = self.run_trace_command(
            ['--path', tmpdir, '--session-name', ''],
        )
        self.assertEqual(1, ret)
        self.assertTracingSessionNotExist('')

        shutil.rmtree(tmpdir)

    def test_base_path_not_exist(self) -> None:
        tmpdir = self.create_test_tmpdir('test_base_path_not_exist')
        session_name = 'test_base_path_not_exist'

        # Base directory should be created if it does not exist
        fake_base_path = os.path.join(tmpdir, 'doesnt_exist')
        ret = self.run_trace_command(
            ['--path', fake_base_path, '--session-name', session_name],
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(fake_base_path, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    def test_no_events(self) -> None:
        tmpdir = self.create_test_tmpdir('test_no_events')
        session_name = 'test_no_events'

        # Enabling no events should result in an error
        ret = self.run_trace_command(
            ['--path', tmpdir, '--ust', '--kernel', '--session-name', session_name],
        )
        self.assertEqual(1, ret)
        self.assertTraceNotExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    def test_unknown_context_field(self) -> None:
        tmpdir = self.create_test_tmpdir('test_unknown_context_field')
        session_name = 'some_nonexistent_context_field'

        # Unknown context field should result in an error
        ret = self.run_trace_command(
            ['--path', tmpdir, '--context', session_name],
        )
        self.assertEqual(1, ret)
        self.assertTraceNotExist(os.path.join(tmpdir, session_name))
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    @unittest.skipIf(not are_tracepoints_included(), 'tracepoints are required')
    def test_explicit_context_fields(self) -> None:
        tmpdir = self.create_test_tmpdir('test_explicit_context_fields')
        session_name = 'test_explicit_context_fields'

        ret = self.run_trace_subcommand(
            [
                'start',
                session_name,
                '--path', tmpdir,
                '--ust', 'ros2:*', TRACE_TEST_ID_TP_NAME,
                '--context', 'vpid', 'vuid',
                '--list',
            ],
        )
        self.assertEqual(0, ret)
        self.run_nodes()
        ret = self.run_trace_subcommand(['stop', session_name])
        self.assertEqual(0, ret)
        self.assertTraceContains(
            os.path.join(tmpdir, session_name),
            expected_field=['vpid', 'vuid'],
        )

        shutil.rmtree(tmpdir)

    def test_append_trace(self) -> None:
        tmpdir = self.create_test_tmpdir('test_append_trace')
        session_name = 'test_append_trace'

        # Generate a normal trace
        ret = self.run_trace_command(
            ['--path', tmpdir, '--session-name', session_name],
        )
        self.assertEqual(0, ret)
        trace_dir = os.path.join(tmpdir, session_name)
        self.assertTraceExist(trace_dir)
        self.assertTracingSessionNotExist(session_name)

        # Generating another trace with the same path should error out
        ret = self.run_trace_command(
            ['--path', tmpdir, '--session-name', session_name],
        )
        self.assertEqual(1, ret)
        self.assertTraceExist(trace_dir)
        self.assertTracingSessionNotExist(session_name)

        # But it should work if we use the '--append-trace' option
        ret = self.run_trace_command(
            [
                '--path', tmpdir,
                '--session-name', session_name,
                '--append-trace',
            ],
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(trace_dir)
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)

    def test_pause_resume_stop_bad_session_name(self) -> None:
        for subcommand in ('pause', 'resume', 'stop'):
            # Session name doesn't exist
            session_name = 'some_nonexistent_session_name'
            ret = self.run_trace_subcommand([subcommand, session_name])
            self.assertEqual(1, ret, f'subcommand: {subcommand}')
            self.assertTracingSessionNotExist(session_name)
            # Session name not provided
            ret = self.run_trace_subcommand([subcommand])
            self.assertEqual(2, ret, f'subcommand: {subcommand}')

    @unittest.skipIf(not are_tracepoints_included(), 'tracepoints are required')
    def test_start_pause_resume_stop(self) -> None:
        tmpdir = self.create_test_tmpdir('test_start_pause_resume_stop')
        session_name = 'test_start_pause_resume_stop'

        # Start tracing and run nodes
        ret = self.run_trace_subcommand(
            [
                'start', session_name,
                '--ust', 'ros2:*', TRACE_TEST_ID_TP_NAME,
                '--path', tmpdir,
            ],
        )
        self.assertEqual(0, ret)
        trace_dir = os.path.join(tmpdir, session_name)
        self.assertTraceExist(trace_dir)
        self.assertTracingSessionExist(session_name)
        self.run_nodes()

        # Pause tracing and check trace
        ret = self.run_trace_subcommand(['pause', session_name])
        self.assertEqual(0, ret)
        self.assertTracingSessionExist(session_name)
        expected_trace_data = [
            ('topic_name', '/ping'),
            ('topic_name', '/pong'),
        ]
        num_events = self.assertTraceContains(trace_dir, expected_field_value=expected_trace_data)

        # Pausing again should give an error but not affect anything
        ret = self.run_trace_subcommand(['pause', session_name])
        self.assertEqual(1, ret)
        self.assertTracingSessionExist(session_name)
        new_num_events = self.assertTraceContains(
            trace_dir,
            expected_field_value=expected_trace_data,
        )
        self.assertEqual(num_events, new_num_events, 'unexpected new events in trace')

        # When not tracing, run nodes again and check that trace didn't change
        self.run_nodes()
        new_num_events = self.assertTraceContains(
            trace_dir,
            expected_field_value=expected_trace_data,
        )
        self.assertEqual(num_events, new_num_events, 'unexpected new events in trace')

        # Resume tracing and run nodes again
        ret = self.run_trace_subcommand(['resume', session_name])
        self.assertEqual(0, ret)
        self.assertTracingSessionExist(session_name)
        self.run_nodes()

        # Resuming tracing again should give an error but not affect anything
        ret = self.run_trace_subcommand(['resume', session_name])
        self.assertEqual(1, ret)
        self.assertTracingSessionExist(session_name)

        # Stop tracing and check that trace changed
        ret = self.run_trace_subcommand(['stop', session_name])
        self.assertEqual(0, ret)
        self.assertTracingSessionNotExist(session_name)
        new_num_events = self.assertTraceContains(
            trace_dir,
            expected_field_value=expected_trace_data,
        )
        self.assertGreater(new_num_events, num_events, 'no new events in trace')

        # Stopping tracing again should give an error but not affect anything
        ret = self.run_trace_subcommand(['stop', session_name])
        self.assertEqual(1, ret)
        self.assertTracingSessionNotExist(session_name)

        shutil.rmtree(tmpdir)
