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
        # Make sure there are no existing tracing sessions before running a test
        self.assertNoTracingSession()

    def tearDown(self) -> None:
        # Make sure there are no leftover tracing sessions after running a test
        # Even if running 'ros2 trace' fails, we do not want any lingering tracing session
        self.assertNoTracingSession()

    def assertNoTracingSession(self) -> None:
        output = self.run_lttng_list()
        # If there is no session daemon, then there are no tracing sessions
        no_session_daemon_available = 'No session daemon is available' in output
        if no_session_daemon_available:
            return
        # Starting from LTTng 2.13, 'tracing session' was replaced with 'recording session'
        # (see lttng-tools e971184)
        no_tracing_sessions = any(
            f'Currently no available {name} session' in output for name in ('tracing', 'recording')
        )
        if not no_tracing_sessions:
            # Destroy tracing sessions if there are any, this way we can continue running tests and
            # avoid possible interference between them
            self.run_lttng_destroy_all()
        self.assertTrue(no_tracing_sessions, 'tracing session(s) exist:\n' + output)

    def assertTraceExist(self, trace_dir: str) -> None:
        self.assertTrue(os.path.isdir(trace_dir), f'trace directory does not exist: {trace_dir}')

    def assertTraceNotExist(self, trace_dir: str) -> None:
        self.assertFalse(os.path.isdir(trace_dir), f'trace directory exists: {trace_dir}')

    def assertTraceContains(
        self,
        trace_dir: str,
        expected_trace_data: List[Tuple[str, str]],
    ) -> None:
        self.assertTraceExist(trace_dir)
        from tracetools_read.trace import get_trace_events
        events = get_trace_events(trace_dir)
        for trace_data in expected_trace_data:
            self.assertTrue(
                any(trace_data in event.items() for event in events),
                f'{trace_data} not found in events: {events}',
            )

    def create_test_tmpdir(self, test_name: str) -> str:
        prefix = self.__class__.__name__ + '__' + test_name
        return tempfile.mkdtemp(prefix=prefix)

    def get_subdirectories(self, directory: str) -> List[str]:
        return [f.name for f in os.scandir(directory) if f.is_dir()]

    def run_lttng_list(self) -> str:
        process = subprocess.run(
            ['lttng', 'list'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding='utf-8',
        )
        return process.stdout + process.stderr

    def run_lttng_destroy_all(self):
        process = subprocess.run(
            ['lttng', 'destroy', '--all'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding='utf-8',
        )
        output = process.stdout + process.stderr
        self.assertEqual(0, process.returncode, f"'lttng destroy' command failed: {output}")

    def run_trace_command_start(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
        wait_for_start: bool = False,
    ) -> subprocess.Popen:
        args = ['ros2', 'trace', *args]
        print('=>running:', args)
        process_env = os.environ.copy()
        process_env['PYTHONUNBUFFERED'] = '1'
        if env:
            process_env.update(env)
        process = subprocess.Popen(
            args,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            encoding='utf-8',
            env=process_env,
        )
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
        stdout, stderr = process.communicate()
        stdout = stdout.strip(' \r\n\t')
        stderr = stderr.strip(' \r\n\t')
        print('=>stdout:\n' + stdout)
        print('=>stderr:\n' + stderr)
        return process.wait()

    def run_trace_command(
        self,
        args: List[str],
        *,
        env: Optional[Dict[str, str]] = None,
    ) -> int:
        process = self.run_trace_command_start(args, env=env)
        return self.run_trace_command_stop(process)

    def test_default(self) -> None:
        tmpdir = self.create_test_tmpdir('test_default')

        # Test with the default session name
        ret = self.run_trace_command(['--path', tmpdir])
        self.assertEqual(0, ret)
        # Check that the trace directory was created
        subdirs = self.get_subdirectories(tmpdir)
        self.assertEqual(1, len(subdirs))
        self.assertTrue(subdirs[0].startswith('session-'))

        # Test with a provided session name
        ret = self.run_trace_command(['--path', tmpdir, '--session-name', 'test_default'])
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir, 'test_default'))

        shutil.rmtree(tmpdir)

    @unittest.skipIf(not are_tracepoints_included(), 'tracepoints are required')
    def test_default_tracing(self) -> None:
        tmpdir = self.create_test_tmpdir('test_default_tracing')

        def run_nodes():
            nodes = [
                Node(
                    package='test_tracetools',
                    executable='test_ping',
                    output='screen',
                ),
                Node(
                    package='test_tracetools',
                    executable='test_pong',
                    output='screen',
                ),
            ]
            ld = LaunchDescription(nodes)
            ls = LaunchService()
            ls.include_launch_description(ld)
            exit_code = ls.run()
            self.assertEqual(0, exit_code)

        # Test with the default session name
        process = self.run_trace_command_start(
            ['--path', tmpdir, '--ust', tracepoints.rcl_subscription_init],
            wait_for_start=True,
        )
        run_nodes()
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
            [
                ('topic_name', '/ping'),
                ('topic_name', '/pong'),
            ],
        )

        # Test with a provided session name
        process = self.run_trace_command_start(
            [
                '--path', tmpdir,
                '--ust', tracepoints.rcl_subscription_init,
                '--session-name', 'test_default_tracing',
            ],
            wait_for_start=True,
        )
        run_nodes()
        ret = self.run_trace_command_stop(process)
        self.assertEqual(0, ret)
        self.assertTraceContains(
            os.path.join(tmpdir, 'test_default_tracing'),
            [
                ('topic_name', '/ping'),
                ('topic_name', '/pong'),
            ],
        )

        shutil.rmtree(tmpdir)

    def test_env_var_ros_trace_dir(self) -> None:
        tmpdir = self.create_test_tmpdir('test_env_var_ros_trace_dir')

        # Env var only
        ret = self.run_trace_command(
            ['--session-name', 'test_env_var_ros_trace_dir'],
            env={'ROS_TRACE_DIR': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir, 'test_env_var_ros_trace_dir'))

        # Env var and argument should use argument
        tmpdir_path = self.create_test_tmpdir('test_env_var_ros_trace_dir__arg')
        ret = self.run_trace_command(
            ['--path', tmpdir_path, '--session-name', 'test_env_var_ros_trace_dir2'],
            env={'ROS_TRACE_DIR': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir_path, 'test_env_var_ros_trace_dir2'))
        self.assertTraceNotExist(os.path.join(tmpdir, 'test_env_var_ros_trace_dir2'))

        shutil.rmtree(tmpdir_path)
        shutil.rmtree(tmpdir)

    def test_env_var_ros_home(self) -> None:
        tmpdir = self.create_test_tmpdir('test_env_var_ros_home')

        ret = self.run_trace_command(
            ['--session-name', 'test_env_var_ros_home'],
            env={'ROS_HOME': tmpdir},
        )
        self.assertEqual(0, ret)
        # Under the 'tracing' directory
        self.assertTraceExist(os.path.join(tmpdir, 'tracing', 'test_env_var_ros_home'))

        # Env var and argument should use argument
        tmpdir_path = self.create_test_tmpdir('test_env_var_ros_home__arg')
        ret = self.run_trace_command(
            ['--path', tmpdir_path, '--session-name', 'test_env_var_ros_home2'],
            env={'ROS_HOME': tmpdir},
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(tmpdir_path, 'test_env_var_ros_home2'))
        self.assertTraceNotExist(os.path.join(tmpdir, 'test_env_var_ros_home2'))

        shutil.rmtree(tmpdir_path)
        shutil.rmtree(tmpdir)

    def test_empty_session_name(self) -> None:
        tmpdir = self.create_test_tmpdir('test_env_var_ros_home')

        # Empty session name should result in an error
        ret = self.run_trace_command(['--session-name', ''])
        self.assertEqual(1, ret)

        shutil.rmtree(tmpdir)

    def test_base_path_not_exist(self) -> None:
        tmpdir = self.create_test_tmpdir('test_base_path_not_exist')

        # Base directory should be created if it does not exist
        fake_base_path = os.path.join(tmpdir, 'doesnt_exist')
        ret = self.run_trace_command(
            ['--path', fake_base_path, '--session-name', 'test_base_path_not_exist'],
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(os.path.join(fake_base_path, 'test_base_path_not_exist'))

        shutil.rmtree(tmpdir)

    def test_unknown_context_field(self) -> None:
        tmpdir = self.create_test_tmpdir('test_unknown_context_field')

        # Unknown context field should result in an error
        ret = self.run_trace_command(
            ['--path', tmpdir, '--context', 'some_nonexistent_context_field'],
        )
        self.assertEqual(1, ret)

        shutil.rmtree(tmpdir)

    def test_append_trace(self) -> None:
        tmpdir = self.create_test_tmpdir('test_append_trace')

        # Generate a normal trace
        ret = self.run_trace_command(
            ['--path', tmpdir, '--session-name', 'test_append_trace'],
        )
        self.assertEqual(0, ret)
        trace_dir = os.path.join(tmpdir, 'test_append_trace')
        self.assertTraceExist(trace_dir)

        # Generating another trace with the same path should error out
        ret = self.run_trace_command(
            ['--path', tmpdir, '--session-name', 'test_append_trace'],
        )
        self.assertEqual(1, ret)
        self.assertTraceExist(trace_dir)

        # But it should work if we use the '--append-trace' option
        ret = self.run_trace_command(
            [
                '--path', tmpdir,
                '--session-name', 'test_append_trace',
                '--append-trace',
            ],
        )
        self.assertEqual(0, ret)
        self.assertTraceExist(trace_dir)

        shutil.rmtree(tmpdir)
