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

import os
import pathlib
import platform
import unittest

from launch import LaunchDescription
from launch import LaunchService

from tracetools_launch.actions.ld_preload import LdPreload


@unittest.skipIf('Linux' != platform.system(), 'Linux-specific test')
class TestLdPreloadAction(unittest.TestCase):

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService(debug=True)
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def __init__(self, *args) -> None:
        super().__init__(
            *args,
        )

    def test_get_shared_lib_path(self) -> None:
        self.assertIsNone(
            LdPreload.get_shared_lib_path('random_lib_that_does_not_exist_I_hope.so')
        )

        libc_path = LdPreload.get_shared_lib_path('libc.so')
        assert libc_path is not None
        print('path:', libc_path)
        path = pathlib.Path(libc_path)
        self.assertTrue(path.exists())
        self.assertEqual('libc.so', path.name)

    def test_action(self) -> None:
        self.assertIsNone(os.environ.get('LD_PRELOAD', None))

        ldpreload_action = LdPreload('random_lib_that_does_not_exist_I_hope.so')
        self.assertEqual('random_lib_that_does_not_exist_I_hope.so', ldpreload_action.lib_name)
        self._assert_launch_no_errors([ldpreload_action])
        self.assertFalse(ldpreload_action.lib_found())
        self.assertIsNone(ldpreload_action.lib_path)
        self.assertIsNone(os.environ.get('LD_PRELOAD', None))

        ldpreload_action = LdPreload('libc.so')
        self.assertEqual('libc.so', ldpreload_action.lib_name)
        self._assert_launch_no_errors([ldpreload_action])
        self.assertTrue(ldpreload_action.lib_found())
        path_env = os.environ.get('LD_PRELOAD', None)
        assert path_env is not None
        path = pathlib.Path(path_env)
        self.assertTrue(path.exists())
        self.assertEqual('libc.so', path.name)
        self.assertEqual(str(path.absolute()), ldpreload_action.lib_path)
        del os.environ['LD_PRELOAD']


if __name__ == '__main__':
    unittest.main()
