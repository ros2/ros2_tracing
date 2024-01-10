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

import unittest

from lttngpy import impl as lttngpy


class TestConstants(unittest.TestCase):

    def test_version(self):
        self.assertRegex(lttngpy.LTTNG_CTL_VERSION, r'^\d+\.\d+\.\d+$')

    def test_error(self):
        # Just check a couple to make sure they're present
        self.assertEqual(10, lttngpy.LTTNG_OK)
        self.assertTrue(0 < len(lttngpy.lttng_strerror(lttngpy.LTTNG_OK)))
        self.assertEqual(11, lttngpy.LTTNG_ERR_UNK)
