# Copyright 2026 Sylvester Kaczmarek
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
from pathlib import Path
import signal

from tracetools_test.lttng import isolated_lttng_home


def test_isolated_lttng_home_restores_existing_value(monkeypatch):
    monkeypatch.setenv('LTTNG_HOME', '/original/lttng/home')

    with isolated_lttng_home() as lttng_home:
        assert os.environ['LTTNG_HOME'] == lttng_home
        assert Path(lttng_home).is_dir()

    assert os.environ['LTTNG_HOME'] == '/original/lttng/home'
    assert not Path(lttng_home).exists()


def test_isolated_lttng_home_restores_unset_value(monkeypatch):
    monkeypatch.delenv('LTTNG_HOME', raising=False)

    with isolated_lttng_home() as lttng_home:
        assert os.environ['LTTNG_HOME'] == lttng_home

    assert 'LTTNG_HOME' not in os.environ


def test_isolated_lttng_home_stops_private_session_daemon(monkeypatch):
    calls = []

    def kill(pid, sig):
        calls.append((pid, sig))
        if sig == 0:
            raise ProcessLookupError

    monkeypatch.setattr(os, 'kill', kill)

    with isolated_lttng_home() as lttng_home:
        lttng_dir = Path(lttng_home) / '.lttng'
        lttng_dir.mkdir()
        (lttng_dir / 'lttng-sessiond.pid').write_text('12345')

    assert calls == [(12345, signal.SIGTERM), (12345, 0)]
