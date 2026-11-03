"""Actual subprocess/pipe handshake with a non-ROS audited-runner substitute."""
import importlib.util
import json
import os
from pathlib import Path
import threading
import time

import pytest


HELPER = Path(__file__).resolve().parents[3] / '.runtime/prewarm_mode_grasp_runner_20261012.py'
spec = importlib.util.spec_from_file_location('prewarm_helper_under_test', str(HELPER))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
SELECTION = dict(mode='unknown', strategy='observe_return', generation=7, stamp_ns=123456)


@pytest.fixture
def fake(tmp_path):
    events = tmp_path / 'events.jsonl'
    path = tmp_path / 'runner.py'
    path.write_text('''import json
from pathlib import Path
EVENTS = Path(%r)
def event(value):
    with EVENTS.open('a') as out:
        out.write(json.dumps(value) + '\\n')
event({'kind': 'import'})
class ModeAwareGraspRunner:
    def __init__(self, mode, strategy, selected):
        self.execution_inflight = False
        self.selected = selected
        event(dict(kind='construct', mode=mode, strategy=strategy, selection=selected))
    def run(self):
        event(dict(kind='run', selection=self.selected))
        if self.selected['stamp_ns'] != 123456:
            raise ValueError('active selection differs from confirmed attempt')
        return 6
    def stop_candidate_computation(self):
        event(dict(kind='stop'))
''' % str(events))
    def read():
        return [json.loads(line) for line in events.read_text().splitlines()] if events.exists() else []
    return path, read


def test_ready_imports_only_then_exactly_one_original_run(fake):
    path, read = fake
    with helper.PrewarmedModeRunner(path) as worker:
        assert read() == [dict(kind='import')]
        assert worker.ready['state'] == 'imports_ready'
        assert worker.run_once(SELECTION) == 6
        with pytest.raises(RuntimeError, match='already released'):
            worker.run_once(SELECTION)
    assert [r['kind'] for r in read()] == ['import', 'construct', 'run']
    assert read()[-1]['selection'] == SELECTION


def test_parent_failure_before_commit_closes_child_without_run(fake):
    path, read = fake
    with pytest.raises(ValueError, match='reset ACK'):
        with helper.PrewarmedModeRunner(path) as worker:
            raise ValueError('reset ACK absent')
    assert worker.process.poll() == 75
    assert read() == [dict(kind='import')]


@pytest.mark.parametrize('mutation', [dict(generation=True), dict(stamp_ns='123456'),
    dict(mode='carton'), dict(strategy='direct'), dict(extra=True), dict(generation=0)])
def test_malformed_selection_fails_before_release(fake, mutation):
    path, read = fake
    with helper.PrewarmedModeRunner(path) as worker:
        with pytest.raises(ValueError):
            worker.run_once(dict(SELECTION, **mutation))
        assert not worker.released
    assert read() == [dict(kind='import')]


def test_original_runner_rejects_wrong_active_selection_and_keeps_cleanup(fake):
    path, read = fake
    with helper.PrewarmedModeRunner(path) as worker:
        assert worker.run_once(dict(SELECTION, stamp_ns=999)) == 10
    assert [r['kind'] for r in read()] == ['import', 'construct', 'run', 'stop']


def test_import_exception_never_reports_ready_or_runs(fake):
    path, read = fake
    path.write_text(path.read_text() + '\nraise RuntimeError("import failure")\n')
    with pytest.raises(EOFError):
        helper.PrewarmedModeRunner(path)
    assert read() == [dict(kind='import')]


def test_release_timeout_and_late_parent_cannot_start(fake):
    path, read = fake
    worker = helper.PrewarmedModeRunner(path, release_timeout=.05)
    assert worker.process.wait(timeout=2) == 75
    with pytest.raises(RuntimeError, match='exited'):
        worker.run_once(SELECTION)
    worker.close()
    assert read() == [dict(kind='import')]


def test_changed_source_after_ready_fails_closed(fake):
    path, read = fake
    with helper.PrewarmedModeRunner(path) as worker:
        path.write_text(path.read_text() + '\n# changed\n')
        with pytest.raises(RuntimeError, match='source changed'):
            worker.run_once(SELECTION)
    assert read() == [dict(kind='import')]


def test_readiness_timeout_reaps_importing_process(fake):
    path, read = fake
    path.write_text('import time\ntime.sleep(30)\n' + path.read_text())
    start = time.monotonic()
    with pytest.raises(TimeoutError):
        helper.PrewarmedModeRunner(path, ready_timeout=.03)
    assert time.monotonic() - start < 5
    assert read() == []


def test_close_after_release_does_not_terminate_running_child(fake):
    path, read = fake
    path.write_text(path.read_text().replace(
        "        return 6", "        import time\n        time.sleep(.8)\n        return 6"))
    worker = helper.PrewarmedModeRunner(path)
    results = []
    thread = threading.Thread(target=lambda: results.append(worker.run_once(SELECTION)))
    thread.start()
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and not any(r['kind'] == 'run' for r in read()):
        time.sleep(.01)
    assert any(r['kind'] == 'run' for r in read())
    worker.close()
    assert worker.process.poll() is None
    thread.join(2)
    assert not thread.is_alive() and results == [6]


@pytest.mark.parametrize('payload', [b'{"bad":true}\n', b'{}\n{}\n', b'x'*4097,
    b'{"version":1,"operation":"run_exact_selection_once","selection":{}}\n'])
def test_child_independently_rejects_malformed_parent_release(fake, payload):
    path, read = fake
    with helper.PrewarmedModeRunner(path) as worker:
        os.write(worker._release_fd, payload)
        os.close(worker._release_fd); worker._release_fd = None
        assert worker.process.wait(timeout=2) == 75
    assert read() == [dict(kind='import')]
