import ast
import dataclasses
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys

import pytest

spec = importlib.util.spec_from_file_location(
    'budgeted_sidecar', Path(__file__).with_name('record_budgeted_full_frames_20261014.py'))
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def wire_record(header, data=b''):
    fields = b''.join(struct.pack('<I', len(k) + 1 + len(v)) + k + b'=' + v
                      for k, v in header.items())
    return struct.pack('<I', len(fields)) + fields + struct.pack('<I', len(data)) + data


def small_indexed_bag(path):
    header = {b'op': b'\x03', b'index_pos': struct.pack('<Q', 0),
              b'conn_count': struct.pack('<I', 0), b'chunk_count': struct.pack('<I', 1)}
    prefix = b'#ROSBAG V2.0\n'
    header[b'index_pos'] = struct.pack('<Q', len(prefix) + len(wire_record(header)))
    index = {b'op': b'\x06', b'ver': struct.pack('<I', 1),
             b'chunk_pos': struct.pack('<Q', 0), b'start_time': struct.pack('<II', 100, 1),
             b'end_time': struct.pack('<II', 101, 2), b'count': struct.pack('<I', 1)}
    path.write_bytes(prefix + wire_record(header) + wire_record(index, struct.pack('<II', 0, 4)))


class Clock:
    def __init__(self):
        self.value = 0.0
    def mono(self):
        return self.value
    def sleep(self, seconds):
        self.value += seconds
    def ns(self):
        return 1_000_000_000_000 + int(self.value * 1e9)


class Writer:
    def __init__(self, output, *, refuses_stop=False, finalize=True, bad_bag=False, overshoot=None):
        self.output, self.pid = output, os.getpid()
        self.refuses_stop, self.finalize, self.bad_bag = refuses_stop, finalize, bad_bag
        self.overshoot = overshoot
        self.signals, self.waits, self.returncode = [], [], None
    def poll(self):
        return self.returncode
    def send_signal(self, sig):
        self.signals.append(sig)
        assert sig == signal.SIGINT
        if self.refuses_stop:
            return
        self.returncode = 0
        active = self.output / 'full_frames.bag.active'
        if active.exists():
            active.unlink()  # Synthetic writer fixture only.
        if self.finalize:
            if self.bad_bag:
                (self.output / 'full_frames.bag').write_bytes(b'incomplete')
            else:
                small_indexed_bag(self.output / 'full_frames.bag')
                if self.overshoot:
                    with (self.output / 'full_frames.bag').open('ab') as f:
                        f.truncate(self.overshoot)
    def wait(self, timeout):
        self.waits.append(timeout)
        if self.returncode is None:
            raise subprocess.TimeoutExpired('owned-test-writer', timeout)
        return self.returncode
    def terminate(self):
        raise AssertionError('must never terminate writer')
    def kill(self):
        raise AssertionError('must never kill writer')


def invocation(tmp_path, *, limits=None, behavior=None, storage=None, on_start=None):
    output = tmp_path / 'new_sidecar'
    limits = limits or m.Limits(duration=1)
    clock = Clock()
    process = Writer(output, **(behavior or {}))
    seen = []
    def popen(command, **kwargs):
        seen.append((command, kwargs))
        if on_start:
            on_start(output, process)
        return process
    def run():
        return m.record(output, limits, storage or (lambda: (8*m.GIB, int(4.5*m.GIB))),
                        popen=popen, monotonic=clock.mono, sleep=clock.sleep, now_ns=clock.ns)
    return run, output, process, seen


def test_direct_owned_writer_lz4_exact_original_topics_and_no_rollover(tmp_path):
    run, output, writer, seen = invocation(tmp_path)
    result = run()
    command, kwargs = seen[0]
    assert command[0] == '/opt/ros/noetic/lib/rosbag/record'
    assert command[-23:] == list(m.TOPICS) and len(set(m.TOPICS)) == 23
    assert '--lz4' in command and '--size=736' in command and '--buffsize=256' in command
    assert not any('split' in item or 'publish' in item for item in command)
    assert kwargs['start_new_session'] and kwargs['env']['ROS_LOG_DIR'] == str(output/'ros')
    assert writer.signals == [signal.SIGINT] and writer.waits == [30.0]
    assert result['stop_reason'] == 'duration_elapsed' and result['recording_closed']
    assert result['coverage_start_ns'] == 100_000_000_001
    assert result['coverage_end_ns'] == 101_000_000_002
    assert result['sha256'] == m.sha(output/'full_frames.bag')
    assert result['within_sidecar_budget'] and result['free_floor_preserved']
    assert result['files_deleted'] is False and result['main_recorder_controlled'] is False
    original = ast.parse(Path(__file__).with_name('record_attempt_full_frames_20261013.py').read_text())
    original_topics = next(ast.literal_eval(node.value) for node in ast.walk(original)
                          if isinstance(node, ast.Assign) and any(
                              isinstance(target, ast.Name) and target.id == 'topics' for target in node.targets))
    assert list(m.TOPICS) == original_topics


@pytest.mark.parametrize('source', ['bytes', 'free'])
def test_budget_signal_stops_only_owned_writer_before_reserved_capacity(tmp_path, monkeypatch, source):
    limits = m.Limits()
    started = [False]
    def on_start(output, writer):
        started[0] = True
    def storage():
        free = limits.min_free_bytes+limits.headroom if started[0] and source == 'free' else 8*m.GIB
        return free, int(4.5*m.GIB)
    if source == 'bytes':
        monkeypatch.setattr(m, 'measured_size', lambda _: limits.max_bytes-limits.headroom)
    run, _, writer, _ = invocation(tmp_path, limits=limits, storage=storage, on_start=on_start)
    result = run()
    assert writer.signals == [signal.SIGINT]
    assert result['stop_reason'] == 'budget_exhausted'
    assert result['budget_trigger'] == ('sidecar_bytes' if source == 'bytes' else 'free_space')
    assert result['buffer_reserve_bytes'] == 288*m.MIB
    assert result['recording_closed'] and not result['main_recorder_controlled']


def test_stop_requested_seals_recorder_without_task_signal(tmp_path):
    run, _, writer, _ = invocation(tmp_path, on_start=lambda out, _: (out/'stop_requested').touch())
    assert run()['stop_reason'] == 'stop_requested'
    assert writer.signals == [signal.SIGINT]


@pytest.mark.parametrize('duration', [0, 601])
def test_duration_rejected_before_process_or_output(tmp_path, duration):
    run, output, _, seen = invocation(tmp_path, limits=m.Limits(duration=duration))
    with pytest.raises(ValueError, match='duration'):
        run()
    assert not seen and not output.exists()


def test_free_floor_cannot_bypass_archive_configuration(tmp_path):
    run, output, _, seen = invocation(tmp_path, limits=m.Limits(min_free_bytes=4*m.GIB))
    with pytest.raises(ValueError, match='archive configuration'):
        run()
    assert not seen and not output.exists()


def test_reviewed_4_5_gib_floor_cannot_be_lowered_by_config(tmp_path):
    run, output, _, seen = invocation(tmp_path, limits=m.Limits(min_free_bytes=4*m.GIB),
                                     storage=lambda: (8*m.GIB, 1*m.GIB))
    with pytest.raises(ValueError, match='archive configuration'):
        run()
    assert not seen and not output.exists()


def test_low_initial_space_rejected_including_buffer_headroom(tmp_path):
    limits = m.Limits()
    run, output, _, seen = invocation(tmp_path, limits=limits,
        storage=lambda: (limits.min_free_bytes+limits.headroom, int(4.5*m.GIB)))
    with pytest.raises(RuntimeError, match='insufficient sidecar'):
        run()
    assert not seen and not output.exists()


def test_output_collision_never_overwrites_existing_evidence(tmp_path):
    run, output, _, seen = invocation(tmp_path)
    output.mkdir(); evidence = output/'result.json'; evidence.write_text('previous')
    with pytest.raises(FileExistsError):
        run()
    assert evidence.read_text() == 'previous' and not seen


def test_graceful_timeout_records_unclosed_state_no_force_or_retry(tmp_path):
    run, output, writer, _ = invocation(tmp_path, behavior=dict(refuses_stop=True))
    with pytest.raises(subprocess.TimeoutExpired):
        run()
    result = json.loads((output/'result.json').read_text())
    assert writer.signals == [signal.SIGINT] and len(writer.waits) == 1
    assert result['recording_closed'] is False and result['writer_may_still_be_running']
    assert result['graceful_wait_timed_out'] and result['coverage_end_ns'] is None


def test_exited_writer_without_final_bag_never_claims_seal(tmp_path):
    run, output, writer, _ = invocation(tmp_path, behavior=dict(finalize=False))
    with pytest.raises(TimeoutError, match='without a finalized bag'):
        run()
    result = json.loads((output/'result.json').read_text())
    assert result['recording_closed'] is False and 'close_verification_error' in result
    assert writer.signals == [signal.SIGINT]


def test_bad_final_bag_is_preserved_without_claiming_coverage(tmp_path):
    run, output, _, _ = invocation(tmp_path, behavior=dict(bad_bag=True))
    with pytest.raises(ValueError, match='ROS V2 bag'):
        run()
    result = json.loads((output/'result.json').read_text())
    assert not result['recording_closed'] and result['coverage_end_ns'] is None
    assert (output/'full_frames.bag').read_bytes() == b'incomplete'


def test_storage_loss_stops_only_own_writer_without_fallback(tmp_path):
    started = [False]
    def storage():
        if started[0]:
            raise RuntimeError('SanStand unavailable')
        return 8*m.GIB, int(4.5*m.GIB)
    run, output, writer, _ = invocation(tmp_path, storage=storage,
        on_start=lambda *_: started.__setitem__(0, True))
    with pytest.raises(RuntimeError, match='SanStand unavailable'):
        run()
    assert writer.signals == [signal.SIGINT]
    # The pre-loss receipt survives; unavailable storage is not replaced.
    assert json.loads((output/'result.json').read_text())['recording_closed'] is False


@pytest.mark.parametrize('failure', [KeyError('min_free_bytes'), AttributeError('get')])
def test_receipt_failure_cannot_prevent_owned_writer_stop(tmp_path, failure):
    started = [False]
    def storage():
        if started[0]:
            raise failure
        return 8*m.GIB, int(4.5*m.GIB)
    run, output, writer, _ = invocation(tmp_path, storage=storage,
        on_start=lambda *_: started.__setitem__(0, True))
    with pytest.raises(type(failure)):
        run()
    assert writer.signals == [signal.SIGINT]
    assert (output/'full_frames.bag').exists()


def test_storage_rejects_output_symlink_created_after_initial_check(tmp_path, monkeypatch):
    root = tmp_path/'external'; (root/'ros_log').mkdir(parents=True)
    monkeypatch.setattr(m, 'ROOT', root)
    output = root/'ros_log'/'new'
    storage = m.Storage(output)
    other = root/'elsewhere'; other.mkdir()
    output.symlink_to(other, target_is_directory=True)
    with pytest.raises(RuntimeError, match='canonical output path changed'):
        storage.check()


@pytest.mark.parametrize('malformed', [[], {}, {'min_free_bytes': None}])
def test_storage_reports_invalid_archive_schema(tmp_path, monkeypatch, malformed):
    from types import SimpleNamespace
    root = tmp_path/'external'
    (root/'ros_log').mkdir(parents=True)
    (root/'grasp_records').mkdir()
    (root/'grasp_records'/'archive_config.json').write_text(json.dumps(malformed))
    monkeypatch.setattr(m, 'ROOT', root)
    monkeypatch.setattr(m, 'MOUNT', root)
    monkeypatch.setattr(Path, 'is_mount', lambda _: True)
    monkeypatch.setattr(m.os, 'statvfs', lambda _: SimpleNamespace(f_flag=0))
    with pytest.raises(RuntimeError, match='configuration schema invalid'):
        m.Storage(root/'ros_log'/'new').check()


def test_measured_size_counts_active_file_without_creating_it(tmp_path):
    out = tmp_path/'record'; out.mkdir()
    (out/'full_frames.bag.active').write_bytes(b'1234')
    assert m.measured_size(out) == 4
    assert not (out/'full_frames.bag').exists()


def test_budget_must_include_pending_buffer_and_data():
    limits = m.Limits()
    with pytest.raises(ValueError, match='one MiB'):
        dataclasses.replace(limits, max_bytes=limits.headroom).validate(int(4.5*m.GIB))


def test_native_size_budget_overshoot_is_reported_not_truncated(tmp_path):
    limits = m.Limits(max_bytes=290*m.MIB, duration=1)
    run, output, _, _ = invocation(tmp_path, limits=limits,
                                  behavior=dict(overshoot=limits.max_bytes+1))
    result = run()
    assert result['recording_closed'] and result['within_sidecar_budget'] is False
    assert (output/'full_frames.bag').stat().st_size == limits.max_bytes+1


def test_native_writer_cap_exit_is_classified_even_between_polls(tmp_path):
    limits = m.Limits(max_bytes=290*m.MIB)
    def exited(out, writer):
        small_indexed_bag(out/'full_frames.bag')
        with (out/'full_frames.bag').open('ab') as stream:
            stream.truncate(2*m.MIB)
        writer.returncode = 0
    run, _, writer, _ = invocation(tmp_path, limits=limits, on_start=exited)
    result = run()
    assert result['stop_reason'] == 'budget_exhausted'
    assert result['budget_trigger'] == 'native_size_limit_observed'
    assert not writer.signals and result['recording_closed']
