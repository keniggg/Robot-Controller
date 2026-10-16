"""Real filesystem cases for the rosbag wrapper/writer close-order boundary."""
import importlib.util
from pathlib import Path
import threading
import time

import pytest


def load_recorder():
    work = next(parent for parent in Path(__file__).resolve().parents
                if (parent / '.runtime/record_attempt_full_frames_20261013.py').is_file())
    spec = importlib.util.spec_from_file_location('record_sidecar_finalize_under_test',
        work / '.runtime/record_attempt_full_frames_20261013.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parent_exit_before_writer_rename_waits_for_real_closed_file(tmp_path):
    recorder = load_recorder()
    bag = tmp_path / 'frames.bag'
    active = tmp_path / 'frames.bag.active'
    payload = b'original complete recording bytes'
    active.write_bytes(payload)
    done = threading.Event()

    def finish_writer():
        time.sleep(.08)
        active.replace(bag)
        done.set()

    thread = threading.Thread(target=finish_writer)
    thread.start()
    try:
        recorder.wait_for_bag_finalize(bag, timeout_sec=1.)
        assert done.wait(.1)
        assert bag.read_bytes() == payload and not active.exists()
    finally:
        thread.join(timeout=2.)
        assert not thread.is_alive()


@pytest.mark.parametrize('final_present,active_present', [(False, False), (False, True), (True, True)])
def test_missing_or_still_active_writer_never_reports_closed(tmp_path, final_present, active_present):
    recorder = load_recorder()
    bag = tmp_path / 'frames.bag'
    active = tmp_path / 'frames.bag.active'
    if final_present:
        bag.write_bytes(b'old final')
    if active_present:
        active.write_bytes(b'still writing')
    before = {path.name: path.read_bytes() for path in tmp_path.iterdir()}
    started = time.monotonic()
    with pytest.raises(TimeoutError, match='did not finalize'):
        recorder.wait_for_bag_finalize(bag, timeout_sec=.08)
    assert time.monotonic() - started < 1.
    assert {path.name: path.read_bytes() for path in tmp_path.iterdir()} == before


def test_already_closed_file_is_preserved(tmp_path):
    recorder = load_recorder()
    bag = tmp_path / 'frames.bag'
    bag.write_bytes(b'closed')
    before = bag.stat()
    recorder.wait_for_bag_finalize(bag, timeout_sec=0.)
    after = bag.stat()
    assert (before.st_ino, before.st_mtime_ns, before.st_size) == (
        after.st_ino, after.st_mtime_ns, after.st_size)


def test_directory_named_bag_is_not_closed_recording(tmp_path):
    recorder = load_recorder()
    bag = tmp_path / 'frames.bag'
    bag.mkdir()
    with pytest.raises(TimeoutError):
        recorder.wait_for_bag_finalize(bag, timeout_sec=0.)
