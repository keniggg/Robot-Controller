"""Publication concurrency and actual ownership contracts, without ROS/SDK."""
from dataclasses import FrozenInstanceError
import threading
import time

import numpy as np
import pytest

from alicia_flexible_grasp.vision.latest_rgbd_publication import (
    LatestRGBDPublication, RGBDSnapshot,
)


def pair(value=1):
    return np.full((6, 8, 3), value, np.uint8), np.full((6, 8), value * 100, np.uint16)


def wait_for(predicate, timeout=1.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.001)
    assert predicate(), 'worker did not reach the expected state'


def test_snapshot_copies_both_noncontiguous_sdk_views_and_is_immutable():
    color, depth = pair(7)
    color, depth = color[:, ::-1], depth[:, ::-1]
    snapshot = RGBDSnapshot.copy_pair(color, depth, 4.25, 1791000000000000001, 1000)
    color[:] = 0; depth[:] = 0
    assert np.all(snapshot.color == 7) and np.all(snapshot.depth == 700)
    assert not np.shares_memory(snapshot.color, color)
    assert not np.shares_memory(snapshot.depth, depth)
    assert snapshot.color.flags.c_contiguous and snapshot.depth.flags.c_contiguous
    assert snapshot.read_completed_monotonic == 4.25
    assert snapshot.host_read_completed_stamp_ns == 1791000000000000001
    with pytest.raises(ValueError):
        snapshot.color[0, 0, 0] = 3
    with pytest.raises(ValueError):
        snapshot.depth.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        snapshot.read_completed_monotonic = 5.


def test_blocked_publisher_does_not_block_reader_and_keeps_only_latest_whole_pair():
    entered, release = threading.Event(), threading.Event()
    delivered = []
    def publish(snapshot):
        delivered.append(snapshot)
        if len(delivered) == 1:
            entered.set()
            assert release.wait(2.)
    worker = LatestRGBDPublication(publish, monotonic=lambda: 10.)
    try:
        assert worker.submit(*pair(1), 10., 1001)
        assert entered.wait(1.)
        done = threading.Event()
        def submit_more():
            for value in range(2, 102):
                assert worker.submit(*pair(value), 10., 1000 + value)
            done.set()
        producer = threading.Thread(target=submit_more, daemon=True); producer.start()
        assert done.wait(.5), 'reader waited for the blocked publisher'
        producer.join(.5)
        before = worker.summary()
        assert before['pending'] == 1 and before['inflight']
        assert before['submitted'] == 101 and before['replaced_pending'] == 99
        release.set()
        wait_for(lambda: worker.summary()['published'] == 2)
        assert [s.host_read_completed_stamp_ns for s in delivered] == [1001, 1101]
        assert [int(s.color[0, 0, 0]) for s in delivered] == [1, 101]
        assert [int(s.depth[0, 0]) for s in delivered] == [100, 10100]
    finally:
        release.set(); worker.close()


@pytest.mark.parametrize('completed,count', [(9.8, 'dropped_expired'), (10.01, 'dropped_future')])
def test_age_checked_on_worker_before_call(completed, count):
    delivered = []
    worker = LatestRGBDPublication(delivered.append, monotonic=lambda: 10.)
    try:
        assert worker.submit(*pair(), completed, 500)
        wait_for(lambda: worker.summary()[count] == 1)
        assert delivered == [] and worker.summary()['failure'] is None
    finally:
        worker.close()


def test_waiting_pending_snapshot_can_expire_while_publisher_is_blocked():
    entered, release = threading.Event(), threading.Event()
    now = [10.]; delivered = []
    def publish(snapshot):
        delivered.append(snapshot); entered.set(); assert release.wait(2.)
    worker = LatestRGBDPublication(publish, monotonic=lambda: now[0])
    try:
        worker.submit(*pair(1), 10., 1); assert entered.wait(1.)
        worker.submit(*pair(2), 10., 2)
        now[0] = 10.151
        release.set()
        wait_for(lambda: worker.summary()['dropped_expired'] == 1)
        assert len(delivered) == 1
    finally:
        release.set(); worker.close()


def test_age_is_rechecked_at_final_publication_admission():
    times = iter([10., 10.151]); delivered = []
    worker = LatestRGBDPublication(delivered.append, monotonic=lambda: next(times))
    try:
        worker.submit(*pair(), 10., 1)
        wait_for(lambda: worker.summary()['dropped_expired'] == 1)
        assert delivered == [] and worker.summary()['failure'] is None
    finally:
        worker.close()


def test_discard_pending_clears_old_mode_pair_without_waiting_for_inflight():
    entered, release = threading.Event(), threading.Event(); delivered = []
    def publish(snapshot):
        delivered.append(snapshot)
        if len(delivered) == 1:
            entered.set(); assert release.wait(2.)
    worker = LatestRGBDPublication(publish, monotonic=lambda: 10.)
    try:
        worker.submit(*pair(1), 10., 1); assert entered.wait(1.)
        worker.submit(*pair(2), 10., 2)
        assert worker.discard_pending() == 1
        assert worker.summary()['publishing'] and worker.summary()['pending'] == 0
        worker.submit(*pair(3), 10., 3); release.set()
        wait_for(lambda: worker.summary()['published'] == 2)
        assert [s.host_read_completed_stamp_ns for s in delivered] == [1, 3]
        assert worker.summary()['dropped_discarded'] == 1
    finally:
        release.set(); worker.close()


def test_discard_revokes_selected_pair_before_final_check():
    entered, release = threading.Event(), threading.Event(); delivered = []; calls = [0]
    def clock():
        calls[0] += 1
        if calls[0] == 2:
            entered.set(); assert release.wait(2.)
        return 10.
    worker = LatestRGBDPublication(delivered.append, monotonic=clock)
    try:
        worker.submit(*pair(1), 10., 1); assert entered.wait(1.)
        assert worker.summary()['inflight'] and not worker.summary()['publishing']
        assert worker.discard_pending() == 0
        release.set()
        wait_for(lambda: worker.summary()['dropped_discarded'] == 1)
        assert delivered == []
        worker.submit(*pair(2), 10., 2)
        wait_for(lambda: worker.summary()['published'] == 1)
        assert delivered[0].host_read_completed_stamp_ns == 2
    finally:
        release.set(); worker.close()


def test_generation_change_during_copy_revokes_pair_and_concurrent_copy_is_bounded(monkeypatch):
    entered, release = threading.Event(), threading.Event(); delivered = []
    original = RGBDSnapshot.copy_pair
    def blocking_copy(cls, *args):
        entered.set(); assert release.wait(2.)
        return original(*args)
    monkeypatch.setattr(RGBDSnapshot, 'copy_pair', classmethod(blocking_copy))
    worker = LatestRGBDPublication(delivered.append, monotonic=lambda: 10.)
    accepted = []
    producer = threading.Thread(target=lambda: accepted.append(worker.submit(*pair(1), 10., 1)),
                                daemon=True)
    try:
        producer.start(); assert entered.wait(1.)
        assert not worker.submit(*pair(2), 10., 2)
        assert worker.summary()['rejected_busy'] == 1
        assert worker.discard_pending() == 0
        release.set(); producer.join(1.)
        assert accepted == [False] and delivered == []
        report = worker.summary()
        assert report['rejected_generation_change'] == 1 and report['submitted'] == 0
    finally:
        release.set(); producer.join(1.); worker.close()


def test_failure_is_latched_pending_is_discarded_and_submission_stops():
    entered, release = threading.Event(), threading.Event()
    def fail(snapshot):
        entered.set(); assert release.wait(2.); raise RuntimeError('TCP writer failed')
    worker = LatestRGBDPublication(fail, monotonic=lambda: 10.)
    try:
        worker.submit(*pair(1), 10., 1); assert entered.wait(1.)
        worker.submit(*pair(2), 10., 2); release.set()
        wait_for(lambda: worker.summary()['failure'] is not None)
        assert not worker.submit(*pair(3), 10., 3)
        wait_for(lambda: not worker.summary()['worker_alive'])
        report = worker.summary()
        assert report['failure'] == 'RuntimeError: TCP writer failed'
        assert report['publish_failures'] == 1 and report['published'] == 0
        assert report['dropped_on_failure'] == 1 and report['pending'] == 0
        assert report['rejected_after_failure'] == 1
    finally:
        release.set(); worker.close()


def test_close_is_bounded_and_truthfully_reports_still_blocked_worker():
    entered, release = threading.Event(), threading.Event()
    def publish(snapshot):
        entered.set(); assert release.wait(2.)
    worker = LatestRGBDPublication(publish, monotonic=lambda: 10.)
    try:
        worker.submit(*pair(1), 10., 1); assert entered.wait(1.)
        worker.submit(*pair(2), 10., 2)
        started = time.monotonic(); report = worker.close(timeout=.02)
        assert time.monotonic() - started < .5
        assert report['worker_alive'] and not report['shutdown_complete']
        assert report['inflight'] and report['dropped_on_close'] == 1
        assert report['pending'] == 0 and not worker.submit(*pair(3), 10., 3)
        release.set()
        finished = worker.close()
        assert not finished['worker_alive'] and finished['shutdown_complete']
        assert finished['published'] == 1 and finished['failure'] is None
    finally:
        release.set(); worker.close()


def test_close_cannot_wait_more_than_one_second_even_if_requested_timeout_is_longer():
    entered, release = threading.Event(), threading.Event()
    def publish(snapshot):
        entered.set(); release.wait(3.)
    worker = LatestRGBDPublication(publish, monotonic=lambda: 10.)
    try:
        worker.submit(*pair(), 10., 1); assert entered.wait(1.)
        started = time.monotonic(); report = worker.close(timeout=100.)
        assert time.monotonic() - started < 1.5
        assert report['worker_alive'] and not report['shutdown_complete']
        assert report['max_pending_pairs'] == 1
        assert report['max_pair_bytes'] == 16 * 1024 * 1024
        assert report['peak_pair_bytes'] == 240
    finally:
        release.set(); worker.close()


def test_duration_summary_measures_real_blocked_publish_with_injected_clock():
    worker = LatestRGBDPublication(lambda snapshot: time.sleep(.025), monotonic=lambda: 10.)
    try:
        worker.submit(*pair(), 10., 1)
        wait_for(lambda: worker.summary()['published'] == 1)
        report = worker.summary()
        assert report['maximum_publish_seconds'] >= .02
        assert report['last_publish_seconds'] == report['total_publish_seconds']
    finally:
        worker.close()


@pytest.mark.parametrize('clock', [lambda: float('nan'), lambda: True])
def test_invalid_clock_latches_instead_of_publishing(clock):
    delivered = []; worker = LatestRGBDPublication(delivered.append, monotonic=clock)
    try:
        worker.submit(*pair(), 10., 1)
        wait_for(lambda: worker.summary()['failure'] is not None)
        assert delivered == [] and worker.summary()['dropped_invalid_clock'] == 1
        assert not worker.submit(*pair(), 10., 2)
    finally:
        worker.close()


@pytest.mark.parametrize('completed,stamp', [(True, 1), (float('nan'), 1), (-1., 1),
                                          (10., True), (10., 0), (10., 1.5)])
def test_invalid_source_identity_is_rejected(completed, stamp):
    with pytest.raises(ValueError):
        RGBDSnapshot.copy_pair(*pair(), completed, stamp, 1000)


def test_pair_shapes_dtypes_and_memory_bound_are_enforced():
    color, depth = pair()
    for bad_color, bad_depth in [(color[:-1], depth), (color.astype(np.float32), depth),
                                 (color, depth.astype(np.float32)), (color[:, :, 0], depth)]:
        with pytest.raises(ValueError):
            RGBDSnapshot.copy_pair(bad_color, bad_depth, 10., 1, 1000)
    with pytest.raises(ValueError):
        RGBDSnapshot.copy_pair(color, depth, 10., 1, color.nbytes + depth.nbytes - 1)


@pytest.mark.parametrize('age', [0., -.1, .150001, float('inf'), True])
def test_age_configuration_cannot_relax_the_150ms_bound(age):
    with pytest.raises(ValueError):
        LatestRGBDPublication(lambda snapshot: None, max_age_sec=age)
