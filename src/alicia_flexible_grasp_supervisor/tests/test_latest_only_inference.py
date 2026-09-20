import math
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT / 'src') not in sys.path:
    sys.path.insert(0, str(ROOT / 'src'))

from alicia_flexible_grasp.vision.latest_only_inference import (
    InferenceTicket,
    LatestOnlyInferenceCoordinator,
)


class FakeClock:
    def __init__(self, now):
        self.now = float(now)

    def __call__(self):
        return self.now


def test_busy_submit_keeps_only_latest_pending_snapshot():
    clock = FakeClock(10.0)
    queue = LatestOnlyInferenceCoordinator(clock=clock)
    queue.start()

    first = queue.submit('frame-1', 9.8, target_epoch=7)
    second = queue.submit('frame-2', 9.9, target_epoch=7)
    third = queue.submit('frame-3', 10.0, target_epoch=7)

    assert first.ticket_to_start.payload == 'frame-1'
    assert second.ticket_to_start is None
    assert third.ticket_to_start is None
    assert third.replaced_request_id == second.pending_request_id
    assert queue.pending_count == 1


def test_completion_starts_latest_pending_without_backlog():
    clock = FakeClock(10.0)
    queue = LatestOnlyInferenceCoordinator(clock=clock)
    queue.start()
    first = queue.submit('frame-1', 9.8, target_epoch=7)
    queue.submit('frame-2', 9.9, target_epoch=7)
    latest = queue.submit('frame-3', 10.0, target_epoch=7)

    done = queue.complete(first.ticket_to_start, now_sec=10.2)

    assert done.accepted is True
    assert done.code == 'ACCEPTED'
    assert done.next_ticket.request_id == latest.pending_request_id
    assert done.next_ticket.payload == 'frame-3'
    assert done.result_age_sec == pytest.approx(0.4)
    assert queue.pending_count == 0


def test_stop_and_age_drop_active_completion():
    clock = FakeClock(10.0)
    queue = LatestOnlyInferenceCoordinator(
        result_max_age_sec=0.5,
        clock=clock,
    )
    queue.start()
    stopped = queue.submit('frame-1', 9.8, target_epoch=7).ticket_to_start
    queue.stop()
    assert queue.complete(stopped, now_sec=10.1).code == 'GENERATION_STALE'

    queue.start()
    old = queue.submit('frame-2', 9.0, target_epoch=8).ticket_to_start
    assert queue.complete(old, now_sec=10.0).code == 'RESULT_EXPIRED'


@pytest.mark.parametrize('stamp', [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_submit_rejects_non_positive_or_non_finite_snapshot_stamp(stamp):
    queue = LatestOnlyInferenceCoordinator(clock=FakeClock(10.0))
    queue.start()

    with pytest.raises(ValueError, match='snapshot_stamp_sec'):
        queue.submit('frame', stamp, target_epoch=1)


def test_unknown_and_mismatched_completion_cannot_release_active_request():
    queue = LatestOnlyInferenceCoordinator(clock=FakeClock(10.0))
    queue.start()
    active = queue.submit('frame-1', 9.8, target_epoch=7).ticket_to_start
    pending = queue.submit('frame-2', 9.9, target_epoch=7)
    unknown = InferenceTicket(
        request_id=active.request_id + 100,
        generation=active.generation,
        snapshot_stamp_sec=active.snapshot_stamp_sec,
        target_epoch=active.target_epoch,
        payload=active.payload,
        submitted_monotonic_sec=active.submitted_monotonic_sec,
    )
    mismatched = InferenceTicket(
        request_id=active.request_id,
        generation=active.generation,
        snapshot_stamp_sec=active.snapshot_stamp_sec + 0.1,
        target_epoch=active.target_epoch,
        payload=active.payload,
        submitted_monotonic_sec=active.submitted_monotonic_sec,
    )

    assert queue.complete(unknown, now_sec=10.0).code == 'UNKNOWN_REQUEST'
    assert queue.complete(mismatched, now_sec=10.0).code == 'REQUEST_MISMATCH'
    completed = queue.complete(active, now_sec=10.0)
    assert completed.accepted is True
    assert completed.next_ticket.request_id == pending.pending_request_id


def test_reset_target_epoch_clears_old_pending_and_rejects_active_result():
    queue = LatestOnlyInferenceCoordinator(clock=FakeClock(10.0))
    queue.start()
    active = queue.submit('old-active', 9.8, target_epoch=7).ticket_to_start
    queue.submit('old-pending', 9.9, target_epoch=7)

    queue.reset_target_epoch(8)
    replacement = queue.submit('new-pending', 10.0, target_epoch=8)
    completed = queue.complete(active, now_sec=10.1, target_epoch=8)

    assert completed.accepted is False
    assert completed.code == 'TARGET_EPOCH_STALE'
    assert completed.next_ticket.request_id == replacement.pending_request_id
    assert completed.next_ticket.target_epoch == 8


def test_invalid_completion_target_epoch_preserves_active_and_pending_order():
    queue = LatestOnlyInferenceCoordinator(clock=FakeClock(10.0))
    queue.start()
    active = queue.submit('active', 9.8, target_epoch=7).ticket_to_start
    pending = queue.submit('pending', 9.9, target_epoch=7)

    with pytest.raises(ValueError, match='target_epoch'):
        queue.complete(active, now_sec=10.0, target_epoch=-1)

    assert queue.pending_count == 1
    retried = queue.complete(active, now_sec=10.0, target_epoch=7)
    assert retried.accepted is True
    assert retried.next_ticket.request_id == pending.pending_request_id
    assert retried.next_ticket.payload == 'pending'
    assert queue.pending_count == 0
    pending_done = queue.complete(
        retried.next_ticket,
        now_sec=10.1,
        target_epoch=7,
    )
    assert pending_done.accepted is True
    assert pending_done.next_ticket is None


def test_stop_clears_pending_and_submit_requires_running_generation():
    queue = LatestOnlyInferenceCoordinator(clock=FakeClock(10.0))
    with pytest.raises(RuntimeError, match='not running'):
        queue.submit('before-start', 9.7, target_epoch=7)

    generation = queue.start()
    active = queue.submit('active', 9.8, target_epoch=7).ticket_to_start
    queue.submit('pending', 9.9, target_epoch=7)
    stopped_generation = queue.stop()

    assert stopped_generation == generation + 1
    assert queue.pending_count == 0
    assert queue.complete(active, now_sec=10.0).next_ticket is None
    with pytest.raises(RuntimeError, match='not running'):
        queue.submit('after-stop', 10.0, target_epoch=7)


def test_reserved_next_is_refreshed_after_slow_local_processing_without_retiming():
    q = LatestOnlyInferenceCoordinator(result_max_age_sec=15., clock=lambda: 10.)
    q.start()
    first = q.submit('first', 10., 1).ticket_to_start
    assert q.claim_for_preparation(first) == (first, None)
    q.submit('reserved', 11., 1)
    next_ticket = q.complete(first, now_sec=12.).next_ticket
    q.submit('newest_after_slow_processing', 30., 1)
    claimed, replaced = q.claim_for_preparation(next_ticket)
    assert replaced == next_ticket.request_id
    assert claimed.payload == 'newest_after_slow_processing'
    assert claimed.snapshot_stamp_sec == 30.
    assert q.pending_count == 0
    # An in-flight request can no longer be replaced.
    pending = q.submit('future', 31., 1)
    assert q.claim_for_preparation(claimed) == (claimed, None)
    assert q.complete(claimed, now_sec=32.).next_ticket.request_id == pending.pending_request_id


@pytest.mark.parametrize('change', ['stop', 'restart', 'target'])
def test_stale_reserved_ticket_cannot_claim_new_generation_or_target(change):
    q = LatestOnlyInferenceCoordinator(clock=lambda: 10.)
    q.start()
    first = q.submit('old', 10., 1).ticket_to_start
    if change == 'stop': q.stop()
    elif change == 'restart':
        q.stop(); q.start(); q.submit('new generation', 11., 1)
    else:
        q.reset_target_epoch(2); q.submit('new target', 11., 2)
    assert q.claim_for_preparation(first) == (first, None)


def test_newer_request_with_older_source_cannot_replace_reserved_frame():
    q = LatestOnlyInferenceCoordinator(clock=lambda: 10.)
    q.start()
    first = q.submit('first', 10., 1).ticket_to_start
    q.submit('out of order source', 9., 1)
    assert q.claim_for_preparation(first) == (first, None)
