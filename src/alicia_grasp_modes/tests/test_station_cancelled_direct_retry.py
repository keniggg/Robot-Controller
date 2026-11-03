"""Real worker termination and real direct polling after measured-station wait.

The optional private-work certificate is deliberately withheld at admission.
This models its supported fail-closed None result; its GEN19 reason was not
recorded. The collection, submission, worker, recovery and poll are real.
No ROS graph, inference service or hardware is used.
"""
from copy import deepcopy
import json
import threading
import time
from types import SimpleNamespace as NS

import pytest

from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_observation_measurement_remote import context
from test_station_compute_work_handoff import configure, rgbd, next_row
from test_unknown_pending_identity import deliver
from test_remote_modes import base


def unretained_submission(c):
    n = c.node
    start = configure(c)
    previous = None
    certificate = n._station_compute_snapshot_work
    n._station_compute_snapshot_work = lambda snapshot: None
    for i in range(3):
        ns = start + i * 200_000_000
        feedback(c, ns-10_000_000)
        feedback(c, ns+10_000_000)
        clock(c, ns+100_000_000)
        row = row_at(c, ns, recovery=previous)
        previous = deepcopy(row['status']['station_compute_wait'])
        rgbd(c, row)
        deliver(n, row)
        submitted = n._poll_direct_snapshot()
        assert submitted is (i == 2), getattr(n, '_direct_snapshot_last_rejection', None)
    n._station_compute_snapshot_work = certificate
    ticket = n._stream_worker_ticket
    assert ticket is not None
    assert not n._station_compute_work_certificate(ticket)
    return start, row, ticket


def complete_worker(n, prepare, accept=None):
    n._prepare_and_predict = prepare
    accepted = []
    n._accept_prediction = accept or accepted.append
    n._stream_worker = threading.Thread(target=n._stream_worker_main, daemon=True)
    n._stream_worker.start()
    deadline = time.perf_counter() + 3.
    while time.perf_counter() < deadline:
        # Metrics precede final cleanup. The worker clears busy and retires
        # the direct slot under this same lock; observe that transaction whole.
        with n._stream_condition:
            if n.pipeline_metrics_pub.messages and not n._stream_worker_busy:
                return json.loads(n.pipeline_metrics_pub.messages[-1].data), accepted
        time.sleep(.002)
    with n._stream_condition:
        assert not n._stream_worker_busy
        assert n.pipeline_metrics_pub.messages
        return json.loads(n.pipeline_metrics_pub.messages[-1].data), accepted


def test_real_worker_pending_cancellation_releases_slot_then_new_source_rejoins(station):
    c = station
    n = c.node
    start, row, ticket = unretained_submission(c)
    identity = n._current_stream_target_identity()
    generation = n._stream_generation
    deadline = n._near_field_phase_deadline_sec
    reference = deepcopy(n._station_compute_reference['contract'])
    clock(c, reference['geometry_deadline_ns'] + 10_000_000)
    assert n._station_compute_poll()
    assert n._station_compute_soft_wait
    failures = []
    n._observe_stream_prediction_failure = lambda *args: failures.append(args)
    try:
        def prepare(actual):
            n._require_stream_ticket_current(actual)
            raise AssertionError('cancelled source must never reach preparation')
        metrics, accepted = complete_worker(n, prepare)
        assert metrics['status'] == 'TARGET_PROCESSING_PENDING'
        assert failures == []
        assert accepted == []
        assert n._direct_near_field_submission_generation is None
        assert n._poll_stream_snapshot() is False  # No new measured join yet.
        assert n._stream_generation == generation
        assert n._current_stream_target_identity() == identity
        assert n._near_field_phase_deadline_sec == deadline
        assert n.last_submitted_stamp_ns == ticket.payload[0].stamp_ns
        assert not n.latest_object_geometry.valid
        assert not n.latest_preview_rich_plan.valid

        # New measured fixture sources pass existing object/mask/status and
        # feedback gates; neither the cancelled batch nor its oldest source
        # can be silently replayed.
        assert not n.submit_stream_snapshot(ticket.payload[0], require_idle_worker=True)
        stop = threading.Event()
        def next_prepare(actual):
            stop.wait(1.)
            raise base.StreamResultCancelled('TEST_FINISHED')
        n._prepare_and_predict = next_prepare
        for i in range(4, 7):
            row = next_row(c, start+i*400_000_000, row['status']['station_compute_wait'])
            # The production poller must collect each freshly completed source
            # inside the original 0.35s window, just as before the cancellation.
            assert n._poll_stream_snapshot() is (i == 6), str(
                getattr(n, '_direct_snapshot_last_rejection', None))
        assert n._pipeline_counters['submitted'] == 2
        assert n.last_submitted_stamp_ns > ticket.payload[0].stamp_ns
        assert n._near_field_phase_deadline_sec == deadline
        stop.set()
    finally:
        n.shutdown_streaming_worker()


@pytest.mark.parametrize('case', ['acceptance_pending', 'geometry_rejected',
                                  'untyped_pending_error', 'different_preparation_cancel'])
def test_real_worker_does_not_reopen_geometry_or_acceptance_failures(station, case):
    c = station
    n = c.node
    _, _, ticket = unretained_submission(c)
    marker = n._direct_near_field_submission_generation
    failures = []
    n._observe_stream_prediction_failure = lambda *args: failures.append(args)
    acceptance_calls = []

    def prepare(actual):
        if case == 'untyped_pending_error':
            raise ValueError('TARGET_PROCESSING_PENDING')
        if case == 'different_preparation_cancel':
            raise base.StreamResultCancelled('TARGET_LOST')
        return NS(ticket=actual)

    def accept(prepared):
        acceptance_calls.append(prepared)
        if case == 'acceptance_pending':
            raise base.StreamResultCancelled('TARGET_PROCESSING_PENDING')
        return {'status': 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'}

    try:
        metrics, _ = complete_worker(n, prepare, accept)
        expected = {'acceptance_pending': 'TARGET_PROCESSING_PENDING',
                    'geometry_rejected': 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'}.get(case, 'PREDICT_FAILED')
        assert metrics['status'] == expected
        assert n._direct_near_field_submission_generation == marker
        assert n._poll_direct_snapshot() is False
        assert n._pipeline_counters['submitted'] == 1
        assert len(acceptance_calls) == int(case in ('acceptance_pending', 'geometry_rejected'))
        assert len(failures) == int(case in ('untyped_pending_error', 'different_preparation_cancel'))
    finally:
        n.shutdown_streaming_worker()


@pytest.mark.parametrize('case', ['stopped', 'deadline', 'generation', 'target_epoch',
    'move_pregrasp', 'missing_registration', 'retired', 'published', 'publication_started',
    'valid_preview', 'manual_takeover', 'control_epoch', 'phase_id', 'phase_start'])
def test_preparation_retry_cannot_cross_original_scope_or_publish_boundary(station, case):
    c = station
    n = c.node
    _, _, ticket = unretained_submission(c)
    if case == 'stopped':
        n.streaming_enabled = False
    elif case == 'deadline':
        clock(c, round(n._near_field_phase_deadline_sec*1e9))
    elif case == 'generation':
        n._stream_generation += 1
    elif case == 'target_epoch':
        n.target_instance_epoch += 1
    elif case == 'move_pregrasp':
        n._station_compute_task_state = 'MOVE_PREGRASP'
    elif case == 'missing_registration':
        n._station_compute_reference = None
    elif case == 'retired':
        n._station_compute_retired = True
    elif case == 'published':
        n._station_compute_work_published = True
    elif case == 'publication_started':
        n._station_compute_publication_started = True
    elif case == 'valid_preview':
        n.latest_preview_rich_plan = NS(valid=True)
    elif case == 'manual_takeover':
        c.params['/gui/joint_direct_mode'] = True
    elif case == 'control_epoch':
        n._measurement_remote_epoch_ns += 1
    elif case == 'phase_id':
        n._station_compute_phase.phase_id += 1
    else:
        from test_observation_measurement_remote import at
        n._station_compute_phase.header.stamp = at(n._station_compute_phase.header.stamp.to_nsec()+1)
    with n._stream_condition:
        assert not n._release_preparation_cancelled_direct_snapshot_locked(
            ticket, base.StreamResultCancelled('TARGET_PROCESSING_PENDING'))


def test_helper_cannot_report_completion_inside_locked_slot_retirement(station, monkeypatch):
    c=station;n=c.node
    _start,_row,ticket=unretained_submission(c)
    clock(c,n._station_compute_reference['contract']['geometry_deadline_ns']+10_000_000)
    assert n._station_compute_poll() and n._station_compute_soft_wait
    original=n._release_preparation_cancelled_direct_snapshot_locked
    entered=threading.Event();allow=threading.Event();finished=threading.Event()
    outcomes=[];errors=[]
    def release(actual,error):
        assert n._stream_condition._is_owned()
        assert n._stream_worker_busy is False
        assert n._direct_near_field_submission_generation==ticket.generation
        assert n.pipeline_metrics_pub.messages
        entered.set()
        assert allow.wait(2.), 'test controller did not release cleanup barrier'
        return original(actual,error)
    monkeypatch.setattr(n,'_release_preparation_cancelled_direct_snapshot_locked',release)
    def prepare(actual):
        n._require_stream_ticket_current(actual)
        raise AssertionError('cancelled source reached preparation')
    def call_helper():
        try:outcomes.append(complete_worker(n,prepare))
        except BaseException as exc:errors.append(exc)
        finally:finished.set()
    caller=threading.Thread(target=call_helper,daemon=True)
    caller.start()
    try:
        assert entered.wait(2.), 'real worker did not enter release hook'
        # This is the exact old flaky read boundary, made deterministic.
        # No production state is fabricated; the real worker owns the lock.
        assert not finished.wait(.1), 'helper reported completion before locked slot retirement'
        allow.set()
        assert finished.wait(2.), 'helper did not observe completed transaction'
        assert not errors and len(outcomes)==1
        metrics,accepted=outcomes[0]
        assert metrics['status']=='TARGET_PROCESSING_PENDING' and accepted==[]
        with n._stream_condition:
            assert n._direct_near_field_submission_generation is None
            assert n._stream_worker_busy is False
            assert n.last_submitted_stamp_ns==ticket.payload[0].stamp_ns
        assert n._poll_stream_snapshot() is False
    finally:
        allow.set();caller.join(2.);n.shutdown_streaming_worker()
        assert not caller.is_alive()


def test_completion_helper_does_not_hide_a_real_missing_slot_release(station, monkeypatch):
    # Deliberately model a real failed cleanup. The helper observes completion,
    # never waits for marker=None; the original assertion must still catch it.
    import traceback
    n = station.node
    monkeypatch.setattr(n, '_release_preparation_cancelled_direct_snapshot_locked',
                        lambda ticket, error: False)
    with pytest.raises(AssertionError) as failure:
        test_real_worker_pending_cancellation_releases_slot_then_new_source_rejoins(station)
    frames = traceback.extract_tb(failure.value.__traceback__)
    assert frames[-1].name == 'test_real_worker_pending_cancellation_releases_slot_then_new_source_rejoins'
    assert frames[-1].line.strip() == 'assert n._direct_near_field_submission_generation is None'
    assert n._direct_near_field_submission_generation is not None
    assert n._stream_worker_busy is False
