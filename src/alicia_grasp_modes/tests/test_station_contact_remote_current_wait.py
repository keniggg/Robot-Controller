"""Receipt construction crossing its current-source window, no ROS or motion.

The real context, ticket, wait, quarantine and three-topic registration execute.
Synthetic pixels/plan geometry isolate admission, not physical grasp quality.
"""
from copy import deepcopy
import sys
import threading

import pytest

from test_station_compute_wait_remote import station
from test_observation_measurement_remote import context
from test_station_compute_work_handoff import next_row
from test_gen11_contact_remote_receipt import published_contact
from test_remote_modes import base
from alicia_grasp_modes.station_contact_admission import validate_receipt
from alicia_grasp_modes import station_compute_wait_remote as implementation


def cross_after_real_context(c, monkeypatch, fault):
    n = c.node
    original = n._station_compute_context
    crossed = []
    mono_offset = [0.]
    original_mono = implementation.time.monotonic
    monkeypatch.setattr(implementation.time, 'monotonic',
                        lambda: original_mono() + mono_offset[0])

    def checked(contract, *args, **kwargs):
        result = original(contract, *args, **kwargs)
        if (not crossed and sys._getframe(1).f_code.co_name
                == '_station_contact_computation_receipt'):
            crossed.append(True)
            record = n._station_compute_reference
            if fault == 'geometry_ros':
                c.clock[0] = (contract['geometry_deadline_ns'] + 1_000_000) / 1e9
            elif fault == 'geometry_exact':
                c.clock[0] = contract['geometry_deadline_ns'] / 1e9
            elif fault == 'geometry_mono':
                mono_offset[0] = record['geometry_monotonic'] - original_mono() + .001
            elif fault == 'identity_ros':
                c.clock[0] = contract['identity_deadline_ns'] / 1e9
            elif fault == 'identity_mono':
                mono_offset[0] = record['identity_monotonic'] - original_mono() + .001
            elif fault == 'clock_back':
                c.clock[0] -= .01
            else:
                raise AssertionError(fault)
        return result

    monkeypatch.setattr(n, '_station_compute_context', checked)
    return crossed


def start_receipt(n, plan, ticket, monkeypatch):
    entered = threading.Event()
    done = threading.Event()
    results = []
    wait = n._stream_condition.wait

    def tracked_wait(*args, **kwargs):
        entered.set()
        return wait(*args, **kwargs)

    monkeypatch.setattr(n._stream_condition, 'wait', tracked_wait)

    def run():
        try:
            results.append(n._station_contact_computation_receipt(plan, ticket))
        except Exception as exc:
            results.append(exc)
        finally:
            done.set()

    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    return entered, done, results, worker


def finish(n, worker, done):
    if worker.is_alive():
        with n._stream_condition:
            n._station_compute_retire('test cleanup after assertion')
            n._stream_condition.notify_all()
        assert done.wait(3), 'receipt worker leaked after cancellation'
    worker.join(3)
    assert not worker.is_alive()


@pytest.mark.parametrize('fault', ['geometry_ros', 'geometry_exact', 'geometry_mono'])
def test_receipt_context_crossing_releases_both_locks_then_uses_real_successor(station, monkeypatch, fault):
    c = station
    n = c.node
    t, row, ticket, prepared, plan = published_contact(c)
    original_plan = deepcopy(plan)
    reference = n._station_compute_reference
    old = deepcopy(reference['contract'])
    deadlines = (reference['geometry_monotonic'], reference['identity_monotonic'])
    crossed = cross_after_real_context(c, monkeypatch, fault)
    entered, done, results, worker = start_receipt(n, plan, ticket, monkeypatch)
    try:
        assert entered.wait(2), results
        assert crossed == [True]
        assert not done.is_set()
        assert n._station_compute_pending and n._station_compute_soft_wait
        assert n._station_compute_reference is reference
        assert reference['contract'] == old
        assert reference['identity_monotonic'] == deadlines[1]
        assert reference['geometry_monotonic'] == deadlines[0]
        # These real callbacks acquire the original stream/geometry locks.
        successor = next_row(c, old['source_ns'] + 400_000_000, old)
        assert done.wait(3), results
        worker.join(3)
        assert len(results) == 1 and isinstance(results[0], dict), results
        receipt = results[0]
        assert validate_receipt(receipt, plan, now_ns=n._ros_source_clock_ns()) == c.binding
        assert receipt['current']['contract'] == successor['status']['station_compute_wait']
        assert receipt['issued_ns'] < receipt['current']['contract']['geometry_deadline_ns']
        assert [x['contract']['source_ns'] for x in receipt['sources']] == [t, t+400_000_000, t+800_000_000]
        assert plan == original_plan
        assert receipt['plan_source_ns'] == original_plan.header.stamp.to_nsec()
        assert receipt['current']['contract']['phase_deadline_ns'] == old['phase_deadline_ns']
        assert receipt['current']['contract']['session_deadline_ns'] == old['session_deadline_ns']
        assert not receipt['motion_authorized']
        assert not n._station_compute_work_published
        assert not n._station_compute_publication_started
    finally:
        finish(n, worker, done)


@pytest.mark.parametrize('fault', ['identity_ros', 'identity_mono', 'clock_back'])
def test_context_crossing_identity_or_backward_clock_never_becomes_geometry_wait(station, monkeypatch, fault):
    c = station
    n = c.node
    _, _, ticket, _, plan = published_contact(c)
    crossed = cross_after_real_context(c, monkeypatch, fault)
    with pytest.raises(ValueError, match='station_compute_wait_'):
        n._station_contact_computation_receipt(plan, ticket)
    assert crossed == [True]
    assert not n._station_compute_work_published
    assert not n._station_compute_publication_started


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'phase', 'identity', 'historical_source'])
def test_crossed_window_wait_does_not_mask_real_owner_or_historical_fault(station, monkeypatch, fault):
    c = station
    n = c.node
    _, row, ticket, _, plan = published_contact(c)
    cross_after_real_context(c, monkeypatch, 'geometry_ros')
    entered, done, results, worker = start_receipt(n, plan, ticket, monkeypatch)
    try:
        assert entered.wait(2), results
        if fault == 'manual':
            c.params['/gui/joint_direct_mode'] = True
        elif fault == 'epoch':
            n._measurement_remote_epoch_ns += 1
        elif fault == 'phase':
            n._station_compute_task_state = 'MOVE_PREGRASP'
        elif fault == 'identity':
            c.clock[0] = n._station_compute_reference['contract']['identity_deadline_ns'] / 1e9
        else:
            changed = deepcopy(row['object'])
            changed.pose_base.pose.position.x += .01
            n.object_cb(changed)
        with n._stream_condition:
            n._stream_condition.notify_all()
        assert done.wait(3), results
        worker.join(3)
        assert len(results) == 1 and isinstance(results[0], base.StreamResultCancelled), results
        assert not n._station_compute_work_published
        assert not n._station_compute_publication_started
    finally:
        finish(n, worker, done)


def test_original_preserve_identity_generation_change_cancels_receipt_wait(station, monkeypatch):
    c = station
    n = c.node
    _, _, ticket, _, plan = published_contact(c)
    cross_after_real_context(c, monkeypatch, 'geometry_ros')
    entered, done, results, worker = start_receipt(n, plan, ticket, monkeypatch)
    try:
        assert entered.wait(2), results
        reference = n._station_compute_reference
        identity = n._current_stream_target_identity()
        session = deepcopy(n._observe_return_session)
        generation = n._stream_generation
        assert n._station_compute_scope()
        assert n._advance_target_instance_epoch('MODEL_RELOADED', preserve_identity=True)
        assert n._stream_generation != generation
        assert n._current_stream_target_identity() == identity
        assert n._station_compute_reference is reference
        assert n._observe_return_session == session
        assert n._station_compute_scope()
        assert done.wait(3), results
        worker.join(3)
        assert len(results) == 1 and isinstance(results[0], base.StreamResultCancelled), results
        # ModeAware checks the station certificate before the base generation
        # code. The changed generation invalidates private work, so its
        # original cancellation precedence is processing-pending cancellation.
        assert results[0].code == 'TARGET_PROCESSING_PENDING'
        assert not n._station_compute_work_certificate(ticket)
        assert not n._station_compute_work_published
        assert not n._station_compute_publication_started
    finally:
        finish(n, worker, done)
