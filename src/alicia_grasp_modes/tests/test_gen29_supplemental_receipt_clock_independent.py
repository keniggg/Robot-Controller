"""Actual remote MRO: elapsed context work must wait for a real new RGB-D join."""
from copy import deepcopy
import inspect
import threading

import pytest

from test_observation_measurement_remote import context
from test_station_compute_wait_remote import station, clock, feedback, row_at
from test_observation_station_snapshot_scope import submitted_observation
from test_station_compute_work_handoff import prepared_for, rgbd
from test_gen11_contact_remote_receipt import contact_plan
from test_remote_modes import base
from test_unknown_pending_identity import deliver
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id


@pytest.mark.parametrize('outcome', ['fresh', 'manual', 'phase_deadline', 'no_new_source'])
def test_context_elapsed_across_deadline_waits_then_checks_actual_owner(station, monkeypatch, outcome):
    c = station
    n = c.node
    _, row, ticket = submitted_observation(c)
    assert n._activate_prepared_geometry(prepared_for(n, ticket))
    plan = contact_plan(n, ticket)
    plan.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    plan.fused_view_count = 5
    plan.plan_id = compute_plan_id(plan)
    original_plan = deepcopy(plan)
    legacy = base.PoseArray()
    legacy.header = deepcopy(plan.header)
    reference = n._station_compute_reference
    original_contract = deepcopy(reference['contract'])
    original_deadlines = (reference['geometry_monotonic'], reference['identity_monotonic'])
    crossed = threading.Event()
    pending = threading.Event()
    done = threading.Event()
    errors = []
    original_context = n._station_compute_context
    original_quarantine = n._station_compute_quarantine

    def elapsed_context(contract, *args, **kwargs):
        result = original_context(contract, *args, **kwargs)
        caller = inspect.currentframe().f_back.f_code.co_name
        if not crossed.is_set() and caller in (
                '_station_supplement_computation_receipt', '_publish_preview_plan', 'straightline'):
            # Inject elapsed validation work only after the real context check.
            # Every source/owner/identity/plan check remains active.
            clock(c, contract['geometry_deadline_ns'] + 8_400_000)
            crossed.set()
        return result

    def quarantine(*args, **kwargs):
        result = original_quarantine(*args, **kwargs)
        if crossed.is_set() and n._station_compute_pending:
            pending.set()
        return result

    monkeypatch.setattr(n, '_station_compute_context', elapsed_context)
    monkeypatch.setattr(n, '_station_compute_quarantine', quarantine)

    def run():
        try:
            n._publish_preview_plan(plan, legacy, ticket=ticket)
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()

    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    try:
        assert pending.wait(2), [repr(e) for e in errors]
        assert crossed.is_set() and not done.wait(.025), [repr(e) for e in errors]
        with n._stream_condition:
            assert n._station_compute_soft_wait
            assert reference['contract'] == original_contract
            assert (reference['geometry_monotonic'], reference['identity_monotonic']) == original_deadlines
            assert not any(m.valid and m.plan_id == plan.plan_id for m in n.preview_rich_plan_pub.messages)
            assert not n._station_compute_publication_started
            assert not n._station_compute_work_published
        if outcome == 'fresh':
            stamp = original_contract['source_ns'] + 900_000_000
            feedback(c, stamp - 10_000_000)
            feedback(c, stamp + 10_000_000)
            clock(c, stamp + 100_000_000)
            successor = row_at(c, stamp, recovery=original_contract)
            rgbd(c, successor)
            deliver(n, successor)
        elif outcome == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif outcome == 'phase_deadline': clock(c, original_contract['phase_deadline_ns'])
        else:
            # The shared fixture stores seconds as float; +1 ms avoids a
            # conversion landing 1 ns before the intended expired boundary.
            clock(c, original_contract['identity_deadline_ns'] + 1_000_000)
            assert n._ros_source_clock_ns() >= original_contract['identity_deadline_ns'], (
                n._ros_source_clock_ns(), original_contract['identity_deadline_ns'], c.clock[0])
        with n._stream_condition:
            n._stream_condition.notify_all()
        assert done.wait(2), [repr(e) for e in errors]
        worker.join(timeout=1)
        publications = [m for m in n.preview_rich_plan_pub.messages if m.valid and m.plan_id == plan.plan_id]
        if outcome == 'fresh':
            assert errors == [] and publications == [original_plan]
            held = n._station_supplement_proposal
            assert held['published'] and held['plan'] == original_plan
            receipt = held['receipt']
            assert receipt['current']['contract']['source_ns'] == stamp
            assert receipt['plan_source_ns'] == original_plan.header.stamp.to_nsec()
            assert len(receipt['sources']) == 5
            assert receipt['current']['contract']['phase_deadline_ns'] == original_contract['phase_deadline_ns']
            assert receipt['current']['contract']['session_deadline_ns'] == original_contract['session_deadline_ns']
        else:
            assert errors and publications == []
            assert n._station_supplement_proposal is None
        assert plan == original_plan
        assert not n._station_compute_publication_started and not n._station_compute_work_published
    finally:
        if not done.is_set():
            c.params['/gui/joint_direct_mode'] = True
            with n._stream_condition:
                n._stream_condition.notify_all()
            done.wait(2)
            worker.join(timeout=2)
