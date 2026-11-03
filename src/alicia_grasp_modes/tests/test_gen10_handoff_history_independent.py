"""Finite cross-view callback history through the actual task MRO.

Only session transport and the already-covered dual-ACK transport are inert.
These tests assert callback ownership/lifetime, not physical admission.
"""
from copy import deepcopy

import pytest

from alicia_grasp_modes.station_compute_reacquisition import build_contract
from test_gen10_handoff_authority_independent import (
    ready, actual_publish_owner, pending_messages, receive,
    station, configured, frame, deliver, advance, feedback, base, module, at_ns)


def register_station(c, view_index, source_ns):
    n = c.node
    n._station_task_disarm()
    n._observe_return_session['view_index'] = view_index
    n._observation_measurement_request = dict(n._observation_measurement_request,
        state='qualified', view_index=view_index, view_plan_id=c.plan.plan_id,
        lease_id='qualified-view-%d' % view_index,
        source_stamps_ns=[source_ns-600_000_000, source_ns-500_000_000, source_ns-400_000_000])
    n._published_near_field_phase.phase_id = view_index + 10
    n._published_near_field_phase.header.stamp = at_ns(source_ns-200_000_000)
    n._published_near_field_phase.reference_source_stamp = at_ns(source_ns-300_000_000)
    n._near_field_phase_id = view_index + 10
    advance(c, source_ns+50_000_000)
    feedback(c, source_ns-20_000_000)
    feedback(c, source_ns+20_000_000)
    f = frame(c, source_ns)
    b = f['status']['station_compute_wait']['binding']
    b.update(view_index=view_index, lease_id=n._observation_measurement_request['lease_id'],
             phase_id=view_index+10, phase_started_ns=source_ns-200_000_000,
             phase_reference_source_ns=source_ns-300_000_000)
    f['status']['station_compute_wait'] = build_contract(source_ns=source_ns,
        maximum_frame_age_sec=.8, session_deadline_ns=200_000_000_000,
        phase_deadline_ns=190_000_000_000, binding=b)
    deliver(c, f)
    assert n._station_task_record['contract'] == f['status']['station_compute_wait']
    n._station_task_context(n._station_task_record['contract'], admission=True)
    return f


def start_handoff(c, monkeypatch):
    owner = actual_publish_owner(c)
    monkeypatch.setattr(owner, '_observe_return_publish', lambda self, message: None)
    nxt_plan = deepcopy(c.plan)
    nxt_plan.header.stamp = at_ns(c.now[0])
    nxt_plan.object_geometry.header = deepcopy(nxt_plan.header)
    nxt_plan.plan_id = base.compute_plan_id(nxt_plan)
    nxt = dict(c.node._observe_return_session,
        view_index=c.node._observe_return_session['view_index']+1,
        view_plan_id=nxt_plan.plan_id)
    c.node._observe_return_publish(nxt)
    return nxt, nxt_plan


def close_handoff(c, nxt, nxt_plan, monkeypatch):
    n = c.node
    def bind():
        frozen = n._freeze_execution_plan(nxt_plan)
        n._observe_return_session = deepcopy(nxt)
        return frozen
    validation, frozen = n._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', bind)
    assert validation.ok, (validation.code, validation.reason)
    c.plan = frozen
    # The real producer/remote dual-ACK protocol is covered independently;
    # isolate the task's post-ACK cutoff transition at this exact boundary.
    monkeypatch.setattr(actual_publish_owner(c), '_observation_measurement_prepare',
        lambda self, plan: {'independent_inert_ack_transport': True})
    n._observation_measurement_prepare(frozen)
    assert n._station_task_handoff['cutoff_ns'] == c.now[0]
    return n._station_task_handoff


@pytest.mark.parametrize('order', [('object', 'status', 'geometry'), ('status', 'geometry', 'object')])
def test_view_one_late_callbacks_survive_next_handoff_without_rearming_geometry(station, monkeypatch, order):
    c, _ = ready(station)
    f1 = register_station(c, 1, 100_000_000_000)
    stamp, messages = pending_messages(c, f1)
    nxt, candidate = start_handoff(c, monkeypatch)
    advance(c, stamp + 10_000_000)
    first = close_handoff(c, nxt, candidate, monkeypatch)
    f2 = register_station(c, 2, 101_100_000_000)
    before = deepcopy(c.node._station_task_record)
    next_session, next_plan = start_handoff(c, monkeypatch)
    assert first in c.node._station_task_handoff_history
    assert len(c.node._station_task_handoff_history) <= 2
    current = c.node._station_task_handoff
    deadlines = (first['session_deadline_ns'], first['session_mono'], first['cutoff_ns'])
    for name in order:
        receive(c, name, messages[name])
    assert not c.node._execution_authority_revoked
    assert c.node._station_task_record is None
    assert current['contract'] == before['contract']
    assert current['contract']['binding']['view_index'] == 2
    assert not first['unpaired_objects'] and not current['unpaired_objects']
    for name in order:
        receive(c, name, messages[name])
    assert (first['session_deadline_ns'], first['session_mono'], first['cutoff_ns']) == deadlines
    check, result = c.node._invoke_plan_bound_action(c.plan, {},
        'supplemental observation rebind', lambda: 'metadata-only')
    assert check.ok and result == 'metadata-only'


@pytest.mark.parametrize('fault', ['epoch', 'manual', 'session', 'monotonic', 'stop', 'terminal'])
def test_closed_history_never_masks_current_owner_or_terminal_failure(station, monkeypatch, fault):
    c, _ = ready(station)
    f = register_station(c, 1, 100_000_000_000)
    stamp, messages = pending_messages(c, f)
    nxt, candidate = start_handoff(c, monkeypatch)
    advance(c, stamp + 10_000_000)
    token = close_handoff(c, nxt, candidate, monkeypatch)
    if fault == 'epoch':
        c.node._following_epoch_ns += 1
    elif fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'session':
        c.node._observe_return_session['session_id'] = 'other-session'
    elif fault == 'monotonic':
        c.mono[0] = token['session_mono']
    elif fault == 'stop':
        c.node.active = False
    else:
        messages['status'].update(target_lost=True, first_loss={'reason': 'actual_failure'})
        receive(c, 'status', messages['status'])
    assert c.node._station_handoff_pending('geometry', messages['geometry']) is False
    if fault == 'terminal':
        assert c.node._execution_authority_revoked
