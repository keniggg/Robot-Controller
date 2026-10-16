"""Deterministic elapsed-time tests of the actual task ACK and callback path."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_observe_return_task import context, begin, at_ns, task, module
from alicia_grasp_modes.observe_return_session import parse_session


@pytest.fixture(autouse=True)
def zero_wall(monkeypatch):
    monkeypatch.setattr(task.time, 'monotonic', lambda: 0.)


def transition(c, kind):
    begin(c)
    c.now[0] += 1_000_000_000
    old = c.node._observe_return_session
    if kind == 'initial':
        request = deepcopy(old)
    elif kind == 'supplement':
        request = dict(old, stamp_ns=str(c.now[0]), view_index=2,
            view_plan_id='next-view', view_source_stamp_ns=str(c.now[0]-1),
            view_request_id='next-request', previous_gain_sha256='a'*64)
    elif kind == 'returning':
        request = dict(old, phase='returning', stamp_ns=str(c.now[0]))
    else:
        old = dict(old, phase='returning', stamp_ns=str(c.now[0]-1_000_000))
        c.node._observe_return_session = old
        request = dict(old, phase='returned', stamp_ns=str(c.now[0]),
            return_verified_stamp_ns=str(c.now[0]-1),
            contact_deadline_ns=str(c.now[0]+20_000_000_000))
    normalized = parse_session(request, c.selection, c.now[0], old if old != request else None)
    c.node._observe_return_acks.clear()
    return request, dict(normalized, accepted=True, reason='OK')


class WaitingTransport:
    def __init__(self, c, monkeypatch, request, ack, *, after=5.97, frozen_ros=False, mutate=None):
        self.elapsed = 0.
        self.c = c
        self.origin = c.now[0]
        self.ack, self.after = ack, after
        self.warns, self.sent = [], []
        self.done = False
        self.mutate = mutate
        self.frozen_ros = frozen_ros
        monkeypatch.setattr(task.time, 'monotonic', lambda: self.elapsed)
        monkeypatch.setattr(module.rospy, 'logwarn', lambda *a, **kw: self.warns.append(a))
        monkeypatch.setattr(c.node._observe_return_condition, 'wait', self.wait)
        c.node._observe_return_pub = NS(publish=lambda message: self.sent.append(json.loads(message.data)))

    def wait(self, seconds):
        self.elapsed = round(self.elapsed + seconds, 9)
        if not self.frozen_ros:
            self.c.now[0] = self.origin + round(self.elapsed*1e9)
        if self.mutate is not None and self.elapsed >= 1.:
            action, self.mutate = self.mutate, None
            action()
        if not self.done and self.after is not None and self.elapsed >= self.after:
            self.done = True
            self.c.node._observe_return_ack_cb(NS(data=json.dumps(self.ack)))


def phase(c, remaining=12.):
    p = NS(active=True, planning_stage='contact_replan', phase_id=7,
        header=NS(frame_id='base_link', stamp=at_ns(c.now[0]-1)),
        deadline=at_ns(c.now[0]+round(remaining*1e9)))
    c.node._published_near_field_phase = p
    c.node._near_field_active = True
    c.node._near_field_phase_id = p.phase_id
    c.node._near_field_phase_started_ns = p.header.stamp.to_nsec()
    c.node._near_field_phase_deadline_ns = p.deadline.to_nsec()
    return p


@pytest.mark.parametrize('kind', ['initial', 'supplement', 'returning', 'returned'])
@pytest.mark.parametrize('frozen_ros', [False, True])
def test_597_second_real_ack_succeeds_once_warned_without_renewal_or_motion(context, monkeypatch, kind, frozen_ros):
    c = context
    request, ack = transition(c, kind)
    if kind != 'initial':
        phase(c)
    old_session = c.node._observe_return_session
    old_value = deepcopy(old_session)
    prior_gateways = list(c.gateways)
    transport = WaitingTransport(c, monkeypatch, request, ack, frozen_ros=frozen_ros)
    accepted = c.node._observe_return_publish(request)
    assert accepted == ack
    assert 5.97 <= transport.elapsed < 6.03
    assert len(transport.warns) == 1
    assert transport.sent == [request]
    assert c.node._observe_return_session is old_session and old_session == old_value
    assert c.gateways == prior_gateways and not c.replans


@pytest.mark.parametrize('field', ['selection', 'target_track_id', 'target_epoch',
    'deadline_ns', 'view_request_id', 'previous_gain_sha256', 'view_plan_id'])
def test_same_key_ack_with_wrong_binding_is_terminal(context, monkeypatch, field):
    c = context
    request, ack = transition(c, 'supplement')
    if field == 'selection':
        ack[field]['generation'] += 1
    elif isinstance(ack[field], int):
        ack[field] += 1
    else:
        ack[field] = 'foreign'
    transport = WaitingTransport(c, monkeypatch, request, ack, after=1.)
    with pytest.raises(ValueError, match='ACK_BINDING_INVALID'):
        c.node._observe_return_publish(request)
    assert transport.elapsed < 1.06


def test_explicit_rejection_is_terminal_even_after_warning(context, monkeypatch):
    c = context
    request, ack = transition(c, 'supplement')
    ack.update(accepted=False, reason='actual target changed')
    transport = WaitingTransport(c, monkeypatch, request, ack)
    with pytest.raises(ValueError, match='REMOTE_REJECTED: actual target changed'):
        c.node._observe_return_publish(request)
    assert len(transport.warns) == 1


@pytest.mark.parametrize('fault', ['manual', 'stop', 'shutdown', 'selection', 'bound_selection',
    'session_replaced', 'session_mutated', 'request_mutated', 'epoch', 'revoked', 'ros_rewind'])
def test_waiting_stops_on_real_owner_or_clock_change(context, monkeypatch, fault):
    c = context
    request, ack = transition(c, 'supplement')
    c.node._following_epoch_ns = 10
    def mutate():
        if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
        elif fault == 'stop': c.node.active = False
        elif fault == 'shutdown': monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: True)
        elif fault == 'selection': c.params[module.SELECTION_PARAM]['generation'] += 1
        elif fault == 'bound_selection': c.node._mode_bound_selection['generation'] += 1
        elif fault == 'session_replaced': c.node._observe_return_session = deepcopy(c.node._observe_return_session)
        elif fault == 'session_mutated': c.node._observe_return_session['view_plan_id'] = 'other'
        elif fault == 'request_mutated': request['view_request_id'] = 'other'
        elif fault == 'epoch': c.node._following_epoch_ns += 1
        elif fault == 'revoked': c.node._execution_authority_revoked = True
        else: c.now[0] -= 1_000_000_000
    transport = WaitingTransport(c, monkeypatch, request, ack, mutate=mutate)
    with pytest.raises(ValueError, match='OWNER_CHANGED|REVOKED|CLOCK_REWOUND'):
        c.node._observe_return_publish(request)
    assert transport.elapsed < 1.06


@pytest.mark.parametrize('kind', ['initial', 'supplement', 'returned'])
@pytest.mark.parametrize('frozen_ros', [False, True])
def test_no_ack_is_bounded_by_original_contract_even_with_frozen_ros(context, monkeypatch, kind, frozen_ros):
    c = context
    request, ack = transition(c, kind)
    remaining = (int(request['deadline_ns'])-c.now[0])/1e9
    if kind == 'supplement':
        phase(c, 6.25)
        remaining = 6.25
    elif kind == 'returned':
        remaining = 20.
    transport = WaitingTransport(c, monkeypatch, request, ack, after=None, frozen_ros=frozen_ros)
    with pytest.raises(ValueError, match='ACK_TIMEOUT|SESSION_EXPIRED'):
        c.node._observe_return_publish(request)
    assert remaining <= transport.elapsed <= remaining+.051
    assert len(transport.warns) == 1


@pytest.mark.parametrize('key', ['session_id', 'phase', 'stamp_ns'])
def test_stale_or_foreign_ack_does_not_extend_wait_or_become_accepted(context, monkeypatch, key):
    c = context
    request, ack = transition(c, 'supplement')
    phase(c, 7.)
    ack[key] = ack[key]-1 if key == 'stamp_ns' else 'foreign'
    transport = WaitingTransport(c, monkeypatch, request, ack)
    with pytest.raises(ValueError, match='ACK_TIMEOUT'):
        c.node._observe_return_publish(request)
    assert 7. <= transport.elapsed < 7.06 and len(transport.warns) == 1


@pytest.mark.parametrize('fault', ['inactive', 'phase_id', 'start_ns', 'deadline_ns', 'reference'])
def test_original_active_phase_must_remain_exact(context, monkeypatch, fault):
    c = context
    request, ack = transition(c, 'supplement')
    p = phase(c)
    def mutate():
        if fault == 'inactive': c.node._near_field_active = False
        elif fault == 'phase_id': p.phase_id += 1
        elif fault == 'start_ns': p.header.stamp = at_ns(p.header.stamp.to_nsec()+1)
        elif fault == 'deadline_ns': p.deadline = at_ns(p.deadline.to_nsec()+1)
        else: p.header.frame_id = 'different'
    transport = WaitingTransport(c, monkeypatch, request, ack, mutate=mutate)
    with pytest.raises(ValueError, match='CONTACT_PHASE_CHANGED'):
        c.node._observe_return_publish(request)
    assert transport.elapsed < 1.06


def test_original_qualified_wall_deadline_is_not_restarted(context, monkeypatch):
    c = context
    request, ack = transition(c, 'supplement')
    p = phase(c, 50.)
    c.node._qualified_station_phase = dict(phase=deepcopy(p), monotonic_deadline=6.25)
    transport = WaitingTransport(c, monkeypatch, request, ack, after=6.3, frozen_ros=True)
    with pytest.raises(ValueError, match='ACK_TIMEOUT'):
        c.node._observe_return_publish(request)
    assert 6.25 <= transport.elapsed < 6.3


def test_initial_observation_ignores_unrelated_old_phase_and_retired_handoff(context, monkeypatch):
    c = context
    request, ack = transition(c, 'initial')
    phase(c, -1.)
    c.node._qualified_station_phase = dict(phase=object(), monotonic_deadline=-1.)
    c.node._station_task_handoff = dict(cutoff_ns=1, next_session={}, phase_mono=-1.)
    transport = WaitingTransport(c, monkeypatch, request, ack)
    assert c.node._observe_return_publish(request) == ack
    assert len(transport.warns) == 1


def test_frozen_ros_cannot_renew_session_wall_cap_at_next_transition(context, monkeypatch):
    c = context
    request, ack = transition(c, 'initial')
    transport = WaitingTransport(c, monkeypatch, request, ack, frozen_ros=True)
    assert c.node._observe_return_publish(request) == ack
    clock = c.node._observe_return_session_clock
    old = c.node._observe_return_session
    request = dict(old, stamp_ns=str(c.now[0]), phase='returning')
    ack = dict(parse_session(request, c.selection, c.now[0], old), accepted=True, reason='OK')
    transport.ack, transport.after, transport.done = ack, None, False
    with pytest.raises(ValueError, match='ACK_TIMEOUT'):
        c.node._observe_return_publish(request)
    assert transport.elapsed == 180.
    assert c.node._observe_return_session_clock == clock
    assert transport.sent[-1] == request


from test_qualified_station_phase import qualified_context, finish


def test_real_qualified_setter_and_ensure_owner_accept_delayed_supplement(qualified_context, monkeypatch):
    c = qualified_context
    finish(c)
    original_phase = deepcopy(c.node._ensure_qualified_station_phase(c.plan, c.cfg))
    original_qualified = c.node._qualified_station_phase
    c.now[0] += 1_000_000
    old = c.node._observe_return_session
    request = dict(old, stamp_ns=str(c.now[0]), view_index=2, view_plan_id='next',
        view_source_stamp_ns=str(c.now[0]-1), view_request_id='next', previous_gain_sha256='a'*64)
    ack = dict(parse_session(request, c.selection, c.now[0], old), accepted=True, reason='OK')
    transport = WaitingTransport(c, monkeypatch, request, ack)
    assert c.node._observe_return_publish(request) == ack
    assert len(transport.warns) == 1
    assert c.node._qualified_station_phase is original_qualified
    assert c.node._published_near_field_phase == original_phase
    assert c.node._ensure_qualified_station_phase(c.plan, c.cfg) == original_phase
    assert len(c.phases) == 1


from test_gen10_handoff_authority_independent import ready, station, configured, feedback


@pytest.mark.parametrize('fault', [None, 'camera', 'feedback_stale', 'feedback_motion'])
def test_real_station_wrapper_and_live_owner_continue_after_five_seconds(station, monkeypatch, fault):
    station.plan.header.stamp = at_ns(22_000_000_000)
    c, source = ready(station)
    old = c.node._observe_return_session
    old.update(stamp_ns=30_000_000_000, origin_plan_id='origin',
        origin_snapshot_stamp_ns=21_000_000_000, return_verified_stamp_ns=0,
        max_observations=3, view_source_stamp_ns=22_000_000_000,
        view_request_id='view-2', previous_gain_sha256='a'*64)
    request = dict(old, stamp_ns=c.now[0], view_index=3, view_plan_id='next',
        view_source_stamp_ns=c.now[0]-1, view_request_id='view-3', previous_gain_sha256='b'*64)
    ack = dict(parse_session(request, c.selection, c.now[0], old), accepted=True, reason='OK')
    c.node._observe_return_acks.clear()
    c.node._observe_return_pub = NS(publish=lambda message: sent.append(json.loads(message.data)))
    start, wall = c.now[0], c.mono[0]
    sent, warns = [], []
    monkeypatch.setattr(module.rospy, 'logwarn', lambda *args: warns.append(args))
    def wait(seconds):
        elapsed = round(c.mono[0] - wall + seconds, 9)
        c.mono[0] = wall + elapsed
        c.now[0] = start + round(elapsed*1e9)
        if fault == 'camera' and elapsed >= 1.:
            c.params['/camera']['fx'] += 1
        if fault != 'feedback_stale':
            feedback(c, c.now[0], .01 if fault == 'feedback_motion' and elapsed >= 1. else 0.)
        if elapsed >= 5.97:
            c.node._observe_return_ack_cb(NS(data=json.dumps(ack)))
    monkeypatch.setattr(c.node._observe_return_condition, 'wait', wait)
    if fault is not None:
        expected = {'camera': 'station_task_camera_changed', 'feedback_stale': 'station_task_feedback_stale',
                    'feedback_motion': 'station_task_pose_changed'}[fault]
        with pytest.raises(ValueError, match=expected):
            c.node._observe_return_publish(request)
        assert c.node._station_task_handoff is None
        assert c.mono[0] - wall < 1.1
    else:
        assert c.node._observe_return_publish(request) == ack
        assert 5.97 <= c.mono[0] - wall < 6.03 and len(warns) == 1
        handoff = c.node._station_task_handoff
        assert c.node._station_handoff_live() is handoff
        assert handoff['next_session'] == request and handoff['old_session'] == old
        assert handoff['phase_deadline_ns'] == source['status']['station_compute_wait']['phase_deadline_ns']
        assert handoff['cutoff_ns'] is None and not handoff['rebound']
        assert c.node._station_task_record is None and not c.node._execution_authority_revoked
        # ACK completes only metadata transport. Old source and plan have
        # not been refreshed, nor has any action/rebind been called here.
        assert c.node._bound_execution_plan.plan_id == c.plan.plan_id
        assert c.node._observe_return_session == old
    assert sent == [request]
