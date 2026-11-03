"""Reached callback/MRO handoff only; raw qualification is a separate test.

The existing session and registration fixtures run unchanged. Explicit inert
context tokens stand in for the new producer context so these tests prove its
delivery/commit ordering, not RGB-D/model qualification or grasp feasibility.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from test_observe_return_supplement_remote import v2_session
from test_observe_return_remote import configured, base, module, direct_observation


def reached(configured, monkeypatch):
    node, params, clock, session = v2_session(configured, monkeypatch)
    node._observe_return_view_uncertainty_m = .001
    observation = direct_observation(node, 19_000_000_000)
    monkeypatch.setattr(node, '_resolve_reached_view_observation', lambda message: observation)
    phase = NS(active=True, phase_id=1,
        header=NS(stamp=module.rospy.Time.from_sec(20.), frame_id='base_link'),
        deadline=module.rospy.Time.from_sec(70.))
    clock[0] = 21_000_000_000
    return NS(node=node, params=params, clock=clock, session=session,
        observation=observation, phase=phase, before=node._observe_return_saved[0],
        generation=node._stream_generation)


def locks_owned(node):
    return [node._mode_work_lock._is_owned(), node._mode_phase_lock._is_owned(),
            node._stream_condition._is_owned(), node._geometry_state_lock._is_owned()]


def context_hooks(c, monkeypatch, *, prepare_change=None, current=True):
    tokens = dict(raw_geometry=object(), raw_box_model=object())
    context = dict(inputs=tokens, owner='explicit inert handoff fixture')
    events = []
    def prepare(observation):
        assert observation is c.observation
        assert not any(locks_owned(c.node))
        events.append('prepare')
        if prepare_change is not None:
            prepare_change()
        return context
    def validate(observation, captured):
        assert observation is c.observation and captured is context
        assert all(locks_owned(c.node))
        assert c.node._observe_return_phase == 'observing'
        events.append('current')
        return current() if callable(current) else current
    monkeypatch.setattr(c.node, '_prepare_reached_box_registration', prepare, raising=False)
    monkeypatch.setattr(c.node, '_reached_box_registration_current', validate, raising=False)
    return context, tokens.copy(), events


def registration_spy(c, monkeypatch):
    original = base.RemoteGrasp6DNode._register_near_field_surface
    calls = []
    def delivered(node, observation, **kwargs):
        assert node is c.node and observation is c.observation
        assert all(locks_owned(node))
        assert node._observe_return_phase == 'observing_contact'
        calls.append(kwargs.copy())
        # Context qualification/physical policy is deliberately outside this
        # handoff unit. Keep the original fixture's real measured registration.
        return original(node, observation)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_register_near_field_surface', delivered)
    return calls


def assert_not_committed(c, calls):
    assert not calls
    assert c.node._stream_generation == c.generation
    assert c.node._observe_return_saved[0] is c.before
    assert c.node._observe_return_phase == 'observing'
    assert c.node._observe_return_reached_phase_id != c.phase.phase_id


def test_reached_context_is_prepared_outside_locks_then_delivered_at_original_registration_commit(configured, monkeypatch):
    c = reached(configured, monkeypatch)
    context, tokens, events = context_hooks(c, monkeypatch)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert events == ['prepare', 'current']
    assert calls == [tokens]
    assert calls[0]['raw_geometry'] is context['inputs']['raw_geometry']
    assert calls[0]['raw_box_model'] is context['inputs']['raw_box_model']
    assert c.node._latest_registration_evidence.result.ok
    assert c.node._latest_registration_evidence.stamp_ns == c.observation.stamp_ns
    assert c.node._observe_return_saved[0].view_stamps_ns == c.before.view_stamps_ns+(c.observation.stamp_ns,)
    assert not c.node._observe_return_paused
    assert c.phase.deadline.to_nsec() == 70_000_000_000


def test_missing_raw_context_uses_legacy_single_argument_registration(configured, monkeypatch):
    c = reached(configured, monkeypatch)
    monkeypatch.setattr(c.node, '_prepare_reached_box_registration', lambda observation: None, raising=False)
    def forbidden(*args):
        raise AssertionError('missing context must not be treated as qualified metadata')
    monkeypatch.setattr(c.node, '_reached_box_registration_current', forbidden, raising=False)
    original = base.RemoteGrasp6DNode._register_near_field_surface
    calls = []
    # Deliberately no **kwargs: legacy subclass/test compatibility is real.
    def legacy(node, observation):
        calls.append(observation)
        return original(node, observation)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_register_near_field_surface', legacy)
    c.node.near_field_state_cb(c.phase)
    assert calls == [c.observation]
    assert c.node._latest_registration_evidence.result.ok
    assert not c.node._observe_return_paused


def test_invalid_context_cannot_advance_phase_or_register(configured, monkeypatch):
    c = reached(configured, monkeypatch)
    c.node._observe_return_paused = False
    _context, _tokens, events = context_hooks(c, monkeypatch, current=False)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert events == ['prepare', 'current']
    assert_not_committed(c, calls)
    assert c.node._observe_return_paused is False


@pytest.mark.parametrize('change', ['selection', 'session', 'target'])
def test_owner_change_during_unlocked_qualification_does_not_commit(configured, monkeypatch, change):
    c = reached(configured, monkeypatch)
    def revoke():
        c.node._observe_return_paused = False
        if change == 'selection':
            c.params[module.SELECTION_PARAM]['generation'] += 1
        elif change == 'session':
            c.node._observe_return_session = deepcopy(c.node._observe_return_session)
        else:
            identity = c.observation.identity
            foreign = type(identity)(identity.epoch+1, identity.track_id+'-changed')
            monkeypatch.setattr(c.node, '_current_stream_target_identity', lambda: foreign)
    context_hooks(c, monkeypatch, prepare_change=revoke)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert_not_committed(c, calls)
    assert c.node._observe_return_paused is False


@pytest.mark.parametrize('change', ['phase_deadline', 'session_deadline', 'clock_backwards', 'after_context_check'])
def test_added_qualification_time_cannot_commit_past_original_deadline(configured, monkeypatch, change):
    c = reached(configured, monkeypatch)
    c.node._observe_return_paused = False
    def expired():
        if change == 'phase_deadline': c.clock[0] = c.phase.deadline.to_nsec()
        elif change == 'session_deadline': c.node._observe_return_session['deadline_ns'] = c.clock[0]
        elif change == 'clock_backwards': c.clock[0] = c.phase.header.stamp.to_nsec()-1
    def current():
        if change == 'after_context_check': c.clock[0] = c.phase.deadline.to_nsec()
        return True
    context_hooks(c, monkeypatch, prepare_change=expired, current=current)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert_not_committed(c, calls)
    assert c.node._observe_return_paused is False
    assert c.phase.deadline.to_nsec() == 70_000_000_000


@pytest.mark.parametrize('missing_context', [False, True])
@pytest.mark.parametrize('changed_owner', ['raw', 'observation'])
def test_owner_replaced_during_prepare_cannot_commit_even_when_override_returns_none(
        configured, monkeypatch, missing_context, changed_owner):
    c = reached(configured, monkeypatch)
    def replace_owner():
        c.node._observe_return_paused = False
        if changed_owner == 'raw':
            c.node._latest_raw_geometry_source = {'owner': 'new source during qualification'}
        else:
            c.node._latest_target_observation = direct_observation(c.node, c.observation.stamp_ns)
    if missing_context:
        def prepare(observation):
            assert observation is c.observation and not any(locks_owned(c.node))
            replace_owner()
            return None
        monkeypatch.setattr(c.node, '_prepare_reached_box_registration', prepare)
        monkeypatch.setattr(c.node, '_reached_box_registration_current',
            lambda *args: pytest.fail('None override has no qualified context'))
    else:
        _context, _tokens, events = context_hooks(c, monkeypatch, prepare_change=replace_owner)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert_not_committed(c, calls)
    assert c.node._observe_return_paused is False
    if not missing_context:
        assert events == ['prepare']


@pytest.mark.parametrize('missing_context', [False, True])
def test_lifecycle_changed_during_prepare_keeps_new_owner_pause_state(
        configured, monkeypatch, missing_context):
    c = reached(configured, monkeypatch)
    def revoke():
        c.node._stream_generation += 1
        c.node._observe_return_paused = False
    if missing_context:
        def prepare(observation):
            revoke()
            return None
        monkeypatch.setattr(c.node, '_prepare_reached_box_registration', prepare)
    else:
        _context, _tokens, events = context_hooks(c, monkeypatch, prepare_change=revoke)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert not calls
    assert c.node._stream_generation == c.generation+1
    assert c.node._observe_return_saved[0] is c.before
    assert c.node._observe_return_phase == 'observing'
    assert c.node._observe_return_reached_phase_id != c.phase.phase_id
    assert c.node._observe_return_paused is False
    if not missing_context:
        assert events == ['prepare']


@pytest.mark.parametrize('has_observation', [False, True])
@pytest.mark.parametrize('changed_owner', ['lifecycle', 'session'])
def test_resolver_owner_change_cannot_prepare_commit_or_queue_old_retry(
        configured, monkeypatch, has_observation, changed_owner):
    c = reached(configured, monkeypatch)
    queued, prepared = [], []
    def resolve(message):
        assert message is c.phase and not any(locks_owned(c.node))
        if changed_owner == 'lifecycle':
            c.node._stream_generation += 1
        else:
            c.node._observe_return_session = deepcopy(c.node._observe_return_session)
        c.node._observe_return_paused = False
        return c.observation if has_observation else None
    monkeypatch.setattr(c.node, '_resolve_reached_view_observation', resolve)
    monkeypatch.setattr(c.node, '_prepare_reached_box_registration',
        lambda observation: prepared.append(observation))
    monkeypatch.setattr(c.node, '_observe_return_queue_reached_retry',
        lambda *args: queued.append(args))
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert not calls and not prepared and not queued
    assert c.node._stream_generation == c.generation+(changed_owner == 'lifecycle')
    assert c.node._observe_return_saved[0] is c.before
    assert c.node._observe_return_phase == 'observing'
    assert c.node._observe_return_reached_phase_id != c.phase.phase_id
    assert c.node._observe_return_reached_retry is None
    assert c.node._observe_return_paused is False


def test_duplicate_reached_phase_does_not_repeat_qualification_or_append(configured, monkeypatch):
    c = reached(configured, monkeypatch)
    _context, _tokens, events = context_hooks(c, monkeypatch)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    generation = c.node._stream_generation
    after = c.node._observe_return_saved[0]
    c.node.near_field_state_cb(c.phase)
    assert len(calls) == 1 and events == ['prepare', 'current']
    assert c.node._stream_generation == generation
    assert c.node._observe_return_saved[0] is after


def test_delivered_inputs_are_owned_before_intended_phase_advance(configured, monkeypatch):
    c = reached(configured, monkeypatch)
    context, tokens, _events = context_hooks(c, monkeypatch)
    original = c.node._observe_return_advance
    def advance(*args, **kwargs):
        context['inputs']['raw_box_model'] = 'later mutable context'
        return original(*args, **kwargs)
    monkeypatch.setattr(c.node, '_observe_return_advance', advance)
    calls = registration_spy(c, monkeypatch)
    c.node.near_field_state_cb(c.phase)
    assert calls == [tokens]
    assert calls[0]['raw_box_model'] != context['inputs']['raw_box_model']
