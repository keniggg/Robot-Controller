"""Diagnostic run identity follows real start/set_state, without action I/O."""
from copy import deepcopy
import json
from types import MethodType, SimpleNamespace as NS

import pytest

from test_task_modes import configured, module, base


@pytest.fixture
def events(configured, monkeypatch):
    node, params = configured
    now = [10_123_456_789]
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(
        lambda: module.rospy.Time(*divmod(now[0], 10**9))))
    monkeypatch.setattr(module.rospy, 'loginfo', lambda *a, **kw: None)
    states, mirrored, sent_objects = [], [], []
    node.pub = NS(publish=lambda msg: states.append(deepcopy(msg)))
    node.set_state = MethodType(base.GraspTaskNode.set_state, node)
    def publish(msg):
        sent_objects.append(node._task_event_run)
        mirrored.append(json.loads(msg.data))
    node._task_event_pub = NS(publish=publish)
    return NS(node=node, params=params, now=now, states=states, events=mirrored,
              run_at_publication=sent_objects, request=NS(execute=True, plan_id=''))


def assert_same_state(event, state):
    assert event['stamp_ns'] == state.header.stamp.to_nsec()
    assert event['frame_id'] == state.header.frame_id
    assert all(event[key] == getattr(state, key)
               for key in ('state', 'stage', 'active', 'success', 'message'))
    assert event['authority'] == 'diagnostic_only'
    assert event['execution_authorized'] is False


def test_real_start_states_and_inactive_release_keep_one_run_then_clear(events):
    c = events
    def execute(grasp6d_plan):
        assert c.node._mode_start_reserved and c.node.active
        c.node.set_state(base.GraspStages.PLAN_PREGRASP, 'current request planning')
        c.now[0] += 17
        c.node.set_state(base.GraspStages.FAILED, 'NO_SAFE_CONTACT', False)
        return False
    c.node.execute = execute
    result = c.node.start_cb(c.request)
    assert not result.success and result.message == 'NO_SAFE_CONTACT'
    assert len(c.events) == len(c.states) == 3
    assert [v['event_seq'] for v in c.events] == [1, 2, 3]
    assert len({v['task_run_id'] for v in c.events}) == 1
    assert c.events[-1]['active'] is False
    assert all(run is not None for run in c.run_at_publication)
    from alicia_grasp_modes.recorded_task_chain import normalize_event
    for event, state in zip(c.events, c.states):
        assert_same_state(event, state)
        parsed = normalize_event('/grasp/task_event', event, {'recorded_ns': c.now[0]})
        assert parsed['binding']['task_run_id'] == event['task_run_id']
    assert c.node._task_event_run is None and not c.node._mode_start_reserved
    assert c.node._mode_bound_selection is None


def test_same_selection_retry_is_unique_and_duplicate_active_start_is_not_a_run(events):
    c = events; executions = []; ids = []
    def execute(grasp6d_plan):
        run = c.node._task_event_run
        executions.append(run['task_run_id'])
        duplicate = c.node.start_cb(c.request)
        assert not duplicate.success and duplicate.message == 'already active'
        assert c.node._task_event_run is run
        c.node.set_state(base.GraspStages.FAILED, 'retryable failure')
        return False
    c.node.execute = execute
    for _ in range(2):
        c.node.start_cb(c.request)
        ids.append(c.events[-1]['task_run_id'])
    assert len(executions) == 2 and len(set(ids)) == 2
    assert [v['event_seq'] for v in c.events] == [1, 2, 1, 2]
    assert all(v['selection'] == c.node._mode_selection for v in c.events)


def test_direct_first_active_budget_stamp_is_the_exact_original_message(events):
    c = events; forced = 9_000_000_007
    def execute(grasp6d_plan):
        c.node._direct_contact_started_budget = NS(first_active_publication=True, start_stamp_ns=forced)
        c.node.set_state(base.GraspStages.PLAN_PREGRASP, 'first active')
        c.node.set_state(base.GraspStages.FAILED, 'stopped')
        return False
    c.node.execute = execute
    c.node.start_cb(c.request)
    assert c.events[0]['stamp_ns'] == c.states[0].header.stamp.to_nsec() == forced
    assert c.events[1]['stamp_ns'] == c.now[0]


@pytest.mark.parametrize('fault', ['publisher', 'hook', 'context_serialization'])
def test_diagnostic_failure_never_changes_original_state_or_start_result(events, fault):
    c = events
    def fail(*args, **kw): raise RuntimeError('diagnostic transport unavailable')
    if fault == 'publisher': c.node._task_event_pub.publish = fail
    elif fault == 'hook': c.node._task_state_published = fail
    def execute(grasp6d_plan):
        if fault == 'context_serialization': c.node._task_event_run['selection']['bad'] = object()
        c.node.set_state(base.GraspStages.FAILED, 'original failure')
        return False
    c.node.execute = execute
    result = c.node.start_cb(c.request)
    assert result.success is False and result.message == 'original failure'
    assert len(c.states) == 2 and c.states[-1].active is False
    assert c.node._task_event_run is None and not c.node.active


def test_owned_context_is_mirrored_and_foreign_latest_preview_is_never_a_fallback(events):
    c = events
    def execute(grasp6d_plan):
        n = c.node
        n.latest_grasp6d_preview_plan = NS(plan_id='FOREIGN', target_track_id='foreign',
            header=NS(stamp=module.rospy.Time(999)))
        n.set_state(base.GraspStages.PLAN_PREGRASP, 'no bound context')
        assert all(c.events[-1][k] is None for k in ('bound_plan_id', 'target_track_id',
            'source_stamp_ns', 'session_id', 'measurement_request_id', 'measurement_lease_id'))
        n._bound_execution_plan = NS(plan_id='owned-plan', target_track_id='owned-track',
            header=NS(stamp=module.rospy.Time(9, 7)))
        n._bound_execution_plan_id = 'owned-plan'
        n._observe_return_session = dict(selection=deepcopy(n._mode_bound_selection),
            session_id='owned-session', target_epoch=4, target_track_id='owned-track', view_index=1)
        n._observation_measurement_request = dict(selection=deepcopy(n._mode_bound_selection),
            session_id='owned-session', request_id='own-request', lease_id='own-lease')
        n._published_near_field_phase = NS(active=True, phase_id=2,
            reference_target_track_id='owned-track', planning_stage='contact_replan')
        n.set_state(base.GraspStages.PLAN_PREGRASP, 'bound contact')
        return False
    c.node.execute = execute
    c.node.start_cb(c.request)
    e = c.events[1]
    assert e['bound_plan_id'] == 'owned-plan' and e['source_stamp_ns'] == 9_000_000_007
    assert e['target_track_id'] == 'owned-track' and e['target_epoch'] == 4
    assert e['session_id'] == 'owned-session' and e['view_index'] == 1
    assert e['measurement_request_id'] == 'own-request' and e['measurement_lease_id'] == 'own-lease'
    assert e['phase_id'] == 2 and e['planning_stage'] == 'contact_replan'
    assert c.events[-1]['bound_plan_id'] is None  # Original cleanup really cleared it.


@pytest.mark.parametrize('fault', ['selection', 'session_selection', 'measurement_session', 'phase_target'])
def test_changed_context_cannot_borrow_foreign_identity(events, fault):
    c = events
    def execute(grasp6d_plan):
        n = c.node
        selected = deepcopy(n._mode_bound_selection)
        n._observe_return_session = dict(selection=selected, session_id='own-session',
            target_epoch=4, target_track_id='own-track', view_index=1)
        n._observation_measurement_request = dict(selection=deepcopy(selected),
            session_id='own-session', request_id='own-request', lease_id='own-lease')
        n._published_near_field_phase = NS(active=True, phase_id=2,
            reference_target_track_id='own-track', planning_stage='contact_replan')
        if fault == 'selection': n._mode_bound_selection = dict(selected, generation=99)
        elif fault == 'session_selection': n._observe_return_session['selection'] = dict(selected, generation=99)
        elif fault == 'measurement_session': n._observation_measurement_request['session_id'] = 'foreign'
        else: n._published_near_field_phase.reference_target_track_id = 'foreign'
        n.set_state(base.GraspStages.FAILED, 'context changed')
        return False
    c.node.execute = execute
    c.node.start_cb(c.request)
    e = c.events[0]
    if fault in ('selection', 'session_selection'):
        assert e['session_id'] is None and e['target_track_id'] is None
    if fault != 'phase_target': assert e['measurement_request_id'] is None
    if fault != 'measurement_session': assert e['phase_id'] is None


def test_initial_idle_and_rejected_selection_emit_no_invented_run(events):
    c = events
    c.node.set_state(base.GraspStages.IDLE, 'idle')
    c.params[module.SELECTION_PARAM] = dict(c.node._mode_selection, generation=99)
    c.node.execute = lambda **kw: pytest.fail('invalid selection executed')
    assert not c.node.start_cb(c.request).success
    assert c.events == [] and c.node._task_event_run is None


def test_setup_only_adds_diagnostic_string_topic(events, monkeypatch):
    calls = []
    monkeypatch.setattr(module.rospy, 'Publisher', lambda *a, **kw: calls.append((a, kw)) or NS())
    events.node._setup_task_event_io()
    assert calls == [(('/grasp/task_event', module.String), dict(queue_size=32))]


def test_hook_receives_same_message_object_after_original_publish(events):
    c = events; sent = []; seen = []
    c.node.pub.publish = lambda msg: sent.append(msg)
    c.node._task_state_published = lambda msg: seen.append((msg, len(sent)))
    c.node.set_state(base.GraspStages.IDLE, 'literal')
    assert seen == [(sent[0], 1)] and seen[0][0] is sent[0]
