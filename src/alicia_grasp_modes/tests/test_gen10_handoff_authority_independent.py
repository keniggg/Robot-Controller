"""Independent actual ModeAware task callback interleavings, without ROS IO."""
from copy import deepcopy
from itertools import permutations
import json
from types import SimpleNamespace as NS

import pytest

from test_station_compute_wait_task import (
    station, configured, frame, deliver, advance, feedback, base, module, at_ns)


def ready(station):
    c = station
    # The station unit fixture changes its phase/header after loading the
    # recorded geometry. Bind those fixture changes to a genuine plan digest.
    c.plan.object_geometry.header = deepcopy(c.plan.header)
    c.plan.plan_id = base.compute_plan_id(c.plan)
    c.node._bound_execution_plan = deepcopy(c.plan)
    c.node._bound_execution_plan_id = c.plan.plan_id
    c.node._bound_execution_plan_digest = c.node._execution_plan_digest(c.plan)
    c.node.latest_grasp6d_plan = deepcopy(c.plan)
    c.node._observe_return_session['view_plan_id'] = c.plan.plan_id
    c.node._observation_measurement_request['view_plan_id'] = c.plan.plan_id
    # Input visual geometry was already validated before handoff; isolate
    # temporal/callback authority here, retaining original plan integrity,
    # manual/active/revocation/selection/session checks.
    c.node._bound_target_drift_result = lambda *a: base.PlanValidationResult(True)
    f = frame(c)
    deliver(c, f)
    assert c.node._station_task_record is not None
    check = c.node._validate_bound_plan_locked(c.plan, {})
    assert check.ok, (check.code, check.reason)
    return c, f


def actual_publish_owner(c):
    owners = [cls for cls in type(c.node).__mro__
              if cls.__module__ == 'alicia_grasp_modes.observe_return_task'
              and cls.__name__ == 'ObserveReturnTask'
              and '_observe_return_publish' in cls.__dict__]
    assert len(owners) == 1
    return owners[0]


def pending_messages(c, f):
    contract = deepcopy(f['status']['station_compute_wait'])
    stamp = contract['geometry_deadline_ns'] + 1_158_667
    geometry = deepcopy(c.plan.object_geometry)
    geometry.header.stamp = at_ns(stamp)
    geometry.valid = False
    geometry.failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(dict(
        station_compute_wait=contract, station_compute_recovery_chain=[contract]))
    obj = deepcopy(f['object']); obj.header.stamp = at_ns(stamp)
    obj.pose_base.header.stamp = at_ns(stamp); obj.detected = False
    status = deepcopy(f['status'])
    status.update(source_stamp_ns=stamp, color_stamp_ns=stamp, depth_stamp_ns=stamp,
                  tf_lookup_stamp_ns=stamp, state='processing_deadline_pending',
                  detail='processing_deadline_pending', station_compute_wait=contract,
                  station_compute_recovery_chain=[contract], target_lost=False, first_loss=None)
    return stamp, dict(geometry=geometry, object=obj, status=status)


def receive(c, name, message):
    if name == 'geometry':
        c.node.target_geometry_cb(message)
    elif name == 'object':
        c.node.obj_cb(message)
    else:
        c.node._unknown_support_status_cb(NS(data=json.dumps(message)))


@pytest.mark.parametrize('order', list(permutations(('geometry', 'object', 'status'))))
@pytest.mark.parametrize('when', ['during_ack', 'after_ack_before_rebind'])
def test_bound_old_pending_callbacks_do_not_revoke_supplemental_metadata_handoff(
        station, monkeypatch, order, when):
    c, f = ready(station)
    stamp, messages = pending_messages(c, f)
    submitted = []
    def callbacks():
        advance(c, stamp)
        if hasattr(c.node, '_station_handoff_live'):
            c.node._station_handoff_live()
        for name in order:
            receive(c, name, messages[name])
            assert not c.node._execution_authority_revoked, name
        assert c.node._station_task_record is None  # No metric source restored.
        assert c.node.latest_grasp6d_plan is not None
        assert c.node.latest_grasp6d_plan.plan_id == c.plan.plan_id
        assert not c.node._execution_authority_revoked
    def transport(self, message):
        assert self._station_task_record is None
        if when == 'during_ack':
            callbacks()
        submitted.append(message)
    monkeypatch.setattr(actual_publish_owner(c), '_observe_return_publish', transport)
    nxt = dict(c.node._observe_return_session, view_index=3, view_plan_id='next')
    c.node._observe_return_publish(nxt)
    if when == 'after_ack_before_rebind':
        callbacks()
    assert submitted == [nxt]
    validation, result = c.node._invoke_plan_bound_action(
        c.plan, {}, 'supplemental observation rebind', lambda: 'metadata-only-bound')
    assert validation.ok, (validation.code, validation.reason)
    assert result == 'metadata-only-bound'


def published(station, monkeypatch):
    c, f = ready(station)
    monkeypatch.setattr(actual_publish_owner(c), '_observe_return_publish', lambda *a: None)
    nxt = dict(c.node._observe_return_session, view_index=3, view_plan_id='next')
    c.node._observe_return_publish(nxt)
    stamp, messages = pending_messages(c, f)
    advance(c, stamp)
    return c, f, nxt, messages


class ObservedEvent:
    """Instrument Event.wait without replacing the production wait loop."""
    def __init__(self):
        import threading
        self.inner = threading.Event()
        self.entered = threading.Event()
    def clear(self):
        self.inner.clear()
    def set(self):
        self.inner.set()
    def wait(self, timeout):
        self.entered.set()
        return self.inner.wait(timeout)


def start_rebind(c, plan=None, label='supplemental observation rebind'):
    import threading
    gate = ObservedEvent()
    h = getattr(c.node, '_station_task_handoff', None)
    if h is not None:
        h['changed'] = gate
    result, actions, errors = [], [], []
    done = threading.Event()
    def run():
        try:
            result.append(c.node._invoke_plan_bound_action(plan or c.plan, {},
                label, lambda: actions.append('metadata')))
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    return NS(gate=gate, worker=worker, result=result, actions=actions, errors=errors, done=done)


def finish(worker):
    worker.worker.join(1.)
    assert worker.done.is_set(), 'original bounded handoff wait did not exit'
    assert not worker.errors, worker.errors
    assert len(worker.result) == 1
    return worker.result[0][0]


def test_false_object_before_action_waits_for_late_exact_status_without_callback_lock(station, monkeypatch):
    c, _, _, messages = published(station, monkeypatch)
    receive(c, 'object', messages['object'])
    worker = start_rebind(c)
    assert worker.gate.entered.wait(1.), 'action returned before the typed receipt instead of waiting'
    assert not worker.done.is_set() and not worker.actions
    # The real status callback needs both ModeAware and plan locks. Completing
    # it here proves the worker does not hold either lock while waiting.
    receive(c, 'status', messages['status'])
    validation = finish(worker)
    assert validation.ok, (validation.code, validation.reason)
    assert worker.actions == ['metadata']
    assert not c.node._execution_authority_revoked
    assert c.node._station_task_record is None


@pytest.mark.parametrize('fault', ['ros_deadline', 'monotonic_deadline', 'stop', 'epoch', 'manual', 'target', 'selection'])
def test_unpaired_wait_exits_on_original_fault_without_action(station, monkeypatch, fault):
    c, _, _, messages = published(station, monkeypatch)
    receive(c, 'object', messages['object'])
    worker = start_rebind(c)
    assert worker.gate.entered.wait(1.), 'missing typed receipt did not enter the finite wait'
    assert not worker.done.is_set() and not worker.actions
    if fault == 'ros_deadline':
        advance(c, 190_000_000_000)
    elif fault == 'monotonic_deadline':
        c.mono[0] = c.node._station_task_handoff['phase_mono']
    elif fault == 'stop':
        # Only the topic publication behind stop is inert; revoke/active and
        # start->plan lock ordering remain the real stop callback.
        monkeypatch.setattr(c.node, '_set_near_field_active', lambda *a, **k: None)
        assert c.node.stop_cb(NS(emergency=False)).success
    elif fault == 'epoch':
        c.node.following_epoch_cb(NS(frame_id='sdk_reference_epoch', stamp=at_ns(30_000_000_001)))
    elif fault == 'manual':
        c.params['/gui/joint_direct_mode'] = True
    elif fault == 'selection':
        c.params[module.SELECTION_PARAM] = dict(c.selection, generation=c.selection['generation'] + 1)
    else:
        bad = deepcopy(messages['geometry'])
        bad.failure_reason = 'TARGET_LOST: actual identity lost'
        receive(c, 'geometry', bad)
    worker.gate.set()
    validation = finish(worker)
    assert not validation.ok
    assert not worker.actions
    # Even a late valid receipt must not grant fresh authority after failure.
    receive(c, 'status', messages['status'])
    calls = []
    validation, _ = c.node._invoke_plan_bound_action(c.plan, {},
        'supplemental observation rebind', lambda: calls.append('metadata'))
    assert not validation.ok and not calls


def test_false_between_wait_return_and_plan_lock_rechecks_atomically(station, monkeypatch):
    import threading
    c, _, _, messages = published(station, monkeypatch)
    wait_returned, allow_lock = threading.Event(), threading.Event()
    original_wait = c.node._station_handoff_wait
    calls = []
    def instrument_wait():
        original_wait()
        calls.append('returned')
        if len(calls) == 1:
            wait_returned.set()
            assert allow_lock.wait(1.)
    monkeypatch.setattr(c.node, '_station_handoff_wait', instrument_wait)
    worker = start_rebind(c)
    assert wait_returned.wait(1.)
    receive(c, 'object', messages['object'])
    allow_lock.set()
    # The false callback won the actual plan lock after the first empty
    # observation. The orchestration must wait again, not return terminal
    # PENDING and not submit a physical action.
    entered_again = worker.gate.entered.wait(1.)
    receive(c, 'status', messages['status'])
    validation = finish(worker)
    assert entered_again, 'unpaired false in wait->lock gap terminated metadata handoff'
    assert validation.ok, (validation.code, validation.reason)
    assert worker.actions == ['metadata']


@pytest.mark.parametrize('order', list(permutations(('geometry', 'object', 'status', 'preview'))))
def test_successor_source_completed_after_disarm_requires_same_recovery_chain(station, monkeypatch, order):
    c, f = ready(station)
    monkeypatch.setattr(actual_publish_owner(c), '_observe_return_publish', lambda *a: None)
    nxt = dict(c.node._observe_return_session, view_index=3, view_plan_id='next')
    c.node._observe_return_publish(nxt)
    successor = frame(c, 100_300_000_000)
    stamp, messages = pending_messages(c, successor)
    parent = f['status']['station_compute_wait']
    child = successor['status']['station_compute_wait']
    chain = [parent, child]
    messages['status']['station_compute_recovery_chain'] = deepcopy(chain)
    messages['geometry'].failure_reason = 'TARGET_PROCESSING_PENDING: ' + json.dumps(dict(
        station_compute_wait=child, station_compute_recovery_chain=chain))
    plan = deepcopy(c.plan)
    plan.header.stamp = at_ns(stamp)
    plan.valid = False
    plan.diagnostic = 'TARGET_PROCESSING_PENDING: original source is waiting'
    messages['preview'] = plan
    advance(c, stamp)
    for name in order:
        if name == 'preview':
            c.node.grasp6d_preview_plan_cb(messages[name])
        else:
            receive(c, name, messages[name])
        assert not c.node._execution_authority_revoked, name
    called = []
    validation, _ = c.node._invoke_plan_bound_action(c.plan, {},
        'supplemental observation rebind', lambda: called.append('metadata'))
    assert validation.ok, (validation.code, validation.reason)
    assert called == ['metadata'] and c.node._station_task_record is None


def rebound(station, monkeypatch):
    c, f = ready(station)
    session = c.node._observe_return_session
    session.update(origin_plan_id=c.plan.plan_id, origin_snapshot_stamp_ns=c.plan.header.stamp.to_nsec(),
                   view_source_stamp_ns=c.plan.header.stamp.to_nsec(), stamp_ns=c.now[0],
                   max_observations=3, view_request_id='fixture-next-view', previous_gain_sha256='0'*64)
    c.node._observe_return_origin_proof = dict(epoch_ns=c.node._following_epoch_ns)
    candidate = deepcopy(c.plan)
    candidate.header.stamp = at_ns(c.now[0])
    candidate.object_geometry.header = deepcopy(candidate.header)
    candidate.poses[0].position.x += .001
    candidate.plan_id = base.compute_plan_id(candidate)
    nxt = dict(session, view_index=3, view_plan_id=candidate.plan_id,
               view_source_stamp_ns=candidate.header.stamp.to_nsec())
    monkeypatch.setattr(actual_publish_owner(c), '_observe_return_publish', lambda *a: None)
    c.node._observe_return_publish(nxt)
    def bind():
        frozen = c.node._freeze_execution_plan(candidate)
        c.node._observe_return_session = deepcopy(nxt)
        return frozen
    validation, frozen = c.node._invoke_plan_bound_action(c.plan, {},
        'supplemental observation rebind', bind)
    assert validation.ok, (validation.code, validation.reason)
    c.node._last_state_message = 'admitting supplemental observation view'
    return c, f, frozen


@pytest.mark.parametrize('reject', [None, 'unknown', 'remote', 'missing'])
def test_real_double_measurement_ack_required_before_closed_handoff(station, monkeypatch, reject):
    from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
    c, _, candidate = rebound(station, monkeypatch)
    requests = []
    def publish(message):
        request = json.loads(message.data)
        requests.append(request)
        assert c.node._station_task_handoff['cutoff_ns'] is None
        for participant in PARTICIPANTS:
            if reject == 'missing' and participant == 'remote':
                continue
            response = measurement_ack(request, participant, accepted=participant != reject)
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(response)))
        if reject == 'missing':
            # Exhaust the original 5 s acknowledgement wait without wall sleep.
            original_wait = c.node._observe_return_condition.wait
            def expire(timeout):
                c.mono[0] += 5.
                return original_wait(0.)
            monkeypatch.setattr(c.node._observe_return_condition, 'wait', expire)
    c.node._observation_measurement_pub = NS(publish=publish)
    if reject is not None:
        with pytest.raises(ValueError):
            c.node._observation_measurement_prepare(candidate)
        assert c.node._station_task_handoff['cutoff_ns'] is None
        return
    prepared = c.node._observation_measurement_prepare(candidate)
    assert len(requests) == 1
    assert requests[0]['state'] == 'executing'
    assert requests[0]['motion_authorized'] is False
    assert requests[0]['metric_geometry_authority'] is False
    assert c.node._station_task_handoff['cutoff_ns'] == c.now[0]
    assert len(prepared['acks']) == 2 and not prepared['consumed']
    # A closed old-station receipt must not pin the subsequent contact plan,
    # phase or measured pose. Original plan integrity and commit checks run.
    c.node._near_field_phase_id += 1
    c.node._near_field_phase_deadline_sec = 195.
    contact = deepcopy(candidate)
    contact.diagnostic = 'CONTACT_EXECUTION_PLAN'
    from test_grasp_task_sequence import GraspTaskSequenceTest
    # Explicit synthetic bilateral fixture evidence, not a new measurement.
    contact = GraspTaskSequenceTest._mark_contact_geometry(contact)
    contact = c.node._freeze_execution_plan(contact, allow_target_occlusion=True)
    called = []
    validation, _ = c.node._invoke_plan_bound_action(contact, {},
        'strict pregrasp offline sentinel', lambda: called.append('checked-only'))
    assert validation.ok, (validation.code, validation.reason)
    assert called == ['checked-only']


@pytest.mark.parametrize('typed', [False, True])
def test_invalid_rich_execution_plan_is_never_a_pending_preview_exception(station, monkeypatch, typed):
    c, _, _, messages = published(station, monkeypatch)
    plan = deepcopy(c.plan)
    plan.header.stamp = deepcopy(messages['geometry'].header.stamp)
    plan.valid = False
    plan.diagnostic = (messages['geometry'].failure_reason if typed else 'TARGET_LOST: real rich invalidation')
    c.node.grasp6d_plan_cb(plan)
    assert c.node._execution_authority_revoked
    calls = []
    validation, _ = c.node._invoke_plan_bound_action(c.plan, {},
        'supplemental observation rebind', lambda: calls.append('metadata'))
    assert not validation.ok and not calls


@pytest.mark.parametrize('phase', ['new_observation', 'new_contact'])
@pytest.mark.parametrize('arrival', ['before_wait', 'wait_lock_gap'])
def test_closed_old_false_waits_for_old_status_at_subsequent_real_action_gate(
        station, monkeypatch, phase, arrival):
    import threading
    from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
    c, f, candidate = rebound(station, monkeypatch)
    def publish(message):
        request = json.loads(message.data)
        for participant in PARTICIPANTS:
            ack = measurement_ack(request, participant, accepted=True)
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))
    c.node._observation_measurement_pub = NS(publish=publish)
    stamp, messages = pending_messages(c, f)
    advance(c, stamp + 100_000_000)
    c.node._observation_measurement_prepare(candidate)
    assert c.node._station_task_handoff['cutoff_ns'] > stamp
    if phase == 'new_contact':
        from test_grasp_task_sequence import GraspTaskSequenceTest
        candidate.diagnostic = 'CONTACT_EXECUTION_PLAN'
        candidate = GraspTaskSequenceTest._mark_contact_geometry(candidate)
        c.node._near_field_phase_id += 1
        c.node._near_field_phase_deadline_sec = 195.
        candidate = c.node._freeze_execution_plan(candidate, allow_target_occlusion=True)
    original = c.node._validate_bound_plan_locked(candidate, {})
    assert original.ok, (original.code, original.reason)
    wait_returned, allow_lock = threading.Event(), threading.Event()
    if arrival == 'wait_lock_gap':
        original_wait = c.node._station_handoff_wait
        counter = []
        def instrument_wait():
            original_wait()
            counter.append(True)
            if len(counter) == 1:
                wait_returned.set()
                assert allow_lock.wait(1.)
        monkeypatch.setattr(c.node, '_station_handoff_wait', instrument_wait)
    else:
        receive(c, 'object', messages['object'])
    worker = start_rebind(c, candidate, 'strict next phase offline sentinel')
    if arrival == 'wait_lock_gap':
        reached = wait_returned.wait(1.)
        if reached:
            receive(c, 'object', messages['object'])
        allow_lock.set()
        assert reached, 'nonhistorical gate skipped original bounded handoff wait'
    entered = worker.gate.entered.wait(1.)
    receive(c, 'status', messages['status'])
    validation = finish(worker)
    assert entered, 'late old unpaired false immediately terminated subsequent action'
    assert validation.ok, (validation.code, validation.reason)
    assert worker.actions == ['metadata']
    assert not c.node._execution_authority_revoked


def contact_ready_after_real_hook(station, monkeypatch):
    from alicia_grasp_modes.observe_return_measurement import PARTICIPANTS, measurement_ack
    from test_grasp_task_sequence import GraspTaskSequenceTest
    c, f, observation = rebound(station, monkeypatch)
    def publish(message):
        request = json.loads(message.data)
        for participant in PARTICIPANTS:
            c.node._observation_measurement_ack_cb(NS(data=json.dumps(
                measurement_ack(request, participant, accepted=True))))
    c.node._observation_measurement_pub = NS(publish=publish)
    stamp, messages = pending_messages(c, f)
    advance(c, stamp + 100_000_000)
    c.node._observation_measurement_prepare(observation)
    contact = deepcopy(observation)
    contact.header.stamp = at_ns(c.now[0])
    contact.object_geometry.header = deepcopy(contact.header)
    contact.diagnostic = 'CONTACT_EXECUTION_PLAN'
    contact = GraspTaskSequenceTest._mark_contact_geometry(contact)
    contact = GraspTaskSequenceTest._mark_valid_3d(contact)
    c.node._near_field_phase_id += 1
    c.node._near_field_phase_deadline_sec = 195.
    reports = []
    # The registered-contact audit input is already checked upstream. Keep the
    # actual before-freeze orchestration while isolating its asynchronous audit
    # lookup; these tests do not certify registration or collision evidence.
    def report(plan, **kwargs):
        assert plan.plan_id == contact.plan_id and kwargs == {'observed_contact': True}
        reports.append(plan.plan_id)
        return {'fixture': 'upstream audit admission'}
    monkeypatch.setattr(c.node, '_observe_return_report', report)
    assert c.node._before_near_field_contact_freeze(contact, observation, {}) is None
    assert reports == [contact.plan_id]
    owner = next(cls for cls in type(c.node).__mro__
                 if cls.__name__ == 'GraspTaskNode' and '_freeze_execution_plan' in cls.__dict__)
    original = owner._freeze_execution_plan
    freezes = []
    def count(self, plan, *args, **kwargs):
        freezes.append(plan.plan_id)
        return original(self, plan, *args, **kwargs)
    monkeypatch.setattr(owner, '_freeze_execution_plan', count)
    return c, observation, contact, messages, freezes


@pytest.mark.parametrize('arrival', ['before_wait', 'wait_lock_gap'])
def test_real_near_field_loop_after_hook_waits_once_for_late_old_status(station, monkeypatch, arrival):
    import threading
    c, observation, contact, messages, freezes = contact_ready_after_real_hook(station, monkeypatch)
    select_contact = actual_near_field_loop(c, observation, contact, monkeypatch)
    wait_returned, allow_lock, done = threading.Event(), threading.Event(), threading.Event()
    gate = ObservedEvent()
    c.node._station_task_handoff['changed'] = gate
    if arrival == 'before_wait':
        receive(c, 'object', messages['object'])
    else:
        original_wait = c.node._station_handoff_wait
        waits = []
        def instrument_wait():
            original_wait()
            waits.append(True)
            if len(waits) == 1:
                wait_returned.set()
                assert allow_lock.wait(1.)
        monkeypatch.setattr(c.node, '_station_handoff_wait', instrument_wait)
    results, errors = [], []
    def freeze():
        try:
            results.append(select_contact())
        except BaseException as exc:
            errors.append(exc)
        finally:
            done.set()
    worker = threading.Thread(target=freeze, daemon=True)
    worker.start()
    if arrival == 'wait_lock_gap':
        reached = wait_returned.wait(1.)
        if reached:
            receive(c, 'object', messages['object'])
        allow_lock.set()
        assert reached, 'direct freeze did not enter the original bounded wait'
    entered = gate.entered.wait(1.)
    assert not freezes, 'base freeze committed before same-source pairing'
    receive(c, 'status', messages['status'])
    worker.join(1.)
    assert done.is_set() and not errors, errors
    assert entered, 'direct contact freeze failed instead of waiting for paired status'
    assert len(results) == 1 and results[0] is not None and freezes == [contact.plan_id]
    assert c.node._bound_execution_plan.plan_id == contact.plan_id


@pytest.mark.parametrize('fault', ['stop', 'geometry', 'epoch', 'manual'])
def test_fault_between_real_contact_hook_and_direct_freeze_cannot_reset_authority(station, monkeypatch, fault):
    c, observation, contact, messages, freezes = contact_ready_after_real_hook(station, monkeypatch)
    if fault == 'stop':
        monkeypatch.setattr(c.node, '_set_near_field_active', lambda *a, **k: None)
        assert c.node.stop_cb(NS(emergency=False)).success
    elif fault == 'geometry':
        lost = deepcopy(messages['geometry'])
        lost.header.stamp = at_ns(c.now[0])
        lost.failure_reason = 'TARGET_LOST: actual post-hook loss'
        c.node.target_geometry_cb(lost)
    elif fault == 'epoch':
        c.node.following_epoch_cb(NS(frame_id='sdk_reference_epoch', stamp=at_ns(30_000_000_001)))
    else:
        c.params['/gui/joint_direct_mode'] = True
    if fault in ('stop', 'geometry'):
        assert c.node._execution_authority_revoked
    with pytest.raises(ValueError):
        c.node._freeze_execution_plan(contact, allow_target_occlusion=True)
    assert not freezes
    assert c.node._bound_execution_plan.plan_id == observation.plan_id
    if fault in ('stop', 'geometry'):
        assert c.node._execution_authority_revoked


def actual_near_field_loop(c, observation, contact, monkeypatch):
    c.node._near_field_phase_started_sec = 99.4
    c.node._published_near_field_phase.phase_id = c.node._near_field_phase_id
    c.node._published_near_field_phase.deadline = at_ns(195_000_000_000)
    notices = []
    c.node._frozen_contact_pub = NS(publish=lambda m: notices.append(json.loads(m.data)))
    monkeypatch.setattr(c.node, '_set_near_field_preview_stream', lambda *a: True)
    # The original preview-copy/audit input is isolated here; real before-hook,
    # full bound guard, freeze, after-hook and frozen-contact notice are used.
    monkeypatch.setattr(c.node, '_copy_near_field_preview_candidate',
        lambda *a: (base.PlanValidationResult(True, age_sec=0.), deepcopy(contact)))
    config = dict(near_field_strategy='single_snapshot_direct', near_field_replan_enabled=True,
                  near_field_replan_required=True, near_field_replan_timeout_sec=90.)
    def select():
        result = c.node._maybe_rebind_near_field_grasp6d_plan(config, {}, observation)
        if result is not None:
            assert len(notices) == 1 and notices[0]['plan_id'] == contact.plan_id
        return result
    return select


@pytest.mark.parametrize('fault', ['stop', 'geometry', 'epoch', 'manual', 'phase_deadline'])
def test_actual_near_field_loop_fault_after_hook_cannot_freeze_or_publish(station, monkeypatch, fault):
    c, observation, contact, messages, freezes = contact_ready_after_real_hook(station, monkeypatch)
    select = actual_near_field_loop(c, observation, contact, monkeypatch)
    original = c.node._before_near_field_contact_freeze
    def before(*args):
        result = original(*args)
        assert result is None
        if fault == 'stop':
            monkeypatch.setattr(c.node, '_set_near_field_active', lambda *a, **k: None)
            assert c.node.stop_cb(NS(emergency=False)).success
        elif fault == 'geometry':
            lost = deepcopy(messages['geometry'])
            lost.header.stamp = at_ns(c.now[0])
            lost.failure_reason = 'TARGET_LOST: actual post-hook loss'
            c.node.target_geometry_cb(lost)
        elif fault == 'epoch':
            c.node.following_epoch_cb(NS(frame_id='sdk_reference_epoch', stamp=at_ns(30_000_000_001)))
        elif fault == 'phase_deadline':
            advance(c, 195_000_000_000)
        else:
            c.params['/gui/joint_direct_mode'] = True
        return result
    monkeypatch.setattr(c.node, '_before_near_field_contact_freeze', before)
    try:
        result = select()
    except ValueError as exc:
        # The nested freeze independently rejects epoch changes even when
        # the generic original bound validator has no epoch field.
        assert fault == 'epoch' and 'control_epoch_changed_at_freeze' in str(exc)
    else:
        assert result is None
    assert not freezes and c.node._bound_execution_plan.plan_id == observation.plan_id
    if fault in ('stop', 'geometry'):
        assert c.node._execution_authority_revoked


def test_actual_contact_loop_unpaired_status_stops_at_current_phase_deadline_equality(station, monkeypatch):
    import threading
    c, observation, contact, messages, freezes = contact_ready_after_real_hook(station, monkeypatch)
    select = actual_near_field_loop(c, observation, contact, monkeypatch)
    gate = ObservedEvent()
    c.node._station_task_handoff['changed'] = gate
    receive(c, 'object', messages['object'])
    results, errors = [], []
    def run():
        try:
            results.append(select())
        except BaseException as exc:
            errors.append(exc)
    worker = threading.Thread(target=run, daemon=True)
    worker.start()
    assert gate.entered.wait(1.)
    # This new phase expires before the immutable 200 s session. Neither the
    # old token nor the wait may borrow that later session deadline.
    advance(c, 195_000_000_000)
    gate.set()
    worker.join(1.)
    assert not worker.is_alive() and not errors
    assert results == [None] and not freezes
    assert c.node._bound_execution_plan.plan_id == observation.plan_id
