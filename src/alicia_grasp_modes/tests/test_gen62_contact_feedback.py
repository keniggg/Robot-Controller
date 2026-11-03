"""Task-local feedback and the contact handoff's final submission boundary.

The state injections exercise safety contracts independently of the recorded
2.24 second failure. They do not claim to reproduce that elapsed delay.
"""
from copy import deepcopy
from collections import deque
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time as clock_time

import pytest
from sensor_msgs.msg import JointState

import alicia_grasp_modes.contact_station_task as task_module
from alicia_grasp_modes.contact_station_task import ContactStationTaskMixin
from alicia_grasp_modes.rgb_observation_source import JOINT_NAMES
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_grasp_modes.contact_station_retirement import ack


def stamp(ns):
    return task_module.rospy.Time(ns // 1_000_000_000, ns % 1_000_000_000)


def joint(ns, values, names=None, frame='sdk_measured'):
    row = JointState()
    row.header.stamp, row.header.frame_id = stamp(ns), frame
    row.name = list(JOINT_NAMES if names is None else names)
    row.position = list(values)
    return row


def feedback_context(monkeypatch, values=None):
    clock = [10_000_000_000]
    monkeypatch.setattr(task_module.rospy.Time, 'now', staticmethod(lambda: stamp(clock[0])))
    node = ContactStationTaskMixin()
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = 8_000_000_000
    q = [0.] * 6 if values is None else list(values)
    node._following_accepted_history = [joint(clock[0] - (127-i)*10_000_000, q)
                                        for i in range(128)]
    return NS(node=node, clock=clock, q=q, epoch=node._following_epoch_ns)


@pytest.mark.parametrize('values', [
    [0., 0., 0., 0., 0., 0.],
    [-.7, .25, 1.1, -.6, .8, -.3],
    [.9, -.4, -.8, .3, -.2, .6],
])
@pytest.mark.parametrize('reverse_names', [False, True])
def test_local_bounded_history_uses_joint_names_and_measured_continuity(monkeypatch, values, reverse_names):
    c = feedback_context(monkeypatch, values)
    if reverse_names:
        for row in c.node._following_accepted_history:
            row.name.reverse()
            row.position.reverse()
    source, vector = c.node._contact_station_feedback(c.epoch, c.clock[0]-150_000_000, c.q)
    assert source == c.clock[0]
    assert vector == values
    assert len(c.node._following_accepted_history) == 128


@pytest.mark.parametrize('fault,reason', [
    ('stale', 'FRESH_FEEDBACK_REQUIRED'),
    ('future', 'FRESH_FEEDBACK_REQUIRED'),
    ('wrong_epoch', 'CONTROL_EPOCH_CHANGED'),
    ('empty', 'FRESH_FEEDBACK_REQUIRED'),
    ('wrong_frame', 'FRESH_FEEDBACK_REQUIRED'),
    ('pre_epoch', 'FRESH_FEEDBACK_REQUIRED'),
    ('since_without_sample', 'FRESH_FEEDBACK_REQUIRED'),
    ('missing_joint', 'FEEDBACK_INVALID'),
    ('duplicate_joint', 'FEEDBACK_INVALID'),
    ('nonfinite', 'FEEDBACK_INVALID'),
    ('out_and_back', 'MOVED_BEFORE_SUBMISSION'),
    ('external_fresh_only', 'FRESH_FEEDBACK_REQUIRED'),
])
def test_fresh_transport_does_not_replace_task_local_feedback_evidence(monkeypatch, fault, reason):
    c = feedback_context(monkeypatch)
    rows = c.node._following_accepted_history
    since = c.clock[0]-150_000_000
    if fault in ('stale', 'external_fresh_only'):
        c.clock[0] += 1_600_000_001
        if fault == 'external_fresh_only':
            c.node.latest_accepted_joint_state = joint(c.clock[0], c.q)
    elif fault == 'future':
        rows[-1].header.stamp = stamp(c.clock[0]+1)
    elif fault == 'wrong_epoch':
        c.node._following_epoch_ns += 1
    elif fault == 'empty':
        rows.clear()
    elif fault == 'wrong_frame':
        for row in rows:
            row.header.frame_id = 'commanded'
    elif fault == 'pre_epoch':
        for row in rows:
            row.header.stamp = stamp(c.epoch-1)
    elif fault == 'since_without_sample':
        since = c.clock[0]+1
    elif fault == 'missing_joint':
        rows[-1].name.pop()
        rows[-1].position.pop()
    elif fault == 'duplicate_joint':
        rows[-1].name[-1] = rows[-1].name[0]
    elif fault == 'nonfinite':
        rows[-1].position[2] = float('nan')
    else:
        rows[-2].position[1] += .006
        assert rows[-1].position == c.q
    with pytest.raises(ValueError, match=reason):
        c.node._contact_station_feedback(c.epoch, since, c.q)


@pytest.mark.parametrize('age_ns,allowed', [(800_000_000, True), (1_600_000_000, True), (1_600_000_001, False)])
def test_contact_feedback_retains_its_inclusive_configured_age_boundary(monkeypatch, age_ns, allowed):
    c = feedback_context(monkeypatch)
    c.clock[0] += age_ns
    if allowed:
        assert c.node._contact_station_feedback(c.epoch)[1] == c.q
    else:
        with pytest.raises(ValueError, match='FRESH_FEEDBACK_REQUIRED'):
            c.node._contact_station_feedback(c.epoch)


@pytest.mark.parametrize('change,allowed', [(.004, True), (.00400001, False)])
def test_contact_feedback_retains_its_original_movement_boundary(monkeypatch, change, allowed):
    c = feedback_context(monkeypatch)
    c.node._following_accepted_history[-1].position[0] = change
    if allowed:
        assert c.node._contact_station_feedback(c.epoch, c.clock[0], c.q)[1][0] == change
    else:
        with pytest.raises(ValueError, match='MOVED_BEFORE_SUBMISSION'):
            c.node._contact_station_feedback(c.epoch, c.clock[0], c.q)


def live_context(monkeypatch):
    # Reuse the historical producer/consumer setup so measurement and plan
    # digests retain their original schemas. Only the blocking validator below
    # is replaced to inject a state change at its return boundary.
    from test_contact_station_retirement import producer_context, remote_context
    c = producer_context(monkeypatch)
    _, request, plan = remote_context(c)
    node = ContactStationTaskMixin()
    node._observation_measurement_request = deepcopy(c.node._observation_measurement_status['request'])
    node._observation_measurement_request.update(target_track_id=plan.target_track_id)
    request = dict(request, measurement_sha256=measurement_digest(node._observation_measurement_request),
                   plan_sha256=committed_plan_digest(plan))
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = request['control_epoch_ns']
    start = task_module.rospy.Time.now().to_nsec()
    node._following_accepted_history = [joint(start, [0.]*6)]
    owner = [True]
    def owner_live():
        if not owner[0]:
            raise ValueError('CONTACT_TEST_OWNER_REVOKED')
    node._observe_return_live = owner_live
    return NS(node=node, request=request, plan=plan, clock=c.clock, start=start,
              q=[0.]*6, owner=owner)


@pytest.mark.parametrize('fault', ['deadline', 'epoch', 'owner', 'feedback_out_and_back',
                                  'plan', 'measurement'])
def test_state_changed_inside_original_bound_validation_cannot_pass_final_live_check(monkeypatch, fault):
    c = live_context(monkeypatch)
    visited = []
    def validate(*args):
        visited.append(True)
        if fault == 'deadline':
            c.clock[0] += .801
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        elif fault == 'owner':
            c.owner[0] = False
        elif fault == 'feedback_out_and_back':
            c.node._following_accepted_history.extend([
                joint(c.start+1, [.006, 0., 0., 0., 0., 0.]), joint(c.start+2, c.q)])
            c.clock[0] += .001
        elif fault == 'plan':
            c.plan.poses[0].position.x += .01
        else:
            c.node._observation_measurement_request['target_epoch'] += 1
        return NS(ok=True, code='', reason='')
    c.node._validate_bound_plan_locked = validate
    with pytest.raises(ValueError):
        c.node._contact_station_live(c.plan, c.request, {}, c.start, c.q)
    assert visited


def restore(message, values):
    import genpy
    from roslib.message import get_message_class
    for name, kind in zip(message.__slots__, message._slot_types):
        value = values[name]
        if kind in ('time', 'duration'):
            cls = genpy.Time if kind == 'time' else genpy.Duration
            value = cls(value['ns']//10**9, value['ns'] % 10**9)
        elif kind.endswith('[]') and '/' in kind:
            cls = get_message_class(kind[:-2])
            value = [restore(cls(), row) for row in value]
        elif '/' in kind:
            value = restore(get_message_class(kind)(), value)
        setattr(message, name, value)
    return message


def recorded_context(monkeypatch):
    """Actual frozen inputs with an explicit, controlled callback schedule.

    These are recorder timestamps, not evidence of the original task cache or
    its scheduling. ACK transport is inert; its exact message validator runs.
    """
    from std_msgs.msg import String
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    from alicia_grasp_modes.observe_return_task import ObserveReturnTask
    fixture = Path(__file__).with_name('fixtures')/'gen62_contact_feedback_recorded.json'
    raw = fixture.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == 'cdb856cc85f58c0eae21ad1125ccd13268c4e02fb0b8038286fc08b678d4edac'
    data = json.loads(raw)
    assert data['local_callback_times_available'] is False
    plan = restore(Grasp6DPlan(), data['plan'])
    request = data['request']
    assert committed_plan_digest(plan) == request['plan_sha256']
    assert measurement_digest(data['measurement']) == request['measurement_sha256']
    assert request['deadline_ns'] - request['stamp_ns'] == 800_000_000
    c = NS(node=ObserveReturnTask(), plan=plan, original_request=deepcopy(request),
           ros=[request['stamp_ns']], mono=[100.], owner=[True], in_publish=False,
           requests=[], submitted=[], validated=[], waits=[], dropped_owners=set(),
           on_publish=None, on_validate=None, on_ack=None, on_wait=None)
    # Preserve original_request as evidence; this run builds a new request.
    c.current_deadline_ns = min(request['stamp_ns'] + 1_600_000_000,
                                data['measurement']['deadline_ns'])
    monkeypatch.setattr(task_module.rospy.Time, 'now', staticmethod(lambda: stamp(c.ros[0])))
    monkeypatch.setattr(task_module, 'time', NS(monotonic=lambda: c.mono[0],
        monotonic_ns=lambda: int(c.mono[0]*10**9), thread_time_ns=clock_time.thread_time_ns))
    n = c.node
    n._mode_bound_selection = deepcopy(request['selection'])
    n._observation_measurement_request = deepcopy(data['measurement'])
    n._following_evidence_lock = threading.RLock()
    n._following_epoch_ns = request['control_epoch_ns']
    n._following_accepted_history = deque((restore(JointState(), row['message'])
        for row in data['accepted']), maxlen=128)
    n._contact_station_acks = {}
    n._published_near_field_phase = NS(active=True, planning_stage='contact_replan',
        reference_target_track_id=plan.target_track_id, phase_id=request['phase_id'],
        header=NS(stamp=stamp(request['phase_started_ns'])),
        deadline=stamp(data['measurement']['deadline_ns']))
    class ControlledCondition:
        def __init__(self):
            self.lock = threading.RLock()
        def __enter__(self):
            return self.lock.__enter__()
        def __exit__(self, *args):
            return self.lock.__exit__(*args)
        def notify_all(self):
            pass
        def wait(self, seconds):
            c.waits.append(seconds)
            assert len(c.waits) < 200, 'handoff exceeded its original finite budget'
            c.mono[0] += seconds
            if c.on_wait:
                c.on_wait(seconds)
    n._observe_return_condition = ControlledCondition()
    def owner_live():
        if not c.owner[0]:
            raise ValueError('CONTACT_TEST_OWNER_REVOKED')
    n._observe_return_live = owner_live
    n._observation_measurement_live = lambda value: None
    def validate(*args):
        c.validated.append((c.ros[0], c.mono[0], c.in_publish))
        if c.on_validate:
            c.on_validate()
        return NS(ok=True, code='', reason='')
    n._validate_bound_plan_locked = validate
    def publish(message):
        value = json.loads(message.data)
        c.requests.append(deepcopy(value))
        if c.on_publish:
            c.on_publish(value)
        for participant in ('unknown', 'remote'):
            if participant in c.dropped_owners:
                continue
            response = ack(value, participant)
            if c.on_ack:
                c.on_ack(value, response)
            n._contact_station_ack_cb(String(data=json.dumps(response)))
    n._contact_station_pub = NS(publish=publish)
    original_publish = n._contact_station_publish
    def traced_publish(*args, **kwargs):
        c.in_publish = True
        try:
            return original_publish(*args, **kwargs)
        finally:
            c.in_publish = False
    n._contact_station_publish = traced_publish
    def execute(pose, enabled):
        c.submitted.append((deepcopy(pose), enabled))
        return NS(success=True)
    c.execute = n._wrap_pose_submission_executor(GraspStages.MOVE_PREGRASP, plan, {}, execute)
    return c


def test_recorded_inputs_reach_original_strict_executor_only_after_both_exact_phases(monkeypatch):
    c = recorded_context(monkeypatch)
    plan_before = deepcopy(c.plan)
    result = c.execute(c.plan.poses[0], True)
    assert result.success and c.submitted == [(c.plan.poses[0], True)]
    assert [row['state'] for row in c.requests] == ['prepare', 'retire']
    for row in c.requests:
        assert row['deadline_ns'] == c.current_deadline_ns
        assert row['plan_sha256'] == c.original_request['plan_sha256']
        assert row['measurement_sha256'] == c.original_request['measurement_sha256']
        assert row['motion_authorized'] is False and row['metric_geometry_authority'] is False
    assert set(c.requests[1]['prepared_acks']) == {'unknown', 'remote'}
    assert c.node._contact_station_retired_measurement == c.original_request['measurement_sha256']
    assert c.plan == plan_before


@pytest.mark.parametrize('fault', ['deadline', 'monotonic_only', 'epoch', 'owner',
                                  'feedback_out_and_back', 'plan', 'measurement'])
def test_original_executor_never_receives_state_invalidated_during_final_validation(monkeypatch, fault):
    c = recorded_context(monkeypatch)
    injected = []
    def changed():
        # The final call is outside both original publish loops, so this does
        # not depend on their validation count or ACK arrival permutations.
        if c.in_publish or not c.requests or c.requests[-1]['state'] != 'retire':
            return
        if injected:
            return
        injected.append(True)
        if fault == 'deadline':
            c.ros[0] = c.current_deadline_ns
        elif fault == 'monotonic_only':
            c.mono[0] += 1.601
        elif fault == 'epoch':
            c.node._following_epoch_ns += 1
        elif fault == 'owner':
            c.owner[0] = False
        elif fault == 'feedback_out_and_back':
            stable = deepcopy(c.node._following_accepted_history[-1])
            moved = deepcopy(stable)
            moved.header.stamp = stamp(c.ros[0]-1)
            moved.position[0] += .006
            stable.header.stamp = stamp(c.ros[0])
            c.node._following_accepted_history.extend([moved, stable])
        elif fault == 'plan':
            c.plan.poses[0].position.x += .01
        else:
            c.node._observation_measurement_request['target_epoch'] += 1
    c.on_validate = changed
    with pytest.raises(ValueError):
        c.execute(c.plan.poses[0], True)
    assert injected and c.submitted == []
    assert getattr(c.node, '_contact_station_retired_measurement', None) is None


@pytest.mark.parametrize('ros_mode', ['forward', 'frozen', 'rollback'])
def test_prepare_retire_and_final_commit_share_one_original_monotonic_budget(monkeypatch, ros_mode):
    c = recorded_context(monkeypatch)
    rolled_back = []
    def delayed(value):
        if value['state'] == 'prepare':
            c.mono[0] += .90
            if ros_mode != 'frozen':
                c.ros[0] += 200_000_000 if ros_mode == 'rollback' else 100_000_000
        else:
            c.mono[0] += .71
            if ros_mode == 'forward':
                c.ros[0] += 100_000_000
    def rollback_during_prepare():
        if (ros_mode == 'rollback' and c.in_publish and c.requests[-1]['state'] == 'prepare'
                and not rolled_back):
            c.ros[0] -= 100_000_000
            rolled_back.append(True)
    c.on_publish = delayed
    c.on_validate = rollback_during_prepare
    with pytest.raises(ValueError):
        c.execute(c.plan.poses[0], True)
    assert [row['state'] for row in c.requests] == ['prepare', 'retire']
    assert all(row['deadline_ns'] == c.current_deadline_ns for row in c.requests)
    assert c.ros[0] < c.current_deadline_ns
    assert c.ros[0] >= c.requests[-1]['stamp_ns']
    assert c.mono[0]-100. > 1.6 and c.submitted == []


@pytest.mark.parametrize('missing', ['unknown', 'remote'])
def test_frozen_ros_clock_never_extends_wait_for_either_missing_owner(monkeypatch, missing):
    c = recorded_context(monkeypatch)
    c.dropped_owners.add(missing)
    with pytest.raises(ValueError):
        c.execute(c.plan.poses[0], True)
    assert [row['state'] for row in c.requests] == ['prepare']
    assert c.ros[0] == c.original_request['stamp_ns']
    assert 1.6 <= c.mono[0]-100. < 1.62
    assert c.submitted == []


@pytest.mark.parametrize('field', ['participant', 'plan_source_ns', 'plan_sha256', 'measurement_sha256'])
@pytest.mark.parametrize('phase', ['prepare', 'retire'])
def test_both_ack_phases_keep_exact_owner_source_plan_and_measurement_bindings(monkeypatch, field, phase):
    c = recorded_context(monkeypatch)
    def corrupt(request, response):
        if response['participant'] != 'remote' or request['state'] != phase:
            return
        if field == 'participant':
            response[field] = 'unknown'
        else:
            different = deepcopy(request)
            if field == 'plan_source_ns':
                different[field] -= 1
            else:
                different[field] = '0'*64
            response.clear()
            response.update(ack(different, 'remote'))
    c.on_ack = corrupt
    with pytest.raises(ValueError):
        c.execute(c.plan.poses[0], True)
    assert c.submitted == []


@pytest.mark.parametrize('fault,reason', [
    ('stale', 'CONTACT_STATION_FRESH_FEEDBACK_REQUIRED'),
    ('epoch', 'CONTACT_STATION_CONTROL_EPOCH_CHANGED'),
    ('invalid', 'CONTACT_STATION_FEEDBACK_INVALID'),
    ('drift', 'CONTACT_STATION_MOVED_BEFORE_SUBMISSION'),
    ('since', 'CONTACT_STATION_FRESH_FEEDBACK_REQUIRED'),
])
def test_failure_diagnostic_is_bounded_after_unlock_and_has_no_execution_authority(monkeypatch, fault, reason):
    c = feedback_context(monkeypatch)
    class ObservedLock:
        def __init__(self):
            self.depth = 0
        def __enter__(self):
            self.depth += 1
        def __exit__(self, *args):
            self.depth -= 1
    lock = ObservedLock()
    c.node._following_evidence_lock = lock
    logged = []
    def log(fmt, encoded):
        assert lock.depth == 0
        logged.append(json.loads(encoded))
    monkeypatch.setattr(task_module.rospy, 'logwarn', log)
    since = c.clock[0]-150_000_000
    if fault == 'stale':
        c.clock[0] += 1_600_000_001
    elif fault == 'epoch':
        c.node._following_epoch_ns += 1
    elif fault == 'invalid':
        c.node._following_accepted_history[-1].name[0] = 'missing'
    elif fault == 'drift':
        c.node._following_accepted_history[-1].position[0] = .005
    else:
        since = c.clock[0]+1
    with pytest.raises(ValueError, match='^'+reason+'$'):
        c.node._contact_station_feedback(c.epoch, since, c.q)
    assert len(logged) == 1
    diagnostic = logged[0]
    assert diagnostic == c.node._contact_station_feedback_diagnostic
    assert diagnostic['reason'] == reason
    assert diagnostic['diagnostic_only'] is True and diagnostic['execution_authorized'] is False
    assert 1 <= len(diagnostic['stages']) <= 8
    assert len(json.dumps(diagnostic)) < 4096
    assert not {'positions', 'history', 'rows', 'plan', 'geometry'} & set(diagnostic)
    assert diagnostic['stages'][-1]['stage'] == 'rejected'
    assert all(row['elapsed_ns'] >= 0 and row['elapsed_thread_cpu_ns'] >= 0
               for row in diagnostic['stages'])


@pytest.mark.parametrize('fault', ['logger', 'encoder'])
def test_diagnostic_failure_cannot_replace_or_suppress_original_rejection(monkeypatch, fault):
    c = feedback_context(monkeypatch)
    c.clock[0] += 1_600_000_001
    def broken(*args, **kwargs):
        raise RuntimeError('injected diagnostic failure')
    if fault == 'logger':
        monkeypatch.setattr(task_module.rospy, 'logwarn', broken)
    else:
        monkeypatch.setattr(task_module, 'json', NS(dumps=broken))
    with pytest.raises(ValueError, match='^CONTACT_STATION_FRESH_FEEDBACK_REQUIRED$'):
        c.node._contact_station_feedback(c.epoch)


@pytest.mark.parametrize('timer', ['monotonic_ns', 'thread_time_ns'])
@pytest.mark.parametrize('stale', [False, True])
def test_diagnostic_timer_failure_preserves_original_accept_or_reject(monkeypatch, timer, stale):
    c = feedback_context(monkeypatch)
    if stale:
        c.clock[0] += 1_600_000_001
    def broken():
        raise RuntimeError('injected diagnostic timer failure')
    clocks = {name: getattr(task_module.time, name)
              for name in ('monotonic', 'monotonic_ns', 'thread_time_ns')}
    clocks[timer] = broken
    monkeypatch.setattr(task_module, 'time', NS(**clocks))
    if stale:
        with pytest.raises(ValueError, match='^CONTACT_STATION_FRESH_FEEDBACK_REQUIRED$'):
            c.node._contact_station_feedback(c.epoch)
    else:
        assert c.node._contact_station_feedback(c.epoch)[1] == c.q


def test_successful_feedback_adds_no_transport_queries_or_history_diagnostic(monkeypatch):
    c = feedback_context(monkeypatch)
    unexpected = []
    def forbidden(*args, **kwargs):
        unexpected.append((args, kwargs))
        raise AssertionError('unexpected transport or parameter access')
    for name in ('Publisher', 'Subscriber', 'get_param', 'set_param', 'logwarn'):
        monkeypatch.setattr(task_module.rospy, name, forbidden)
    assert c.node._contact_station_feedback(c.epoch)[1] == c.q
    assert not unexpected
    assert not hasattr(c.node, '_contact_station_feedback_diagnostic')


def test_recorded_feedback_with_controlled_post_snapshot_pause_still_rejects_before_submission(monkeypatch):
    c = recorded_context(monkeypatch)
    original_copy = task_module.deepcopy
    paused = []
    def copy_with_pause(value):
        result = original_copy(value)
        if (c.requests and not paused and isinstance(value, list) and value
                and isinstance(value[0], JointState)):
            c.ros[0] = 1792661385188850164
            c.mono[0] += 2.239521504
            paused.append(True)
        return result
    monkeypatch.setattr(task_module, 'deepcopy', copy_with_pause)
    with pytest.raises(ValueError, match='^CONTACT_STATION_FRESH_FEEDBACK_REQUIRED$'):
        c.execute(c.plan.poses[0], True)
    assert paused and c.submitted == []
    assert [row['state'] for row in c.requests] == ['prepare']
    diagnostic = c.node._contact_station_feedback_diagnostic
    assert diagnostic['snapshot_count'] == diagnostic['filtered_count'] == 128
    assert diagnostic['latest_source_age_ns'] > 1_600_000_000
    assert diagnostic['request']['deadline_ns'] == c.current_deadline_ns
    assert diagnostic['execution_authorized'] is False


def test_epoch_callback_after_snapshot_unlock_revokes_the_copied_feedback(monkeypatch):
    c = feedback_context(monkeypatch)
    entered = []
    lock = c.node._following_evidence_lock
    def clock_after_epoch_callback():
        if not entered:
            assert not lock._is_owned(), 'inject only after the snapshot lock is released'
            with lock:
                c.node._following_epoch_ns += 1
                c.node._following_accepted_history.clear()
            entered.append(True)
        return stamp(c.clock[0])
    monkeypatch.setattr(task_module.rospy.Time, 'now', staticmethod(clock_after_epoch_callback))
    with pytest.raises(ValueError, match='^CONTACT_STATION_CONTROL_EPOCH_CHANGED$'):
        c.node._contact_station_feedback(c.epoch)
    assert entered and not c.node._following_accepted_history


def test_wait_for_final_epoch_lock_cannot_refresh_or_reuse_an_expired_snapshot(monkeypatch):
    c = feedback_context(monkeypatch)
    original_source = c.clock[0]
    c.node._following_accepted_history = deque(c.node._following_accepted_history, maxlen=128)
    class DelayedReacquire:
        def __init__(self):
            self.lock = threading.RLock()
            self.entries = 0
        def __enter__(self):
            self.entries += 1
            if self.entries == 2:
                # Model a callback holding the lock ahead of this final
                # acquisition. Its new sample cannot replace already checked
                # continuity rows or renew their original freshness bound.
                with self.lock:
                    c.clock[0] += 1_600_000_001
                    c.node._following_accepted_history.append(joint(c.clock[0], c.q))
            return self.lock.__enter__()
        def __exit__(self, *args):
            return self.lock.__exit__(*args)
    lock = DelayedReacquire()
    c.node._following_evidence_lock = lock
    with pytest.raises(ValueError, match='^CONTACT_STATION_FRESH_FEEDBACK_REQUIRED$'):
        c.node._contact_station_feedback(c.epoch, original_source, c.q)
    assert lock.entries == 2
    assert c.node._following_accepted_history[-1].header.stamp.to_nsec() == c.clock[0]
    diagnostic = c.node._contact_station_feedback_diagnostic
    assert diagnostic['latest_source_ns'] == original_source
    assert diagnostic['initial_validation_ros_ns'] == original_source
    assert diagnostic['validation_ros_ns'] == c.clock[0]
    assert diagnostic['latest_source_age_ns'] == 1_600_000_001
