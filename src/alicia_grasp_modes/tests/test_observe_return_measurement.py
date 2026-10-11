"""Non-authorizing measurement lifecycle, with inert task motion interfaces."""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS
from contextlib import nullcontext

import pytest

from alicia_grasp_modes import observe_return_measurement as contract
from alicia_grasp_modes import observe_return_task as task


def request():
    return dict(version=1, state='executing', lease_id='lease', request_id='lease:executing',
        selection=dict(mode='unknown', strategy='observe_return', generation=3, stamp_ns=1_000_000_000),
        session_id='session', target_epoch=2, target_track_id='target', origin_plan_id='origin',
        origin_snapshot_stamp_ns=2_000_000_000, view_plan_id='view', view_source_stamp_ns=3_000_000_000,
        view_index=1, plan_sha256='a'*64, control_epoch_ns=1_500_000_000,
        deadline_ns=30_000_000_000, stamp_ns=4_000_000_000, motion_kind='observation',
        arrival_stamp_ns=0, capture_deadline_ns=0, metric_geometry_authority=False, motion_authorized=False)


def arrived(previous):
    return dict(previous, state='requalifying', request_id='lease:requalifying', stamp_ns=10_000_000_000,
        arrival_stamp_ns=10_000_000_000, capture_deadline_ns=15_000_000_000,
        settle=dict(epoch_ns=previous['control_epoch_ns'], stationary_window_start_ns=9_000_000_000,
                    stationary_window_end_ns=9_900_000_000))


def parse(value, previous=None, now=None):
    return contract.parse_measurement_request(value, request()['selection'],
        value['stamp_ns'] if now is None else now, previous)


def test_exact_binding_and_three_new_original_source_frames_complete_without_authority():
    first = parse(request())
    second = parse(arrived(first), first)
    sources = [10_100_000_000, 10_200_000_000, 10_300_000_000]
    for participant in contract.PARTICIPANTS:
        ack = contract.measurement_ack(second, participant, accepted=True, qualified=True, source_stamps_ns=sources)
        assert contract.validate_measurement_ack(ack, second, participant, sources[-1], require_qualified=True) == ack
        assert not ack['motion_authorized'] and not ack['metric_geometry_authority']
    final = dict(second, state='qualified', request_id='lease:qualified', stamp_ns=10_400_000_000,
                 source_stamps_ns=sources)
    assert parse(final, second)['source_stamps_ns'] == sources


@pytest.mark.parametrize('key,value', [('target_epoch', 3), ('control_epoch_ns', 1_600_000_000),
    ('view_plan_id', 'other'), ('plan_sha256', 'b'*64), ('deadline_ns', 31_000_000_000),
    ('origin_snapshot_stamp_ns', 2_100_000_000), ('target_track_id', 'other'), ('view_index', 2),
    ('motion_kind', 'return'), ('lease_id', 'other'), ('session_id', 'other')])
def test_transition_cannot_change_frozen_binding(key, value):
    first = parse(request())
    second = arrived(first)
    second[key] = value
    with pytest.raises(ValueError, match='BINDING_CHANGED'):
        parse(second, first)


@pytest.mark.parametrize('field', ['metric_geometry_authority', 'motion_authorized'])
def test_receipt_never_has_motion_or_geometry_authority(field):
    value = request()
    value[field] = True
    with pytest.raises(ValueError, match='AUTHORITY_FORBIDDEN'):
        parse(value)


@pytest.mark.parametrize('fault', ['short', 'old', 'duplicate', 'future', 'expired', 'bool', 'different_request',
                                  'wrong_participant', 'unqualified', 'rejected'])
def test_ack_is_not_fresh_multiframe_geometry_by_itself(fault):
    second = parse(arrived(parse(request())), parse(request()))
    stamps = [10_100_000_000, 10_200_000_000, 10_300_000_000]
    ack = contract.measurement_ack(second, 'unknown', accepted=True, qualified=True, source_stamps_ns=stamps)
    now = 10_400_000_000
    if fault == 'short': ack['source_stamps_ns'].pop()
    elif fault == 'old': ack['source_stamps_ns'][0] = second['arrival_stamp_ns']
    elif fault == 'duplicate': ack['source_stamps_ns'][1] = ack['source_stamps_ns'][0]
    elif fault == 'future': ack['source_stamps_ns'][-1] = now + 1
    elif fault == 'expired': now = second['capture_deadline_ns']
    elif fault == 'bool': ack['source_stamps_ns'][0] = True
    elif fault == 'different_request': ack['request_id'] = 'other'
    elif fault == 'wrong_participant': ack['participant'] = 'remote'
    elif fault == 'unqualified': ack['qualified'] = False
    else: ack['accepted'] = False
    with pytest.raises(ValueError):
        contract.validate_measurement_ack(ack, second, 'unknown', now, require_qualified=True)


def test_expired_lease_may_be_revoked_but_never_requalified():
    first = parse(request())
    stop = dict(first, state='revoked', request_id='stop', stamp_ns=31_000_000_000)
    assert parse(stop, first)['state'] == 'revoked'
    with pytest.raises(ValueError):
        parse(arrived(first), first, now=31_000_000_000)


@pytest.mark.parametrize('fault', ['old_settle', 'wrong_epoch', 'long_capture', 'capture_renewal'])
def test_arrival_cannot_reuse_old_stillness_or_renew_its_budget(fault):
    first = parse(request())
    second = arrived(first)
    if fault == 'old_settle': second['settle']['stationary_window_start_ns'] = first['stamp_ns']
    elif fault == 'wrong_epoch': second['settle']['epoch_ns'] += 1
    elif fault == 'long_capture': second['capture_deadline_ns'] = second['arrival_stamp_ns'] + 10_000_000_001
    else:
        previous = parse(second, first)
        second = dict(previous, state='qualified', request_id='done', stamp_ns=11_000_000_000,
            source_stamps_ns=[10_100_000_000, 10_200_000_000, 10_300_000_000],
            capture_deadline_ns=previous['capture_deadline_ns'] + 1)
        first = previous
    with pytest.raises(ValueError):
        parse(second, first)


class Backend:
    def _plan_and_execute_pose(self, *args, **kwargs):
        self.events.append('original_preflight')
        response = kwargs['execute_pose'](args[2], True)
        if not response.success:
            return False
        self.events.append('original_settle')
        return self.backend_result


class Node(task.ObserveReturnTask, Backend):
    pass


def inert_node():
    node = Node()
    node.events = []
    node.backend_result = True
    node._observation_measurement_scope = lambda plan: True
    node._observation_measurement_begin = lambda plan, *a: node.events.append('dual_ack')
    node._validate_bound_plan_locked = lambda *a: (node.events.append('original_commit_gate') or NS(ok=True))
    node._observation_measurement_finish = lambda *a: node.events.append('fresh_original_multiframe')
    node._observation_measurement_revoke = lambda reason: node.events.append('revoke')
    node.set_state = lambda *a: node.events.append('failed')
    return node


def execute(node, success=True):
    def strict(*args):
        node.events.append('strict_motion')
        return NS(success=success)
    return node._plan_and_execute_pose(1, 'view', object(), lambda *a: None, 'view',
        execution_plan=object(), execute_pose=strict, gcfg={})


def test_task_waits_for_ack_after_original_preflight_and_rechecks_before_actual_call():
    node = inert_node()
    assert execute(node)
    assert node.events == ['original_preflight', 'dual_ack', 'original_commit_gate',
                           'strict_motion', 'original_settle', 'fresh_original_multiframe']


@pytest.mark.parametrize('failure', ['ack', 'gate', 'motion', 'settle', 'requalification'])
def test_no_motion_or_continuation_after_lifecycle_failure(failure):
    node = inert_node()
    if failure == 'ack':
        node._observation_measurement_begin = lambda *a: (_ for _ in ()).throw(ValueError('unknown rejected'))
    elif failure == 'gate':
        node._validate_bound_plan_locked = lambda *a: NS(ok=False, code='TARGET_STALE', reason='old')
    elif failure == 'settle': node.backend_result = False
    elif failure == 'requalification':
        node._observation_measurement_finish = lambda *a: (_ for _ in ()).throw(ValueError('4mm gate failed'))
    assert execute(node, success=failure != 'motion') is False
    assert 'revoke' in node.events
    if failure in ('ack', 'gate'):
        assert 'strict_motion' not in node.events
    assert 'fresh_original_multiframe' not in node.events


def test_unrelated_direct_or_silhouette_execution_uses_original_path_only():
    node = inert_node()
    node._observation_measurement_scope = lambda plan: False
    assert execute(node)
    assert node.events == ['original_preflight', 'strict_motion', 'original_settle']


def test_task_cannot_use_ack_before_actual_bound_object_has_arrived(monkeypatch):
    node = Node()
    node._observe_return_condition = threading.Condition(threading.RLock())
    node._mode_bound_selection = request()['selection']
    node._observation_measurement_request = parse(request())
    node._observation_measurement_live = lambda *a: None
    node._configured_target_observation_validity = lambda cfg: 5.
    node._last_motion_settle_sample = arrived(request())['settle']
    times = iter([10_000_000_000, 13_000_000_000, 13_000_000_000])
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: NS(to_nsec=lambda: next(times))))
    node._observation_measurement_publish = lambda req, **kw: [dict(source_stamps_ns=[10_100_000_000,
        10_200_000_000, 10_300_000_000])] * 2
    with pytest.raises(ValueError, match='TASK_SOURCE_MISSING'):
        node._observation_measurement_finish(object(), {})


@pytest.fixture
def transport(monkeypatch):
    node = Node()
    node._initialize_observe_return_state()
    node._mode_bound_selection = request()['selection']
    now = [4_000_000_000]
    clock = lambda ns: NS(to_nsec=lambda: ns)
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: clock(now[0])))
    monkeypatch.setattr(task, 'committed_plan_digest', lambda plan: 'a'*64)
    node._observe_return_live = lambda: None
    node._following_evidence_lock = threading.Lock()
    node._following_epoch_ns = 1_500_000_000
    node._observe_return_session = dict(version=2, phase='observing', session_id='session',
        target_epoch=2, target_track_id='target', origin_plan_id='origin',
        origin_snapshot_stamp_ns=2_000_000_000, view_plan_id='view', view_index=1,
        deadline_ns=30_000_000_000)
    node._observe_return_origin_proof = dict(epoch_ns=node._following_epoch_ns)
    node._grasp6d_plan_guard = nullcontext
    node._object_source_stamp = lambda obj: obj.header.stamp
    node._observed_target_matches_plan = lambda plan, obj, cfg: obj.track == 'target'
    publications = []
    def publish(message):
        payload = json.loads(message.data)
        publications.append(payload)
        stamps = []
        if payload['state'] == 'requalifying':
            stamps = [payload['arrival_stamp_ns'] + i*100_000_000 for i in (1, 2, 3)]
            now[0] = stamps[-1] + 1
            node.latest_obj = NS(detected=True, track='target', header=NS(stamp=clock(stamps[-1])))
        for participant in contract.PARTICIPANTS:
            ack = contract.measurement_ack(payload, participant, accepted=True,
                qualified=payload['state'] == 'requalifying', source_stamps_ns=stamps)
            node._observation_measurement_ack_cb(NS(data=json.dumps(ack)))
    node._observation_measurement_pub = NS(publish=publish)
    plan = NS(plan_id='view', header=NS(stamp=clock(3_000_000_000)))
    return node, now, publications, plan


def test_real_task_receipts_bind_settle_and_actual_object_before_release(transport):
    node, now, publications, plan = transport
    first = node._observation_measurement_begin(plan)
    now[0] = 10_000_000_000
    node._last_motion_settle_sample = arrived(first)['settle']
    node._observation_measurement_finish(plan, {})
    assert [row['state'] for row in publications] == ['executing', 'requalifying', 'qualified']
    assert publications[1]['capture_deadline_ns'] == 13_000_000_000
    assert node.latest_obj.header.stamp.to_nsec() == publications[-1]['source_stamps_ns'][-1]
    assert all(row['target_epoch'] == first['target_epoch'] and row['deadline_ns'] == first['deadline_ns']
               and row['plan_sha256'] == first['plan_sha256'] for row in publications)


@pytest.mark.parametrize('timeout', [True, False, '3', 0., -1., 10.001, float('nan'), float('inf')])
def test_capture_budget_is_independent_finite_and_bounded(transport, timeout):
    node, now, _, plan = transport
    first = node._observation_measurement_begin(plan)
    now[0] = 10_000_000_000
    node._last_motion_settle_sample = arrived(first)['settle']
    with pytest.raises(ValueError, match='CAPTURE_BUDGET_INVALID'):
        node._observation_measurement_finish(plan, dict(unknown_observe_return_requalification_timeout_sec=timeout))


def test_epoch_change_during_wait_retires_receipt_instead_of_accepting_new_frames(transport):
    node, _, _, plan = transport
    first = node._observation_measurement_begin(plan)
    node._following_epoch_ns += 1
    with pytest.raises(ValueError, match='CONTROL_EPOCH_CHANGED'):
        node._observation_measurement_live(first)


from test_observe_return_task import context, begin, evidence


def test_ordinary_return_runs_same_handoff_only_after_original_return_proof(context):
    c = context
    begin(c)
    c.node._observation_measurement_scope = lambda plan: True
    events = []
    c.node._observation_measurement_begin = lambda plan, kind: events.append(('begin', kind))
    c.node._wait_for_motion_settle = lambda label: events.append(('settle', label)) or True
    c.node._observation_measurement_finish = lambda plan, cfg: events.append(('qualified', plan.plan_id))
    _, _, contact = evidence('initial_contact')
    contact.header.stamp = task.rospy.Time(23)
    c.now[0] = 24_000_000_000
    result = c.node._before_near_field_contact_freeze(contact, c.plan, {'unknown_observe_return_return_to_origin': True})
    assert result.get('retry'), c.states
    assert events == [('begin', 'return'), ('settle', 'observe-return measured origin'),
                      ('qualified', c.plan.plan_id)]
    assert c.node._observe_return_session['phase'] == 'returned'


def test_invalid_return_proof_never_enters_measurement_recovery(context):
    c = context
    begin(c)
    c.node._observation_measurement_scope = lambda plan: True
    c.node._observation_measurement_begin = lambda *a: None
    c.node._observation_measurement_finish = lambda *a: pytest.fail('invalid proof reached recovery')
    c.transport['return_proof'] = dict(c.proof, code='OBSERVE_RETURN_MEASURED_ARRIVAL',
        post_command_feedback_valid=False)
    _, _, contact = evidence('initial_contact')
    c.now[0] = 24_000_000_000
    assert c.node._before_near_field_contact_freeze(contact, c.plan, {'unknown_observe_return_return_to_origin': True}) == {'failed': True}
