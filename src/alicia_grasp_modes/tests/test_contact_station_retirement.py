"""Exact contact handoff removes stationary privilege, never visual checks."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock
import json

import pytest
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from alicia_grasp_modes.contact_station_retirement import parse_request, ack, validate_ack
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from test_observation_arrival_station import completed
from test_observation_measurement_unknown import next_frame
from test_locked_rgb_hold import module, latest_status


def producer_context(monkeypatch):
    node, clock, feed, joint, params, marker = completed(monkeypatch)
    measurement = node._observation_measurement_status['request']
    phase = NS(active=True, planning_stage='contact_replan', phase_id=1,
        header=NS(stamp=module.rospy.Time.from_sec(clock[0]-.1)),
        deadline=module.rospy.Time.from_sec(clock[0]+10), reference_target_track_id=measurement['target_track_id'])
    node._station_compute_phase = phase
    node._station_compute_task_state = NS(state='MOVE_PREGRASP', active=True)
    node._contact_station_ack_pub = Mock()
    now = module.rospy.Time.now().to_nsec()
    req = dict(version=1, state='prepare', request_id='contact:1', selection=measurement['selection'],
        session_id=measurement['session_id'], target_track_id=measurement['target_track_id'],
        target_epoch=measurement['target_epoch'], control_epoch_ns=measurement['control_epoch_ns'],
        measurement_sha256=measurement_digest(measurement), plan_id='contact-plan', plan_sha256='b'*64,
        plan_source_ns=node.last_good_source_stamp_ns, phase_id=1, phase_started_ns=phase.header.stamp.to_nsec(),
        stage='MOVE_PREGRASP', stamp_ns=now, deadline_ns=now+800_000_000,
        metric_geometry_authority=False, motion_authorized=False, controller_motion_confirmed=False)
    return NS(node=node, clock=clock, feed=feed, joint=joint, params=params, marker=marker, req=req, phase=phase)


def send(node, req):
    node._contact_station_request_cb(String(data=json.dumps(req)))
    return json.loads(node._contact_station_ack_pub.publish.call_args[0][0].data)


def retirement(req):
    return dict(req, state='retire', prepared_acks={p:ack(req, p) for p in ('unknown', 'remote')})


def test_exact_two_owner_handshake_only_retires_station_then_fresh_rgbd_tracks(monkeypatch):
    c = producer_context(monkeypatch)
    anchor, source = c.node.tracker.anchor, c.node.last_good_source_stamp_ns
    count = c.node.pubs['object'].publish.call_count
    assert send(c.node, c.req)['accepted']
    assert c.node._locked_rgb_hold_reference['observation_arrival_retry'] is c.marker
    assert send(c.node, retirement(c.req))['accepted']
    assert 'observation_arrival_retry' not in c.node._locked_rgb_hold_reference
    assert c.node.pubs['object'].publish.call_count == count
    assert c.node.last_good_source_stamp_ns == source
    next_frame(c.node, c.clock, c.joint, q=[.006, 0., 0., 0., 0., 0.])
    assert latest_status(c.node)['state'] == 'ready', latest_status(c.node)
    assert c.node.tracker.anchor is anchor and not c.node.tracker.lost
    assert c.node.last_good_source_stamp_ns > source
    assert 'observation_arrival_retry' not in (c.node._locked_rgb_hold_reference or {})


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'selection', 'track', 'plan_source', 'phase',
    'stage', 'lost', 'config', 'camera', 'movement', 'out_and_back', 'stale'])
def test_preparation_never_hides_real_station_fault(monkeypatch, fault):
    c = producer_context(monkeypatch)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': c.node._rgb_interest_epoch += 1
    elif fault == 'selection': c.params['/grasp_mode/selection']['generation'] += 1
    elif fault == 'track': c.req['target_track_id'] = 'wrong'
    elif fault == 'plan_source': c.req['plan_source_ns'] -= 1_000_000_000
    elif fault == 'phase': c.phase.phase_id += 1
    elif fault == 'stage': c.node._station_compute_task_state.state = 'APPROACH_TARGET'
    elif fault == 'lost': c.node.tracker.lost = True
    elif fault == 'config': c.node.config.support_threshold_m += .001
    elif fault == 'camera': c.params['/camera']['fx'] += 1
    elif fault in ('movement', 'out_and_back'):
        c.joint(c.clock[0], [.005, 0., 0., 0., 0., 0.])
        if fault == 'out_and_back': c.joint(c.clock[0], [0.] * 6)
    else: c.clock[0] += .81
    before = c.node._contact_station_ack_pub.publish.call_count
    c.node._contact_station_request_cb(String(data=json.dumps(c.req)))
    if c.node._contact_station_ack_pub.publish.call_count > before:
        assert not json.loads(c.node._contact_station_ack_pub.publish.call_args[0][0].data)['accepted']
    assert getattr(c.node, '_contact_station_version', 0) == 0


@pytest.mark.parametrize('fault', ['ack', 'plan', 'epoch', 'q', 'manual', 'replay'])
def test_commit_rechecks_binding_after_prepare(monkeypatch, fault):
    c = producer_context(monkeypatch)
    assert send(c.node, c.req)['accepted']
    req = retirement(c.req)
    if fault == 'ack': req['prepared_acks']['remote']['accepted'] = False
    elif fault == 'plan': req['plan_sha256'] = 'c'*64
    elif fault == 'epoch': c.node._rgb_interest_epoch += 1
    elif fault == 'q': c.joint(c.clock[0], [.006, 0., 0., 0., 0., 0.])
    elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    else:
        assert send(c.node, req)['accepted']
        before = getattr(c.node, '_contact_station_version', 0)
        c.node._contact_station_request_cb(String(data=json.dumps(req)))
        assert c.node._contact_station_version == before
        return
    before = c.node._contact_station_ack_pub.publish.call_count
    c.node._contact_station_request_cb(String(data=json.dumps(req)))
    if c.node._contact_station_ack_pub.publish.call_count > before:
        assert not json.loads(c.node._contact_station_ack_pub.publish.call_args[0][0].data)['accepted']
    assert getattr(c.node, '_contact_station_version', 0) == 0


@pytest.mark.parametrize('retire', [False, True])
def test_gen14_tf_window_joint_callback_race_requires_exact_handoff(monkeypatch, retire):
    from test_observation_arrival_frame_completion_guards import pending_source
    c = producer_context(monkeypatch)
    pending_source(c.node, c.clock, c.joint)
    original = c.node.buffer.lookup_transform
    count, source = c.node.pubs['object'].publish.call_count, c.node.last_good_source_stamp_ns
    injected = [False]
    def during_tf(*args, **kwargs):
        result = original(*args, **kwargs)
        if injected[0]:
            return result
        injected[0] = True
        assert c.node._arrival_frame_completion is not None
        if retire:
            assert send(c.node, c.req)['accepted']
            assert send(c.node, retirement(c.req))['accepted']
        c.joint(c.clock[0], [.005, 0., 0., 0., 0., 0.])
        return result
    monkeypatch.setattr(c.node.buffer, 'lookup_transform', during_tf)
    c.node.process(None)
    if retire:
        assert not c.node.tracker.lost and c.node.first_loss is None
        assert c.node.pubs['object'].publish.call_count == count
        assert c.node.last_good_source_stamp_ns == source
    else:
        assert c.node.tracker.lost
        assert 'station_frame_completion_qualification_failed' in c.node.first_loss['reason']


def test_retirement_does_not_hide_later_missing_target(monkeypatch):
    c = producer_context(monkeypatch)
    assert send(c.node, c.req)['accepted'] and send(c.node, retirement(c.req))['accepted']
    c.clock[0] += .2
    c.feed(target_depth=0.)
    assert not c.node.pubs['object'].publish.call_args[0][0].detected


def test_unknown_one_shot_is_outside_handoff_scope():
    from alicia_grasp_modes.contact_station_task import ContactStationTaskMixin
    node = ContactStationTaskMixin()
    plan = NS(diagnostic='CONTACT_EXECUTION_PLAN', model_choice='unknown_tabletop')
    assert not node._contact_station_scope(plan, GraspStages.MOVE_PREGRASP)


def remote_context(c):
    from test_unknown_pending_identity import consumer_for, publication, deliver
    from test_remote_modes import base
    node = consumer_for(c.node, c.clock, c.params)
    node._ros_source_clock_ns = lambda: module.rospy.Time.now().to_nsec()
    deliver(node, publication(c.node))
    identity = node._current_stream_target_identity()
    measurement = deepcopy(c.node._observation_measurement_status['request'])
    measurement.update(target_epoch=identity.epoch, target_track_id=identity.track_id)
    req = dict(c.req, target_epoch=identity.epoch, target_track_id=identity.track_id,
               measurement_sha256=measurement_digest(measurement))
    node._measurement_remote_request = measurement
    node._measurement_remote_epoch_ns = req['control_epoch_ns']
    node._observe_return_session = dict(phase='observing', session_id=req['session_id'],
        deadline_ns=measurement['deadline_ns'])
    node._observe_return_phase = 'observing_contact'
    node._observe_return_diagnostic_observation_only = False
    node._station_compute_task_state = 'MOVE_PREGRASP'
    node.robot_execution_active = True
    node._station_compute_phase = deepcopy(c.phase)
    node._station_compute_phase.reference_target_track_id = identity.track_id
    node._contact_station_ack_pub = Mock()
    plan = base.Grasp6DPlan()
    plan.valid, plan.plan_id, plan.diagnostic = True, req['plan_id'], 'CONTACT_EXECUTION_PLAN'
    plan.model_choice, plan.target_track_id = 'unknown_tabletop', identity.track_id
    plan.header.stamp = module.rospy.Time(req['plan_source_ns']//10**9, req['plan_source_ns']%10**9)
    plan.header.frame_id = 'base_link'
    plan.poses = [Pose() for _ in range(4)]
    node.latest_rich_plan = plan
    req['plan_sha256'] = committed_plan_digest(plan)
    return node, req, plan


@pytest.mark.parametrize('fault', [None, 'plan_digest', 'plan_source', 'track', 'phase', 'manual', 'epoch',
                                  'pending', 'lost', 'state', 'measurement', 'retired'])
def test_remote_requires_actual_published_plan_and_qualified_identity(monkeypatch, fault):
    c = producer_context(monkeypatch)
    node, req, plan = remote_context(c)
    if fault == 'plan_digest': plan.poses[0].position.x += .01
    elif fault == 'plan_source': plan.header.stamp = module.rospy.Time(2)
    elif fault == 'track': plan.target_track_id = 'wrong'
    elif fault == 'phase': node._station_compute_phase.phase_id += 1
    elif fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': node._measurement_remote_epoch_ns += 1
    elif fault == 'pending': node._station_compute_pending = True
    elif fault == 'lost': node._unknown_identity_retired_selection = node._mode_selection
    elif fault == 'state': node._station_compute_task_state = 'APPROACH_TARGET'
    elif fault == 'measurement': node._measurement_remote_request['state'] = 'requalifying'
    elif fault == 'retired': node._contact_station_retired_measurement = req['measurement_sha256']
    response = send(node, req)
    assert response['accepted'] is (fault is None), response
    if fault is None:
        assert send(node, retirement(req))['accepted']
        assert node._contact_station_retired_measurement == req['measurement_sha256']
        assert not node._station_compute_retired
        assert node._unknown_identity_retired_selection is None
        assert node._station_compute_reference is None


@pytest.mark.parametrize('fault', [None, 'q_prepare', 'q_retire', 'epoch', 'cancel', 'rejected_ack', 'execute_failed'])
def test_actual_task_wrapper_never_submits_after_failed_handoff(monkeypatch, fault):
    import threading
    from alicia_grasp_modes.observe_return_task import ObserveReturnTask
    from alicia_grasp_modes.rgb_observation_source import JOINT_NAMES
    c = producer_context(monkeypatch)
    _, req, plan = remote_context(c)
    calls = []
    class Backend:
        def _plan_and_execute_pose(self, *args, **kwargs):
            calls.append('preflight')
            executor = self._wrap_pose_submission_executor(args[0],
                kwargs['execution_plan'], kwargs['gcfg'], kwargs['execute_pose'])
            return executor(args[2], True).success
    class Task(ObserveReturnTask, Backend):
        pass
    node = Task()
    node._observe_return_condition = threading.Condition(threading.RLock())
    node._contact_station_acks = {}
    node._observation_measurement_request = deepcopy(c.node._observation_measurement_status['request'])
    node._observation_measurement_request.update(target_track_id=plan.target_track_id)
    node._mode_bound_selection = deepcopy(c.req['selection'])
    node._published_near_field_phase = deepcopy(c.phase)
    node._published_near_field_phase.reference_target_track_id = plan.target_track_id
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = req['control_epoch_ns']
    row = JointState()
    row.header.frame_id, row.header.stamp = 'sdk_measured', module.rospy.Time.now()
    row.name, row.position = list(JOINT_NAMES), [0.] * 6
    node._following_accepted_history = [row]
    node._observe_return_live = lambda: None
    node._observation_measurement_live = lambda value: None
    node._observation_measurement_scope = lambda value: False
    node._validate_bound_plan_locked = lambda *a: NS(ok='cancel' not in calls, code='CANCELLED', reason='cancel')
    node.set_state = lambda *a: calls.append(('state', a))
    def publish(message):
        request = json.loads(message.data)
        calls.append(request['state'])
        if ((fault == 'q_prepare' and request['state'] == 'prepare')
                or (fault == 'q_retire' and request['state'] == 'retire')):
            node._following_accepted_history[-1].position[0] += .006
        if request['state'] == 'retire' and fault == 'epoch': node._following_epoch_ns += 1
        if request['state'] == 'retire' and fault == 'cancel': calls.append('cancel')
        for p in ('unknown', 'remote'):
            response = ack(request, p, accepted=not (fault == 'rejected_ack' and p == 'remote'))
            node._contact_station_ack_cb(String(data=json.dumps(response)))
    node._contact_station_pub = NS(publish=publish)
    def execute(*args):
        calls.append('strict_submit')
        return NS(success=fault != 'execute_failed')
    result = node._plan_and_execute_pose(GraspStages.MOVE_PREGRASP, 'contact', NS(), None, 'settle',
        execution_plan=plan, gcfg={}, execute_pose=execute)
    assert ('strict_submit' in calls) is (fault in (None, 'execute_failed')), calls
    assert result is (fault is None), calls
    if fault is None:
        assert calls[:4] == ['preflight', 'prepare', 'retire', 'strict_submit']


@pytest.mark.parametrize('participant', ['unknown', 'remote'])
@pytest.mark.parametrize('later', ['MOVE_PREGRASP', 'FAILED', 'timeout'])
def test_prepare_may_precede_active_motion_state_without_retiring_early(monkeypatch, participant, later):
    import alicia_grasp_modes.contact_station_retirement as contract
    c = producer_context(monkeypatch)
    if participant == 'remote':
        node, req, _ = remote_context(c)
        node._station_compute_task_state = 'PLAN_PREGRASP'
    else:
        node, req = c.node, c.req
        node._station_compute_task_state.state = 'PLAN_PREGRASP'
    before = getattr(node, '_contact_station_version', 0)
    def sleep(_duration):
        assert getattr(node, '_contact_station_version', 0) == before
        assert node._contact_station_ack_pub.publish.call_count == 0
        if later == 'timeout': c.clock[0] += .81
        elif participant == 'remote': node._station_compute_task_state = later
        else: node._station_compute_task_state.state = later
    monkeypatch.setattr(contract.time, 'sleep', sleep)
    response = send(node, req)
    assert response['accepted'] is (later == 'MOVE_PREGRASP'), response
    assert getattr(node, '_contact_station_version', 0) == before


def spatial_context(monkeypatch):
    c = producer_context(monkeypatch)
    assert send(c.node, c.req)['accepted']
    assert send(c.node, retirement(c.req))['accepted']
    assert c.node._contact_spatial_context is not None
    return c


def test_contact_spatial_proposal_uses_new_depth_and_never_arrival_permissions(monkeypatch):
    c = spatial_context(monkeypatch)
    context = c.node._contact_spatial_context
    assert set(context['spatial_reference']) == {'footprint_base', 'support_plane_base', 'height_m'}
    assert not context['spatial_reference']['footprint_base'].flags.writeable
    assert not context['spatial_reference']['support_plane_base'].flags.writeable
    source_before = c.node.last_good_source_stamp_ns
    calls = []
    original = module.refine_candidate
    def capture(*args, **kwargs):
        calls.append(kwargs)
        return original(*args, **kwargs)
    monkeypatch.setattr(module, 'refine_candidate', capture)
    # More than the completed handshake's .8 sec, still inside its unchanged
    # finite contact phase. Every intermediate frame supplies new geometry.
    for index in range(6):
        next_frame(c.node, c.clock, c.joint, q=[.006*(index+1), 0., 0., 0., 0., 0.])
        assert latest_status(c.node)['state'] == 'ready', latest_status(c.node)
    assert c.node.last_good_source_stamp_ns > source_before + 800_000_000
    assert calls and all(k.get('contact_spatial_reference') is context['spatial_reference']
                         and not k.get('observation_arrival_retry', False) for k in calls)
    assert c.node._arrival_frame_completion is None
    assert 'observation_arrival_retry' not in (c.node._locked_rgb_hold_reference or {})
    assert c.node._contact_spatial_context is context


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'selection', 'track', 'phase', 'phase_start',
    'stage', 'inactive', 'lost', 'config', 'camera', 'anchor', 'plane', 'model', 'deadline', 'source_age'])
def test_contact_spatial_owner_fault_permanently_revokes_without_geometry_authority(monkeypatch, fault):
    c = spatial_context(monkeypatch)
    context = c.node._contact_spatial_context
    source = c.node.latest
    before = c.node.pubs['object'].publish.call_count
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': c.node._rgb_interest_epoch += 1
    elif fault == 'selection': c.params['/grasp_mode/selection']['generation'] += 1
    elif fault == 'track': c.phase.reference_target_track_id = 'wrong'
    elif fault == 'phase': c.phase.phase_id += 1
    elif fault == 'phase_start': c.phase.header.stamp += module.rospy.Duration(.01)
    elif fault == 'stage': c.node._station_compute_task_state.state = 'PLAN_PREGRASP'
    elif fault == 'inactive': c.node._station_compute_task_state.active = False
    elif fault == 'lost': c.node.tracker.lost = True
    elif fault == 'config': c.node.config.support_threshold_m += .001
    elif fault == 'camera': c.params['/camera']['fx'] += 1
    elif fault == 'anchor': c.node.tracker.anchor.position_base[0] += .001
    elif fault == 'plane': c.node.tracker.plane_base[3] += .001
    elif fault == 'model': c.params['/robot_description'] = '<robot name="new"/>'
    elif fault == 'deadline': c.phase.deadline = module.rospy.Time.now()
    else: c.clock[0] += .81
    with pytest.raises(ValueError, match='revoked_before_commit'):
        c.node._contact_spatial_reference(source, c.params['/camera'], expected=context)
    assert c.node._contact_spatial_context is None
    assert c.node.pubs['object'].publish.call_count == before
    assert not context['binding']['metric_geometry_authority']


@pytest.mark.parametrize('change', ['PLAN_PREGRASP', 'CLOSE_GRIPPER', 'STOP', 'phase', 'epoch'])
def test_contact_spatial_callback_revoke_cannot_be_revived_by_restoring_state(monkeypatch, change):
    from std_msgs.msg import Header
    c = spatial_context(monkeypatch)
    if change == 'phase':
        phase = deepcopy(c.phase); phase.active = False
        c.node._station_compute_phase_cb(phase)
        c.node._station_compute_phase_cb(c.phase)
    elif change == 'epoch':
        c.node._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
    else:
        c.node._station_compute_state_cb(NS(state=change, active=True))
        c.node._station_compute_state_cb(NS(state='MOVE_PREGRASP', active=True))
    assert c.node._contact_spatial_context is None


def test_contact_spatial_scope_is_rechecked_after_rgb_before_publish(monkeypatch):
    c = spatial_context(monkeypatch)
    original = module.refine_candidate
    def cancel(*args, **kwargs):
        result = original(*args, **kwargs)
        c.node._station_compute_state_cb(NS(state='STOP', active=False))
        return result
    monkeypatch.setattr(module, 'refine_candidate', cancel)
    source = c.node.last_good_source_stamp_ns
    next_frame(c.node, c.clock, c.joint, q=[.006,0.,0.,0.,0.,0.])
    assert c.node.last_good_source_stamp_ns == source
    assert not c.node.pubs['object'].publish.call_args[0][0].detected
    assert 'contact_spatial_context_revoked_before_commit' in latest_status(c.node)['detail']


@pytest.mark.parametrize('arrival,observation', [(True, None), (False, {})])
def test_contact_spatial_proposal_cannot_be_combined_with_arrival_authority(monkeypatch, arrival, observation):
    c = spatial_context(monkeypatch)
    captured=[]
    original=module.refine_candidate
    def capture(*args, **kwargs):
        captured.append((args, kwargs))
        return original(*args, **kwargs)
    monkeypatch.setattr(module, 'refine_candidate', capture)
    next_frame(c.node,c.clock,c.joint,q=[.006,0.,0.,0.,0.,0.])
    args,kwargs=captured[-1]
    kwargs=dict(kwargs,observation_arrival_retry=arrival,observation_spatial_reference=observation)
    with pytest.raises(ValueError,match='contact_spatial_reference_is_not_arrival_authority'):
        original(*args,**kwargs)
