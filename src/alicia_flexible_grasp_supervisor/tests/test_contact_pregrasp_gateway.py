"""Actual service/callback flow with explicit synthetic planner/physics doubles.

Recorded numerical selection and real continuous-geometry validators have their
own tests. These cases check ownership, immutable output and admission ordering.
"""
from copy import deepcopy
import json
import threading
import time
from types import MethodType, SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from test_pregrasp_gateway import make_gateway
from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module
from alicia_flexible_grasp.robot.pregrasp_gateway import PregraspCompensationGateway
from alicia_flexible_grasp.robot.observation_path_guard import CommittedObservationContexts
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages


def path(start, target):
    p = RobotTrajectory(); p.joint_trajectory.joint_names = list(ARM_NAMES)
    p.joint_trajectory.points = [
        JointTrajectoryPoint(positions=list(start), time_from_start=rospy.Duration(0)),
        JointTrajectoryPoint(positions=list(target), time_from_start=rospy.Duration(2))]
    return p


def case(monkeypatch, *, receipt=True):
    node, planner, plan, state, calls, params = make_gateway(monkeypatch)
    old_context = node._pregrasp_context(plan)
    plan.model_choice = 'unknown_tabletop'
    plan.header.stamp = rospy.Time.from_sec(2.)
    plan.object_geometry.header = deepcopy(plan.header)
    plan.plan_id = compute_plan_id(plan)
    cfg = dict(measured_endpoint_position_tolerance_m=.006,
        unknown_measured_endpoint_position_tolerance_m=.010,
        measured_endpoint_orientation_tolerance_deg=5.,
        pregrasp_cartesian_compensation_enabled=True,
        unknown_pregrasp_feedforward_enabled=True,
        unknown_pregrasp_held_axes_feedforward_enabled=True,
        unknown_approach_held_axes_feedforward_enabled=True,
        unknown_contact_cartesian_compensation_enabled=True,
        measured_endpoint_check_enabled=True, contact_endpoint_precision_enabled=True)
    params.update({'/robot_description':old_context['model'], '/grasp':cfg,
        '/grasp_mode/selection':dict(mode='unknown',generation=20,stamp_ns=1_500_000_000),
        '/grasp_6d/remote/gripper_geometry':{}, '/gripper':{},
        '/robot/observation_tracking_contract_enabled':True})
    node._observation_context_lock = threading.RLock()
    node._observation_contexts = CommittedObservationContexts()
    node._observation_contexts.set_active(True)
    node._control_reference_epoch_ns = 1_000_000_000
    node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP
    node._compensation_task_label = 'planning 6D pregrasp'
    node._manual_control_active = lambda:state.get('manual',False)
    node._require_live_observation_task = mock.Mock()
    node._fresh_actuation_status = lambda:'CONFIRMED:test'
    node._pregrasp_context = MethodType(PregraspCompensationGateway._pregrasp_context,node)
    node._validate_pregrasp_context = MethodType(PregraspCompensationGateway._validate_pregrasp_context,node)
    node._unknown_pregrasp_velocity_limits = lambda _planner:{'Joint1':.08}
    node._validate_frozen_observation_path = mock.Mock(return_value={
        'controller_constraints_snapshot':{'goal_time':1.}})
    node._contact_command_collision_check = mock.Mock(return_value={'checked':True})
    robot = RobotState(); robot.joint_state.name = list(ARM_NAMES)+['right_finger']
    robot.joint_state.position = state['actual'].tolist()+[.04975]
    planner.robot = NS(get_current_state=lambda:deepcopy(robot))
    target = state['sdk']+np.array([4,0,-4,2,0,4])*Q
    planner._plan_pose_from_start_state = mock.Mock(side_effect=lambda start, _goal, **_:
        (path(start.joint_state.position[:6], state['sdk']), 'seed'))
    planner._plan_joint_from_start_state = mock.Mock(side_effect=lambda start, q, **_:
        (path(start.joint_state.position[:6],q), 'exact'))
    selection = mock.Mock(side_effect=lambda *_a,**_k:(target.copy(),{'reselected':True}))
    monkeypatch.setattr(module,'select_held_pregrasp_endpoint',selection)
    def proof(*args):
        args[-1]['success']=True
    monkeypatch.setattr(module,'predict_held_approach',mock.Mock(side_effect=proof))
    curve = mock.Mock(return_value={'command':{},'predicted_actual':{}})
    monkeypatch.setattr(module,'validate_selected_pregrasp_path',curve)
    original = planner.plan_and_execute_joint_probe
    def execute(q,execute,**kwargs):
        planner.observation_path_validator(path(kwargs['start_joint_positions'],q))
        return original(q,execute,**kwargs)
    planner.plan_and_execute_joint_probe = mock.Mock(side_effect=execute)
    monkeypatch.setattr(module.rospy,'is_shutdown',lambda:False)
    if receipt:node._committed_observation_plan_cb(deepcopy(plan))
    return NS(node=node,planner=planner,plan=plan,state=state,calls=calls,params=params,
              target=target,select=selection,curve=curve)


def test_readonly_and_execute_recompute_fresh_goal_keep_complete_path_and_arrival_separate(monkeypatch):
    c=case(monkeypatch); fixed=deepcopy(c.plan)
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False))
    assert r.success,r.message
    assert json.loads(r.message)['code']=='CONTACT_PREGRASP_CHECKED_ONLY'
    assert c.state['moves']==0 and not c.calls and c.planner._last_pose_plan is None
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert r.success,r.message
    report=json.loads(r.message)
    assert report['code']=='CONTACT_PREGRASP_EXECUTED'
    assert report['complete_execution_path_checked'] is True
    assert report['endpoint_confirmation_required'] is True
    assert not report['hardware_response_verified'] and not report['real_grasp_success']
    assert c.select.call_count==2 and c.planner._plan_pose_from_start_state.call_count==2
    assert c.curve.call_count==2 and c.node._validate_frozen_observation_path.call_count==1
    np.testing.assert_array_equal(c.calls[0][0],c.target)
    assert c.state['moves']==1 and c.plan==fixed and c.planner._last_pose_plan is None
    assert c.planner.observation_path_guard_required is False
    assert c.planner.observation_tracking_contract_required is False
    assert not c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True)).success
    assert c.state['moves']==1


def test_late_actual_plan_callback_releases_service_without_renewing_deadline(monkeypatch):
    c=case(monkeypatch,receipt=False)
    waiting=threading.Event(); context=[]
    original=c.node._wait_for_contact_pregrasp_receipt
    def wait(ctx):
        context.append((ctx,ctx['deadline']));waiting.set();return original(ctx)
    c.node._wait_for_contact_pregrasp_receipt=wait
    result=[]
    worker=threading.Thread(target=lambda:result.append(c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False))))
    worker.start()
    assert waiting.wait(1.)
    c.node._committed_observation_plan_cb(deepcopy(c.plan))
    worker.join(2.)
    assert not worker.is_alive() and len(result)==1
    assert result[0].success,result[0].message
    assert context[0][0]['deadline']==context[0][1]
    assert c.state['moves']==0


@pytest.mark.parametrize('field',['plan_sha256','source_ns','epoch','selection'])
def test_stale_or_wrong_published_receipt_never_selects_or_moves(monkeypatch,field):
    c=case(monkeypatch)
    receipt=c.node._contact_pregrasp_receipts[c.plan.plan_id]
    receipt[field]='wrong' if field in ('plan_sha256','selection') else receipt[field]+1
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not r.success and 'PUBLISHED_PLAN_MISMATCH' in r.message
    c.select.assert_not_called();assert not c.calls and c.state['moves']==0


@pytest.mark.parametrize('fault',['epoch','manual','selection','plan','source','owner','config'])
def test_context_change_during_selection_cannot_plan_or_submit_old_goal(monkeypatch,fault):
    c=case(monkeypatch)
    def select(*_a,**_k):
        if fault=='epoch':c.node._control_reference_epoch_ns+=1
        if fault=='manual':c.state['manual']=True
        if fault=='selection':c.params['/grasp_mode/selection']=dict(mode='unknown',generation=21)
        if fault=='plan':c.node._compensation_visual_context['plan'].poses[0].position.x+=.001
        if fault=='source':c.node._compensation_visual_context['plan'].header.stamp=rospy.Time(1)
        if fault=='owner':c.node._pregrasp_task_stage=GraspStages.IDLE
        if fault=='config':c.params['/grasp']=dict(c.params['/grasp'],measured_endpoint_check_enabled=False)
        return c.target.copy(),{'reselected':True}
    c.select.side_effect=select
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not r.success,r.message
    assert not c.calls and c.state['moves']==0


@pytest.mark.parametrize('failure',['curve_collision','tracking_path','fixed_target','new_feedback','submit_failure'])
def test_failed_selected_path_or_changed_submission_never_uses_pose_fallback(monkeypatch,failure):
    c=case(monkeypatch)
    if failure=='curve_collision':c.curve.side_effect=ValueError('predicted_actual collision')
    if failure=='tracking_path':c.node._validate_frozen_observation_path.side_effect=ValueError('tracking envelope')
    original=c.planner.plan_and_execute_joint_probe.side_effect
    def execute(q,execute,**kw):
        if failure=='fixed_target':q=np.asarray(q)+Q
        if failure=='new_feedback':c.state['actual'][0]+=3*Q
        if failure=='submit_failure':
            c.planner.observation_path_validator(path(kw['start_joint_positions'],q))
            kw['before_execute']();return False,'controller refused'
        return original(q,execute,**kw)
    c.planner.plan_and_execute_joint_probe.side_effect=execute
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not r.success,r.message
    assert not c.calls and c.state['moves']==0
    if failure=='submit_failure':
        assert not c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True)).success


def test_missing_receipt_expires_without_reselect_or_parameter_write(monkeypatch):
    c=case(monkeypatch,receipt=False)
    original=c.node._pregrasp_context
    def context(*args):
        ctx=original(*args);ctx['deadline']=time.monotonic()+.025;return ctx
    c.node._pregrasp_context=context
    r=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not r.success,r.message
    c.select.assert_not_called();assert not c.calls
    assert not any(k.startswith('/supervisor/contact_pregrasp_executions') for k in c.params)


def test_invalid_published_plan_revokes_receipt_before_later_service(monkeypatch):
    c=case(monkeypatch);invalid=deepcopy(c.plan);invalid.valid=False
    c.node._committed_observation_plan_cb(invalid)
    assert c.node._contact_pregrasp_receipts=={}
    assert c.node._observation_contexts.blocked


def at_current_pose(c):
    from tf.transformations import quaternion_from_matrix
    fk=module.SerialUrdfFk(c.params['/robot_description'],ARM_NAMES)
    t=fk(c.state['actual']);p=c.plan.poses[0]
    p.position.x,p.position.y,p.position.z=t[:3,3]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quaternion_from_matrix(t)
    c.plan.plan_id=compute_plan_id(c.plan)
    c.node._committed_observation_plan_cb(deepcopy(c.plan))


def test_same_current_joint_start_uses_fresh_shared_proof_without_motion(monkeypatch):
    c=case(monkeypatch);at_current_pose(c)
    def proof(*args):
        args[-1].update(success=True,predicted_pregrasp={
            'predicted_command_positions_rad':list(args[2]['sdk_positions_rad'])})
    module.predict_held_approach.side_effect=proof
    for execute in (False,True):
        result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=execute))
        assert result.success,result.message
        report=json.loads(result.message)
        assert report['code']==('CONTACT_PREGRASP_ALREADY_SATISFIED' if execute else 'CONTACT_PREGRASP_CHECKED_ONLY')
        assert report['no_motion_submitted'] and report['current_joint_start_verified']
        assert not report['complete_execution_path_checked'] and not report['hardware_response_verified']
    assert module.predict_held_approach.call_count==4
    c.select.assert_not_called();c.planner._plan_pose_from_start_state.assert_not_called()
    assert c.state['moves']==0 and not c.calls


def test_shared_prediction_with_pose0_correction_cannot_claim_no_motion(monkeypatch):
    c=case(monkeypatch);at_current_pose(c)
    def proof(*args):
        q=np.asarray(args[2]['sdk_positions_rad']).copy();q[0]+=Q
        args[-1].update(success=True,predicted_pregrasp={'predicted_command_positions_rad':q.tolist()})
    module.predict_held_approach.side_effect=proof
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False))
    assert result.success,result.message
    assert json.loads(result.message).get('no_motion_submitted') is not True
    c.select.assert_called_once()


@pytest.mark.parametrize('change',['epoch','q','config'])
def test_current_joint_reuse_is_revalidated_after_readonly_call(monkeypatch,change):
    c=case(monkeypatch);at_current_pose(c)
    def proof(*args):
        args[-1].update(success=True,predicted_pregrasp={
            'predicted_command_positions_rad':list(args[2]['sdk_positions_rad'])})
    module.predict_held_approach.side_effect=proof
    assert c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False)).success
    if change=='epoch':c.node._control_reference_epoch_ns+=1
    elif change=='config':c.params['/grasp']['unknown_approach_held_axes_feedforward_enabled']=False
    else:
        original=c.node._endpoint_following_snapshot
        calls=[0]
        def snapshot(*a,**kw):
            calls[0]+=1
            if calls[0]==2:c.state['actual'][0]+=3*Q
            return original(*a,**kw)
        c.node._endpoint_following_snapshot=snapshot
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not result.success,result.message
    assert c.state['moves']==0 and not c.calls


@pytest.mark.parametrize('fault',['ingest_raises','ingest_clears','terminal'])
def test_receipt_is_cleared_with_rejected_ingest_or_terminal_owner(monkeypatch,fault):
    c=case(monkeypatch)
    if fault=='ingest_raises':
        c.node._observation_contexts.ingest=mock.Mock(side_effect=ValueError('invalid geometry'))
        c.node._committed_observation_plan_cb(deepcopy(c.plan))
    elif fault=='ingest_clears':
        c.node._observation_contexts.active_scene=NS(track_id='other')
        c.node._committed_observation_plan_cb(deepcopy(c.plan))
    else:c.node._revoke_observation_task()
    assert c.node._contact_pregrasp_receipts=={}
    if fault=='terminal':
        assert not c.node._observation_contexts.active and c.node._observation_task_requires_inactive
    else:assert c.node._observation_contexts.blocked


@pytest.mark.parametrize('fault',['manual','epoch','deadline'])
def test_late_success_after_execution_context_changed_is_rejected(monkeypatch,fault):
    c=case(monkeypatch);original=c.planner.plan_and_execute_joint_probe.side_effect
    def execute(*args,**kwargs):
        result=original(*args,**kwargs)
        if fault=='manual':c.state['manual']=True
        elif fault=='epoch':c.node._control_reference_epoch_ns+=1
        else:c.node._compensation_visual_context['deadline']=time.monotonic()-1.
        return result
    c.planner.plan_and_execute_joint_probe.side_effect=execute
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not result.success,result.message
    assert c.state['moves']==1
    assert json.loads(result.message)['code']=='CONTACT_PREGRASP_REJECTED'


def captured_session(c, monkeypatch, version=2):
    """Synthetic captured origin; the real registered callback validates v2."""
    from std_msgs.msg import String
    from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
    c.params['/grasp_mode/selection']['strategy']='observe_return'
    c.params['/grasp']['unknown_observe_return_session_version']=version
    observation=deepcopy(c.plan);observation.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    observation.header.stamp=rospy.Time.from_sec(1.8)
    observation.object_geometry.header=deepcopy(observation.header)
    observation.plan_id=compute_plan_id(observation)
    scene=FrozenObservationScene.from_plan(observation)
    origin=dict(scene=scene,plan=observation,selection=deepcopy(c.params['/grasp_mode/selection']),
        epoch=c.node._control_reference_epoch_ns,revocation=c.node._observation_contexts.revocation,
        identity='d'*64,deadline=time.monotonic()+120.,config=deepcopy(c.params['/grasp']),
        model=c.params['/robot_description'],session_id='captured-session')
    c.node._observe_return_origin=origin
    if version==2:
        raw=dict(version=2,phase='observing',selection=deepcopy(origin['selection']),
            session_id=origin['session_id'],origin_plan_id=scene.plan_id,
            origin_snapshot_stamp_ns=scene.source_ns,target_track_id=scene.track_id,target_epoch=0,
            stamp_ns=int(c.state['now']*1e9),deadline_ns=int((c.state['now']+15.)*1e9),
            max_observations=3,view_index=1,view_plan_id=scene.plan_id,
            view_source_stamp_ns=scene.source_ns,view_request_id='initial:'+scene.plan_id,
            previous_gain_sha256='0'*64,return_verified_stamp_ns=0,contact_deadline_ns=0)
        c.node._observe_return_view_session_cb(String(data=json.dumps(raw)))
        assert origin['view_session']['deadline_ns']==raw['deadline_ns']
    c.node._committed_observation_plan_cb(deepcopy(c.plan))
    return origin


def test_readonly_and_true_calls_keep_original_outer_session_deadline(monkeypatch):
    c=case(monkeypatch);origin=captured_session(c,monkeypatch)
    contexts=[];original=c.node._bind_contact_pregrasp_session
    def bind(context):
        original(context);contexts.append(context)
    c.node._bind_contact_pregrasp_session=bind
    first=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False))
    assert first.success,first.message
    c.state['now']+=2.
    second=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert second.success,second.message
    for result in (first,second):
        bound=json.loads(result.message)['outer_session_deadline']
        assert bound['session_id']==origin['session_id']
        assert bound['ros_deadline_ns']==origin['view_session']['deadline_ns']
    assert contexts[1]['deadline']<contexts[0]['deadline']
    assert c.state['moves']==1 and c.select.call_count==2


@pytest.mark.parametrize('timing',['selection','before_submit'])
def test_true_internal_work_crossing_outer_deadline_never_submits(monkeypatch,timing):
    c=case(monkeypatch);origin=captured_session(c,monkeypatch)
    assert c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False)).success
    expire=lambda:c.state.update(now=origin['view_session']['deadline_ns']*1e-9+.01)
    if timing=='selection':
        original=c.select.side_effect
        def select(*a,**kw):
            value=original(*a,**kw);expire();return value
        c.select.side_effect=select
    else:
        original=c.node._validate_frozen_observation_path
        def guard(*a,**kw):
            value=original(*a,**kw);expire();return value
        c.node._validate_frozen_observation_path=guard
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not result.success,result.message
    assert c.state['moves']==0 and not c.calls
    assert not any(k.startswith('/supervisor/contact_pregrasp_executions') for k in c.params)


@pytest.mark.parametrize('change',['deadline','session','origin','epoch','revocation','track','missing_v2'])
def test_captured_outer_session_cannot_be_replaced_or_extended(monkeypatch,change):
    c=case(monkeypatch);origin=captured_session(c,monkeypatch)
    if change=='missing_v2':origin.pop('view_session')
    else:
        original=c.select.side_effect
        def select(*a,**kw):
            value=original(*a,**kw)
            if change=='deadline':origin['deadline']+=120.
            elif change=='session':origin['view_session']['session_id']='other-session'
            elif change=='origin':c.node._observe_return_origin=deepcopy(origin)
            elif change=='epoch':origin['epoch']+=1
            elif change=='revocation':origin['revocation']+=1
            elif change=='track':
                origin['scene']=NS(track_id='other',digest=origin['scene'].digest)
            return value
        c.select.side_effect=select
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=True))
    assert not result.success,result.message
    assert c.state['moves']==0 and not c.calls


def test_v1_origin_uses_its_earlier_immutable_deadline(monkeypatch):
    c=case(monkeypatch);origin=captured_session(c,monkeypatch,version=1)
    origin['deadline']=time.monotonic()+20.
    result=c.node.handle_contact_pregrasp(NS(plan=c.plan,execute=False))
    assert result.success,result.message
    bound=json.loads(result.message)['outer_session_deadline']
    assert bound['monotonic_deadline']==origin['deadline']
    assert bound['ros_deadline_ns'] is None
