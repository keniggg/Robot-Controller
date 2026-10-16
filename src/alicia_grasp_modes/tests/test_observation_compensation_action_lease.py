"""Actual task MRO/invoke/validation chain; RGB geometry and RPC are fixtures.

The GEN5 31.914-second observation age is transplanted, not represented as an
exact reconstruction of the unrecorded task-private last-object timestamp.
"""
from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import pytest
import rospy

WORK=Path.cwd()
SOURCE=Path(os.environ.get('GEN5_SOURCE_ROOT',str(WORK)))
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path))
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module
    spec.loader.exec_module(module);return module
base=load('_alicia_original_grasp_task',SOURCE/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py')
task=load('alicia_grasp_modes.observe_return_task',SOURCE/'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py')
mode=load('_gen5_mode_task',WORK/'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py')
import test_grasp_task_sequence as factory_module
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest


@pytest.fixture
def scene(monkeypatch):
    factory=factory_module.GraspTaskSequenceTest()
    now=[40.55]
    selection=dict(mode='unknown',strategy='observe_return',generation=5,stamp_ns=1_000_000_000)
    params={mode.SELECTION_PARAM:selection,'/grasp_6d/target_observation_validity_sec':8.}
    monkeypatch.setattr(rospy,'get_param',lambda k,d=None:deepcopy(params.get(k,d)))
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(now[0])))
    monkeypatch.setattr(rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(rospy,'wait_for_service',lambda *a,**k:None)
    monkeypatch.setattr(rospy,'loginfo',lambda *a,**k:None)
    n=mode.ModeAwareGraspTaskNode.__new__(mode.ModeAwareGraspTaskNode)
    n._initialize_mode_boundary();n._mode_selection=deepcopy(selection)
    n._mode_bound_selection=deepcopy(selection);n._mode_ready=True;n.active=True
    plan=factory._rich_plan(stamp_sec=8.)
    plan.model_choice='unknown_tabletop';plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    plan.plan_id=compute_plan_id(plan);n._freeze_execution_plan(plan)
    n.latest_grasp6d_plan=deepcopy(plan)
    n.latest_obj=factory._object(stamp_sec=now[0]-31.914)
    n.latest_obj_time=n.latest_obj.header.stamp
    n.latest_target_geometry=deepcopy(plan.object_geometry)
    n._following_evidence_lock=threading.RLock();n._following_epoch_ns=2_000_000_000
    n._observe_return_origin_plan=deepcopy(plan);n._observe_return_active_view_plan=deepcopy(plan)
    n._observe_return_origin_proof=dict(code='OBSERVE_RETURN_ORIGIN_CAPTURED',physical_motion_submitted=False,
        session_id='session',plan_id=plan.plan_id,target_track_id=plan.target_track_id,
        plan_digest=committed_plan_digest(plan),model_sha256='b'*64,origin_goal_sha256='c'*64,
        epoch_ns=n._following_epoch_ns,origin_accepted_stamp_ns=8_900_000_000,
        origin_joint_positions_rad=[0.]*6,origin_tool_pose=dict(frame_id='base_link',
            position_m=[.1,0.,.2],quaternion_xyzw=[0.,0.,0.,1.]))
    session=dict(version=2,phase='observing',selection=deepcopy(selection),session_id='session',
        target_epoch=4,target_track_id=plan.target_track_id,origin_plan_id=plan.plan_id,
        origin_snapshot_stamp_ns=plan.header.stamp.to_nsec(),view_plan_id=plan.plan_id,
        view_source_stamp_ns=plan.header.stamp.to_nsec(),view_index=1,deadline_ns=608_300_000_000)
    n._observe_return_session=deepcopy(session)
    receipt=dict(version=1,state='executing',selection=deepcopy(selection),session_id='session',
        target_epoch=4,target_track_id=plan.target_track_id,origin_plan_id=plan.plan_id,
        origin_snapshot_stamp_ns=plan.header.stamp.to_nsec(),view_plan_id=plan.plan_id,
        view_source_stamp_ns=plan.header.stamp.to_nsec(),view_index=1,deadline_ns=session['deadline_ns'],
        plan_sha256=committed_plan_digest(plan),control_epoch_ns=n._following_epoch_ns,
        stamp_ns=9_042_902_946,lease_id='session:observation:9042902946',
        request_id='session:observation:9042902946:executing',motion_kind='observation',
        arrival_stamp_ns=0,capture_deadline_ns=0,metric_geometry_authority=False,motion_authorized=False)
    n._observation_measurement_request=deepcopy(receipt)
    n._last_motion_settle_sample=dict(epoch_ns=n._following_epoch_ns,
        stationary_window_start_ns=39_500_000_000,stationary_window_end_ns=40_400_000_000)
    def state(stage,label):n.stage=stage;n._last_state_message=label
    n.set_state=state;calls=[];behavior={}
    def service(bound,execute):
        calls.append((bound.plan_id,execute))
        if behavior.get('during_service'):behavior['during_service']()
        stamp=rospy.Time.now().to_nsec()
        return NS(success=True,message=json.dumps(dict(code='OBSERVATION_ENDPOINT_READY',
            plan_id=plan.plan_id,plan_digest=committed_plan_digest(plan),
            fixed_goal_pose=list(task.pose_values(plan.poses[0])),control_epoch_ns=receipt['control_epoch_ns'],
            session_id=receipt['session_id'],lease_id=receipt['lease_id'],view_index=receipt['view_index'],
            physical_motion_submitted=False,real_grasp_success=False,completion_stamp_ns=stamp,
            accepted_stamp_ns=stamp-10_000_000,view_audit={'fixture':True})))
    monkeypatch.setattr(rospy,'ServiceProxy',lambda *args:service)
    return NS(n=n,plan=plan,now=now,params=params,receipt=receipt,calls=calls,behavior=behavior,
        cfg=dict(observation_endpoint_compensation_enabled=True,target_observation_validity_sec=8.))


def test_actual_mro_31_second_motion_reaches_compensation_without_refreshing_old_target(scene):
    s=scene;before=s.n.latest_obj_time.to_nsec()
    s.n._compensate_observation_before_measurement(s.plan,s.cfg)
    assert s.calls==[(s.plan.plan_id,True)]
    assert s.n.latest_obj_time.to_nsec()==before
    assert s.n._observation_measurement_request==s.receipt
    assert not s.n._execution_authority_revoked
    assert s.n._observation_compensation_action_scope is None


def test_other_bound_action_keeps_original_eight_second_age_limit(scene):
    s=scene
    result,response=s.n._invoke_plan_bound_action(s.plan,s.cfg,'unrelated action',lambda:s.calls.append('other'))
    assert not result.ok and result.code=='TARGET_STALE'
    assert response is None and not s.calls


@pytest.mark.parametrize('fault', ['future', 'missing_stamp', 'before_plan', 'too_old_at_begin',
    'post_begin_stale', 'source_mismatch', 'not_detected', 'invalid_geometry', 'wrong_track',
    'drifting_center', 'geometry_loss_latch', 'manual', 'inactive', 'revoked_authority',
    'bad_digest', 'revoked_lease', 'qualified_lease', 'return_lease', 'wrong_epoch',
    'wrong_selection', 'session_selection', 'wrong_view', 'changed_source', 'wrong_origin', 'changed_deadline',
    'old_settle', 'expired_session'])
def test_real_frozen_gate_and_current_contradictions_still_reject_before_service(scene,fault):
    s=scene;n=s.n
    if fault in ('future','before_plan','too_old_at_begin','post_begin_stale','source_mismatch'):
        age={'future':-1.,'before_plan':34.,'too_old_at_begin':31.914,'post_begin_stale':20.,'source_mismatch':31.913}[fault]
        value=rospy.Time.from_sec(s.now[0]-age)
        n.latest_obj_time=value
        if fault!='source_mismatch':n.latest_obj.header.stamp=value;n.latest_obj.pose_base.header.stamp=value
        if fault=='too_old_at_begin':n._observation_measurement_request['stamp_ns']=20_000_000_000
    elif fault=='missing_stamp':n.latest_obj_time=None
    elif fault=='not_detected':n.latest_obj.detected=False
    elif fault=='invalid_geometry':n.latest_target_geometry.valid=False
    elif fault=='wrong_track':n.latest_target_geometry.target_track_id='g0-t9'
    elif fault=='drifting_center':n.latest_obj.pose_base.pose.position.x+=.1
    elif fault=='geometry_loss_latch':n._unknown_target_loss=dict(selection=deepcopy(n._mode_bound_selection),reason='TARGET_LOST: explicit geometry contradiction')
    elif fault=='manual':s.params['/gui/joint_direct_mode']=True
    elif fault=='inactive':n.active=False
    elif fault=='revoked_authority':n._execution_authority_revoked=True
    elif fault=='bad_digest':s.plan.poses[0].position.x+=.01
    elif fault=='revoked_lease':n._observation_measurement_request['state']='revoked'
    elif fault=='qualified_lease':n._observation_measurement_request['state']='qualified'
    elif fault=='return_lease':n._observation_measurement_request['motion_kind']='return'
    elif fault=='wrong_epoch':n._following_epoch_ns+=1
    elif fault=='wrong_selection':s.params[mode.SELECTION_PARAM]['generation']+=1
    elif fault=='session_selection':n._observe_return_session['selection']['generation']+=1
    elif fault=='wrong_view':n._observe_return_session['view_index']+=1
    elif fault=='changed_source':n._observation_measurement_request['view_source_stamp_ns']+=1
    elif fault=='wrong_origin':n._observe_return_origin_proof['plan_digest']='f'*64
    elif fault=='changed_deadline':n._observation_measurement_request['deadline_ns']+=1
    elif fault=='old_settle':n._last_motion_settle_sample['stationary_window_start_ns']=9_000_000_000
    elif fault=='expired_session':s.now[0]=609.
    with pytest.raises((ValueError,TypeError,AttributeError)):
        n._compensate_observation_before_measurement(s.plan,s.cfg)
    assert not s.calls
    assert getattr(n,'_observation_compensation_action_scope',None) is None


@pytest.mark.parametrize('fault',['exception','manual','epoch','lease','deadline'])
def test_late_service_return_cannot_keep_scope_or_old_authority(scene,fault):
    s=scene
    def change():
        if fault=='exception':raise RuntimeError('simulated RPC exception')
        if fault=='manual':s.params['/gui/joint_direct_mode']=True
        if fault=='epoch':s.n._following_epoch_ns+=1
        if fault=='lease':s.n._observation_measurement_request['lease_id']='replacement'
        if fault=='deadline':s.now[0]=609.
    s.behavior['during_service']=change
    with pytest.raises((ValueError,RuntimeError)):
        s.n._compensate_observation_before_measurement(s.plan,s.cfg)
    assert len(s.calls)==1 and s.n._observation_compensation_action_scope is None


def test_scope_is_not_retained_for_the_next_unrelated_action(scene):
    s=scene;s.n._compensate_observation_before_measurement(s.plan,s.cfg)
    result,response=s.n._invoke_plan_bound_action(s.plan,s.cfg,'other action',lambda:s.calls.append('other'))
    assert not result.ok and result.code=='TARGET_STALE' and response is None
    assert len(s.calls)==1


def test_new_fresh_same_target_still_uses_original_geometry_checks(scene):
    s=scene;stamp=rospy.Time.from_sec(s.now[0]-.1)
    s.n.latest_obj_time=stamp;s.n.latest_obj.header.stamp=stamp;s.n.latest_obj.pose_base.header.stamp=stamp
    s.n._compensate_observation_before_measurement(s.plan,s.cfg)
    assert len(s.calls)==1 and s.n.latest_obj_time==stamp


def test_wait_for_service_cannot_replace_original_executing_lease(scene,monkeypatch):
    s=scene
    def replace(*a,**kw):
        s.n._observation_measurement_request.update(lease_id='replacement',request_id='replacement:executing')
    monkeypatch.setattr(rospy,'wait_for_service',replace)
    with pytest.raises(ValueError,match='LEASE_CHANGED'):
        s.n._compensate_observation_before_measurement(s.plan,s.cfg)
    assert not s.calls

