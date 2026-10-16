"""Actual geometric rejection metadata, transport, and no-submit boundary."""
from copy import deepcopy
from dataclasses import replace
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest import mock
import numpy as np
import pytest
from alicia_flexible_grasp.robot import observation_path_guard as guard
from alicia_flexible_grasp.robot import observation_tracking_contract as tracking
from test_observation_path_guard import model_xml, gripper, point, trajectory, scene
from test_pregrasp_precision_preference import case, run
from test_observation_branch_hint import fixture as planner_fixture


def collision_report():
    fk=guard.SerialUrdfFk(model_xml(),['J1'])
    plan=trajectory([point([0.],0.,[0.],[0.]),point([.6],30.,[0.],[0.])])
    hold=point([0.],v=[0.],a=[0.])
    box=guard._stage_boxes(fk([.6]),gripper(),.05,'y','z')[0]
    frozen=replace(scene(),center=box[1],size=np.array([.005]*3))
    report=tracking.qualify_contract_path(plan,hold,frozen,fk,gripper(),.05,
        {'J1':{'trajectory':.12}},.5,trajectory_stop_reserve=True)
    assert report['ok'] is False and report['failure_location']=='path'
    assert report['candidate_rejection_code']=='TARGET_OBB_COLLISION'
    assert report['candidate_rejection_at_start'] is False
    assert all(r['candidate_rejection_code']=='TARGET_OBB_COLLISION' for r in report['tracking_contract_search'])
    return report


def test_actual_collision_has_metadata_through_both_pure_contract_layers():
    collision_report()


@pytest.mark.parametrize('kind',['support','start_collision','budget'])
def test_other_actual_guard_reasons_keep_start_or_no_normal_type(kind):
    fk=guard.SerialUrdfFk(model_xml(),['J1'])
    tr=trajectory([point([0.],0.,[0.],[0.]),point([.6],30.,[0.],[0.])])
    hold=point([0.],v=[0.],a=[0.]);frozen=scene()
    opts={}
    if kind=='support':frozen=replace(frozen,offset=-.4)
    elif kind=='start_collision':
        box=guard._stage_boxes(fk([0.]),gripper(),.05,'y','z')[0]
        frozen=replace(frozen,center=box[1],size=np.array([.005]*3))
    else:opts['max_checks']=1
    with pytest.raises(guard.ObservationPathError) as caught:
        guard.validate_observation_trajectory(tr,frozen,fk,gripper(),.05,hold,**opts)
    if kind=='budget':assert not hasattr(caught.value,'candidate_rejection_code')
    else:
        assert caught.value.candidate_rejection_at_start is True
        assert caught.value.candidate_rejection_code==('SUPPORT_CLEARANCE_COLLISION' if kind=='support' else 'TARGET_OBB_COLLISION')


def gateway_exception(monkeypatch,report):
    # Execute the real gateway rejection boundary. Geometry itself was
    # evaluated by collision_report; this fixture injects that immutable
    # report at the contracted checker boundary, not a successful path.
    path=Path(__file__).resolve().parents[1]/'scripts/motion_gateway_node.py'
    spec=importlib.util.spec_from_file_location('preference_test_gateway',str(path))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    from test_stationary_following import XML,NAMES
    node=module.MotionGateway.__new__(module.MotionGateway)
    node._observation_context_lock=threading.RLock()
    node._require_live_observation_task=lambda:None
    node._manual_control_active=lambda:False
    node.joint_names=NAMES+['right_finger']
    node._observation_measured_opening=lambda:.05
    node._observation_controller_hold=lambda names:point([0.]*6,v=[0.]*6,a=[0.]*6)
    node._unknown_observation_view_audit=lambda *_:None
    parameters={'/robot_description':XML,'/alicia_controller/constraints':{n:{'trajectory':.12,'goal':.035} for n in NAMES},'/alicia_controller/stop_trajectory_duration':.5}
    original=module.rospy.get_param
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:parameters.get(key,original(key,default)))
    monkeypatch.setattr(tracking,'qualify_contract_path',lambda *a,**k:deepcopy(report))
    planner=NS(manipulator=NS(get_planning_frame=lambda:'base_link',get_end_effector_link=lambda:'tool0'),_observation_branch_signature=lambda:('model',),observation_tracking_contract_required=True)
    tr=trajectory([point([0.]*6,0.,[0.]*6,[0.]*6),point([.01]*6,3.,[0.]*6,[0.]*6)],NAMES)
    with pytest.raises(guard.ObservationPathError) as caught:
        node._validate_frozen_observation_path((scene(),1),tr,planner,context_validator=lambda:None)
    assert caught.value.candidate_path_rejection==report
    return caught.value


@pytest.mark.parametrize('fault',['none','submitted','start','unknown_code','invalid_context','stale','owner','epoch','hard_miss'])
def test_actual_typed_report_gateway_to_pregrasp_noop_scope(monkeypatch,fault):
    report=collision_report()
    values=case(monkeypatch,.00999 if fault=='hard_miss' else .009333)
    node,planner,plan,state,calls,params,context=values
    if fault=='start':report['candidate_rejection_at_start']=True
    if fault=='unknown_code':report['candidate_rejection_code']='INVALID_JOINT_LIMIT'
    error=gateway_exception(monkeypatch,report)
    if fault=='invalid_context':error=guard.ObservationPathError('OWNER_CHANGED')
    node._validate_pregrasp_step_path=mock.Mock(side_effect=error)
    snapshot=node._endpoint_following_snapshot;count=[0]
    def fresh(*args,**kwargs):
        value=snapshot(*args,**kwargs);count[0]+=1
        if count[0]==2:
            if fault=='stale':value['accepted_stamp_ns']-=600000000
            if fault=='owner':raise ValueError('OWNER_CHANGED')
            if fault=='epoch':value['epoch_ns']+=1
            if fault=='hard_miss':value['accepted_positions_rad'][2]-=np.pi/2048
        return value
    node._endpoint_following_snapshot=fresh
    def failed(*args,**kwargs):
        if fault=='submitted':kwargs['before_execute']()
        try:planner.observation_path_validator(object())
        except guard.ObservationPathError as exc:return False,str(exc)
        raise AssertionError('physical rejection was lost')
    planner.plan_and_execute_joint_probe=failed
    result,audit=run(values)
    assert result.success is (fault=='none'),audit
    if fault=='none':
        assert audit['no_motion_submitted'] and audit['steps_completed']==0
        assert audit['preference_path_rejection']['candidate_rejection_code']=='TARGET_OBB_COLLISION'
    assert state['moves']==0


@pytest.mark.parametrize('fault',['none','owner','not_ready','already_submitted'])
def test_real_planner_failure_before_submission_can_only_abandon_optional_step(monkeypatch,fault):
    values=case(monkeypatch)
    node,planner,plan,state,calls,params,context=values
    actual,arm,_,_=planner_fixture()
    arm.current_joint_values=state['actual'].tolist();arm.fail_plan=True
    if fault=='not_ready':actual.ready=False;actual.error='not ready'
    invoked=[]
    def invoke(*args,**kwargs):
        before=kwargs['before_execute']
        def boundary():invoked.append(True);before()
        if fault=='already_submitted':boundary()
        kwargs['before_execute']=boundary
        result=actual.plan_and_execute_joint_probe(*args,**kwargs)
        if fault=='owner':node._validate_pregrasp_context.side_effect=ValueError('OWNER_CHANGED')
        return result
    planner.plan_and_execute_joint_probe=invoke
    result,audit=run(values)
    assert result.success is (fault=='none'),audit
    if fault=='none':
        assert audit['no_motion_submitted'] and audit['preference_path_rejection']=={'candidate_rejection_code':'JOINT_PROBE_PLAN_FAILED'}
    assert not arm.executed_plans
    if fault!='already_submitted':assert not invoked
