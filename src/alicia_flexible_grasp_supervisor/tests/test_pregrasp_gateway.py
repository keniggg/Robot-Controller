from copy import deepcopy
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy

from test_pregrasp_compensation import recorded_fixture
from test_endpoint_correction import sample
from test_motion_gateway_controller_start import MotionGateway
import test_grasp_task_sequence as task_fixture
from test_gateway_tracking_probe import path_fixture
from alicia_flexible_grasp.robot import pregrasp_gateway as module
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts, CorrectionStep
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, committed_plan_digest
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q

def _task_fixture():
    return task_fixture.GraspTaskSequenceTest()


def make_gateway(monkeypatch, response=True, day='20260919'):
    fk,sdk,actual,goal=recorded_fixture(day)
    fixture=_task_fixture(); plan=fixture._rich_plan()
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    p=plan.poses[0]
    p.position.x,p.position.y,p.position.z=goal[:3]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=goal[3:]
    plan.plan_id=compute_plan_id(plan)
    node=MotionGateway.__new__(MotionGateway)
    node._planner_lock=threading.RLock()
    parameters={}
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:parameters.get(key,default))
    monkeypatch.setattr(module.rospy,'set_param',lambda key,value:parameters.update({key:value}))
    state=dict(now=10.,sdk=sdk.copy(),actual=actual.copy(),moves=0)
    monkeypatch.setattr(module.rospy,'get_time',lambda:state['now'])
    ctx=dict(scene=NS(plan_id=plan.plan_id),epoch=1_000_000_000,plan=plan,
        config=dict(measured_endpoint_position_tolerance_m=.006,measured_endpoint_orientation_tolerance_deg=5.),
        model=fk.xml if hasattr(fk,'xml') else __import__('json').loads(
            (__import__('pathlib').Path(__file__).parent/'fixtures/pregrasp_following_20260919.json').read_text())['robot_description'])
    node._pregrasp_context=lambda _:ctx
    node._validate_pregrasp_context=mock.Mock()
    node._pregrasp_reference=lambda:tuple(sdk_counts(state['sdk']))
    def snapshot(*args,**kwargs):
        kwargs['context_validator']()
        if kwargs.get('after_ns'):state['now']+=1.
        return sample(fk,state['sdk'],state['actual'],state['now'])
    node._endpoint_following_snapshot=snapshot
    node._configure_controller_reference_guard=mock.Mock()
    node._ensure_trajectory_controllers_started=mock.Mock(return_value=(True,'ready'))
    node._synchronize_trajectory_controller_to_feedback=mock.Mock(return_value=(True,'ready'))
    planner=NS(observation_path_guard_required=False,observation_path_validator=None,
        strict_execution_max_joint_velocity_rad_s=.12,observation_tracking_contract_required=False,
        observation_trajectory_executor=NS())
    calls=[]
    def execute(goal,execute,**kwargs):
        calls.append((goal,execute))
        assert planner.observation_path_guard_required and planner.observation_tracking_contract_required
        assert callable(planner.observation_path_validator)
        if execute:
            kwargs['before_execute']()
            state['moves']+=1;state['now']+=2.
            delta=np.asarray(goal)-state['sdk']
            state['sdk']=np.asarray(goal)
            if callable(response):state['actual']+=response(delta.copy())
            elif response:state['actual']+=delta
        return True,'controller result'
    planner.plan_and_execute_joint_probe=execute
    node._ensure_planner=lambda:planner
    return node,planner,plan,state,calls,parameters


def test_gateway_converges_only_on_new_measured_response_and_restores_planner(monkeypatch):
    node,planner,plan,state,calls,parameters=make_gateway(monkeypatch,day='20260920_wrist')
    original=committed_plan_digest(plan)
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    report=json.loads(result.message)
    assert result.success,report
    assert report['code']=='PREGRASP_MEASURED_CONVERGED' and report['position_error_m']<=.006
    assert len(calls)==7 and state['moves']==7
    assert all(sdk_counts(goal)[4]==1118 for goal,_ in calls)
    assert original==committed_plan_digest(plan)
    assert planner.observation_path_guard_required is False
    assert planner.observation_tracking_contract_required is False
    assert planner.strict_execution_max_joint_velocity_rad_s==.12
    assert not hasattr(planner,'observation_execution_authorized')
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'ALREADY_CONSUMED' in result.message and len(calls)==7


def test_no_response_leaves_original_goal_and_stops_after_one_step(monkeypatch):
    node,planner,plan,state,calls,_=make_gateway(monkeypatch,response=False)
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'NO_BOUNDED_DIRECTIONAL_RESPONSE' in result.message
    assert len(calls)==1 and state['moves']==1
    assert json.loads(result.message)['no_implicit_rollback_or_torque_disable']


def test_unsettled_response_reports_completed_command_without_reusing_old_measurement(monkeypatch):
    node,_,plan,state,calls,parameters=make_gateway(monkeypatch)
    snapshot=node._endpoint_following_snapshot
    def unsettled(*args,**kwargs):
        if kwargs.get('after_ns'):
            raise ValueError('ENDPOINT_CORRECTION_FEEDBACK_UNAVAILABLE: accepted feedback is not stationary')
        return snapshot(*args,**kwargs)
    node._endpoint_following_snapshot=unsettled
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    report=json.loads(result.message)
    assert not result.success and state['moves']==1 and len(calls)==1
    assert report['steps_completed']==1 and report['post_command_feedback_valid'] is False
    assert report['completed_command_target_counts']==sdk_counts(state['sdk']).tolist()
    assert report['last_validated_evidence']['steps_completed']==0
    assert 'position_error_m' not in report and 'measured_counts' not in report
    assert not node.handle_compensate_pregrasp(NS(plan=plan,execute=True)).success
    assert len(calls)==1


def test_dry_run_never_connects_controller_or_consumes_episode(monkeypatch):
    node,_,plan,state,calls,parameters=make_gateway(monkeypatch)
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=False))
    assert result.success and len(calls)==1 and calls[0][1] is False
    assert state['moves']==0 and not parameters
    assert json.loads(result.message)['physical_path_authorized'] is False
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()


def test_changed_reference_cannot_be_overwritten(monkeypatch):
    node,_,plan,state,calls,_=make_gateway(monkeypatch)
    node._pregrasp_reference=lambda:(0,)*6
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'REFERENCE_CHANGED' in result.message and not calls


def test_revoked_authority_between_steps_cannot_issue_another(monkeypatch):
    node,_,plan,state,calls,_=make_gateway(monkeypatch)
    def validate(_):
        if state['moves']:raise ValueError('manual takeover')
    node._validate_pregrasp_context=validate
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    assert not result.success and 'manual takeover' in result.message and len(calls)==1


def real_context(monkeypatch):
    plan=_task_fixture()._rich_plan(stamp_sec=9.)
    plan.diagnostic='CONTACT_EXECUTION_PLAN';plan.plan_id=compute_plan_id(plan)
    scene=FrozenObservationScene.from_contact_plan(plan)
    cfg=dict(pregrasp_cartesian_compensation_enabled=True,measured_endpoint_check_enabled=True)
    params={'/grasp':cfg,'/robot/observation_tracking_contract_enabled':True,'/robot_description':'model'}
    monkeypatch.setattr(rospy,'get_param',lambda key,default=None:params.get(key,default))
    monkeypatch.setattr(rospy,'get_time',lambda:10.)
    node=MotionGateway.__new__(MotionGateway);node._observation_context_lock=threading.RLock()
    node._require_live_observation_task=mock.Mock();node._manual_control_active=lambda:False
    node._pregrasp_task_stage=GraspStages.MOVE_PREGRASP
    node._control_reference_epoch_ns=1_000_000_000
    node._observation_contexts=NS(revocation=3)
    node._fresh_actuation_status=lambda:'CONFIRMED: test'
    context=dict(scene=scene,plan=plan,epoch=1_000_000_000,revocation=3,
                 model='model',config=deepcopy(cfg),deadline=time.monotonic()+60.)
    return node,context,params


@pytest.mark.parametrize('fault',['none','manual','stage','epoch','revocation','disabled','model',
                                    'plan','deadline','actuation','driver_trim','tracking','disconnect'])
def test_real_context_gate_rejects_authority_configuration_and_geometry_changes(monkeypatch,fault):
    node,context,params=real_context(monkeypatch)
    if fault=='manual':node._manual_control_active=lambda:True
    if fault=='stage':node._pregrasp_task_stage=GraspStages.APPROACH_TARGET
    if fault=='epoch':node._control_reference_epoch_ns+=1
    if fault=='revocation':node._observation_contexts.revocation+=1
    if fault=='disabled':params['/grasp']['pregrasp_cartesian_compensation_enabled']=False
    if fault=='model':params['/robot_description']='other'
    if fault=='plan':context['plan'].poses[0].position.x+=.001
    if fault=='deadline':context['deadline']=time.monotonic()-1
    if fault=='actuation':node._fresh_actuation_status=lambda:'PENDING'
    if fault=='driver_trim':params['/alicia_d_driver_node/endpoint_feedback_trim_enabled']=True
    if fault=='tracking':params['/robot/observation_tracking_contract_enabled']=False
    if fault=='disconnect':node._require_live_observation_task.side_effect=ValueError('disconnected')
    if fault=='none':node._validate_pregrasp_context(context)
    else:
        with pytest.raises(ValueError):node._validate_pregrasp_context(context)


@pytest.mark.parametrize('fault',['none','joint2','held_axis_excursion','endpoint','excursion','reference','collision','time'])
def test_step_path_checks_exact_goal_held_axis_and_continuous_local_box(monkeypatch,fault):
    node,planner,scene,trajectory=path_fixture(monkeypatch)
    target=np.array([4,0,4,0,0,0]);positions=target*Q
    trajectory.joint_trajectory.points[-1].positions=positions.tolist()
    step=CorrectionStep((2048,)*6,tuple((2048+target).tolist()),(2048,)*6,1,1)
    node._validate_pregrasp_context=mock.Mock();node._pregrasp_reference=lambda:(2048,)*6
    node._validate_frozen_observation_path=mock.Mock(return_value={'controller_constraints_snapshot':{'goal_time':.5}})
    context=dict(scene=scene,revocation=1,deadline=time.monotonic()+60,initial_sdk_counts=(2048,)*6)
    if fault=='joint2':
        trajectory.joint_trajectory.points[0].positions[1]=Q
        node._observation_controller_hold=lambda _:NS(positions=(0,Q,0,0,0,0),velocities=(0,)*6,accelerations=(0,)*6)
    if fault=='endpoint':trajectory.joint_trajectory.points[-1].positions[0]+=Q
    if fault=='excursion':trajectory.joint_trajectory.points[-1].velocities[0]=.1
    if fault=='held_axis_excursion':trajectory.joint_trajectory.points[-1].velocities[4]=.01
    if fault=='reference':node._pregrasp_reference=lambda:(2049,)*6
    if fault=='collision':node._validate_frozen_observation_path.side_effect=ValueError('collision')
    if fault=='time':context['deadline']=time.monotonic()+1.
    if fault=='none':
        assert node._validate_pregrasp_step_path(context,step,trajectory,planner)['pregrasp_fixed_plan_id']==scene.plan_id
        assert callable(node._validate_frozen_observation_path.call_args.kwargs['context_validator'])
    else:
        with pytest.raises(ValueError):node._validate_pregrasp_step_path(context,step,trajectory,planner)


def test_contact_scene_constructor_does_not_admit_contact_to_observation_route():
    plan=_task_fixture()._rich_plan();plan.diagnostic='CONTACT_EXECUTION_PLAN';plan.plan_id=compute_plan_id(plan)
    assert FrozenObservationScene.from_contact_plan(plan).plan_id==plan.plan_id
    with pytest.raises(ValueError):FrozenObservationScene.from_plan(plan)
    plan.poses[0].position.x+=.001
    with pytest.raises(ValueError):FrozenObservationScene.from_contact_plan(plan)


def test_gateway_retains_one_episode_budget_while_holding_unresponsive_axis(monkeypatch):
    def partial_response(delta):
        counts=np.rint(delta/Q).astype(int);counts[4]=0
        if abs(counts[2])==4:counts[2]=int(np.sign(counts[2])*3)
        return counts*Q
    node,planner,plan,state,calls,parameters=make_gateway(monkeypatch,partial_response,'20260920')
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=True))
    evidence=json.loads(result.message)
    assert result.success,evidence
    assert evidence['held_joints']==['Joint2','Joint5']
    assert 1<len(calls)<=12 and len(parameters)==1
    assert evidence['position_error_m']<=.006
    n=len(calls)
    assert not node.handle_compensate_pregrasp(NS(plan=plan,execute=True)).success
    assert len(calls)==n
