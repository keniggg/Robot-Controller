from copy import deepcopy
import json
from unittest import mock

import pytest

import test_grasp_task_sequence as fixture
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values

task=fixture.grasp_task_node


def task_case():
    node=task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active=True
    node._bound_execution_plan=None
    node._position_only_execute_globally_enabled=lambda:False
    node._validate_bound_plan=lambda *args,**kwargs:task.PlanValidationResult(True)
    node._invoke_plan_bound_action=lambda _p,_c,_l,action:(task.PlanValidationResult(True),action())
    node._wait_for_motion_settle=mock.Mock(return_value=True)
    node._set_contact_endpoint_precision=mock.Mock(return_value=True)
    node._compensate_pregrasp_endpoint=mock.Mock(return_value=True)
    node._wait_for_measured_endpoint_contract=mock.Mock(return_value=(True,.004,2.))
    node._record_and_validate_measured_endpoint=mock.Mock(return_value=True)
    node.set_state=mock.Mock()
    plan=fixture.GraspTaskSequenceTest()._rich_plan()
    plan.diagnostic='CONTACT_EXECUTION_PLAN'
    pose=fixture.GraspTaskSequenceTest()._pose(0.)
    pose.pose=deepcopy(plan.poses[0])
    cfg=dict(measured_endpoint_check_enabled=True,contact_endpoint_precision_enabled=True,
        measured_endpoint_position_tolerance_m=.006,measured_endpoint_orientation_tolerance_deg=5.,
        pregrasp_cartesian_compensation_enabled=True)
    move=mock.Mock(return_value=fixture.FakeServiceResponse(True))
    return node,plan,pose,cfg,move


def run_case(node,plan,pose,cfg,move,stage=task.GraspStages.MOVE_PREGRASP):
    return node._plan_and_execute_pose(stage,'pregrasp',pose,move,'pregrasp',
        execution_plan=plan,gcfg=cfg,execute_pose=move)


def test_pregrasp_uses_software_compensation_then_original_arrival_gate_without_driver_trim():
    node,plan,pose,cfg,move=task_case()
    events=[]
    node._wait_for_motion_settle.side_effect=lambda *_:events.append('settle') or True
    node._compensate_pregrasp_endpoint.side_effect=lambda *_:events.append('compensate') or True
    node._wait_for_measured_endpoint_contract.side_effect=lambda *_:events.append('verify') or (True,.004,2.)
    assert run_case(node,plan,pose,cfg,move)
    assert events==['settle','compensate','verify']
    node._set_contact_endpoint_precision.assert_not_called()
    node._compensate_pregrasp_endpoint.assert_called_once_with(plan,cfg)
    node._record_and_validate_measured_endpoint.assert_called_once()
    assert node._wait_for_measured_endpoint_contract.call_args.args[1:3]==(.006,5.)


@pytest.mark.parametrize('kind',['disabled','far_field','approach'])
def test_software_compensation_is_scoped_to_contact_pregrasp(kind):
    node,plan,pose,cfg,move=task_case();stage=task.GraspStages.MOVE_PREGRASP
    if kind=='disabled':cfg['pregrasp_cartesian_compensation_enabled']=False
    if kind=='far_field':plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    if kind=='approach':stage=task.GraspStages.APPROACH_TARGET
    assert run_case(node,plan,pose,cfg,move,stage)
    node._compensate_pregrasp_endpoint.assert_not_called()
    if kind!='far_field':assert node._set_contact_endpoint_precision.call_count==2


@pytest.mark.parametrize('fault',['not_settled','correction','final_arrival','goal_mismatch'])
def test_failed_compensation_cannot_authorize_next_stage(fault):
    node,plan,pose,cfg,move=task_case()
    if fault=='not_settled':node._wait_for_motion_settle.return_value=False
    if fault=='correction':node._compensate_pregrasp_endpoint.return_value=False
    if fault=='final_arrival':node._wait_for_measured_endpoint_contract.return_value=(False,.015,2.)
    if fault=='goal_mismatch':pose.pose.position.x+=.01
    assert not run_case(node,plan,pose,cfg,move)
    node._set_contact_endpoint_precision.assert_not_called()
    if fault=='goal_mismatch':assert move.call_count==1 and move.call_args.args[1] is False
    if fault in ('not_settled','goal_mismatch'):node._compensate_pregrasp_endpoint.assert_not_called()


@pytest.mark.parametrize('fault',['none','wrong_plan','wrong_goal','planned_only','failure','transport'])
def test_task_requires_gateway_confirmation_for_same_frozen_pose(monkeypatch,fault):
    node,plan,_,cfg,_=task_case()
    report=dict(code='PREGRASP_MEASURED_CONVERGED',plan_id=plan.plan_id,
                fixed_goal_pose=list(pose_values(plan.poses[0])))
    if fault=='wrong_plan':report['plan_id']='other'
    if fault=='wrong_goal':report['fixed_goal_pose'][0]+=.01
    if fault=='planned_only':report['code']='PREGRASP_PLANNED_ONLY'
    proxy=mock.Mock(return_value=fixture.FakeServiceResponse(fault!='failure',json.dumps(report)))
    if fault=='transport':proxy.side_effect=RuntimeError('transport')
    monkeypatch.setattr(task.rospy,'wait_for_service',mock.Mock())
    monkeypatch.setattr(task.rospy,'ServiceProxy',lambda *_:proxy)
    result=task.GraspTaskNode._compensate_pregrasp_endpoint(node,plan,cfg)
    assert result is (fault=='none')
    assert proxy.call_args.args[0] is not plan
    assert proxy.call_args.args[1] is True


@pytest.mark.parametrize('converged',[True,False])
def test_reused_pregrasp_still_compensates_and_requires_measured_arrival(converged):
    node,plan,pose,cfg,_=task_case()
    node._wait_for_measured_endpoint_contract.return_value=(converged,.004 if converged else .015,2.)
    assert node._validate_reused_contact_pregrasp(plan,pose,cfg) is converged
    node._compensate_pregrasp_endpoint.assert_called_once_with(plan,cfg)
    node._set_contact_endpoint_precision.assert_not_called()
    node._record_and_validate_measured_endpoint.assert_called_once()
