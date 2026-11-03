"""Recorded G50 guide defect; synthetic future feedback never grants authority.

Real selector, predictor, feedforward and gateway methods run. The post-proposal
gateway tests use explicit no-action authority/feedback/path providers. Original
complete path kernels remain exercised by test_gen45_fresh_held_path_chain.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from geometry_msgs.msg import Pose

from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot import pregrasp_compensation as module
from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError, predict_held_axes_endpoint
from alicia_flexible_grasp.robot.held_pregrasp_selection import select_held_pregrasp_endpoint
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk

FIXTURE_SHA='7114e60bcaa5feb6a416f57a9a04b6efa2530d8fb4d20ec148057c99244083ed'


def inputs():
    p=Path(__file__).parent/'fixtures/gen51_g50_held_guide_inputs.json'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==FIXTURE_SHA
    data=json.loads(p.read_text())
    fk=SerialUrdfFk(data['model_xml'],ARM_NAMES)
    assert fk.model_sha256==data['model_sha256']
    return data,fk


def pose(values):
    result=Pose()
    result.position.x,result.position.y,result.position.z=values[:3]
    result.orientation.x,result.orientation.y,result.orientation.z,result.orientation.w=values[3:]
    return result


def selected(index):
    data,fk=inputs();case=data['cases'][index];before=deepcopy(data)
    command,report=select_held_pregrasp_endpoint(fk,case['source_sample'],
        case['pregrasp_goal'],case['approach_goal'],case['nominal_pregrasp_command_rad'],
        position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5))
    assert data==before
    return data,fk,case,command,report


def future_provider(case,report):
    # Explicit conditional provider: source metadata is original, positions
    # are a counterfactual future pregrasp, not an original measured endpoint.
    row=deepcopy(case['source_sample'])
    row['sdk_positions_rad']=report['predicted_pregrasp']['predicted_command_positions_rad']
    row['accepted_positions_rad']=report['predicted_pregrasp']['predicted_positions_rad']
    return row


@pytest.mark.parametrize('index',range(4))
def test_original_four_selectors_and_execution_share_one_qualified_integer_guide(index):
    data,fk,case,command,report=selected(index)
    original=deepcopy(data)
    approach_tolerance=min(.010,.012)
    assert report['success'] and not report['execution_authorized']
    assert report['predicted_pregrasp']['predicted_position_error_m']<=.010
    assert report['predicted_approach']['position_tolerance_m']==approach_tolerance
    assert report['predicted_approach']['predicted_position_error_m']<=approach_tolerance
    sample=future_provider(case,report)
    recovery=PregraspFeedforward(fk,sample,pose(case['approach_goal']),
        position_tolerance_m=approach_tolerance)
    step,evidence=recovery.propose_held_axes()
    predicted=report['predicted_approach']
    assert tuple(step.target_counts)==tuple(sdk_counts(predicted['predicted_command_positions_rad']))
    assert evidence['proposed_delta_counts']==predicted['proposed_delta_counts']
    delta=np.array(step.target_counts)-step.baseline_counts
    assert delta[1]==delta[4]==0 and max(abs(delta))<=32
    assert evidence['predicted_orientation_error_rad']<=math.radians(5)
    assert not evidence['hardware_response_verified']
    assert not evidence['conditional_budget_guide']['path_authorized']
    assert data==original


def failed_approach():
    data,fk=inputs()
    old=data['cases'][0]['original_trials'][0]['rejection_evidence']
    return fk,old,(np.asarray(old['initial_sdk_counts'])-2048)*Q


def test_every_already_qualified_old_guide_is_retained_without_new_solver(monkeypatch):
    from test_pregrasp_feedforward import gen6_held_axes_case
    recovery,_,_=gen6_held_axes_case()
    monkeypatch.setattr(module,'_normalized_remaining_budget_guide',
        mock.Mock(side_effect=AssertionError('qualified original must remain unchanged')))
    step,report=recovery.propose_held_axes()
    assert report['proposed_delta_counts']==[-4,0,28,-2,0,-32]
    module._normalized_remaining_budget_guide.assert_not_called()
    assert step is recovery.step


@pytest.mark.parametrize('fault',['nan','inf','wrong_shape','outside_remaining_box'])
def test_solver_output_never_overrides_original_quantized_rejection(monkeypatch,fault):
    import scipy.optimize
    fk,old,sdk=failed_approach()
    values=dict(nan=[float('nan')]*4,inf=[float('inf')]*4,
        wrong_shape=[0.]*3,outside_remaining_box=[1000*Q]*4)[fault]
    monkeypatch.setattr(scipy.optimize,'minimize',
        lambda *a,**k:NS(success=True,x=np.array(values)))
    with pytest.raises(HeldAxesPredictionError,match='DOES_NOT_REACH_FIXED_GOAL') as caught:
        predict_held_axes_endpoint(fk,sdk,old['virtual_start_positions_rad'],old['fixed_goal_pose'],
            position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5))
    assert caught.value.evidence['proposed_delta_counts']==old['proposed_delta_counts']
    assert not caught.value.evidence['execution_authorized']


@pytest.mark.parametrize('fault',['spent_steps','spent_total','additional_held','contact_merit'])
def test_shared_fallback_inherits_remaining_budget_and_separate_contact_policy(monkeypatch,fault):
    fk,old,sdk=failed_approach()
    measured=np.asarray(old['virtual_start_positions_rad'])
    goal=module.pose_matrix(old['fixed_goal_pose'])
    def residual(q):
        actual=fk(q)
        return float(np.linalg.norm(actual[:3,3]-goal[:3,3])),math.acos(float(np.clip(
            (np.trace(goal[:3,:3].T@actual[:3,:3])-1)/2,-1,1)))
    held=(0,1,4) if fault=='additional_held' else (1,4)
    spy=mock.Mock(wraps=module._normalized_remaining_budget_guide)
    monkeypatch.setattr(module,'_normalized_remaining_budget_guide',spy)
    delta,error=module.remaining_budget_guide(fk,residual,measured,sdk_counts(sdk),sdk_counts(sdk),
        maximum_steps=12,steps=12 if fault=='spent_steps' else 0,maximum_step_counts=4,
        maximum_total_counts=0 if fault=='spent_total' else 32,held_axes=held,
        response_gains=np.ones(6),tolerances=(.006,math.radians(5)),
        cost=(lambda q:100.) if fault=='contact_merit' else None)
    assert all(delta[i]==0 for i in held) and max(abs(delta))<=32
    if fault in ('spent_steps','spent_total'):
        assert not np.any(delta)
    if fault=='contact_merit':
        spy.assert_not_called()


@pytest.mark.parametrize('fault',['authority','expired_feedback_provider','path_collision_provider'])
def test_new_guide_cannot_cross_real_gateway_post_proposal_denial(monkeypatch,fault):
    from test_pregrasp_feedforward import held_gateway_case
    from test_endpoint_correction import sample as sample_provider
    # Use the recorded failed guide's actual start, not an endpoint selected
    # under today's wider request. At the explicit 6 mm request the shared
    # normalized fallback must really qualify before any denial is injected.
    fk,old,sdk=failed_approach()
    assert old['predicted_position_error_m']>.006
    node,planner,plan,state,calls,params,context=held_gateway_case(monkeypatch)
    state['sdk']=sdk.copy()
    state['actual']=np.array(old['virtual_start_positions_rad'])
    plan.poses[1]=pose(old['fixed_goal_pose'])
    plan.plan_id=compute_plan_id(plan)
    context.update(model=inputs()[0]['model_xml'],plan=plan,pose_index=1)
    context['scene'].plan_id=plan.plan_id
    context['config'].update(unknown_approach_held_axes_feedforward_enabled=True,
        unknown_contact_cartesian_compensation_enabled=True,
        contact_endpoint_precision_enabled=True,measured_endpoint_check_enabled=True,
        unknown_measured_endpoint_position_tolerance_m=.006)
    node._pregrasp_context=lambda _plan,index=0:context
    triggered=[]
    original=module._normalized_remaining_budget_guide
    def fallback(*args,**kwargs):
        value=original(*args,**kwargs)
        if value is not None:
            triggered.append(value[0].tolist())
            if fault=='authority':
                node._validate_pregrasp_context.side_effect=ValueError('manual owner revoked after guide')
        return value
    monkeypatch.setattr(module,'_normalized_remaining_budget_guide',fallback)
    def snapshot(*args,**kwargs):
        kwargs['context_validator']()
        if triggered and fault=='expired_feedback_provider':
            raise ObservationPathError('ENDPOINT_CORRECTION_FEEDBACK_UNAVAILABLE: stale provider')
        return sample_provider(fk,state['sdk'],state['actual'],state['now'])
    node._endpoint_following_snapshot=snapshot
    denied_path=mock.Mock(side_effect=ObservationPathError('TARGET_OBB_COLLISION explicit frozen-path provider'))
    node._validate_pregrasp_step_path=denied_path
    submitted=[]
    def joint_probe(goal,execute,**kwargs):
        calls.append((goal,execute))
        try:
            if fault=='path_collision_provider':
                planner.observation_path_validator(NS())
            kwargs['before_execute']()
        except (ValueError,ObservationPathError) as exc:
            return False,str(exc)
        submitted.append(goal)
        raise AssertionError('denied request must never reach inert controller submission')
    planner.plan_and_execute_joint_probe=joint_probe
    result=node.handle_compensate_approach(NS(plan=plan,execute=True))
    assert triggered, 'new shared fallback must actually have qualified before denial'
    assert not result.success and not submitted and state['moves']==0 and not params
    if fault=='path_collision_provider':denied_path.assert_called_once()
    planner._plan_cartesian_from_start_state.assert_not_called()
