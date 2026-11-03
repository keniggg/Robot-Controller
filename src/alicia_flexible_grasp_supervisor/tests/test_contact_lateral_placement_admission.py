"""New placement recipes retain original final admission and request ownership."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from test_contact_height_window_proposals import fixture, remote


def with_lateral_proposal():
    values=fixture();node=values[0]
    def proposal(plan,failure,geometry,parameters):
        original=geometry['planned_T_base_tool0'];normal=np.asarray(geometry['support_normal_base'])
        d=np.array([.0003,.0001,0.]);d-=normal*(normal@d)
        target=original.copy();target[:3,3]+=d
        return target,dict(proposed=True,normal_translation_m=0.,translation_base_m=d.tolist(),
            policy='explicit_test_placement_provider',motion_authorized=False)
    node._contact_lateral_window_proposal=mock.Mock(side_effect=proposal)
    return values


def test_lateral_recipe_is_rehashed_remeasured_and_fully_replanned_without_target_change():
    node,plan,prepared,runtime,_,_,calls,_,_=with_lateral_proposal();original=deepcopy(plan)
    _,result,_=node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert result.reachable and len(calls)==2
    trial=runtime['contact_height_window_search']['proposals'][0]
    delta=np.array(trial['placement']['translation_base_m'])
    assert trial['accepted'] and trial['geometry_gate']['ok']
    assert runtime['final_contact_height_margin']['ok']
    assert plan.object_geometry==original.object_geometry and plan.plan_id!=original.plan_id
    assert plan.plan_id==remote.compute_plan_id(plan)
    for before,after in zip(original.poses,plan.poses):
        assert before.orientation==after.orientation
        np.testing.assert_allclose([after.position.x-before.position.x,after.position.y-before.position.y,
            after.position.z-before.position.z],delta,atol=1e-15)
    assert plan.refinement_translation_m==pytest.approx(original.refinement_translation_m+np.linalg.norm(delta))
    assert all(call[2]==123. for call in calls)


def test_publication_recipe_repeats_complete_check_and_does_not_accumulate_displacement():
    node,source,prepared,runtime,good,bad,calls,_,_=with_lateral_proposal()
    prepared.ticket=NS(request_id=40,generation=9,target_epoch=1)
    def strict(candidate,stages,p,r,operation,deadline_sec):
        calls.append((deepcopy(candidate),operation,deadline_sec));passed=operation=='final_registered_height_window_strict_sequence'
        r['contact_sequence_precheck']=dict(success=passed,failed_stage='grasp',reason=bad.reason,plan_id=candidate.plan_id)
        return (good if passed else bad),{}
    node._final_contact_sequence_evaluation=strict
    selection=deepcopy(source);node._strict_check_final_near_field_plan(selection,prepared,runtime)
    publication=deepcopy(source);node._strict_check_final_near_field_plan(publication,prepared,runtime)
    assert selection==publication and selection!=source and len(calls)==3
    assert calls[1][0]==calls[2][0]==selection
    assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False
    assert runtime['_selected_contact_height_recipe']['proposal_index']==-1


def test_lateral_geometry_rejection_keeps_height_alternatives():
    node,plan,prepared,runtime,_,_,calls,_,_=with_lateral_proposal()
    original=node._registered_sequence_geometry_gate
    count=[0]
    def gate(*args):
        count[0]+=1
        if count[0]==1:return remote.CandidateGateResult(False,'GRIPPER_SWEEP_COLLISION','real overlap',0.,0.,0.,0.,0.,0.,'sweep',0)
        return original(*args)
    node._registered_sequence_geometry_gate=gate
    _,result,_=node._strict_check_final_near_field_plan(plan,prepared,runtime)
    trials=runtime['contact_height_window_search']['proposals']
    assert result.reachable and len(calls)==2
    assert not trials[0]['accepted'] and trials[1]['accepted']


def test_strict_rejection_never_admits_proposal_or_extends_four_trial_budget():
    node,plan,prepared,runtime,_,bad,calls,_,_=with_lateral_proposal();before=deepcopy(plan)
    def strict(candidate,stages,p,r,operation,deadline_sec):
        calls.append((candidate,operation,deadline_sec))
        r['contact_sequence_precheck']=dict(success=False,failed_stage='grasp',reason=bad.reason,plan_id=candidate.plan_id)
        return bad,{}
    node._final_contact_sequence_evaluation=strict
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert plan==before
    assert len(runtime['contact_height_window_search']['proposals'])<=4
    assert not any(x['accepted'] for x in runtime['contact_height_window_search']['proposals'])
    assert all(c[2]==123. for c in calls)


def test_expired_phase_cannot_submit_any_lateral_trial():
    node,plan,prepared,runtime,_,bad,calls,clock,_=with_lateral_proposal()
    def strict(candidate,stages,p,r,operation,deadline_sec):
        calls.append(operation);clock[0]=124.
        r['contact_sequence_precheck']=dict(success=False,failed_stage='grasp',reason=bad.reason,plan_id=candidate.plan_id)
        return bad,{}
    node._final_contact_sequence_evaluation=strict
    with pytest.raises(remote.CandidateContractError):node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert len(calls)==1
