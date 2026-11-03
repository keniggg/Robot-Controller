"""Final measured-height placement; no ROS, planner or physical execution.

R4 fixture is algebraic reconstruction of persisted analytic boundaries, not a
replay of the unavailable fused cloud. Complete geometry uses a separate real
synthetic measured surface; strict RPC outcomes below test admission control.
"""
import ast
from copy import deepcopy
from dataclasses import replace
import inspect
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence, encode_contact_evidence
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
from test_contact_geometry_evidence import final_family_fixture, remote
from test_remote_grasp6d_streaming import tabletop_gripper, MoveItResult
from test_rich_plan_candidate_source import valid_refined_ros_plan

LIMIT = .0026767676132463574
R4 = json.loads((Path(__file__).parent/'fixtures/gen16_r4_final_height_boundaries.json').read_text())


def real_fixture():
    node, prepared, runtime, plan, candidate = final_family_fixture()
    prepared.target_observation = node._latest_target_observation
    plan.refinement_status = 'VALID_3D'
    plan.refinement_translation_m = .001
    # Inside the connected nominal component, with insufficient stress margin.
    for pose in plan.poses:
        pose.position.z += .0005
    node.grasp_config.update(contact_height_registration_margin_enabled=True,
                             handeye_translation_max_error_m=LIMIT)
    return node, prepared, runtime, plan


def positions(plan):
    return np.array([[p.position.x,p.position.y,p.position.z] for p in plan.poses])


@pytest.mark.parametrize('row', R4['rows'], ids=lambda r:'R4_track_%s'%r['track_id'])
def test_r4_original_boundary_reconstruction_and_centering(row):
    g = dict(deepcopy(row['geometry']),gripper=tabletop_gripper())
    before = evaluate_contact_height_margin(translation_evidence_limit_m=LIMIT, **g)
    original = row['original_report']
    assert not before['ok'] and before['nominal']['ok']
    np.testing.assert_allclose(before['allowed_normal_translation_intervals_m'],
        original['allowed_normal_translation_intervals_m'],rtol=0,atol=2e-12)
    for key in ('actual_overlap_m','minimum_support_clearance_m'):
        assert before['nominal'][key] == pytest.approx(original['nominal'][key],abs=2e-12)
        for a,b in zip(before['endpoint_diagnostics'],original['endpoint_diagnostics']):
            assert a[key] == pytest.approx(b[key],abs=2e-12)
    moved,audit = place_contact_height(translation_evidence_limit_m=LIMIT, **g)
    assert moved is not None and audit['after']['ok']
    lo,hi = original['allowed_normal_translation_intervals_m'][0]
    assert audit['normal_translation_m'] == pytest.approx((lo+hi)/2,abs=2e-12)
    assert lo+LIMIT <= audit['normal_translation_m'] <= hi-LIMIT
    assert audit['after']['translation_evidence_limit_m'] == LIMIT
    assert audit['after']['minimum_support_clearance_m'] == .003
    assert audit['after']['minimum_contact_overlap_m'] == .002
    assert not audit['after']['uncertainty_is_certified_bound']
    assert not audit['motion_authorized'] and not audit['target_relocated']
    np.testing.assert_array_equal(moved[:3,:3],np.asarray(g['actual_T_base_tool0'])[:3,:3])


@pytest.mark.parametrize('row',R4['rows'],ids=lambda r:'R4_track_%s'%r['track_id'])
def test_r4_narrower_than_unchanged_stress_interval_still_rejects(row):
    g = dict(deepcopy(row['geometry']),gripper=tabletop_gripper())
    lo,hi = row['original_report']['allowed_normal_translation_intervals_m'][0]
    moved,audit=place_contact_height(translation_evidence_limit_m=(hi-lo)/2+1e-6,**g)
    assert moved is None and not audit['before']['ok']


def test_real_measured_surface_rechecks_complete_four_pose_geometry_and_rebinds_evidence():
    node,p,r,plan=real_fixture(); original=deepcopy(plan); calls=[]
    real=node._registered_sequence_geometry_gate
    def checked(trial,prepared,runtime,sequence):
        calls.append(deepcopy(sequence))
        return real(trial,prepared,runtime,sequence)
    node._registered_sequence_geometry_gate=checked
    node._prepare_final_registered_contact_plan(plan,p,r)
    audit=r['final_contact_height_recenter']
    assert len(calls)==2 and audit['geometry_rechecked'] and audit['applied']
    assert not audit['motion_authorized'] and audit['strict_sequence_recheck_required']
    assert not audit['deadline_extended']
    delta=audit['normal_translation_m']
    np.testing.assert_allclose(positions(plan)-positions(original),np.tile([0.,0.,delta],(4,1)),atol=1e-15)
    for i,stage in enumerate(('pregrasp','approach','grasp','lift')):
        assert getattr(calls[-1],stage).pose==plan.poses[i]
        assert plan.poses[i].orientation==original.poses[i].orientation
    assert plan.object_geometry==original.object_geometry
    assert plan.refinement_translation_m==pytest.approx(.001+abs(delta))
    fresh=decode_contact_evidence(plan,required=True)
    np.testing.assert_allclose(fresh['grasp_tool_pose_base'][:3],positions(plan)[2])
    assert r['final_contact_height_margin']['ok']
    assert r['final_contact_height_margin']['translation_evidence_limit_m']==LIMIT
    assert 'final_strict_plan_id' not in r and 'final_execution_sequence' not in r


@pytest.mark.parametrize('field,value', [('candidate_source','graspnet'),
    ('refinement_status','NOT_EVALUATED'),('model_choice','carton_segment'),
    ('diagnostic','FAR_FIELD_OBSERVATION_PLAN')])
def test_ineligible_plan_cannot_enter_recenter(field,value):
    node,p,r,plan=real_fixture();setattr(plan,field,value);original=deepcopy(plan)
    node._registered_sequence_geometry_gate=lambda *a:pytest.fail('ineligible geometry call')
    assert not node._recenter_final_registered_contact_height(plan,p,r,LIMIT,{})
    assert plan==original and 'final_contact_height_recenter' not in r


def test_already_robust_registered_family_does_not_change_again():
    node,p,r,plan=real_fixture();node._prepare_final_registered_contact_plan(plan,p,r)
    original=deepcopy(plan)
    node._recenter_final_registered_contact_height=lambda *a:pytest.fail('already robust')
    node._prepare_final_registered_contact_plan(plan,p,r)
    assert plan==original and 'final_contact_height_recenter' not in r


def test_translation_correction_bound_rejects_without_running_proposed_geometry():
    node,p,r,plan=real_fixture()
    bound=remote.refinement_registration_policy(remote.RegistrationConfig())['maximum_translation_m']
    plan.refinement_translation_m=bound
    real=node._registered_sequence_geometry_gate;calls=[]
    def gate(*a):calls.append(1);return real(*a)
    node._registered_sequence_geometry_gate=gate
    with pytest.raises(remote.CandidateContractError,match='support-normal translation'):
        node._prepare_final_registered_contact_plan(plan,p,r)
    assert len(calls)==1 and not r['final_contact_height_recenter']['applied']
    assert 'unchanged correction bound' in r['final_contact_height_recenter']['reason']


@pytest.mark.parametrize('lost', ['surface','registration','collision'])
def test_proposed_family_must_pass_fresh_full_geometry(lost):
    node,p,r,plan=real_fixture();original=positions(plan);real=node._registered_sequence_geometry_gate;calls=[]
    def gate(trial,prepared,state,sequence):
        calls.append(1)
        if len(calls)==2:
            if lost=='surface':node._active_multiview_surface=lambda:(None,None)
            elif lost=='registration':node._latest_registration_evidence=None
            else:
                # Genuine complete geometry sees an invalid palm/support pose.
                sequence.pregrasp.pose.position.z-=1.
        return real(trial,prepared,state,sequence)
    node._registered_sequence_geometry_gate=gate
    with pytest.raises(remote.CandidateContractError):node._prepare_final_registered_contact_plan(plan,p,r)
    assert len(calls)==2 and not r['final_contact_height_recenter']['applied']
    np.testing.assert_array_equal(positions(plan),original)
    assert 'final_strict_plan_id' not in r


def test_fresh_measurement_cannot_reuse_preproposal_margin():
    node,p,r,plan=real_fixture();original=positions(plan);real=node._registered_sequence_geometry_gate;calls=[]
    def gate(trial,prepared,state,sequence):
        result=real(trial,prepared,state,sequence);calls.append(1)
        if len(calls)==2:
            evidence=json.loads(state['final_contact_geometry_evidence'])
            lo,hi=evidence['contact_height_bounds_m'];evidence['contact_height_bounds_m']=[lo,lo+.0001]
            state['final_contact_geometry_evidence']=json.dumps(evidence,sort_keys=True,separators=(',',':'))
        return result
    node._registered_sequence_geometry_gate=gate
    with pytest.raises(remote.CandidateContractError):node._prepare_final_registered_contact_plan(plan,p,r)
    assert len(calls)==2 and not r['final_contact_height_recenter']['applied']
    assert not r['final_contact_height_recenter']['after_geometry_recheck']['ok']
    np.testing.assert_array_equal(positions(plan),original)


@pytest.mark.parametrize('expire_at',[0,1,2])
def test_original_phase_deadline_or_stop_rejects_before_commit(expire_at):
    node,p,r,plan=real_fixture();original=positions(plan)
    node.near_field_strategy='single_snapshot_direct';node.near_field_planning_active=True
    p.near_field_phase_id=4;p.near_field_phase_deadline_sec=100.
    now=[100. if expire_at==0 else 99.]
    node._execution_plan_validity_now_sec=lambda:now[0]
    real=node._registered_sequence_geometry_gate;calls=[]
    def gate(*args):
        result=real(*args);calls.append(1)
        if len(calls)==2:
            if expire_at==2:node.near_field_planning_active=False
            else:now[0]=100.
        return result
    node._registered_sequence_geometry_gate=gate
    with pytest.raises(remote.CandidateContractError):node._prepare_final_registered_contact_plan(plan,p,r)
    np.testing.assert_array_equal(positions(plan),original)
    assert len(calls)==(1 if expire_at==0 else 2)
    assert p.near_field_phase_deadline_sec==100.


@pytest.mark.parametrize('fault',['unreachable','collision_free','within_joint_limits','ik_valid','planning_success','complete'])
def test_real_strict_entry_rechecks_changed_family_and_cannot_authorize_incomplete_result(fault):
    node,p,r,plan=real_fixture()
    template=valid_refined_ros_plan()
    for field in ('header','target_track_id','object_geometry','refinement_inlier_count',
                  'refinement_overlap_fraction','refinement_rmse_m','fused_view_count'):
        setattr(plan,field,deepcopy(getattr(template,field)))
    plan.candidate_source_lineage=['tabletop_geometry']
    before_id=remote.compute_plan_id(plan)
    node._prepare_final_registered_contact_plan(plan,p,r)
    plan.plan_id=remote.compute_plan_id(plan);assert plan.plan_id!=before_id
    node._direct_near_field_active=lambda p:True
    node._direct_near_field_candidate_deadline_sec=lambda p,r:123.
    node._reject_inevitable_contact_goal=lambda *a:None
    good=MoveItResult(True,0.,0.,'test IPC response',collision_free=True,
                     within_joint_limits=True,ik_valid=True,planning_success=True)
    result=good if fault=='complete' else (replace(good,reachable=False,failure_code='MOVEIT_TIMEOUT') if fault=='unreachable' else replace(good,**{fault:None}))
    seen=[]
    def strict(candidate,stages,prepared,state,operation,deadline_sec):
        seen.append(operation)
        assert deadline_sec==123. and candidate.plan_id==remote.compute_plan_id(candidate)
        assert [name for name,_ in stages]==['pregrasp','approach','grasp','lift']
        for (_,pose),expected in zip(stages,plan.poses):assert pose.pose==expected
        return result,{}
    node._final_contact_sequence_evaluation=strict
    # Old success cannot survive the real strict entry's clearing and failure.
    r['final_strict_plan_id']='old';r['final_execution_sequence']='old'
    if fault=='complete':
        sequence,result,_=node._strict_check_final_near_field_plan(plan,p,r)
        assert result.reachable and sequence.grasp.pose==plan.poses[2]
        assert r['final_strict_plan_id']==plan.plan_id
    else:
        with pytest.raises(remote.CandidateContractError):node._strict_check_final_near_field_plan(plan,p,r)
        assert 'final_strict_plan_id' not in r and 'final_execution_sequence' not in r
    assert seen==['final_registered_strict_sequence']


def test_caller_keeps_recenter_before_rehash_continuity_and_strict_entry():
    tree=ast.parse(inspect.getsource(remote.RemoteGrasp6DNode._build_selected_preview_bundle).lstrip())
    calls=[]
    for item in ast.walk(tree):
        if isinstance(item,ast.Call):
            name=item.func.attr if isinstance(item.func,ast.Attribute) else getattr(item.func,'id','')
            if name in {'_prepare_final_registered_contact_plan','compute_plan_id','_check_refinement_candidate_continuity','_strict_check_final_near_field_plan'}:calls.append((item.lineno,name))
    names=[name for line,name in sorted(calls)]
    assert names==['_prepare_final_registered_contact_plan','compute_plan_id',
        '_check_refinement_candidate_continuity','_strict_check_final_near_field_plan',
        '_check_refinement_candidate_continuity']
