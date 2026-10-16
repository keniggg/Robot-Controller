"""Bounded placement proposals preserve contact evidence and full admission."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix

from alicia_flexible_grasp.grasp.contact_height_placement import (
    place_contact_height, contact_height_window_proposals)
from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_evidence import encode_contact_evidence
from test_contact_height_placement import case
from test_remote_grasp6d_streaming import remote_node as remote, CandidateGateResult, MoveItResult
from test_final_registered_orientation_resolution import fixture as old_fixture


def robust_geometry():
    geometry = case()
    T, _ = place_contact_height(translation_evidence_limit_m=.001, **geometry)
    assert T is not None
    geometry.update(actual_T_base_tool0=T, planned_T_base_tool0=T)
    return geometry


def test_smaller_upward_then_original_two_proposals_keep_full_error_interval_and_inputs():
    geometry = robust_geometry(); before = deepcopy(geometry)
    proposals = contact_height_window_proposals(translation_evidence_limit_m=.001, **geometry)
    assert len(proposals) == 3
    assert 0 < proposals[0][1]['normal_translation_m'] < proposals[1][1]['normal_translation_m']
    assert proposals[0][1]['normal_translation_m'] == pytest.approx(proposals[1][1]['normal_translation_m']*2./3.)
    assert proposals[2][1]['normal_translation_m'] < 0
    for T, report in proposals:
        assert report['after']['ok'] and report['after']['translation_evidence_limit_m'] == .001
        assert report['after']['minimum_contact_overlap_m'] == .002
        assert report['after']['minimum_support_clearance_m'] == .003
        assert report['motion_authorized'] is False and report['target_relocated'] is False
        np.testing.assert_array_equal(T[:3,:3], before['actual_T_base_tool0'][:3,:3])
        np.testing.assert_allclose(T[:3,3]-before['actual_T_base_tool0'][:3,3],
            report['normal_translation_m']*np.array(geometry['support_normal_base']), atol=1e-15)
    for key in geometry:
        if key != 'gripper': np.testing.assert_array_equal(geometry[key], before[key])


@pytest.mark.parametrize('limit', [True, '0.001', float('nan'), float('inf'), -.001, .05])
def test_bad_or_unavailable_original_margin_cannot_propose(limit):
    assert contact_height_window_proposals(translation_evidence_limit_m=limit, **robust_geometry()) == ()


def fixture():
    node, plan, prepared, runtime, _, good, _, gate, _ = old_fixture()
    geometry = robust_geometry()
    node.gripper_geometry = geometry['gripper']
    node.gripper_physical_open_width_m = .05
    node.gripper_tool_jaw_axis = 'y'; node.gripper_tool_finger_length_axis = 'z'
    node._contact_height_evidence_limit = lambda: .001
    clock = [100.]
    node._execution_plan_validity_now_sec = lambda: clock[0]
    node._direct_near_field_deadline_gate = lambda _p: lambda: clock[0] < 123.
    node._reject_inevitable_contact_goal = mock.Mock()
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = remote.CONTACT_EXECUTION_PLAN
    plan.refinement_status = 'VALID_3D'
    plan.refinement_translation_m = .002
    plan.refinement_inlier_count = 100
    plan.fused_view_count = 2
    plan.refinement_overlap_fraction = .8
    plan.refinement_rmse_m = .001
    plan.refinement_rotation_deg = .05
    T = geometry['actual_T_base_tool0']
    plan.poses = [remote.make_pose_stamped('base_link',
        T[:3,3]+delta*np.array(geometry['support_normal_base']), quaternion_from_matrix(T),
        stamp=plan.header.stamp).pose for delta in (.05,.03,0.,.04)]
    for axis, value in zip('xyz', geometry['support_normal_base']):
        setattr(plan.object_geometry.support_normal_base, axis, float(value))
    plan.object_geometry.support_offset_m = geometry['support_offset_m']
    def encode(candidate):
        return encode_contact_evidence(contact_center_base=geometry['contact_center_base'],
            contact_height_axis_base=geometry['contact_height_axis_base'],
            contact_height_bounds_m=geometry['contact_height_bounds_m'],
            minimum_contact_overlap_m=geometry['minimum_contact_overlap_m'],
            grasp_pose=candidate.poses[2])
    plan.contact_geometry_evidence = encode(plan)
    plan.plan_id = remote.compute_plan_id(plan)
    def full_gate(candidate, _prepared, _runtime, sequence):
        _runtime['final_contact_geometry_evidence'] = encode(candidate)
        return gate
    node._registered_sequence_geometry_gate = mock.Mock(side_effect=full_gate)
    runtime['scored_candidate'] = NS(stable_candidate=NS(candidate_source='tabletop_geometry'))
    reason = 'predicted_command continuous geometry: target OBB collision at 12-end/right_finger'
    bad = MoveItResult(False, 0., 0., reason, failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED')
    calls = []
    def strict(candidate, stages, p, r, operation, deadline_sec):
        calls.append((deepcopy(candidate), operation, deadline_sec))
        assert p is prepared and deadline_sec == 123.
        r['contact_sequence_precheck'] = dict(success=len(calls)>1, failed_stage='grasp',
            reason=reason, plan_id=candidate.plan_id)
        return (bad if len(calls)==1 else good), {}
    node._final_contact_sequence_evaluation = strict
    return node, plan, prepared, runtime, good, bad, calls, clock, geometry


def test_actual_final_admission_rehashes_only_after_new_complete_check():
    node, plan, prepared, runtime, _, _, calls, _, _ = fixture(); original = deepcopy(plan)
    sequence, result, _ = node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert result.reachable and len(calls) == 2
    assert [x[2] for x in calls] == [123.,123.]
    assert plan.plan_id != original.plan_id
    assert plan.plan_id == remote.compute_plan_id(plan) == runtime['final_strict_plan_id']
    delta = runtime['contact_height_window_search']['proposals'][0]['placement']['normal_translation_m']
    for a,b in zip(plan.poses,original.poses):
        assert a.orientation == b.orientation
        np.testing.assert_allclose([a.position.x-b.position.x,a.position.y-b.position.y,a.position.z-b.position.z],
            delta*np.array([plan.object_geometry.support_normal_base.x,plan.object_geometry.support_normal_base.y,plan.object_geometry.support_normal_base.z]),atol=1e-15)
    assert plan.object_geometry == original.object_geometry
    assert plan.refinement_translation_m == pytest.approx(original.refinement_translation_m+abs(delta))
    assert runtime['final_contact_height_margin']['ok']
    assert runtime['final_contact_height_margin']['translation_evidence_limit_m'] == .001
    assert runtime['contact_height_window_search']['deadline_extended'] is False
    assert runtime['contact_height_window_search']['maximum_proposals'] == 3
    assert len(runtime['contact_height_window_search']['proposals']) == 1
    assert sequence.grasp.pose == plan.poses[2]
    assert node._registered_sequence_geometry_gate.call_count == 1
    assert node._reject_inevitable_contact_goal.call_count == 2


def test_original_success_has_no_new_proposal_or_geometry_change():
    node, plan, prepared, runtime, good, _, _, _, _ = fixture(); original = deepcopy(plan)
    node._final_contact_sequence_evaluation = mock.Mock(return_value=(good,{}))
    node._contact_height_window_retry = mock.Mock(side_effect=AssertionError('original passed'))
    runtime['contact_height_window_search'] = {'old_build': True}
    node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert plan == original
    assert 'contact_height_window_search' not in runtime


def test_publication_rebuild_from_same_source_does_not_accumulate_height():
    node, source, prepared, runtime, good, bad, calls, _, _ = fixture()
    prepared.ticket = NS(request_id=40, generation=9, target_epoch=1)
    def strict(candidate, stages, p, r, operation, deadline_sec):
        calls.append((deepcopy(candidate),operation,deadline_sec))
        passed=operation=='final_registered_height_window_strict_sequence'
        r['contact_sequence_precheck']=dict(success=passed,failed_stage='grasp',reason=bad.reason,plan_id=candidate.plan_id)
        return (good if passed else bad),{}
    node._final_contact_sequence_evaluation=strict
    selection=deepcopy(source)
    node._strict_check_final_near_field_plan(selection,prepared,runtime)
    publication=deepcopy(source)  # actual builder reconstructs the frozen source family
    node._strict_check_final_near_field_plan(publication,prepared,runtime)
    assert publication==selection and publication!=source
    assert publication.plan_id==remote.compute_plan_id(publication)
    assert len(calls)==3 and all(c[2]==123. for c in calls)
    assert calls[0][0]==source
    assert calls[1][0]==calls[2][0]==selection
    assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False
    assert len(runtime['contact_height_window_search']['proposals'])==1


@pytest.mark.parametrize('rejections', [1, 2])
def test_necessary_collision_does_not_skip_original_alternatives(rejections):
    node,plan,prepared,runtime,_,_,calls,_,_=fixture()
    def necessary(candidate, state, stage):
        if stage=='final_registered_height_window_proposal' and runtime.get('necessary_rejections', 0)<rejections:
            runtime['necessary_rejections']=runtime.get('necessary_rejections', 0)+1
            raise remote.CandidateContractError('CONTACT_SEQUENCE_PRECHECK_FAILED','proposal inevitable collision')
    node._reject_inevitable_contact_goal=necessary
    _, result, _=node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert result.reachable and len(calls)==2
    proposals=runtime['contact_height_window_search']['proposals']
    assert len(proposals)==rejections+1 and not any(p['accepted'] for p in proposals[:-1]) and proposals[-1]['accepted']
    assert 'inevitable collision' in proposals[0]['reason']


@pytest.mark.parametrize('fault', ['source','not_registered','wrong_phase','nominal_collision','support_collision','wrong_stage','wrong_plan','timeout','missing_failure'])
def test_only_exact_registered_bias_target_collision_is_eligible(fault):
    node, plan, prepared, runtime, _, bad, _, _, _ = fixture()
    runtime['contact_sequence_precheck'] = dict(success=False,failed_stage='grasp',
        reason=bad.reason,plan_id=plan.plan_id)
    if fault=='source':runtime['scored_candidate'].stable_candidate.candidate_source='graspnet'
    if fault=='not_registered':plan.refinement_status='NOT_EVALUATED'
    if fault=='wrong_phase':plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    if fault=='nominal_collision':runtime['contact_sequence_precheck']['reason']='nominal continuous geometry: target OBB collision at 1-end/left_finger'
    if fault=='support_collision':runtime['contact_sequence_precheck']['reason']='predicted_command continuous geometry: support clearance 0.001m'
    if fault=='wrong_stage':runtime['contact_sequence_precheck']['failed_stage']='pregrasp'
    if fault=='wrong_plan':runtime['contact_sequence_precheck']['plan_id']='other'
    if fault=='timeout':bad=replace(bad,failure_code='MOVEIT_TIMEOUT')
    if fault=='missing_failure':runtime.pop('contact_sequence_precheck')
    original=deepcopy(plan)
    assert node._contact_height_window_retry(plan,prepared,runtime,bad,123.) is None
    assert plan == original
    node._registered_sequence_geometry_gate.assert_not_called()


@pytest.mark.parametrize('fault', ['expired','expired_after_check','registration_bound','geometry','both_strict_fail','incomplete_success','changed_contact_evidence'])
def test_failure_retains_original_plan_and_original_budget(fault):
    node, plan, prepared, runtime, good, bad, calls, clock, _ = fixture()
    if fault=='expired':clock[0]=124.
    if fault=='registration_bound':plan.refinement_translation_m=.025;plan.plan_id=remote.compute_plan_id(plan)
    if fault=='geometry':node._registered_sequence_geometry_gate=mock.Mock(return_value=CandidateGateResult(False,'GRIPPER_SWEEP_COLLISION','blocked',.04,0.,.001,1.,0.,0.,'static_envelope',4))
    original_call=node._final_contact_sequence_evaluation
    if fault in ('expired_after_check','both_strict_fail','incomplete_success'):
        def strict(*args,**kwargs):
            result, metrics=original_call(*args,**kwargs)
            if len(calls)>1:
                if fault=='expired_after_check':clock[0]=124.
                if fault=='both_strict_fail':result=bad
                if fault=='incomplete_success':result=replace(good,collision_free=None)
            return result,metrics
        node._final_contact_sequence_evaluation=strict
    if fault=='changed_contact_evidence':node._freeze_final_contact_evidence=lambda p,r:setattr(p,'contact_geometry_evidence','invalid')
    original=deepcopy(plan)
    with pytest.raises(remote.CandidateContractError):node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert plan == original
    assert len(calls)<=4  # Original plus the three bounded placement proposals.
    assert all(x[2]==123. for x in calls)
    assert 'final_execution_sequence' not in runtime
    assert 'final_strict_plan_id' not in runtime


def test_real_gen15_band_preserves_original_margin_and_solves_recorded_height_gap():
    """Rebuild curves for today's proposals, not the saved former 75% poses.

    The failed original MoveIt trajectory was not saved. These independent IK
    curves exercise the production continuous gripper checker; they do not
    certify the whole-arm scene or authorize hardware execution.
    """
    import hashlib
    import math
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    from genpy import Duration
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from alicia_flexible_grasp.robot.observation_path_guard import (
        SerialUrdfFk, FrozenObservationScene, validate_observation_trajectory)

    folder=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen15_contact_prediction')
    if not (folder/'height_full_geometry_and_curve.json').exists():pytest.skip('sealed Gen15 reconstruction is on experiment disk')
    report=json.loads((folder/'height_full_geometry_and_curve.json').read_text())
    endpoint=json.loads((folder/'endpoint_quantification.json').read_text())
    model=folder.parents[1]/'user_grasp_20261008T090751Z/source_snapshot_review/gen28_fk_ik_100ms.model.urdf'
    assert hashlib.sha256(model.read_bytes()).hexdigest()==endpoint['model_sha256']
    fk=SerialUrdfFk(model.read_text(),tuple('Joint%d'%i for i in range(1,7)))
    measured=endpoint['geometry'];registration=np.array(endpoint['registration'])
    normal=np.array(measured['support_normal_base'])
    scene=FrozenObservationScene('gen15_half_window_regression',
        hashlib.sha256((folder/'endpoint_quantification.json').read_bytes()).hexdigest(),
        1,normal,measured['support_offset_m'],
        registration[:3,:3]@measured['obb_center_base_m']+registration[:3,3],
        registration[:3,:3]@measured['R_base_obb'],np.array(measured['obb_size_xyz_m']))

    for row in report['rows'][:2]:
        original=row['trials'][0]
        original_family=[np.array(t) for t in original['T_base_tool0']]
        geometry=robust_geometry();geometry.update(
            actual_T_base_tool0=original_family[2],planned_T_base_tool0=original_family[2],
            contact_center_base=row['contact_center_base'],
            contact_height_axis_base=original_family[2][:3,2],
            contact_height_bounds_m=row['bounds_m'],support_normal_base=normal,
            support_offset_m=measured['support_offset_m'])
        limit=original['height']['translation_evidence_limit_m']
        proposals=contact_height_window_proposals(translation_evidence_limit_m=limit,**geometry)
        assert len(proposals)==3
        assert proposals[0][1]['normal_translation_m']==pytest.approx(.0018588949618017674)
        assert proposals[1][1]['normal_translation_m']==pytest.approx(.00278834244270262)
        assert proposals[2][1]['normal_translation_m']==pytest.approx(-.003062288484736227)
        source=next(r for r in endpoint['rows'] if r['index']==row['index'])
        bias=np.array(source['bias_sdk_minus_encoder_rad'])

        # Original must still collide; the new upward half-window proposal
        # must pass, while the downward proposal must remain rejected.
        outcomes=[]
        for delta in (0.,)+(tuple(p[1]['normal_translation_m'] for p in proposals)):
            family=[t.copy() for t in original_family]
            for t in family:t[:3,3]+=delta*normal
            height=evaluate_contact_height_margin(translation_evidence_limit_m=limit,
                **dict(geometry,actual_T_base_tool0=family[2],planned_T_base_tool0=family[2]))
            assert height['ok'] and height['translation_evidence_limit_m']==limit
            assert height['minimum_contact_overlap_m']==.002
            assert height['minimum_support_clearance_m']==.003
            first,last=family[1:3]
            count=max(2,int(math.ceil(np.linalg.norm(last[:3,3]-first[:3,3])/.003))+1)
            trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=list(fk.names)
            q=np.array(source['offline_ik']['q'])
            for i,alpha in enumerate(np.linspace(0.,1.,count)):
                target=first.copy();target[:3,3]=(1.-alpha)*first[:3,3]+alpha*last[:3,3]
                def residual(joints):
                    actual=fk(joints)
                    return np.r_[actual[:3,3]-target[:3,3],
                        .2*Rotation.from_dcm(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
                fit=least_squares(residual,q,bounds=(fk.limits[:,0],fk.limits[:,1]),
                    max_nfev=100,xtol=1e-12,ftol=1e-12,gtol=1e-12)
                assert np.linalg.norm(residual(fit.x))<1e-7
                q=fit.x
                trajectory.joint_trajectory.points.append(JointTrajectoryPoint(
                    positions=q.tolist(),velocities=[0.]*6,accelerations=[0.]*6,
                    time_from_start=Duration(i)))
            checked=[]
            for shift in (np.zeros(6),bias):
                command=deepcopy(trajectory)
                for point in command.joint_trajectory.points:
                    point.positions=(np.array(point.positions)+shift).tolist()
                try:
                    validate_observation_trajectory(command,scene,fk,geometry['gripper'],
                        .04975,command.joint_trajectory.points[0])
                    checked.append(True)
                except ValueError as exc:
                    assert 'target OBB collision' in str(exc)
                    checked.append(False)
            outcomes.append(checked)
        assert outcomes==[[True,False],[True,True],[True,True],[True,False]]
