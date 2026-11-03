"""Height retry diagnostics preserve every original failure and bound."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from unittest import mock

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_matrix

from test_contact_height_window_proposals import fixture, remote
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, validate_observation_trajectory)
from alicia_flexible_grasp.robot.contact_sequence_precheck import (
    contact_precheck_gripper, predict_contact_command_path)
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals


def capture(monkeypatch):
    reports=[]
    monkeypatch.setattr(remote.rospy,'loginfo',lambda fmt,*a:
                        reports.append(json.loads(a[0])) if 'contact height retry:' in fmt else None)
    return reports


@pytest.mark.parametrize('condition', ['failure_code','failure_success','failed_stage',
    'failure_kind','failure_plan_id','candidate_source','model_choice','registration_status',
    'plan_phase','direct_near_field','deadline'])
def test_exact_first_ineligible_condition_preserves_skip(monkeypatch,condition):
    node,plan,prepared,runtime,_,bad,calls,_,_=fixture();reports=capture(monkeypatch)
    runtime['contact_sequence_precheck']=dict(success=False,failed_stage='grasp',reason=bad.reason,plan_id=plan.plan_id)
    deadline=123.
    if condition=='failure_code':bad=replace(bad,failure_code='MOVEIT_TIMEOUT')
    if condition=='failure_success':runtime['contact_sequence_precheck']['success']=True
    if condition=='failed_stage':runtime['contact_sequence_precheck']['failed_stage']='approach'
    if condition=='failure_kind':runtime['contact_sequence_precheck']['reason']='nominal collision'
    if condition=='failure_plan_id':runtime['contact_sequence_precheck']['plan_id']='other'
    if condition=='candidate_source':runtime['scored_candidate'].stable_candidate.candidate_source='graspnet'
    if condition=='model_choice':plan.model_choice='other'
    if condition=='registration_status':plan.refinement_status='NOT_EVALUATED'
    if condition=='plan_phase':plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    if condition=='direct_near_field':node._direct_near_field_active=lambda _:False
    if condition=='deadline':deadline=0.
    original=deepcopy(plan)
    assert node._contact_height_window_retry(plan,prepared,runtime,bad,deadline) is None
    assert plan==original and not calls
    assert reports[-1]['first_ineligible_condition']==condition
    assert reports[-1]['eligible'] is False and reports[-1]['execution_authorized'] is False
    assert reports[-1]['source_stamp_ns']==plan.header.stamp.to_nsec()
    assert reports[-1]['plan_id']==plan.plan_id and reports[-1]['search'] is None
    node._registered_sequence_geometry_gate.assert_not_called()


@pytest.mark.parametrize('fault',['geometry','deadline','strict'])
def test_rejected_search_survives_runtime_clear_in_log(monkeypatch,fault):
    node,plan,prepared,runtime,_,bad,calls,clock,_=fixture();reports=capture(monkeypatch)
    if fault=='geometry':
        node._registered_sequence_geometry_gate.return_value=remote.CandidateGateResult(
            False,'GRIPPER_SWEEP_COLLISION','original gate rejected',.04,0.,.001,1.,0.,0.,'static_envelope',4)
        node._registered_sequence_geometry_gate.side_effect=None
    if fault=='deadline':clock[0]=124.
    if fault=='strict':
        original=node._final_contact_sequence_evaluation
        def reject(*a,**kw):
            _,metrics=original(*a,**kw);return bad,metrics
        node._final_contact_sequence_evaluation=reject
    before=deepcopy(plan)
    with pytest.raises(remote.CandidateContractError):node._strict_check_final_near_field_plan(plan,prepared,runtime)
    report=reports[-1];runtime.clear()
    assert plan==before and report['eligible'] is True
    assert report['search']['original_plan_id']==before.plan_id
    assert report['search']['deadline_sec']==123.
    assert report['search']['deadline_extended'] is False
    if fault=='deadline':assert 'deadline consumed' in report['search']['reason']
    elif fault=='geometry':assert all(p['geometry_gate']['ok'] is False for p in report['search']['proposals'])
    else:assert all(p['result']['reachable'] is False for p in report['search']['proposals'])


def test_logging_failure_cannot_reject_original_success_or_mask_exception(monkeypatch):
    node,plan,prepared,runtime,_,bad,calls,_,_=fixture()
    monkeypatch.setattr(remote.rospy,'loginfo',mock.Mock(side_effect=RuntimeError('log unavailable')))
    _,result,_=node._strict_check_final_near_field_plan(plan,prepared,runtime)
    assert result.reachable and len(calls)==2
    runtime['contact_sequence_precheck']=dict(success=False,failed_stage='grasp',reason=bad.reason,plan_id=plan.plan_id)
    error=remote.CandidateContractError('CONTACT_HEIGHT_REGISTRATION_MARGIN','original exception')
    node._contact_height_evidence_limit=mock.Mock(side_effect=error)
    with pytest.raises(remote.CandidateContractError) as caught:
        node._contact_height_window_retry(plan,prepared,runtime,bad,123.)
    assert caught.value is error


def test_gen30_retains_75_percent_fallback_when_new_half_proposal_collides():
    """Actual frozen plan; reconstructed curves never claim recorded MoveIt or authority."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    folder=Path(__file__).parent/'fixtures'
    d=json.loads((folder/'gen30_contact_height_counterfactual.json').read_text())
    model=json.loads((folder/'contact_command_collision_20260930.json').read_text())['robot_description']
    assert hashlib.sha256(model.encode()).hexdigest()==d['sample']['model_sha256']
    fk=SerialUrdfFk(model,tuple(d['sample']['joint_names']))
    gripper=contact_precheck_gripper(d['gripper_config'])
    poses=[]
    for values in d['poses_xyz_xyzw']:
        T=quaternion_matrix(values[3:]);T[:3,3]=values[:3];poses.append(T)
    G=quaternion_matrix(d['object_geometry']['pose_xyz_xyzw'][3:]);G[:3,3]=d['object_geometry']['pose_xyz_xyzw'][:3]
    n=np.array(d['object_geometry']['support_normal_base']);e=d['contact_geometry_evidence']
    geometry=dict(gripper=gripper,actual_T_base_tool0=poses[2],planned_T_base_tool0=poses[2],
        contact_center_base=e['contact_center_base'],contact_height_axis_base=e['contact_height_axis_base'],
        contact_height_bounds_m=e['contact_height_bounds_m'],minimum_contact_overlap_m=e['minimum_contact_overlap_m'],
        support_normal_base=n,support_offset_m=d['object_geometry']['support_offset_m'],opening_width_m=d['opening_m'])
    options=contact_height_window_proposals(translation_evidence_limit_m=d['height_limit_m'],**geometry)
    delta=options[0][1]['normal_translation_m']
    assert delta==pytest.approx(d['curves'][1]['normal_translation_m']*2./3.,abs=1e-12)
    assert len(options)==3
    assert options[1][1]['normal_translation_m']==pytest.approx(d['curves'][1]['normal_translation_m'],abs=1e-12)
    assert options[0][1]['after']['ok']
    assert options[0][1]['after']['translation_evidence_limit_m']==d['height_limit_m']
    scene=FrozenObservationScene(d['plan_id'],'diagnostic-counterfactual',d['source_stamp_ns'],n,
        d['object_geometry']['support_offset_m'],G[:3,3],G[:3,:3],np.array(d['object_geometry']['size_xyz_m']))
    # Keep the original rejection and former 75% control, then reconstruct the
    # actual current 50% proposal. Saved 75% curves cannot validate a new pose.
    rows=deepcopy(d['curves']);positions=[]
    q=np.array(rows[0]['nominal_positions_rad'][0])
    for alpha in np.linspace(0.,1.,13):
        target=poses[2].copy()
        target[:3,3]=(1.-alpha)*poses[1][:3,3]+alpha*poses[2][:3,3]+delta*n
        def residual(joints):
            actual=fk(joints)
            return np.r_[actual[:3,3]-target[:3,3],
                .2*Rotation.from_dcm(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
        fit=least_squares(residual,q,bounds=(fk.limits[:,0],fk.limits[:,1]),
            max_nfev=100,xtol=1e-12,ftol=1e-12,gtol=1e-12)
        assert np.linalg.norm(residual(fit.x))<1e-7
        q=fit.x;positions.append(q.tolist())
    rows.append(dict(normal_translation_m=delta,nominal_positions_rad=positions))
    for index,row in enumerate(rows):
        goal=poses[2].copy();goal[:3,3]+=row['normal_translation_m']*n
        path=RobotTrajectory();path.joint_trajectory.joint_names=list(fk.names)
        for i,q in enumerate(row['nominal_positions_rad']):
            expected=(1-i/12)*poses[1][:3,3]+i/12*poses[2][:3,3]+row['normal_translation_m']*n
            np.testing.assert_allclose(fk(q)[:3,3],expected,atol=1e-8)
            path.joint_trajectory.points.append(JointTrajectoryPoint(positions=q,velocities=[0.]*6,
                accelerations=[0.]*6,time_from_start=rospy.Duration(i)))
        goalmsg=Pose()
        for k,v in zip('xyz',goal[:3,3]):setattr(goalmsg.position,k,float(v))
        for k,v in zip('xyzw',d['poses_xyz_xyzw'][2][3:]):setattr(goalmsg.orientation,k,v)
        shifted,proof=predict_contact_command_path(path,fk,d['sample'],goalmsg)
        assert proof['future_following_verified'] is False and proof['execution_authorized'] is False
        validate_observation_trajectory(path,scene,fk,gripper,d['opening_m'],path.joint_trajectory.points[0])
        if index in (0,2):  # Original and new smaller proposal still collide.
            with pytest.raises(ValueError,match='target OBB collision at 11-end/left_finger'):
                validate_observation_trajectory(shifted,scene,fk,gripper,d['opening_m'],shifted.joint_trajectory.points[0])
        else:
            validate_observation_trajectory(shifted,scene,fk,gripper,d['opening_m'],shifted.joint_trajectory.points[0])
