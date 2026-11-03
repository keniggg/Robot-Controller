"""Exact G51 terminal counterexamples; no motion or ROS master access."""
from copy import deepcopy
from itertools import product
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot import contact_sequence_precheck as module
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, wire_digest)
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry


def case(index=0, zero_bias=False):
    data=json.loads((Path(__file__).parent/'fixtures/gen52_g51_contact_collision_inputs.json').read_text())['cases'][index]
    fk=SerialUrdfFk(data['model']['xml'],data['following_sample']['joint_names'])
    assert fk.model_sha256==data['model']['sha256']
    sample=deepcopy(data['following_sample'])
    bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    nominal=RobotTrajectory();nominal.joint_trajectory.joint_names=data['checked_curve']['joint_names']
    for p in data['checked_curve']['points']:
        ns=p['time_from_start_ns']
        nominal.joint_trajectory.points.append(JointTrajectoryPoint(
            positions=(np.array(p['positions'])-bias).tolist(),velocities=p['velocities'],
            accelerations=p['accelerations'],time_from_start=rospy.Duration(ns//10**9,ns%10**9)))
    if zero_bias:
        # Explicit provider counterfactual for the already proved conditional
        # actual curve. No new physical zero-bias measurement is claimed.
        sample['sdk_positions_rad']=list(nominal.joint_trajectory.points[0].positions)
        sample['accepted_positions_rad']=list(nominal.joint_trajectory.points[0].positions)
    scene=FrozenObservationScene(**{k:np.array(v) if k in ('normal','center','rotation','size') else v
        for k,v in data['scene'].items()})
    g=GripperGeometry(**{k:np.array(v) if isinstance(v,list) else v
        for k,v in data['config']['evaluated_gripper'].items()})
    goal=Pose();vals=data['plan_data']['poses_xyz_xyzw'][2]
    for key,value in zip('xyz',vals[:3]):setattr(goal.position,key,value)
    for key,value in zip('xyzw',vals[3:]):setattr(goal.orientation,key,value)
    planner=NS(_retime_cartesian_plan=mock.Mock(side_effect=deepcopy),
        _same_joint_path_geometry=lambda a,b:wire_digest(a)==wire_digest(b))
    return data,fk,sample,nominal,scene,g,goal,planner



def _assert_complete_original_vertex_terminal_separation(fk, command, scene, opening, proof):
    """Reproject every original CAD vertex; the candidate axis is only a hint."""
    assert proof['separated'] is True
    assert proof['padding_each_tool_axis_m'] == .0006
    assert proof['displacement_each_tool_axis_m'] == 0.
    assert proof['execution_authorized'] is False
    assert proof['provenance']['robot_description_sha256'] == fk.model_sha256
    transform = fk(command.joint_trajectory.points[-1].positions)
    parts = fk.finger_cad_model._body_components('left_finger')
    vertices = np.concatenate([
        part.vertices_at_zero + opening * part.velocity_per_opening
        for part in parts])
    offsets = np.asarray(list(product((-.0006, .0006), repeat=3)))
    padded = (vertices[:, None, :] + offsets[None, :, :]).reshape(-1, 3)
    finger = padded @ transform[:3, :3].T
    target = (np.asarray(list(product((-.5, .5), repeat=3))) * scene.size) @ scene.rotation.T
    target += scene.center - transform[:3, 3]
    old_axes = np.concatenate((
        transform[:3, :3].T, scene.rotation.T,
        np.cross(transform[:3, :3].T[:, None, :],
                 scene.rotation.T[None, :, :]).reshape(-1, 3)))
    norms = np.linalg.norm(old_axes, axis=1)
    old_axes = old_axes[norms > 1e-12] / norms[norms > 1e-12, None]
    old_finger, old_target = finger @ old_axes.T, target @ old_axes.T
    lower, upper = old_finger.min(axis=0), old_finger.max(axis=0)
    old_gap = np.maximum(old_target.min(axis=0) - upper,
                         lower - old_target.max(axis=0)).max()
    old_reserve = 64 * np.finfo(float).eps * (1 + max(
        np.max(np.abs(scene.center)), np.max(np.abs(transform[:3, 3])),
        np.max(scene.size), np.max(np.abs(lower)), np.max(np.abs(upper))))
    assert old_gap - old_reserve <= 1e-9  # Old fifteen axes were inconclusive.
    axis = np.asarray(proof['axis_base'])
    assert np.linalg.norm(axis) == pytest.approx(1., abs=1e-12)
    projected_finger, projected_target = finger @ axis, target @ axis
    lo, hi = projected_finger.min(), projected_finger.max()
    target_lo, target_hi = projected_target.min(), projected_target.max()
    reserve = max(old_reserve, 64 * np.finfo(float).eps * (1 + max(
        np.max(np.abs(scene.center)), np.max(np.abs(transform[:3, 3])),
        np.max(scene.size), abs(lo), abs(hi))))
    gap = max(target_lo - hi, lo - target_hi) - reserve
    assert gap > 1e-9
    assert gap == pytest.approx(proof['gap_m'], abs=2e-14)
    assert gap == pytest.approx(.00005333643473869622, abs=1e-12)

@pytest.mark.parametrize('index',range(3))
def test_original_three_counterexamples_reject_exact_terminal_not_claim_first_curve_collision(index):
    d,fk,sample,nominal,scene,g,goal,_=case(index)
    command,_=module.predict_contact_command_path(nominal,fk,sample,goal)
    original_data=deepcopy(d)
    cad_model=fk.finger_cad_model
    if index in (1, 2):
        # Preserve the conservative historical rectangle fallback. Supported
        # CAD endpoint separation is independently checked after restoration.
        assert cad_model is not None
        fk.finger_cad_model=None
    with pytest.raises(module.ObservationPathError,match='target OBB collision.*left_finger') as caught:
        module.contact_curve_terminal_collision(command,fk,scene,g,d['opening_m'])
    value=caught.value.contact_replay_sample
    assert value['endpoint_screen_only'] is True
    assert value['first_collision_along_curve_claimed'] is False
    assert value['sample_selection_policy']=='first_tested_terminal_not_first_collision_along_curve'
    assert value['other_ik_branches_rejected'] is False
    assert value['checker_options']=={}
    assert value['validation_kind']=='terminal_endpoint_screen'
    assert value['checked_point_count']==1
    assert value['continuous_validation_executed'] is False
    assert value['trajectory_point_index']==len(command.joint_trajectory.points)-1
    assert value['trajectory_point_time_ns']==d['checked_curve']['points'][-1]['time_from_start_ns']
    np.testing.assert_array_equal(value['joint_positions_rad'],command.joint_trajectory.points[-1].positions)
    np.testing.assert_allclose(value['T_base_tool0'],fk(value['joint_positions_rad']),atol=1e-15)
    if index==0:
        assert d['first_failed_sample']['label']=='12-end'
        assert value['label']=='13-end'  # Terminal-first ordering is explicit.
    fk.finger_cad_model=cad_model
    if index==1:
        # The old fifteen axes were inconclusive; complete original-vertex
        # projection proves this terminal clear without certifying its path.
        report=module.contact_curve_terminal_collision(command,fk,scene,g,d['opening_m'])
        assert report['checked'] is True and report['collision_at_terminal'] is False
        assert report['scope']=='same_proposed_curve_endpoint_only'
        assert report['path_verified'] is False
        proof=next(v['proof'] for v in report['cad_target_projection_proofs']
                   if v['body']=='left_finger')
        _assert_complete_original_vertex_terminal_separation(
            fk,command,scene,d['opening_m'],proof)
    if index==2:
        # This is only endpoint clearance. It never certifies the full curve.
        import hashlib
        hull_fixture=Path(__file__).parent/'fixtures/gen55_finger_cad_actual_receipts.json'
        assert hashlib.sha256(hull_fixture.read_bytes()).hexdigest()=='f4c4e8f3f6624d85427de54026fc7dfc925fe14a6f104dd1b255d6efed0d12da'
        hull_data=json.loads(hull_fixture.read_text())
        assert d['opening_m']==hull_data['opening_m']
        report=module.contact_curve_terminal_collision(command,fk,scene,g,d['opening_m'])
        assert report['checked'] is True and report['collision_at_terminal'] is False
        assert report['scope']=='same_proposed_curve_endpoint_only'
        assert report['path_verified'] is False
        proof=next(v['proof'] for v in report['cad_target_projection_proofs']
                   if v['body']=='left_finger')
        T=fk(command.joint_trajectory.points[-1].positions)
        vertices=np.asarray(hull_data['hulls']['left_finger'])@T[:3,:3].T+T[:3,3]
        axes=list(T[:3,:3].T)+list(scene.rotation.T)
        axes += [np.cross(u,v) for u in T[:3,:3].T for v in scene.rotation.T]
        gaps=[]
        for axis in axes:
            norm=np.linalg.norm(axis)
            if norm<=1e-12:continue
            axis=axis/norm;projection=vertices@axis
            mid=float(scene.center@axis)
            radius=float(np.abs(scene.rotation.T@axis)@(scene.size/2.))
            gaps.append(max(mid-radius-projection.max(),projection.min()-mid-radius))
        assert max(gaps)==pytest.approx(.0006552818936254919,abs=1e-12)
        assert proof['separated'] is True
        assert proof['gap_m']==pytest.approx(max(gaps),abs=1e-12)
        assert proof['padding_each_tool_axis_m']==.0006
        assert proof['displacement_each_tool_axis_m']==0.
        assert proof['provenance']['robot_description_sha256']==fk.model_sha256
        assert proof['execution_authorized'] is False
    assert d==original_data  # Historical failure/inputs are not reclassified.


@pytest.mark.parametrize('index',range(3))
def test_same_branch_terminal_failure_retimes_once_and_preserves_typed_replay(index,monkeypatch):
    d,fk,sample,nominal,scene,g,goal,planner=case(index)
    original=wire_digest(nominal),deepcopy(sample)
    real_continuous=module.validate_observation_trajectory
    cad_model=fk.finger_cad_model
    if index in (1, 2):
        # Retain every original failure/receipt assertion for the old box.
        assert cad_model is not None
        fk.finger_cad_model=None
    proof=mock.Mock(side_effect=AssertionError('unnecessary continuous proof'))
    collision=mock.Mock(side_effect=AssertionError('no whole-arm service'))
    monkeypatch.setattr(module,'validate_observation_trajectory',proof)
    def retime(curve):
        curve.joint_trajectory.header.seq=22
        for p in curve.joint_trajectory.points:
            ns=2*p.time_from_start.to_nsec()
            p.time_from_start=rospy.Duration(ns//10**9,ns%10**9)
        return curve
    planner._retime_cartesian_plan.side_effect=retime
    planner._same_joint_path_geometry=lambda a,b: (
        [list(p.positions) for p in a.joint_trajectory.points]
        ==[list(p.positions) for p in b.joint_trajectory.points])
    sink=mock.Mock(return_value={'diagnostic_only':True,'execution_authorized':False})
    with pytest.raises(module.ContactPrecheckError,match='predicted_command continuous geometry: target OBB collision') as caught:
        module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
            RobotState(),collision,lambda:5.,failure_sink=sink)
    assert caught.value.code=='CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert caught.value.contact_replay_sample['endpoint_screen_only'] is True
    assert caught.value.contact_replay_receipt['execution_authorized'] is False
    assert sink.call_args[0][3:]==('predicted_command','grasp')
    retimed=sink.call_args[0][1]
    assert retimed.joint_trajectory.header.seq==22
    assert caught.value.contact_replay_sample['trajectory_point_time_ns']==retimed.joint_trajectory.points[-1].time_from_start.to_nsec()
    assert caught.value.contact_replay_sample['trajectory_point_time_ns']==2*d['checked_curve']['points'][-1]['time_from_start_ns']
    proof.assert_not_called();collision.assert_not_called();planner._retime_cartesian_plan.assert_called_once()
    assert original==(wire_digest(nominal),sample)
    fk.finger_cad_model=cad_model
    if index in (1, 2):
        # With the measured CAD union, both full continuous proofs must run.
        # A deliberate downstream refusal must still prevent any success.
        # Nominal is the original fixture's bias reconstruction, not wire data.
        reports=[]
        def complete_proof(*args,**kwargs):
            report=real_continuous(*args,**kwargs)
            reports.append(report)
            return report
        proof=mock.Mock(side_effect=complete_proof)
        monkeypatch.setattr(module,'validate_observation_trajectory',proof)
        planner._retime_cartesian_plan.reset_mock();sink.reset_mock()
        downstream_refusal=RuntimeError('isolated whole-arm gate refusal')
        collision=mock.Mock(side_effect=downstream_refusal)
        with pytest.raises(RuntimeError) as refusal:
            module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
                RobotState(),collision,lambda:5.,failure_sink=sink)
        assert refusal.value is downstream_refusal
        assert proof.call_count==2 and planner._retime_cartesian_plan.call_count==2
        collision.assert_called_once();sink.assert_not_called()
        command,_=module.predict_contact_command_path(nominal,fk,sample,goal)
        for expected,call,report in zip((nominal,command),proof.call_args_list,reports):
            retimed=call[0][0]
            assert len(retimed.joint_trajectory.points)==len(expected.joint_trajectory.points)
            for actual,source in zip(retimed.joint_trajectory.points,expected.joint_trajectory.points):
                np.testing.assert_array_equal(actual.positions,source.positions)
                assert actual.time_from_start.to_nsec()==2*source.time_from_start.to_nsec()
            assert retimed.joint_trajectory.header.seq==22
            assert report['trajectory_sha256']==wire_digest(retimed)
            assert report['scope']=='continuous_commanded_Link6_Link7_Link8_only'
            assert report['model_sha256']==fk.model_sha256
            assert report['segments']==len(expected.joint_trajectory.points)-1
            assert report['interval_checks']>report['segments']
            assert report['opening_width_m']==d['opening_m']
            assert report['target_cad_projection']['target_envelope_unchanged'] is True
            assert report['target_cad_projection']['support_and_palm_checks_unchanged'] is True
        assert original==(wire_digest(nominal),sample)


def test_clear_endpoints_cannot_hide_intermediate_collision(monkeypatch):
    d,fk,sample,nominal,scene,g,goal,planner=case(zero_bias=True)
    # Explicit synthetic curve: original safe actual terminal, with one
    # original colliding actuator endpoint as an interior waypoint.
    nominal.joint_trajectory.points[7].positions=list(d['checked_curve']['points'][-1]['positions'])
    for p in nominal.joint_trajectory.points:p.velocities=[0.]*6;p.accelerations=[0.]*6
    assert module.contact_curve_terminal_collision(nominal,fk,scene,g,d['opening_m'])['checked'] is True
    real=module.validate_observation_trajectory;proof=mock.Mock(side_effect=real)
    monkeypatch.setattr(module,'validate_observation_trajectory',proof)
    collision=mock.Mock()
    with pytest.raises(module.ContactPrecheckError,match='nominal continuous geometry: target OBB collision'):
        module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
            RobotState(),collision,lambda:5.)
    assert proof.call_count==1
    planner._retime_cartesian_plan.assert_called_once();collision.assert_not_called()


def test_clear_endpoint_still_runs_both_original_full_proofs_and_collision_gate(monkeypatch):
    d,fk,sample,nominal,scene,g,goal,planner=case(zero_bias=True)
    proof=mock.Mock(wraps=module.validate_observation_trajectory)
    monkeypatch.setattr(module,'validate_observation_trajectory',proof)
    collision=mock.Mock(return_value=23)
    report=module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
        RobotState(),collision,lambda:5.)
    assert proof.call_count==2 and planner._retime_cartesian_plan.call_count==2
    collision.assert_called_once();assert report['collision_state_checks']==23
    assert all(v['path_verified'] is False for v in report['same_curve_terminal_screens'].values())
    assert report['execution_authorized'] is False and report['future_following_verified'] is False


@pytest.mark.parametrize('fault',('deadline','cancel'))
def test_deadline_or_control_cancel_before_screen_is_unchanged(fault,monkeypatch):
    d,fk,sample,nominal,scene,g,goal,planner=case()
    screen=mock.Mock(side_effect=AssertionError('screen cannot bypass cancellation'))
    monkeypatch.setattr(module,'contact_curve_terminal_collision',screen)
    error=module.ContactPrecheckError('original deadline','MOVEIT_TIMEOUT') if fault=='deadline' else RuntimeError('original control revoke')
    def remaining():raise error
    with pytest.raises(type(error)) as caught:
        module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
            RobotState(),mock.Mock(),remaining)
    assert caught.value is error
    screen.assert_not_called();planner._retime_cartesian_plan.assert_not_called()


def test_cancel_between_two_endpoint_screens_cannot_become_a_collision():
    d,fk,sample,nominal,scene,g,goal,planner=case()
    calls=[];error=RuntimeError('control revoked before second endpoint')
    def remaining():
        calls.append(1)
        if len(calls)==2:raise error
        return 5.
    with pytest.raises(RuntimeError) as caught:
        module.validate_contact_prediction(nominal,fk,sample,goal,scene,g,d['opening_m'],planner,
            RobotState(),mock.Mock(),remaining)
    assert caught.value is error and len(calls)==2
    planner._retime_cartesian_plan.assert_not_called()
