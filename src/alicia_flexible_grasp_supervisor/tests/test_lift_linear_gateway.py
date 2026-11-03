"""Offline gateway/task contracts. Every transport and hardware dependency is fake."""
from copy import deepcopy
import json
import threading
import time
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from moveit_msgs.msg import RobotState

from alicia_flexible_grasp.robot import lift_linear_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, wire_digest
from alicia_flexible_grasp.robot.lift_moveit_adapter import KIND,POLICY
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD
from test_lift_path_continuity import recorded,point
from test_motion_gateway_controller_start import MotionGateway
from test_pregrasp_task_compensation import task_case,task


def case(monkeypatch,capabilities=True):
    data,fk,plan,tr,sample,opening=recorded()
    sample.update(measurement_kind='stationary_sdk_following_not_executed_endpoint',
        accepted_gripper_opening_m=opening,opening_stamp_ns=sample['accepted_stamp_ns'])
    cfg=dict(unknown_lift_cartesian_compensation_enabled=True,measured_endpoint_check_enabled=True)
    bodies={name:dict(frame_id='tool0',T_tool_box=np.eye(4).tolist(),size_xyz_m=[.01]*3,
                       source_sha256='a'*64) for name in ('camera','mount')} if capabilities else {}
    constraints={n:dict(trajectory=.12,goal=.035) for n in ARM_NAMES}
    constraints.update(goal_time=2.,stopped_velocity_tolerance=.01)
    params={'/grasp':cfg,'/robot_description':data['robot_description'],
        '/grasp_mode/selection':dict(mode='unknown',strategy='direct',selection_id='current'),
        '/grasp_6d/remote/gripper_geometry':data['parameters']['/grasp_6d']['remote']['gripper_geometry'],
        module.BODY_PARAM:bodies,'/robot/observation_tracking_contract_enabled':True,
        '/alicia_d_driver_node/endpoint_feedback_trim_enabled':False,
        '/alicia_controller/constraints':constraints,'/alicia_controller/stop_trajectory_duration':.1}
    monkeypatch.setattr(module.rospy,'get_param',lambda name,default=None:deepcopy(params.get(name,default)))
    monkeypatch.setattr(module.rospy,'get_time',lambda:sample['accepted_stamp_ns']*1e-9+.05)
    monkeypatch.setattr(module.rospy,'is_shutdown',lambda:False)
    node=MotionGateway.__new__(MotionGateway)
    node._planner_lock=threading.RLock();node._observation_context_lock=threading.RLock()
    node._control_reference_epoch_ns=sample['epoch_ns']
    node._observation_contexts=NS(revocation=1)
    node._pregrasp_task_stage=module.GraspStages.LIFT_OBJECT
    node._compensation_task_label='planning linear 6D lift'
    node._require_live_observation_task=mock.Mock()
    node._manual_control_active=lambda:False
    node._fresh_actuation_status=lambda:'CONFIRMED:unit_test'
    node._endpoint_following_snapshot=mock.Mock(side_effect=lambda *a,**k:deepcopy(sample))
    node._observation_controller_hold=lambda names:point(sample['sdk_positions_rad'])
    node._ensure_trajectory_controllers_started=mock.Mock(return_value=(True,'fake running'))
    state=RobotState();state.joint_state.name=list(ARM_NAMES)+['right_finger']
    state.joint_state.position=list(sample['accepted_positions_rad'])+[opening]
    planner=NS(robot=NS(get_current_state=lambda:deepcopy(state)),
        _plan_cartesian_from_start_state=mock.Mock(return_value=(deepcopy(tr),'planned')),
        _retime_cartesian_plan=deepcopy,_last_pose_plan=None,
        _same_joint_path_geometry=lambda a,b:np.array_equal(
            [p.positions for p in a.joint_trajectory.points],[p.positions for p in b.joint_trajectory.points]),
        _pose_xyz_text=lambda pose:'lift',observation_trajectory_executor=object())
    planner._remember_pose_plan=lambda pose,path,kind:setattr(planner,'_last_pose_plan',dict(plan=path,kind=kind))
    def execute(*args,**kwargs):
        assert planner.observation_tracking_contract_required is True
        assert planner.observation_path_guard_required is True
        path=deepcopy(planner._last_pose_plan['plan'])
        audit=planner.observation_path_validator(path)
        assert planner.observation_execution_authorized()
        kwargs['before_execute']()
        planner.observation_submission_revalidate(path,audit)
        return True,'fake proof-bound action submitted'
    planner._execute_cached_pose_plan=mock.Mock(side_effect=execute)
    node._ensure_planner=mock.Mock(return_value=planner)
    # Geometry is independently tested with actual models in the pure suite.
    # This fake isolates gateway ordering and proof/transport binding only.
    def pure_result(nominal,shifted,plan,frozen,fk,sample,opening,gripper,bodies):
        return {key:dict(trajectory_sha256=wire_digest(path),scene_sha256=frozen.scene.digest,
            model_sha256=fk.model_sha256,accepted_opening_m=opening,stationary_world_target_removed=False,
            attachment_confirmed=False) for key,path in [('nominal',nominal),('shifted_with_sdk_bridge',shifted)]}
    pure=mock.Mock(side_effect=pure_result)
    monkeypatch.setattr(module,'validate_lift_pair',pure)
    def provider(**kwargs):
        frozen=kwargs['frozen'];shifted=kwargs['shifted']
        contract=dict(policy=POLICY,stop_trajectory_duration_sec=kwargs['stop_duration'])
        return dict(trajectory_sha256=wire_digest(shifted),
            nominal_trajectory_sha256=wire_digest(kwargs['nominal']),scene_sha256=frozen.scene.digest,
            model_sha256=fk.model_sha256,epoch_ns=frozen.epoch_ns,opening_width_m=opening,
            kind=KIND,whole_arm_collision_method='sampled_exact_Noetic_curve_two_request_local_hypotheses',
            hardware_following_swept_volume_certified=False,hardware_stopping_swept_volume_certified=False,
            sdk_hold_positions_rad=sample['sdk_positions_rad'],moveit_scene_sha256='scene',
            missing_rigid_tool_body_models=sorted({'camera','mount'}-{b.name for b in kwargs['bodies']}),
            includes_sdk_bridge=True,stationary_world_target_removed=False,attachment_confirmed=False,
            controller_constraints_snapshot=deepcopy(kwargs['constraints']),
            execution_tracking_contract=contract)
    node._closed_lift_contract_validator=mock.Mock(side_effect=provider)
    node._closed_lift_contract_validator.scene_digest=mock.Mock(return_value='scene')
    return node,planner,plan,sample,params,pure


def req(plan,execute=False):return NS(plan=deepcopy(plan),execute=execute)


def test_disabled_route_does_not_plan_start_controller_or_create_publisher(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    params['/grasp'][module.ENABLED_KEY]=False
    response=node.handle_contact_lift(req(plan))
    assert not response.success and 'LIFT_COMPENSATION_DISABLED' in response.message
    node._ensure_planner.assert_not_called();node._ensure_trajectory_controllers_started.assert_not_called()


def test_missing_optional_body_models_are_explicit_diagnostic_coverage(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch,capabilities=False)
    response=node.handle_contact_lift(req(plan))
    assert response.success,response.message
    result=json.loads(response.message)
    assert result['proof']['missing_rigid_tool_body_models']==['camera','mount']
    assert not result['motion_authorized'] and not result['attachment_confirmed']
    node._ensure_trajectory_controllers_started.assert_not_called();p._execute_cached_pose_plan.assert_not_called()


def test_complete_fake_certificate_routes_full_path_once_through_contracted_executor(monkeypatch):
    node,p,plan,s,params,pure=case(monkeypatch)
    response=node.handle_contact_lift(req(plan));assert response.success,response.message
    result=json.loads(response.message)
    assert not result['motion_authorized'] and not result['proof']['motion_authorized']
    cached=deepcopy(node._lift_linear_cache['trajectory'])
    assert len(cached.joint_trajectory.points)==13
    np.testing.assert_allclose(cached.joint_trajectory.points[0].positions,s['sdk_positions_rad'])
    node._ensure_trajectory_controllers_started.assert_not_called()
    assert node.handle_contact_lift(req(plan,True)).success
    assert not node.handle_contact_lift(req(plan,True)).success
    p._execute_cached_pose_plan.assert_called_once()
    assert node._lift_linear_cache is None and p._last_pose_plan is None
    assert node._compensation_visual_context is None
    assert not hasattr(p,'observation_path_validator')
    assert pure.call_count==2


@pytest.mark.parametrize('fault',['epoch','manual','phase','config','model','geometry','bodies','selection','revocation'])
def test_authority_change_consumes_cache_without_execution(monkeypatch,fault):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    if fault=='epoch':node._control_reference_epoch_ns+=1
    if fault=='manual':node._manual_control_active=lambda:True
    if fault=='phase':node._pregrasp_task_stage=module.GraspStages.APPROACH_TARGET
    if fault=='config':params['/grasp']['new_key']=1
    if fault=='model':params['/robot_description']+=' '
    if fault=='geometry':params['/grasp_6d/remote/gripper_geometry']['support_clearance_m']=.004
    if fault=='bodies':params[module.BODY_PARAM]['camera']['source_sha256']='b'*64
    if fault=='selection':params['/grasp_mode/selection']['selection_id']='other'
    if fault=='revocation':node._observation_contexts.revocation+=1
    assert not node.handle_contact_lift(req(plan,True)).success
    assert node._lift_linear_cache is None
    node._ensure_trajectory_controllers_started.assert_not_called();p._execute_cached_pose_plan.assert_not_called()


@pytest.mark.parametrize('fault',['sdk','accepted','opening','opening_stamp'])
def test_new_feedback_cannot_rewrite_frozen_bias_or_opening(monkeypatch,fault):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    if fault=='sdk':s['sdk_positions_rad'][1]+=SDK_QUANTUM_RAD
    if fault=='accepted':s['accepted_positions_rad'][1]+=SDK_QUANTUM_RAD
    if fault=='opening':s['accepted_gripper_opening_m']+=.00005
    if fault=='opening_stamp':s['opening_stamp_ns']+=1
    assert not node.handle_contact_lift(req(plan,True)).success
    p._execute_cached_pose_plan.assert_not_called()


@pytest.mark.parametrize('fault',['single_occupancy','attachment','whole_arm','curve','contract'])
def test_incomplete_or_unbound_provider_result_never_gets_cached(monkeypatch,fault):
    node,p,plan,s,params,_=case(monkeypatch)
    original=node._closed_lift_contract_validator.side_effect
    def bad(**kwargs):
        audit=original(**kwargs)
        if fault=='single_occupancy':audit['stationary_world_target_removed']=True
        if fault=='attachment':audit['attachment_confirmed']=True
        if fault=='whole_arm':audit['whole_arm_collision_method']='endpoint_only'
        if fault=='curve':audit['trajectory_sha256']='other'
        if fault=='contract':audit['hardware_following_swept_volume_certified']=True
        return audit
    node._closed_lift_contract_validator.side_effect=bad
    assert not node.handle_contact_lift(req(plan)).success
    assert node._lift_linear_cache is None
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_final_timing_changes_reprove_every_point_without_roundtrip_position_loss(monkeypatch):
    node,p,plan,s,params,pure=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    original=p._execute_cached_pose_plan.side_effect
    def retimed(*args,**kwargs):
        tr=p._last_pose_plan['plan']
        for row in tr.joint_trajectory.points:
            row.time_from_start*=2
            row.velocities=[v/2 for v in row.velocities]
            row.accelerations=[a/4 for a in row.accelerations]
        return original(*args,**kwargs)
    p._execute_cached_pose_plan.side_effect=retimed
    assert node.handle_contact_lift(req(plan,True)).success
    nominal,shifted=pure.call_args.args[:2]
    offset=np.array(s['sdk_positions_rad'])-s['accepted_positions_rad']
    for a,b in zip(nominal.joint_trajectory.points,shifted.joint_trajectory.points):
        np.testing.assert_allclose(np.array(b.positions)-a.positions,offset,atol=1e-15)
        assert a.time_from_start==b.time_from_start and a.velocities==b.velocities


def test_loss_scope_accepts_only_target_lost_during_same_lift_context(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    msg=deepcopy(plan);msg.valid=False;msg.diagnostic='TARGET_LOST: occluded'
    assert node._compensation_allows_visual_loss(msg)
    msg.diagnostic='SUPPORT_GEOMETRY_CONTRADICTION: positive residual'
    assert not node._compensation_allows_visual_loss(msg)
    msg.diagnostic='TARGET_LOST';msg.target_track_id='other'
    assert not node._compensation_allows_visual_loss(msg)


@pytest.mark.parametrize('enabled',[False,True])
def test_task_lift_routes_only_when_explicitly_enabled_and_keeps_measured_gate(enabled):
    node,plan,pose,cfg,legacy=task_case();plan.model_choice='unknown_tabletop'
    pose.pose=deepcopy(plan.poses[3]);cfg[module.ENABLED_KEY]=enabled
    route=mock.Mock(return_value=NS(success=True,message='fake'))
    node._contact_lift_service=mock.Mock(return_value=route)
    assert node._plan_and_execute_pose(task.GraspStages.LIFT_OBJECT,'linear 6D lift',pose,legacy,
        '6D lift',execution_plan=plan,gcfg=cfg,execute_pose=legacy)
    if enabled:
        assert route.call_count==2;legacy.assert_not_called()
    else:
        assert legacy.call_count==2;node._contact_lift_service.assert_not_called()
    node._record_and_validate_measured_endpoint.assert_called_once()
    node._compensate_pregrasp_endpoint.assert_not_called()


@pytest.mark.parametrize('fault',['plan_rejection','execute_rejection','wrong_goal','invalid_config'])
def test_opted_in_lift_failure_never_falls_back_to_old_path(fault):
    node,plan,pose,cfg,legacy=task_case();plan.model_choice='unknown_tabletop'
    pose.pose=deepcopy(plan.poses[3]);cfg[module.ENABLED_KEY]=True
    route=mock.Mock(return_value=NS(success=True,message='fake'))
    if fault=='plan_rejection':route.return_value=NS(success=False,message='missing geometry')
    if fault=='execute_rejection':route.side_effect=[NS(success=True,message='planned'),NS(success=False,message='failed')]
    if fault=='wrong_goal':pose.pose.position.z+=.001
    if fault=='invalid_config':cfg[module.ENABLED_KEY]='true'
    node._contact_lift_service=mock.Mock(return_value=route)
    assert not node._plan_and_execute_pose(task.GraspStages.LIFT_OBJECT,'linear 6D lift',pose,legacy,
        '6D lift',execution_plan=plan,gcfg=cfg,execute_pose=legacy)
    legacy.assert_not_called();node._compensate_pregrasp_endpoint.assert_not_called()


def test_current_scene_change_after_proof_rejects_before_fake_send(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    node._closed_lift_contract_validator.scene_digest.return_value='changed_scene'
    result=node.handle_contact_lift(req(plan,True))
    assert not result.success and 'LIFT_MOVEIT_SCENE_CHANGED_BEFORE_SEND' in result.message


def test_absent_override_connects_real_readonly_adapter_instead_of_permanent_missing_provider(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch,capabilities=False)
    provider=node._closed_lift_contract_validator;del node._closed_lift_contract_validator
    connect=mock.Mock(return_value=provider)
    monkeypatch.setattr(module.ClosedLiftMoveItAdapter,'connect',connect)
    result=node.handle_contact_lift(req(plan))
    assert result.success,result.message
    connect.assert_called_once_with(params['/robot_description'])
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_scene_change_while_waiting_for_submission_sample_is_caught_before_send(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    calls=[0]
    def sample(*args,**kwargs):
        calls[0]+=1
        # execute admission, final path guard, final before-send sample
        if calls[0]==3:node._closed_lift_contract_validator.scene_digest.return_value='changed_in_sample_wait'
        return deepcopy(s)
    node._endpoint_following_snapshot.side_effect=sample
    result=node.handle_contact_lift(req(plan,True))
    assert not result.success and 'SCENE_CHANGED_DURING_SUBMISSION' in result.message


def test_feedback_expiry_during_final_scene_rpc_is_caught(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    assert node.handle_contact_lift(req(plan)).success
    count=[0]
    def digest():
        count[0]+=1
        if count[0]==2:monkeypatch.setattr(module.rospy,'get_time',lambda:s['accepted_stamp_ns']*1e-9+1.)
        return 'scene'
    node._closed_lift_contract_validator.scene_digest.side_effect=digest
    result=node.handle_contact_lift(req(plan,True))
    assert not result.success and 'FEEDBACK_EXPIRED_DURING_SCENE_CHECK' in result.message


def test_provided_wrong_body_frame_is_rejected_not_treated_as_missing_model(monkeypatch):
    node,p,plan,s,params,_=case(monkeypatch)
    params[module.BODY_PARAM]['camera']['frame_id']='camera_link'
    response=node.handle_contact_lift(req(plan))
    assert not response.success and 'LIFT_RIGID_BODY_FRAME_INVALID' in response.message
    node._ensure_trajectory_controllers_started.assert_not_called()
