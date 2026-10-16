"""Independent RGB owner and planner sequencing; no ROS master or actuation."""
from copy import deepcopy
from dataclasses import asdict,replace
from types import SimpleNamespace as NS
from unittest.mock import Mock
from pathlib import Path
import base64
import json
import threading
import time

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot import rgb_acquisition_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError


def request():
    return dict(version=1, request_id='first', expected_source_id='a'*64,
                selection=dict(mode='unknown',strategy='observe_return',generation=2,stamp_ns=1),
                duration_sec=120.,max_observations=3)


def reserve(previous=None, req=None, now=10**9, mono=10.):
    req = req or request()
    return module.reserve_episode(previous, req, 'a'*64, request()['selection'], now, mono)


def test_new_selection_requires_new_freeze_even_after_previous_settled_view():
    previous = reserve()
    previous.update(phase='awaiting_measured_target', reference_id='old-reference')
    incoming = request()
    assert module.continues_rgb_episode(previous, incoming)
    incoming['selection'] = dict(incoming['selection'], generation=3, stamp_ns=2_000_000_000)
    incoming['request_id'] = 'new-generation'
    assert not module.continues_rgb_episode(previous, incoming)
    current = module.reserve_episode(previous, incoming, 'a'*64, incoming['selection'],
        3_000_000_000, 12.)
    assert current['attempts'] == 1 and 'reference_id' not in current
    assert current['selection'] == incoming['selection']


def test_same_selection_timestamp_string_keeps_original_continuation():
    previous = reserve()
    previous['phase'] = 'awaiting_measured_target'
    previous['selection']['stamp_ns'] = str(previous['selection']['stamp_ns'])
    assert module.continues_rgb_episode(previous, request())


def test_rejected_plan_or_empty_ledger_requires_freeze():
    previous = reserve()
    previous['phase'] = 'planning_rejected'
    assert not module.continues_rgb_episode(previous, request())
    assert not module.continues_rgb_episode({}, request())


def test_episode_persists_budget_and_deadline_after_rejected_plan():
    first = reserve()
    first['phase'] = 'planning_rejected'
    req = request(); req['request_id'] = 'second'
    second = reserve(first, req, now=20*10**9,mono=29.)
    assert second['attempts'] == 2
    assert second['deadline_ns'] == first['deadline_ns']
    assert second['deadline_monotonic'] == first['deadline_monotonic']
    second['phase'] = 'planning_rejected'; req['request_id'] = 'third'
    third = reserve(second, req, now=22*10**9,mono=30.)
    third['phase'] = 'planning_rejected'; req['request_id'] = 'fourth'
    with pytest.raises(ObservationPathError,match='EXHAUSTED'):
        reserve(third, req,now=23*10**9,mono=31.)


@pytest.mark.parametrize('field,value', [('duration_sec',180.001),('duration_sec',True),
    ('duration_sec',float('nan')),('duration_sec',0),('max_observations',4),('max_observations',True)])
def test_finite_budget_cannot_be_relaxed(field,value):
    req=request(); req[field]=value
    with pytest.raises(ObservationPathError,match='BUDGET_INVALID'):
        reserve(req=req)


@pytest.mark.parametrize('phase',['reserved','executing','revoked','awaiting_measured_target'])
def test_new_request_is_not_an_ownership_reset(phase):
    first=reserve(); first['phase']=phase
    req=request(); req['request_id']='new'
    with pytest.raises(ObservationPathError):
        reserve(first,req)


@pytest.mark.parametrize('now,mono',[(0,20.),(2*10**9,9.),(121*10**9,20.),(2*10**9,130.)])
def test_either_clock_expiry_or_rollback_rejects(now,mono):
    first=reserve(); first['phase']='planning_rejected'
    req=request(); req['request_id']='next'
    with pytest.raises(ObservationPathError,match='EXPIRED'):
        reserve(first,req,now=now,mono=mono)


def test_xmlrpc_nanoseconds_are_lossless_and_selection_is_parsed():
    req=request(); req['selection']['stamp_ns']=str(req['selection']['stamp_ns'])
    first=reserve(req=req)
    first['selection']['stamp_ns']=10**18
    result=module.ros_parameter_value(first)
    assert result['selection']['stamp_ns']==str(10**18)
    assert result['attempts']==1


def trajectory(velocities=0.):
    result=RobotTrajectory(); result.joint_trajectory.joint_names=list(module.ARM_NAMES)
    result.joint_trajectory.points=[JointTrajectoryPoint(positions=[0.]*6,velocities=[velocities]*6,
        time_from_start=rospy.Duration(0.)), JointTrajectoryPoint(positions=[.05]*6,
        velocities=[-velocities]*6,time_from_start=rospy.Duration(3.))]
    return result


def test_intermediate_cubic_flip_is_rejected_despite_small_endpoints():
    hold=NS(positions=[0.]*6)
    assert module.continuous_excursion(trajectory(),hold)<.06
    with pytest.raises(ObservationPathError,match='NONLOCAL'):
        module.continuous_excursion(trajectory(2.),hold)


def node_fixture(monkeypatch):
    from alicia_flexible_grasp.robot import rgb_acquisition_artifacts
    monkeypatch.setattr(rgb_acquisition_artifacts,'require_storage',lambda:None)
    monkeypatch.setattr(rgb_acquisition_artifacts,'save_artifacts',lambda *a,**k:'/external/fixture')
    node=module.RgbAcquisitionGateway(); node.planner=NS(tag='original')
    node._rgb_lock=threading.RLock()
    node._command_reference_lock=threading.RLock()
    node._planner_operation_lock=lambda:threading.RLock()
    node._manual_control_active=lambda:False
    node._fresh_actuation_status=lambda:'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    node._rgb_revocation=0
    params={module.REQUEST_PARAM:request()}
    target=PoseStamped(); target.header.frame_id='base_link';target.pose.orientation.w=1.
    params[module.REQUEST_PARAM]['target_sha256']=module.wire_digest(target)
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:deepcopy(params.get(key,default)))
    monkeypatch.setattr(module.rospy,'set_param',lambda key,val:params.__setitem__(key,deepcopy(val)))
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    node._rgb_wait_initial=Mock(return_value=dict(source_id='a'*64,
        source_context=dict(selection=request()['selection']),q_source=[0.]*6))
    node._rgb_require_owner=Mock()
    node._rgb_status_pub=Mock()
    node._rgb_freeze_reference=Mock()
    node._configure_observation_timing=lambda planner:('timing',)
    node._observation_timing_current=lambda planner,timing:True
    node._configure_controller_reference_guard=Mock()
    node._ensure_trajectory_controllers_started=Mock(return_value=(True,'started'))
    node._synchronize_trajectory_controller_to_feedback=Mock(return_value=(True,'synced'))
    node._wait_for_stationary_rgb_feedback=Mock(return_value=(True,'stationary'))
    captured=dict(source_stamp_ns=10**10,source_rgb_sha256='b'*64,source_depth_sha256='c'*64)
    node._rgb_capture_after_plan=Mock(return_value=captured)
    node._rgb_validate_path=Mock(return_value={'trajectory_sha256':'e'*64})
    def submission(episode,*args):
        node._rgb_update_ledger(episode,'executing')
    node._rgb_submission=Mock(side_effect=submission)
    node._rgb_authorized=lambda *args:True
    planner=NS(observation_trajectory_executor=object(),_last_pose_plan=None)
    planner.move_to_pose=Mock(return_value=(True,'planned'))
    def execute(target):
        plan=trajectory()
        audit=planner.observation_path_validator(plan)
        planner.observation_submission_revalidate(plan,audit)
        planner._last_observation_path_audit=audit
        return True,'executed'
    planner.execute_cached_strict_pose=Mock(side_effect=execute)
    node._rgb_observation_planner=lambda:planner
    return node,planner,params,NS(target=target,execute=True)


def prefix_authority():
    node=module.RgbAcquisitionGateway()
    node._rgb_lock=threading.RLock()
    node._command_reference_lock=threading.RLock()
    node._rgb_revocation=0
    status=['PENDING:POSITIVE_ENABLE_REQUESTED']
    node._fresh_actuation_status=lambda:status[0]
    node._control_reference_epoch_ns=5
    episode=reserve();episode['q_source']=[0.]*6
    captured=dict(source_context=dict(epoch_ns=5))
    return node,episode,captured,status


def prefix_trajectory():
    plan=trajectory()
    plan.joint_trajectory.points[-1].positions=[.025]*6
    return plan


@pytest.mark.parametrize('status', ['', 'PENDING:AWAITING_ENCODER_RESPONSE',
    'UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT','DISABLED:EXPLICIT_TORQUE_OFF',
    'OVERHEAT_BLOCKED:FAULT'])
def test_rgb_prefix_never_admits_missing_faulted_or_already_running_probe(status):
    node,episode,captured,current=prefix_authority();current[0]=status
    with pytest.raises(ObservationPathError,match='ACTUATION_UNCONFIRMED'):
        node._rgb_admit_actuation_prefix(episode,captured)
    assert getattr(node,'_rgb_prefix_capability',None) is None


@pytest.mark.parametrize('changed', ['epoch','attempt','observation','consumed'])
def test_rgb_pending_prefix_requires_first_unused_budget_and_same_real_epoch(changed):
    node,episode,captured,status=prefix_authority()
    if changed=='epoch': node._control_reference_epoch_ns=6
    if changed=='attempt': episode['attempts']=2
    if changed=='observation': episode['observation_index']=2
    if changed=='consumed': episode['actuation_prefix_attempted']=True
    with pytest.raises(ObservationPathError,match='ACTUATION_PREFIX'):
        node._rgb_admit_actuation_prefix(episode,captured)


def test_rgb_pending_exception_belongs_only_to_same_live_episode_and_submitted_curve():
    node,episode,captured,status=prefix_authority()
    assert node._rgb_admit_actuation_prefix(episode,captured)
    node._rgb_require_actuation(episode)
    with pytest.raises(ObservationPathError,match='UNCONFIRMED'):
        node._rgb_require_actuation(deepcopy(episode))
    plan=prefix_trajectory()
    node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))
    capability=node._rgb_prefix_capability
    episode['actuation_prefix_trajectory_sha256']=capability['trajectory_sha256']
    capability['phase']='submitted'
    status[0]='PENDING:AWAITING_ENCODER_RESPONSE'
    node._rgb_require_actuation(episode)
    plan.joint_trajectory.points[-1].positions[0]+=.001
    with pytest.raises(ObservationPathError,match='REVOKED'):
        node._rgb_require_actuation(episode)


@pytest.mark.parametrize('change',['pending_started_before_send','epoch','source','fault','confirmation_regressed'])
def test_rgb_prefix_state_or_binding_revocation_cannot_recover_by_restoring_current_values(change):
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    if change=='pending_started_before_send':status[0]='PENDING:AWAITING_ENCODER_RESPONSE'
    if change=='epoch':node._control_reference_epoch_ns=6
    if change=='source':episode['source_id']='b'*64
    if change=='fault':status[0]='UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
    if change=='confirmation_regressed':
        status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE';node._rgb_require_actuation(episode)
        status[0]='PENDING:COMMAND_SYNCHRONIZED'
    with pytest.raises(ObservationPathError,match='REVOKED'):
        node._rgb_require_actuation(episode)
    status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    node._control_reference_epoch_ns=5;episode['source_id']='a'*64
    with pytest.raises(ObservationPathError,match='REVOKED'):
        node._rgb_require_actuation(episode)


@pytest.mark.parametrize('events',[
    ['UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT','PENDING:POSITIVE_ENABLE_REQUESTED'],
    ['DISABLED:EXPLICIT_TORQUE_OFF','PENDING:COMMAND_SYNCHRONIZED'],
    ['CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE','PENDING:COMMAND_SYNCHRONIZED'],
    ['UNCONFIRMED:ENCODER_RESPONSE_LOST','CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'],
])
def test_actual_actuation_callback_latches_all_prefix_excursions_between_gate_polls(monkeypatch,events):
    from test_motion_gateway_controller_start import motion_gateway_node
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    monkeypatch.setattr(motion_gateway_node.rospy,'get_time',lambda:10.)
    node._fresh_actuation_status=lambda:node._actuation_status
    # No polling gate runs between these actual subscriber callback invocations.
    for value in events:
        motion_gateway_node.MotionGateway._actuation_cb(node,NS(data=value))
    assert node._actuation_status==events[-1]
    assert node._rgb_prefix_capability['revoked'] is True
    with pytest.raises(ObservationPathError,match='REVOKED'):
        node._rgb_require_actuation(episode)


def test_prefix_epoch_or_manual_event_latches_capability_itself():
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    node._rgb_acquisition_revoke_event()
    assert node._rgb_revocation==1
    assert node._rgb_prefix_capability['revoked'] is True
    status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    with pytest.raises(ObservationPathError,match='REVOKED'):
        node._rgb_require_actuation(episode)


def test_actuation_callback_hook_does_not_change_ordinary_status_or_need_initialized_rgb(monkeypatch):
    from test_motion_gateway_controller_start import motion_gateway_node
    monkeypatch.setattr(motion_gateway_node.rospy,'get_time',lambda:10.)
    node=module.RgbAcquisitionGateway();node._command_reference_lock=threading.RLock()
    motion_gateway_node.MotionGateway._actuation_cb(node,NS(data='PENDING:COMMAND_SYNCHRONIZED'))
    assert node._actuation_status=='PENDING:COMMAND_SYNCHRONIZED'
    assert node._actuation_received_sec==10.
    assert getattr(node,'_rgb_prefix_capability',None) is None


def test_prefix_measured_excursion_checks_real_desired_bridge_instead_of_inventing_measured_bridge():
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    plan=prefix_trajectory();desired=NS(positions=[.004,0.,0.,0.,0.,0.])
    plan.joint_trajectory.points[-1].positions[0]=.018283333333333332
    plan.joint_trajectory.points[-1].velocities[0]=-.0152
    plan.joint_trajectory.points[-1].accelerations=[0.]*6
    # Each invented bridge would appear inside its own .025 bound, while
    # the actual controller bridge overshoots .025 about measured Joint1=0.
    assert module.continuous_excursion(plan,desired)<=.025+1e-12
    assert module.continuous_excursion(plan,NS(positions=[0.]*6))<=.025+1e-12
    with pytest.raises(ObservationPathError,match='NONLOCAL'):
        node._rgb_prefix_curve(episode,plan,desired)


@pytest.mark.parametrize('changed',['endpoint','retimed_overshoot','controller_bridge','measured_origin','too_short'])
def test_rgb_prefix_final_curve_and_bridge_have_their_own_small_continuous_bound(changed):
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    plan=prefix_trajectory();desired=NS(positions=[0.]*6)
    if changed=='endpoint':plan.joint_trajectory.points[-1].positions[0]=.026
    if changed=='retimed_overshoot':
        plan.joint_trajectory.points[0].velocities=[.1]*6
        plan.joint_trajectory.points[-1].velocities=[-.1]*6
    if changed=='controller_bridge':desired.positions=[-.003]*6
    if changed=='measured_origin':episode['q_source']=[-.003]*6
    if changed=='too_short':plan.joint_trajectory.points[-1].positions=[.019]*6
    with pytest.raises(ObservationPathError,match='NONLOCAL|INSUFFICIENT'):
        node._rgb_prefix_curve(episode,plan,desired)
    assert node._rgb_prefix_capability['trajectory_sha256'] is None


def test_rgb_prefix_seals_final_curve_once_and_keeps_terminal_target_distinct():
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    plan=prefix_trajectory()
    audit=node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))
    assert audit['terminal_joint_positions']==[.025]*6
    assert audit['requested_target_reached'] is False
    assert audit['continuous_command_excursion_rad']<=.025+1e-12
    plan.joint_trajectory.points[-1].time_from_start=rospy.Duration(4.)
    with pytest.raises(ObservationPathError,match='CURVE_CHANGED'):
        node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))


def test_rgb_prefix_measured_bound_uses_plan_joint_order():
    node,episode,captured,status=prefix_authority()
    episode['q_source']=[.1,.2,.3,.4,.5,.6]
    node._rgb_admit_actuation_prefix(episode,captured)
    plan=prefix_trajectory();plan.joint_trajectory.joint_names=list(reversed(module.ARM_NAMES))
    origin=list(reversed(episode['q_source']))
    plan.joint_trajectory.points[0].positions=origin
    plan.joint_trajectory.points[-1].positions=[q+.025 for q in origin]
    assert node._rgb_prefix_curve(episode,plan,NS(positions=origin))['maximum_joint_delta_rad']==.025


def test_early_prefix_curve_rejection_retains_exact_curve_and_model_without_archive_io(monkeypatch):
    from alicia_flexible_grasp.robot import rgb_acquisition_artifacts
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    node._rgb_require_owner=lambda *args:None
    node._observation_measured_opening=lambda:.05
    node._observation_controller_hold=lambda names:NS(positions=[0.]*6)
    planner=NS(manipulator=NS(get_planning_frame=lambda:'base_link',get_end_effector_link=lambda:'tool0'))
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:'exact model bytes' if key=='/robot_description' else default)
    monkeypatch.setattr(module,'SerialUrdfFk',lambda *args:object())
    monkeypatch.setattr(module,'gripper_contract_mismatch_reason',lambda *args,**kwargs:'')
    monkeypatch.setattr(rgb_acquisition_artifacts,'save_artifacts',lambda *args,**kwargs:pytest.fail('archive I/O during proof'))
    plan=prefix_trajectory();plan.joint_trajectory.points[-1].positions[0]=.026
    fingerprint=module.wire_digest(plan)
    with pytest.raises(ObservationPathError,match='NONLOCAL'):
        node._rgb_validate_path(episode,captured,plan,planner,('timing',))
    assert module.wire_digest(node._rgb_artifact_plan)==fingerprint
    assert node._rgb_artifact_model=='exact model bytes'
    plan.joint_trajectory.points[-1].positions[0]=0.
    assert module.wire_digest(node._rgb_artifact_plan)==fingerprint


def prefix_rpc_fixture(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch)
    status=['PENDING:POSITIVE_ENABLE_REQUESTED']
    node._fresh_actuation_status=lambda:status[0]
    node._control_reference_epoch_ns=5
    node._rgb_wait_initial.return_value['source_context']['epoch_ns']=5
    # Keep admission and real status/capability checks even with synthetic
    # image/scene fixtures. The fake executor traverses the actual callbacks.
    node._rgb_require_owner=lambda episode,**kwargs:node._rgb_require_actuation(episode)
    def validate(episode,captured,plan,*args):
        prefix=node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))
        return dict(trajectory_sha256=module.wire_digest(plan),actuation_prefix=prefix)
    node._rgb_validate_path=Mock(side_effect=validate)
    def submit(episode,captured,plan,audit,*args):
        node._rgb_require_actuation(episode)
        capability=node._rgb_prefix_capability
        node._rgb_update_ledger(episode,'executing',
            actuation_prefix_trajectory_sha256=capability['trajectory_sha256'])
        capability['phase']='submitted'
    node._rgb_submission=Mock(side_effect=submit)
    def execute(target):
        plan=prefix_trajectory()
        audit=planner.observation_path_validator(plan)
        planner.observation_submission_revalidate(plan,audit)
        planner._last_observation_path_audit=audit
        status[0]='PENDING:AWAITING_ENCODER_RESPONSE'
        assert planner.observation_execution_authorized()
        status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        return True,'bounded prefix completed'
    planner.execute_cached_observation_prefix=Mock(side_effect=execute)
    node._rgb_authorized=lambda episode,*args:module.RgbAcquisitionGateway._rgb_authorized(
        node,episode,*args)
    return node,planner,params,req,status


def retry_rpc_fixture(monkeypatch):
    """Real owner, reservations, reference install and submission across RPCs.

    Images/MoveIt/transport are offline fixtures; no test claims real movement.
    """
    import hashlib
    node,planner,params,req,status=prefix_rpc_fixture(monkeypatch)
    clock=[10.,100.]
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[1])
    context=dict(selection=request()['selection'],epoch_ns=5)
    source=module.digest(context)
    params[module.REQUEST_PARAM]['expected_source_id']=source
    params.update({'/robot/observation_tracking_contract_enabled':True,
        '/alicia_d_driver_node/endpoint_feedback_trim_enabled':False,
        '/robot_description':'model','/alicia_controller/constraints':{},
        '/alicia_controller/stop_trajectory_duration':.5,'/grasp_6d/remote/gripper_geometry':{}})
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda key,default=None:deepcopy(params.get(key,default)))
    node._rgb_context=lambda stamp:deepcopy(context)
    node._observation_context_lock=threading.RLock()
    node._observation_task_connection_available=lambda:True
    node._observation_task_requires_inactive=False
    node._observation_contexts=NS(active=False)
    node._observation_task_state_stamp_ns=1
    node._rgb_measured_arm_snapshot=lambda:[0.]*6
    node._rgb_require_owner=module.RgbAcquisitionGateway._rgb_require_owner.__get__(node)
    vertices=[[x,y,z] for x in (.1,.2) for y in (.1,.2) for z in (.1,.2)]
    volume=dict(vertices_base_m=vertices)
    masks=[np.ones((20,20),dtype=bool)]
    counter=[9_500_000_000]
    def capture(*args,reference_id='',**kwargs):
        counter[0]+=50_000_000
        return dict(source_id=source,source_context=deepcopy(context),q_source=[0.]*6,
            source_stamp_ns=counter[0],reference_id=reference_id,masks=deepcopy(masks),
            source_rgb_sha256='b'*64,source_depth_sha256='c'*64,
            interest=dict(rgb_search_basis='fixed_reference_volume_projection' if reference_id else 'image_center',
                reference_id=reference_id,association_sha256='f'*64))
    node._rgb_wait_initial=Mock(side_effect=capture)
    node._rgb_capture_after_plan=Mock(side_effect=lambda episode,initial:capture(reference_id=episode['reference_id']))
    def freeze(episode,initial):
        episode.update(reference_id='d'*64,reference_observation_set_sha256='e'*64,
            reference_volume_json=json.dumps(volume),reference_volumes_json=json.dumps([volume]))
        node._rgb_install_reference_visibility(episode)
        initial['reference_id']=episode['reference_id']
        node._rgb_update_ledger(episode,'reserved')
    node._rgb_freeze_reference=Mock(side_effect=freeze)
    node._observation_measured_opening=lambda:.05
    node._observation_controller_hold=lambda names:NS(positions=[0.]*6)
    monkeypatch.setattr(module,'SerialUrdfFk',lambda *args:object())
    def validate(episode,captured,plan,*args):
        node._rgb_prefix_capability['retry_stage']='path_preflight'
        prefix=node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))
        return dict(trajectory_sha256=module.wire_digest(plan),actuation_prefix=prefix,
            model_sha256=hashlib.sha256(b'model').hexdigest(),
            execution_tracking_contract=dict(stop_trajectory_duration_sec=.5,joint_error_bounds_rad=[.091]*6),
            controller_constraints_snapshot={},gripper_geometry_snapshot={},opening_width_m=.05,
            rgb_interest_visibility=dict(node._rgb_reference_visibility(captured),
                all_original_reference_geometry_visible=True))
    node._rgb_validate_path=Mock(side_effect=validate)
    node._rgb_fresh_path_audit=Mock(return_value=dict(scene_sha256='new',certified_minimum_clearance_m=.0035))
    node._rgb_submission=module.RgbAcquisitionGateway._rgb_submission.__get__(node)
    def next_request(token):
        params[module.REQUEST_PARAM].update(reservation_request_id=token,
            reference_id='d'*64,observation_index=1)
    return node,planner,params,req,status,clock,next_request


@pytest.mark.parametrize('failure',['plan','path'])
def test_preflight_retry_preserves_original_owner_reference_budget_then_submits_once(monkeypatch,failure):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    if failure=='plan':planner.move_to_pose.return_value=(False,'no path')
    else:
        original=node._rgb_validate_path.side_effect
        def reject(*args):
            original(*args)
            raise ObservationPathError('path preflight rejects')
        node._rgb_validate_path.side_effect=reject
    first=node.handle_rgb_acquisition_pose_strict(req)
    before=deepcopy(params[module.LEDGER_PARAM])
    capability=node._rgb_prefix_capability
    assert not first.success and before['phase']=='preflight_rejected'
    assert before['actuation_prefix_submission_started'] is False
    proof=module.validate_prefix_preflight_receipt(before,now_ns=10**10,monotonic_now=100.)
    assert proof['q_source']==[0.]*6 and capability['phase']=='retry_ready'
    planner.move_to_pose.return_value=(True,'planned')
    if failure=='path':node._rgb_validate_path.side_effect=original
    next_request('reservation-two')
    second=node.handle_rgb_acquisition_pose_strict(req)
    after=params[module.LEDGER_PARAM]
    assert second.success and after['phase']=='awaiting_measured_target'
    assert after['request_id']==before['request_id']=='first'
    assert after['reference_id']==before['reference_id']=='d'*64
    assert after['request_ids']==['first','reservation-two'] and after['attempts']==2
    assert after['actuation_prefix_submission_started'] is True
    assert 'actuation_prefix_preflight' not in after
    assert after['observation_index']==1 and after['requested_target_reached'] is False
    for key in ('created_ns','deadline_ns','created_monotonic','deadline_monotonic','duration_sec','max_observations'):
        assert after[key]==before[key]
    node._rgb_freeze_reference.assert_called_once()
    assert node._rgb_wait_initial.call_args.kwargs['reference_id']=='d'*64
    assert node._rgb_fresh_path_audit.call_count==1
    planner.execute_cached_strict_pose.assert_not_called()
    assert node._rgb_prefix_capability is None


def test_three_preflight_failures_charge_all_attempts_without_reference_or_clock_refresh(monkeypatch):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    sources=[]
    for index in range(1,4):
        if index>1:next_request('reservation-'+str(index))
        assert not node.handle_rgb_acquisition_pose_strict(req).success
        ledger=params[module.LEDGER_PARAM]
        assert ledger['phase']=='preflight_rejected' and ledger['attempts']==index
        assert ledger['created_ns']=='10000000000' and ledger['deadline_ns']=='130000000000'
        sources.append(int(ledger['actuation_prefix_preflight']['last_capture_source_stamp_ns']))
    assert sources==sorted(set(sources)) and len(sources)==3
    next_request('reservation-four')
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success and 'EXHAUSTED' in result.message
    assert params[module.LEDGER_PARAM]['attempts']==3
    assert planner.move_to_pose.call_count==3
    node._rgb_freeze_reference.assert_called_once()
    node._rgb_fresh_path_audit.assert_not_called()


@pytest.mark.parametrize('failure',['insufficient_direction','retime'])
def test_actual_prefix_clip_or_retime_rejection_can_replan_without_any_submission(monkeypatch,failure):
    from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    original_execute=planner.execute_cached_observation_prefix
    plan=prefix_trajectory();plan.joint_trajectory.points[-1].positions=[.1]*6
    planner._matching_cached_strict_pose_plan=lambda pose:({'plan':plan},'')
    planner.manipulator=NS(get_current_joint_values=lambda:[0.]*6)
    planner.strict_execution_retime_enabled=True
    planner.controller_reference_guard_required=True
    offset=.01 if failure=='insufficient_direction' else 0.
    planner._controller_reference_snapshot=lambda names:NS(positions=[offset]*6,velocities=[0.]*6,accelerations=[0.]*6)
    planner._retime_strict_execution_plan=Mock(return_value=(None,'retimer could not produce path'))
    planner.execute_cached_observation_prefix=lambda target:MoveItPlanner.execute_cached_observation_prefix(planner,target)
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success
    assert ('insufficient directional command delta' if failure=='insufficient_direction' else 'RETIMING_FAILED') in result.message
    assert params[module.LEDGER_PARAM]['phase']=='preflight_rejected'
    assert node._rgb_prefix_capability['submission_started'] is False
    node._rgb_validate_path.assert_not_called()
    node._rgb_fresh_path_audit.assert_not_called()
    planner.execute_cached_observation_prefix=original_execute
    next_request('reservation-two')
    assert node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['attempts']==2
    node._rgb_fresh_path_audit.assert_called_once()
    node._rgb_freeze_reference.assert_called_once()


@pytest.mark.parametrize('failure',['archive_unavailable','fault_during_archive'])
def test_preflight_evidence_failure_or_fault_cannot_leave_reusable_live_capability(monkeypatch,failure):
    from alicia_flexible_grasp.robot import rgb_acquisition_artifacts
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    def archive(*args,**kwargs):
        if failure=='archive_unavailable':raise OSError('external evidence volume unavailable')
        node._rgb_prefix_actuation_event('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
        node._rgb_prefix_actuation_event('CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE')
        status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
        return '/external/fixture'
    monkeypatch.setattr(rgb_acquisition_artifacts,'save_artifacts',archive)
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    capability=node._rgb_prefix_capability
    assert capability is None or capability['revoked'] is True
    if failure=='archive_unavailable':assert params[module.LEDGER_PARAM]['phase']=='revoked'
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert planner.move_to_pose.call_count==1


def test_unreserved_retry_capture_timeout_preserves_original_receipt_and_reservation_token(monkeypatch):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    ledger=deepcopy(params[module.LEDGER_PARAM]);capability=node._rgb_prefix_capability
    capture=node._rgb_wait_initial.side_effect
    node._rgb_wait_initial.side_effect=ObservationPathError('fresh initial RGB frame timeout')
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]==ledger
    assert node._rgb_prefix_capability is capability and not capability['revoked']
    node._rgb_wait_initial.side_effect=capture
    planner.move_to_pose.return_value=(True,'planned')
    assert node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['request_ids']==['first','reservation-two']
    assert params[module.LEDGER_PARAM]['attempts']==2


@pytest.mark.parametrize('change',['wall_deadline','ros_deadline','source_context','measured_pose','epoch'])
def test_retry_live_owner_changes_are_terminal_without_new_reservation(monkeypatch,change):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    if change=='wall_deadline':clock[1]=220.
    if change=='ros_deadline':clock[0]=130.
    if change=='source_context':node._rgb_context=lambda stamp:dict(changed=True)
    if change=='measured_pose':node._rgb_measured_arm_snapshot=lambda:[0.,0.,.0076699,0.,0.,0.]
    if change=='epoch':node._control_reference_epoch_ns=6
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['phase']=='revoked'
    assert params[module.LEDGER_PARAM]['attempts']==1
    assert node._rgb_prefix_capability is None
    assert planner.move_to_pose.call_count==1


def test_preflight_state_topic_keeps_complete_receipt_without_repeating_large_frozen_geometry(monkeypatch):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    params[module.REQUEST_PARAM]['request_id']='o'*128
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    episode=node._rgb_prefix_capability['episode']
    # The API accepts four full volumes and 128-character reservation IDs.
    # Keep their exact ledger strings while the consumer's state message
    # carries the complete receipt and original geometry fingerprints only.
    for key in ('reference_volume_json','reference_volumes_json'):
        episode[key]='preserved exact geometry '*2000
    node._rgb_update_ledger(episode,'preflight_rejected')
    event=json.loads(node._rgb_status_pub.publish.call_args[0][0].data)
    assert len(node._rgb_status_pub.publish.call_args[0][0].data.encode())<16384
    for key in ('reference_volume_json','reference_volumes_json'):
        assert params[module.LEDGER_PARAM][key]==episode[key]
        assert params[module.STATUS_PARAM][key]==episode[key]
        assert key not in event
    assert event['reference_id']==episode['reference_id']
    assert event['reference_vertices_sha256']==episode['reference_vertices_sha256']
    assert module.validate_prefix_preflight_receipt(event)==episode['actuation_prefix_preflight']


@pytest.mark.parametrize('events',[
    ['UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT','PENDING:POSITIVE_ENABLE_REQUESTED'],
    ['CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE','PENDING:COMMAND_SYNCHRONIZED'],
    ['UNCONFIRMED:ENCODER_RESPONSE_LOST','CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'],
])
@pytest.mark.parametrize('rejections',[1,2])
def test_idle_retry_capability_latches_all_status_excursions_before_confirmed_fast_path(monkeypatch,events,rejections):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    for index in range(rejections):
        if index:next_request('reservation-two')
        assert not node.handle_rgb_acquisition_pose_strict(req).success
    capability=node._rgb_prefix_capability
    for event in events:
        status[0]=event
        node._rgb_prefix_actuation_event(event)
    assert capability['revoked'] is True
    next_request('reservation-next')
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success and params[module.LEDGER_PARAM]['phase']=='revoked'
    assert planner.move_to_pose.call_count==rejections
    planner.execute_cached_strict_pose.assert_not_called()
    assert node._rgb_prefix_capability is None


@pytest.mark.parametrize('change',['missing_ref','different_ref','center','different_source','epoch',
    'mask','same_stamp','q','ledger_q','ledger_deadline','ledger_ref','ledger_attempts','restart','manual_event','epoch_event'])
def test_preflight_retry_never_refreezes_or_accepts_changed_source_receipt_or_lost_private_owner(monkeypatch,change):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    capability=node._rgb_prefix_capability
    previous=deepcopy(params[module.LEDGER_PARAM])
    if change.startswith('ledger_'):
        ledger=params[module.LEDGER_PARAM]
        if change=='ledger_q':ledger['actuation_prefix_preflight']['q_source'][0]=.001
        if change=='ledger_deadline':
            ledger['deadline_ns']=str(int(ledger['deadline_ns'])+10**9)
            ledger['actuation_prefix_preflight']['deadline_ns']=ledger['deadline_ns']
        if change=='ledger_ref':
            ledger['reference_id']='c'*64
            ledger['actuation_prefix_preflight']['reference_id']='c'*64
        if change=='ledger_attempts':ledger['attempts']=2
    elif change=='restart':node._rgb_prefix_capability=None
    elif change in ('manual_event','epoch_event'):
        node._rgb_acquisition_revoke_event()
    else:
        fresh=node._rgb_wait_initial.side_effect(reference_id='d'*64)
        if change=='missing_ref':fresh['reference_id']=''
        if change=='different_ref':fresh['reference_id']='c'*64
        if change=='center':fresh['interest']['rgb_search_basis']='image_center'
        if change=='different_source':fresh['source_id']='c'*64
        if change=='epoch':fresh['source_context']['epoch_ns']=6
        if change=='mask':fresh['masks']=[np.zeros((20,20),dtype=bool)]
        if change=='same_stamp':fresh['source_stamp_ns']=int(previous['actuation_prefix_preflight']['last_capture_source_stamp_ns'])
        if change=='q':fresh['q_source'][2]=.0076699
        node._rgb_wait_initial.side_effect=None
        node._rgb_wait_initial.return_value=fresh
    next_request('reservation-two')
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success
    assert planner.move_to_pose.call_count==1
    node._rgb_freeze_reference.assert_called_once()
    assert node._rgb_prefix_capability is None
    node._rgb_fresh_path_audit.assert_not_called()


@pytest.mark.parametrize('failure',['persist','fault_during_persist','unknown_transport','postsend_failure'])
def test_submission_or_uncertain_execution_never_issues_no_submission_retry_receipt(monkeypatch,failure):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    if failure in ('persist','fault_during_persist'):
        original=node._rgb_update_ledger
        def persist(episode,phase,**values):
            if phase=='executing':
                assert node._rgb_prefix_capability['submission_started'] is True
                if failure=='persist':raise OSError('durable store failed')
                status[0]='UNCONFIRMED:ENCODER_RESPONSE_LOST'
                node._rgb_prefix_actuation_event(status[0])
            return original(episode,phase,**values)
        node._rgb_update_ledger=persist
    else:
        def execute(target):
            plan=prefix_trajectory()
            audit=planner.observation_path_validator(plan)
            planner.observation_submission_revalidate(plan,audit)
            if failure=='unknown_transport':raise OSError('send completion unknown')
            return False,'controller aborted after submission'
        planner.execute_cached_observation_prefix.side_effect=execute
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    ledger=params[module.LEDGER_PARAM]
    assert ledger['phase']=='revoked' and ledger['actuation_prefix_submission_started'] is True
    assert 'actuation_prefix_preflight' not in ledger
    assert node._rgb_prefix_capability is None
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert planner.move_to_pose.call_count==1


def test_fault_during_durable_no_submission_receipt_revokes_instead_of_issuing_live_retry(monkeypatch):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    original=node._rgb_update_ledger
    def persist(episode,phase,**values):
        original(episode,phase,**values)
        if phase=='preflight_rejected':
            node._rgb_prefix_actuation_event('UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
            node._rgb_prefix_actuation_event('CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE')
            status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    node._rgb_update_ledger=persist
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['phase']=='revoked'
    assert node._rgb_prefix_capability is None
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert planner.move_to_pose.call_count==1


@pytest.mark.parametrize('phase',['planning_rejected','revoked'])
def test_legacy_attempted_prefix_without_explicit_no_submission_receipt_stays_terminal(monkeypatch,phase):
    node,planner,params,req,status,clock,next_request=retry_rpc_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no path')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    params[module.LEDGER_PARAM]['phase']=phase
    params[module.LEDGER_PARAM].pop('actuation_prefix_preflight')
    status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    next_request('reservation-two')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['phase']==phase
    assert planner.move_to_pose.call_count==1


def test_rgb_rpc_prefix_is_one_charged_submission_and_never_continues_full_cached_path(monkeypatch):
    node,planner,params,req,status=prefix_rpc_fixture(monkeypatch)
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert result.success and 'PREFIX_CONFIRMED_REQUIRES_FRESH_REPLAN' in result.message
    planner.execute_cached_observation_prefix.assert_called_once_with(req.target)
    planner.execute_cached_strict_pose.assert_not_called()
    planner.move_to_pose.assert_called_once()
    ledger=params[module.LEDGER_PARAM]
    assert ledger['attempts']==1 and ledger['observation_index']==1
    assert ledger['phase']=='awaiting_measured_target'
    assert ledger['receipt']=='actuation_prefix_confirmed_requires_fresh_replan'
    assert ledger['requested_target_reached'] is False and 'target_pose_sha256' not in ledger
    assert ledger['actuation_prefix']['terminal_joint_positions']==[.025]*6
    assert params[module.STATUS_PARAM]['measured_target_established'] is False
    assert params[module.STATUS_PARAM]['contact_authorized'] is False
    assert node._rgb_prefix_capability is None


@pytest.mark.parametrize('failure',['scene_rejected','execution_failed','no_measured_response','lost_after_settle'])
def test_rgb_prefix_failure_cannot_claim_confirmation_refund_or_continue(monkeypatch,failure):
    node,planner,params,req,status=prefix_rpc_fixture(monkeypatch)
    if failure=='scene_rejected':
        node._rgb_validate_path.side_effect=ObservationPathError('scene rejected')
    if failure=='execution_failed':
        planner.execute_cached_observation_prefix.side_effect=None
        planner.execute_cached_observation_prefix.return_value=(False,'action failed')
    if failure=='no_measured_response':
        def no_response(episode):
            status[0]='UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
            node._rgb_require_actuation(episode)
        node._rgb_wait_prefix_confirmation=no_response
    if failure=='lost_after_settle':
        def settle():
            if planner.execute_cached_observation_prefix.call_count:
                status[0]='UNCONFIRMED:ENCODER_RESPONSE_LOST'
            return True,'stationary'
        node._wait_for_stationary_rgb_feedback.side_effect=settle
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success
    planner.execute_cached_strict_pose.assert_not_called()
    ledger=params[module.LEDGER_PARAM]
    assert ledger['attempts']==1 and ledger['actuation_prefix_attempted'] is True
    assert ledger['phase'] in ('revoked','planning_rejected')
    assert ledger.get('receipt')!='actuation_prefix_confirmed_requires_fresh_replan'
    assert node._rgb_prefix_capability is None


@pytest.mark.parametrize('failure',['manual','revocation','active_task','source','expired','pose'])
def test_pending_prefix_keeps_every_original_rgb_owner_gate(monkeypatch,failure):
    node,episode,captured,status=prefix_authority()
    context=dict(epoch_ns=5,other='unchanged')
    episode.update(source_id=module.digest(context),revocation=0)
    node._rgb_revocation=0
    node._rgb_context=lambda stamp:context
    node._manual_control_active=lambda:failure=='manual'
    node._observation_context_lock=threading.RLock()
    node._observation_task_connection_available=lambda:True
    node._observation_task_requires_inactive=False
    node._observation_contexts=NS(active=failure=='active_task')
    node._observation_task_state_stamp_ns=1
    node._rgb_measured_arm_snapshot=lambda:[.01 if failure=='pose' else 0.]*6
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda key,default=None:
        True if key=='/robot/observation_tracking_contract_enabled' else False)
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    monkeypatch.setattr(module.time,'monotonic',lambda:11.)
    node._rgb_admit_actuation_prefix(episode,captured)
    if failure=='revocation':node._rgb_revocation=1
    if failure=='source':context['other']='changed'
    if failure=='expired':episode['deadline_monotonic']=10.5
    with pytest.raises(ObservationPathError):
        node._rgb_require_owner(episode)


@pytest.mark.parametrize('respond',[True,False])
def test_rgb_prefix_wait_requires_fresh_measured_confirmation_and_has_fixed_deadline(monkeypatch,respond):
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    node._rgb_prefix_curve(episode,prefix_trajectory(),NS(positions=[0.]*6))
    capability=node._rgb_prefix_capability
    capability['phase']='submitted'
    episode['actuation_prefix_trajectory_sha256']=capability['trajectory_sha256']
    status[0]='PENDING:AWAITING_ENCODER_RESPONSE'
    clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    def advance(seconds):
        clock[0]+=seconds
        if respond and clock[0]>=10.1:status[0]='CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE'
    monkeypatch.setattr(module.time,'sleep',advance)
    node._rgb_require_owner=lambda current,**kwargs:node._rgb_require_actuation(current)
    if respond:
        node._rgb_wait_prefix_confirmation(episode)
        assert 10.1<=clock[0]<10.2
    else:
        with pytest.raises(ObservationPathError,match='NO_MEASURED_RESPONSE'):
            node._rgb_wait_prefix_confirmation(episode)
        assert 11.<=clock[0]<11.03


def test_rgb_prefix_next_view_requires_fresh_association_and_retains_original_budget(monkeypatch):
    node,planner,params,req,status=prefix_rpc_fixture(monkeypatch)
    assert node.handle_rgb_acquisition_pose_strict(req).success
    previous=params[module.LEDGER_PARAM]
    previous['reference_id']='original'
    incoming=request();incoming.update(reference_id='original',observation_index=2)
    ack=dict(status='supplement_ready',request_id='first',reference_id='original',source_id='a'*64,
        source_stamp_ns=str(11*10**9),association_sha256='b'*64,
        deficit_code='associated_region_lacks_measured_3d',observation_index=1)
    kwargs=dict(now_ns=12*10**9,mono=previous['created_monotonic']+2.,continuation=ack)
    fresh=module.reserve_episode(previous,incoming,'a'*64,incoming['selection'],**kwargs)
    assert fresh['attempts']==2 and fresh['observation_index']==2
    assert fresh['reference_id']=='original'
    for key in ('deadline_ns','deadline_monotonic','created_ns','created_monotonic'):
        assert fresh[key]==previous[key]
    stale=dict(ack,source_stamp_ns=previous['settled_ns'])
    kwargs['continuation']=stale
    with pytest.raises(ObservationPathError,match='POST_SETTLE_ASSOCIATION'):
        module.reserve_episode(previous,incoming,'a'*64,incoming['selection'],**kwargs)
    status[0]='PENDING:POSITIVE_ENABLE_REQUESTED'
    with pytest.raises(ObservationPathError,match='ALREADY_CONSUMED'):
        node._rgb_admit_actuation_prefix(fresh,dict(source_context=dict(epoch_ns=5)))


@pytest.mark.parametrize('change',['none','curve','audit_kind','fresh_proof','freshness','status','replay'])
def test_actual_prefix_submission_reproves_the_sealed_curve_before_issuing_pending_capability(monkeypatch,change):
    import hashlib
    node,episode,captured,status=prefix_authority()
    node._rgb_admit_actuation_prefix(episode,captured)
    plan=prefix_trajectory()
    prefix=node._rgb_prefix_curve(episode,plan,NS(positions=[0.]*6))
    node._rgb_require_owner=lambda current,**kwargs:node._rgb_require_actuation(current)
    node._observation_timing_current=lambda *args:True
    node._observation_measured_opening=lambda:.05
    node._observation_controller_hold=lambda names:NS(positions=[0.]*6)
    node._rgb_artifact_frames={}
    persisted=[]
    def persist(current,phase,**values):
        current.update(phase=phase,**values)
        persisted.append(deepcopy(current))
    node._rgb_update_ledger=Mock(side_effect=persist)
    binding=dict(reference_id='ref',reference_vertices_sha256='v'*64,
        reference_observation_set_sha256='s'*64)
    episode.update(binding)
    node._rgb_reference_visibility=lambda source:binding
    contract=dict(stop_trajectory_duration_sec=.5,joint_error_bounds_rad=[.091]*6)
    constraints={'snapshot':'unchanged'}
    params={'/robot_description':'model','/alicia_controller/constraints':constraints,
        '/alicia_controller/stop_trajectory_duration':.5,'/grasp_6d/remote/gripper_geometry':{}}
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:params.get(key,default))
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    monkeypatch.setattr(module,'SerialUrdfFk',lambda *args:object())
    captured={'source_stamp_ns':10**10}
    audit=dict(trajectory_sha256=module.wire_digest(plan),actuation_prefix=prefix,
        model_sha256=hashlib.sha256(b'model').hexdigest(),execution_tracking_contract=deepcopy(contract),
        controller_constraints_snapshot=constraints,gripper_geometry_snapshot={},opening_width_m=.05,
        rgb_interest_visibility=dict(binding,all_original_reference_geometry_visible=True))
    def prove(current,source,sent,*args,**kwargs):
        assert module.wire_digest(sent)==node._rgb_prefix_capability['trajectory_sha256']
        assert args[-1]==contract['joint_error_bounds_rad']
        if change=='fresh_proof':raise ObservationPathError('new obstacle rejects prefix')
        if change=='freshness':source['source_stamp_ns']=10**9
        if change=='status':status[0]='UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT'
        return dict(scene_sha256='new',certified_minimum_clearance_m=.0035)
    node._rgb_fresh_path_audit=Mock(side_effect=prove)
    if change=='curve':plan.joint_trajectory.points[-1].positions[0]=.024
    if change=='audit_kind':audit['actuation_prefix']['kind']='ordinary_full_path'
    if change in ('none','replay'):
        node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
        assert len(persisted)==1
        assert persisted[0]['actuation_prefix_trajectory_sha256']==module.wire_digest(plan)
        assert node._rgb_prefix_capability['phase']=='submitted'
        status[0]='PENDING:AWAITING_ENCODER_RESPONSE'
        node._rgb_require_actuation(episode)
        if change=='replay':
            with pytest.raises(ObservationPathError,match='ALREADY_SUBMITTED'):
                node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
            assert len(persisted)==1
    else:
        with pytest.raises((ObservationPathError,ValueError)):
            node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
        assert not persisted
        assert node._rgb_prefix_capability['phase']=='preparing'


def test_explicit_execute_false_cannot_create_owner_or_start_controllers(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch);req.execute=False
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success
    assert module.LEDGER_PARAM not in params
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_freeze_ack_required_before_controllers_or_planning(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch)
    node._rgb_freeze_reference.side_effect=ObservationPathError('ACK missing')
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner.move_to_pose.assert_not_called()
    assert params[module.LEDGER_PARAM]['attempts']==1


def test_real_rpc_scopes_distinct_owner_and_waits_measured_settle(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch)
    original=node.planner
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert result.success
    assert node.planner is original
    assert params[module.LEDGER_PARAM]['phase']=='awaiting_measured_target'
    assert params[module.STATUS_PARAM]['measured_target_established'] is False
    assert params[module.STATUS_PARAM]['contact_authorized'] is False
    assert node._wait_for_stationary_rgb_feedback.call_count==2
    planner.move_to_pose.assert_called_once_with(req.target,execute=False,allow_fallbacks=False)
    node._rgb_capture_after_plan.assert_called_once()
    node._rgb_validate_path.assert_called_once()
    node._rgb_submission.assert_called_once()
    assert planner.observation_path_validator is None


def test_planning_failure_never_calls_executor_and_next_request_retains_budget(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch)
    planner.move_to_pose.return_value=(False,'no collision free path')
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    planner.execute_cached_strict_pose.assert_not_called()
    assert params[module.LEDGER_PARAM]['phase']=='planning_rejected'
    deadline=params[module.LEDGER_PARAM]['deadline_ns']
    params[module.REQUEST_PARAM]['request_id']='second'
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    assert params[module.LEDGER_PARAM]['attempts']==2
    assert params[module.LEDGER_PARAM]['deadline_ns']==deadline


def test_target_mutation_rejected_before_capture_and_enable_handoff(monkeypatch):
    node,planner,params,req=node_fixture(monkeypatch);req.target.pose.position.x=1.
    assert not node.handle_rgb_acquisition_pose_strict(req).success
    node._rgb_wait_initial.assert_not_called()
    node._ensure_trajectory_controllers_started.assert_not_called()


def test_rate_limited_status_preserves_only_fresh_ready_source(monkeypatch):
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock()
    ready=dict(target_locked=False,observation_interest=dict(observation_interest_ready=True,source_stamp_ns=1))
    node._rgb_acquisition_status_cb(NS(data=json.dumps(ready)))
    node._rgb_acquisition_status_cb(NS(data=json.dumps(dict(observation_interest=dict(reason='rgb_observation_rate_limited')))))
    assert node._rgb_ready_status==ready
    node._rgb_acquisition_status_cb(NS(data=json.dumps(dict(observation_interest=dict(reason='measured_arm_pose_changed')))))
    assert node._rgb_ready_status is None


def test_manual_round_trip_has_latched_revocation_not_just_current_checkbox():
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock();node._rgb_revocation=0
    node._rgb_acquisition_revoke_event();node._rgb_acquisition_revoke_event()
    assert node._rgb_revocation==2


def continuation_fixture():
    first=reserve();first.update(phase='awaiting_measured_target',reference_id='original',
                                 settled_ns=str(5*10**9),observation_index=1)
    req=request();req.update(reference_id='original',observation_index=2)
    ack=dict(status='supplement_ready',request_id='first',reference_id='original',source_id='a'*64,
             source_stamp_ns=str(6*10**9),association_sha256='b'*64,
             deficit_code='associated_region_lacks_measured_3d',observation_index=1)
    return first,req,ack


def test_associated_supplement_retains_original_reference_deadline_and_budget():
    first,req,ack=continuation_fixture()
    second=module.reserve_episode(first,req,'a'*64,request()['selection'],7*10**9,16.,continuation=ack)
    assert second['reference_id']=='original'
    assert second['request_id']=='first'
    assert second['observation_index']==2
    assert second['attempts']==2
    assert second['deadline_ns']==first['deadline_ns']
    assert second['deadline_monotonic']==first['deadline_monotonic']
    assert second['settled_ns']==first['settled_ns']


@pytest.mark.parametrize('field,value', [('reference_id','replacement'),('request_id','replacement'),
    ('source_id','c'*64),('source_stamp_ns',str(4*10**9)),('association_sha256',''),
    ('deficit_code','collision'),('status','frozen'),('observation_index',2)])
def test_supplement_cannot_reanchor_or_use_non_visual_failure(field,value):
    first,req,ack=continuation_fixture();ack[field]=value
    with pytest.raises(ObservationPathError,match='ASSOCIATION'):
        module.reserve_episode(first,req,'a'*64,request()['selection'],7*10**9,16.,continuation=ack)


def test_injected_scene_validator_is_used_without_constructing_old_target(monkeypatch):
    from alicia_flexible_grasp.robot import observation_preview
    plan=trajectory();hold=NS(positions=[0.]*6)
    scene=NS(normal=[0.,0.,1.],offset=0.)
    monkeypatch.setattr(observation_preview,'observation_following_support_bound',
                        lambda *args:dict(ok=True))
    validator=Mock(return_value={'trajectory_sha256':module.wire_digest(plan)})
    original=Mock(side_effect=AssertionError('old scene validator must not run'))
    monkeypatch.setattr(observation_preview,'validate_observation_trajectory',original)
    report=observation_preview.qualify_candidate_path(plan,hold,scene,object(),object(),.05,[.12]*6,
                                                       path_validator=validator)
    assert report['ok']
    validator.assert_called_once();original.assert_not_called()
    assert validator.call_args.kwargs['joint_error_bounds_rad']==[.12]*6


@pytest.mark.parametrize('q', [[.1,.2,.3,.9],[.3,-.4,.1,.8],[-.1,.65,.42,.31]])
def test_tf_source_matrix_uses_producer_canonical_arithmetic(q):
    q=np.asarray(q,dtype=float);q=q/np.sqrt(float(q@q))
    x,y,z,w=q/np.sqrt(float(q@q))
    expected=np.eye(4)
    expected[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    expected[:3,3]=[.1,.2,.3]
    message=NS(transform=NS(rotation=NS(**dict(zip('xyzw',q))),translation=NS(x=.1,y=.2,z=.3)))
    assert np.array_equal(module._matrix(message),expected)


@pytest.mark.parametrize('failure_kind',['none','stale','proof_budget'])
def test_fresh_reproof_preserves_bound_action_band_and_replaces_geometry(monkeypatch,failure_kind):
    import hashlib
    node=module.RgbAcquisitionGateway();node._rgb_require_owner=Mock()
    node._observation_timing_current=lambda *args:True
    node._observation_measured_opening=lambda:.05
    node._observation_controller_hold=lambda names:NS(positions=[0.]*6)
    node._rgb_update_ledger=Mock();node._rgb_artifact_frames={}
    binding=dict(reference_id='ref',reference_vertices_sha256='v'*64,
                 reference_observation_set_sha256='s'*64)
    node._rgb_reference_visibility=Mock(return_value=binding)
    episode=dict(binding)
    constraints={'snapshot':'unchanged'};contract=dict(stop_trajectory_duration_sec=.5,
                                                        joint_error_bounds_rad=[.091]*6)
    params={'/robot_description':'model','/alicia_controller/constraints':constraints,
            '/alicia_controller/stop_trajectory_duration':.5,'/grasp_6d/remote/gripper_geometry':{}}
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:params.get(key,default))
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    monkeypatch.setattr(module,'SerialUrdfFk',lambda *args:object())
    plan=trajectory();captured={'source_stamp_ns':10**9}
    audit=dict(trajectory_sha256=module.wire_digest(plan),model_sha256=hashlib.sha256(b'model').hexdigest(),
        execution_tracking_contract=deepcopy(contract),controller_constraints_snapshot=constraints,
        gripper_geometry_snapshot={},opening_width_m=.05,scene_sha256='old',
        following_support={'old':True},command_derivative_bounds={'old_command':'unchanged'},
        rgb_interest_visibility=dict(binding,all_original_reference_geometry_visible=True))
    def prove(episode,source,*args,**kwargs):
        if failure_kind=='proof_budget':
            from alicia_grasp_modes.rgb_noncontact_scene import RgbNoncontactError
            raise RgbNoncontactError('RGB_NONCONTACT_LOCAL_OBSTACLE_PROOF_BUDGET_EXHAUSTED')
        source['source_stamp_ns']=10**9 if failure_kind=='stale' else 10**10
        return dict(scene_sha256='new',source_stamp_ns=source['source_stamp_ns'],
                    following_support={'new':True},certified_minimum_clearance_m=.0035)
    node._rgb_fresh_path_audit=Mock(side_effect=prove)
    if failure_kind=='proof_budget':
        with pytest.raises(ObservationPathError,match='PROOF_BUDGET_EXHAUSTED'):
            node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
        node._rgb_update_ledger.assert_not_called()
        node._rgb_fresh_path_audit.assert_called_once()
        assert node._rgb_fresh_path_audit.call_args.args[-1]==contract['joint_error_bounds_rad']
        assert audit['execution_tracking_contract']==contract and audit['scene_sha256']=='old'
    elif failure_kind=='stale':
        with pytest.raises(ValueError,match='frame_limit'):
            node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
        node._rgb_update_ledger.assert_not_called()
    else:
        node._rgb_submission(episode,captured,plan,audit,object(),('timing',))
        assert audit['scene_sha256']=='new'
        assert audit['following_support']=={'new':True}
        assert audit['execution_tracking_contract']==contract
        assert audit['command_derivative_bounds']=={'old_command':'unchanged'}
        assert node._rgb_fresh_path_audit.call_args.args[-1]==contract['joint_error_bounds_rad']
        node._rgb_update_ledger.assert_called_once()


@pytest.mark.parametrize('visible',[True,False])
def test_submission_source_requires_its_own_complete_volume_visibility(monkeypatch,visible):
    from alicia_grasp_modes import rgb_noncontact_scene
    node=module.RgbAcquisitionGateway();node._rgb_artifact_frames={};node._rgb_require_owner=Mock()
    corners=np.array(list(__import__('itertools').product((0.,1.),repeat=3)))
    volume=NS(vertices_base_m=corners)
    fresh=dict(volume=volume,volumes=(volume,),interest={'observation_set_sha256':'b'*64,
        'interest_volumes':[dict(vertices_base_m=corners.tolist())],
        'interest_volume':dict(vertices_base_m=corners.tolist())},
        depth_raw=np.ones((2,2)),depth_scale=1.,source_stamp_ns=10**10,
        source_depth_sha256='a'*64,cfg=NS(support_threshold_m=.002,depth_min_m=.03,depth_max_m=2.),
        source_context=dict(tool_camera=np.eye(4),camera=dict(fx=1.,fy=1.,cx=0.,cy=0.,width=2,height=2)),
        limits=dict(minimum_depth_m=.07,maximum_depth_m=.5,margin_px=0.))
    node._rgb_capture_after_plan=Mock(return_value=fresh)
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    monkeypatch.setattr(rgb_noncontact_scene,'build_rgb_noncontact_scene',Mock(return_value='fresh'))
    monkeypatch.setattr(rgb_noncontact_scene,'retain_observed_obstacles',Mock(return_value='union'))
    monkeypatch.setattr(rgb_noncontact_scene,'validate_rgb_noncontact_trajectory',Mock(return_value={'scene':'new'}))
    view=Mock(return_value={'checked':'new-source'})
    if not visible:
        view.side_effect=ObservationPathError('fresh volume out of FOV')
    monkeypatch.setattr(module,'validate_observation_target_trajectory',view)
    def run():
        return node._rgb_fresh_path_audit(dict(request_id='r'),dict(proof_scene='old'),trajectory(),
            object(),object(),.05,NS(positions=[0.]*6),[.09]*6)
    if visible:
        result=run()['rgb_interest_visibility']
        assert result['all_members_visible'] and result['members']==[{'checked':'new-source'}]
    else:
        with pytest.raises(ObservationPathError,match='out of FOV'):
            run()
    assert np.array_equal(view.call_args.args[3],volume.vertices_base_m)


def member_fixture():
    from alicia_grasp_modes.rgb_observation_region import RgbObservationRegion,RgbObservationRegions
    regions=[]
    for x,score in [(40,.1),(60,.15)]:
        mask=np.zeros((100,100),dtype=bool);mask[45:55,x:x+10]=True
        regions.append(RgbObservationRegion(mask,(x,45,10,10),(x+4.5,49.5),100,
            1.,.8,30.,.1,score,('gray',),None))
    policy=module.CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.25)
    proposals=RgbObservationRegions(tuple(regions),None,'ambiguous_rgb_regions',(100,100),2)
    metadata=module.observation_set_metadata(regions)
    interest=dict(rgb_interest_members=metadata,observation_set_sha256=module.observation_set_digest(regions),
                  region_mask_sha256=metadata[0]['mask_sha256'])
    return interest,proposals,policy


def test_independent_set_capture_retains_ambiguity_without_inventing_single_object():
    interest,proposals,policy=member_fixture()
    regions=module.verify_rgb_interest_members(interest,proposals,policy)
    assert len(regions)==2 and proposals.selected is None
    assert regions[0].mask is proposals.candidates[0].mask
    assert not any(region.object_identity_confirmed for region in regions)


@pytest.mark.parametrize('change',['legacy','mask','bbox','omit_current_competitor','duplicate','order'])
def test_set_capture_rejects_member_forgery_and_omitted_ambiguity(change):
    interest,proposals,policy=member_fixture();interest=deepcopy(interest)
    members=list(interest['rgb_interest_members']);interest['rgb_interest_members']=members
    if change=='legacy': interest.pop('rgb_interest_members')
    elif change=='mask': members[1]['mask_sha256']='f'*64
    elif change=='bbox': members[1]['bbox_xywh'][0]+=1
    elif change=='duplicate': members[1]=deepcopy(members[0])
    elif change=='order': members.reverse()
    elif change=='omit_current_competitor':
        one=(proposals.candidates[0],)
        interest.update(rgb_interest_members=module.observation_set_metadata(one),
            observation_set_sha256=module.observation_set_digest(one))
    with pytest.raises(ObservationPathError):
        module.verify_rgb_interest_members(interest,proposals,policy)


def test_stable_superset_can_retain_real_current_member_after_score_margin_changes():
    interest,proposals,policy=member_fixture()
    second=replace(proposals.candidates[1],score=.4)
    proposals=replace(proposals,candidates=(proposals.candidates[0],second),
                      selected_index=0,reason='observation_region_found')
    assert len(module.verify_rgb_interest_members(interest,proposals,policy))==2


def test_fresh_member_masks_require_unique_complete_correspondence():
    _,proposals,_=member_fixture();first,second=[r.mask for r in proposals.candidates]
    module.require_same_rgb_member_masks((second,first),(first,second))
    for current,prior in [((first,),(first,second)),((first,first),(first,first)),
                          ((first,np.zeros_like(second)),(first,second))]:
        with pytest.raises(ObservationPathError):
            module.require_same_rgb_member_masks(current,prior)


def member_volume_fixture():
    interest,proposals,policy=member_fixture()
    T=np.array([[0.,-1.,0.,0.],[0.,0.,1.,0.],[-1.,0.,0.,.35],[0.,0.,0.,1.]])
    K=np.array([[400.,0.,50.],[0.,400.,50.],[0.,0.,1.]])
    provenance=module.SupportPlaneProvenance(10**9,(0.,0.,1.,0.),'frame',800,.9,.001,4)
    volumes=tuple(module.RGBObservationVolume(r.bbox_xywh,(100,100),K,T,(0.,0.,1.,0.),
        provenance,.12,10**9,(.07,.5)) for r in proposals.candidates)
    envelope=replace(volumes[0],bbox_xywh=(40,45,30,10))
    interest.update(interest_volumes=[asdict(v) for v in volumes],interest_volume=asdict(envelope))
    arguments=dict(ns=10**9,K=K,T=T,cfg=NS(foreground_max_m=.12),
                   limits=dict(minimum_depth_m=.07,maximum_depth_m=.5))
    return interest,proposals,arguments


def test_member_volume_reconstruction_keeps_envelope_separate_from_real_members():
    interest,proposals,arguments=member_volume_fixture()
    volumes,envelope=module.decode_rgb_interest_volumes(interest,proposals.candidates,**arguments)
    assert len(volumes)==2 and envelope.bbox_xywh==(40.,45.,30.,10.)
    assert all(v.bbox_xywh!=envelope.bbox_xywh for v in volumes)


@pytest.mark.parametrize('change',['missing','swap','support','envelope','vertices'])
def test_member_volume_source_or_envelope_forgery_rejects(change):
    interest,proposals,arguments=member_volume_fixture()
    if change=='missing': interest.pop('interest_volumes')
    elif change=='swap': interest['interest_volumes'].reverse()
    elif change=='support': interest['interest_volumes'][1]['support_provenance']['source_record_id']='other'
    elif change=='envelope': interest['interest_volume']=deepcopy(interest['interest_volumes'][0])
    elif change=='vertices': interest['interest_volumes'][1]['vertices_base_m']=np.zeros((8,3)).tolist()
    with pytest.raises(ObservationPathError):
        module.decode_rgb_interest_volumes(interest,proposals.candidates,**arguments)


def test_all_member_trajectory_visibility_is_required(monkeypatch):
    interest,proposals,arguments=member_volume_fixture()
    volumes,envelope=module.decode_rgb_interest_volumes(interest,proposals.candidates,**arguments)
    captured=dict(interest=interest,volumes=volumes,volume=envelope,
        source_context=dict(tool_camera=np.eye(4),camera=dict(fx=400.,fy=400.,cx=50.,cy=50.,width=100,height=100)),
        limits=dict(roi_margin_px=0.,minimum_depth_m=.07,maximum_depth_m=.5))
    node=module.RgbAcquisitionGateway()
    validator=Mock(side_effect=[dict(ok=True),ObservationPathError('second hypothesis leaves FOV')])
    monkeypatch.setattr(module,'validate_observation_target_trajectory',validator)
    with pytest.raises(ObservationPathError,match='second hypothesis'):
        node._rgb_member_visibility(captured,trajectory(),object(),NS(positions=[0.]*6))
    assert validator.call_count==2
    assert np.array_equal(validator.call_args.args[3],volumes[1].vertices_base_m)


def test_identity_ambiguity_is_visual_information_deficit_only():
    first,req,ack=continuation_fixture();ack['deficit_code']='observation_identity_ambiguous'
    result=module.reserve_episode(first,req,'a'*64,request()['selection'],7*10**9,16.,continuation=ack)
    assert result['attempts']==2 and result['reference_id']=='original'


def capture_fixture(monkeypatch):
    interest,proposals,arguments=member_volume_fixture()
    policy=module.CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.25)
    ns=arguments['ns'];source_time=rospy.Time.from_sec(ns/1e9)
    selection=dict(mode='unknown',strategy='observe_return',generation=2,stamp_ns=100)
    camera=dict(width=100,height=100,fx=400.,fy=400.,cx=50.,cy=50.,
                align_depth_to_color=True,frame_id='camera_link',depth_scale=.0001)
    description='<robot name="test">'+''.join(
        '<joint name="%s" type="revolute"><parent link="%s"/><child link="%s"/>'
        '<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>'
        %(name,'base_link' if i==0 else 'link%d'%i,'tool0' if i==5 else 'link%d'%(i+1))
        for i,name in enumerate(module.ARM_NAMES))+'</robot>'
    monkeypatch.setattr(module.rospy,'get_param_cached',lambda key,default=None:
        description if key=='/robot_description' else default)
    context=dict(epoch_ns=200,selection=selection,camera=camera,
        tool_camera=arguments['T'].tolist(),model_sha256=module.hashlib.sha256(description.encode()).hexdigest(),
        perception={'center_radius_fraction':.25},observation_view_parameters=module.VIEW_DEFAULTS)
    source_id=module.digest(context)
    interest.update(observation_interest_ready=True,source_stamp_ns=ns,
        source_id=source_id,source_context=context,q_source=[0.]*6,T_base_camera=arguments['T'].tolist(),
        rgb_search_policy=asdict(policy),rgb_search_basis='initial_center_policy',
        support_metrics={'support_ring_outer_radius_px':70.},
        rgb_track=dict(confirmed=True,source_id=source_id,generation=2,
                       source_stamps_ns=[ns-200_000_000,ns-100_000_000,ns]))
    images={kind:NS(kind=kind,header=NS(frame_id='camera_link',stamp=source_time)) for kind in ('rgb','depth')}
    arrays=dict(rgb=np.zeros((100,100,3),dtype=np.uint8),depth=np.full((100,100),3500,dtype=np.uint16))
    interest.update(source_rgb_sha256=module.hashlib.sha256(arrays['rgb'].tobytes()).hexdigest(),
        source_depth_sha256=module.hashlib.sha256(arrays['depth'].tobytes()).hexdigest())
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock()
    node._rgb_ready_status=dict(observation_interest=interest,target_locked=False,target_lost=False)
    node._rgb_pairs={ns:images};node._rgb_reference_ack=None
    node._rgb_bridge=NS(imgmsg_to_cv2=lambda message,encoding:arrays[message.kind])
    node._rgb_context=lambda stamp:deepcopy(context)
    node._endpoint_feedback_lock=threading.RLock()
    messages=[NS(header=NS(frame_id='sdk_measured',stamp=rospy.Time.from_sec(t)),
                 name=list(module.ARM_NAMES),position=[0.]*6) for t in (.95,1.05)]
    node._endpoint_feedback_history={'accepted':messages}
    node._rgb_measured_arm_snapshot=lambda:np.zeros(6)
    node._observation_view_tf_buffer=NS(lookup_transform=lambda *args:arguments['T'])
    monkeypatch.setattr(module,'_matrix',lambda value:value)
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time.from_sec(1.05))
    monkeypatch.setattr(module,'propose_rgb_observation_regions',Mock(return_value=proposals))
    monkeypatch.setattr(module,'measured_support_coverage',Mock(return_value=dict(
        spatial_quadrants=4,measured_support_pixel_count=1000)))
    return node,source_id,proposals


def test_capture_binds_actual_independent_members_and_their_own_volumes(monkeypatch):
    node,source_id,proposals=capture_fixture(monkeypatch)
    captured=node._rgb_capture(source_id)
    assert len(captured['volumes'])==len(captured['masks'])==2
    assert captured['mask'] is captured['masks'][0]
    assert np.count_nonzero(captured['mask'])==100
    assert all(tuple(v.bbox_xywh)==tuple(r.bbox_xywh) for v,r in zip(captured['volumes'],proposals.candidates))
    assert captured['volume'].bbox_xywh==(40.,45.,30.,10.)
    assert np.count_nonzero(captured['masks'][0]|captured['masks'][1])==200
    assert module.propose_rgb_observation_regions.call_count==1


@pytest.mark.parametrize('change',['legacy','stale','different_tf','different_q','no_feedback_bracket','omit_member','rgb_bytes','depth_bytes'])
def test_capture_rejects_stale_source_and_set_contract_breaks(monkeypatch,change):
    node,source_id,proposals=capture_fixture(monkeypatch)
    interest=node._rgb_ready_status['observation_interest']
    if change=='legacy': interest.pop('rgb_interest_members')
    elif change=='stale': monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(2))
    elif change=='different_tf': interest['T_base_camera'][0][3]+=.001
    elif change=='different_q': interest['q_source'][0]=.01
    elif change=='no_feedback_bracket': node._endpoint_feedback_history['accepted'].pop()
    elif change=='rgb_bytes': interest['source_rgb_sha256']='f'*64
    elif change=='depth_bytes': interest['source_depth_sha256']='f'*64
    elif change=='omit_member':
        region=proposals.candidates[0]
        interest.update(rgb_interest_members=module.observation_set_metadata((region,)),
                        observation_set_sha256=module.observation_set_digest((region,)))
    with pytest.raises((ValueError,ObservationPathError)):
        node._rgb_capture(source_id)


@pytest.mark.parametrize('match',[True,False])
def test_freeze_ack_must_bind_exact_set_and_preserve_all_original_volumes(monkeypatch,match):
    node,source_id,_=capture_fixture(monkeypatch);captured=node._rgb_capture(source_id)
    episode=dict(request_id='r',source_id=source_id,selection=captured['source_context']['selection'],
                 deadline_ns=str(120*10**9),deadline_monotonic=time.monotonic()+5.,max_observations=3)
    node._rgb_reference_pub=Mock();node._rgb_require_owner=Mock();node._rgb_update_ledger=Mock()
    node._rgb_reference_ack=dict(version=1,request_id='r',source_id=source_id,
        source_stamp_ns=str(captured['source_stamp_ns']),reference_id='fixed',status='frozen',
        observation_set_sha256=captured['interest']['observation_set_sha256'] if match else 'f'*64)
    if not match:
        with pytest.raises(ObservationPathError,match='FREEZE_REJECTED'):
            node._rgb_freeze_reference(episode,captured)
        return
    node._rgb_freeze_reference(episode,captured)
    request=json.loads(node._rgb_reference_pub.publish.call_args.args[0].data)
    assert request['observation_set_sha256']==captured['interest']['observation_set_sha256']
    assert episode['reference_observation_set_sha256']==request['observation_set_sha256']
    assert len(json.loads(episode['reference_volumes_json']))==2
    assert captured['reference_id']=='fixed'


def install_fresh_audit_fixture(monkeypatch, node, fail_at=None):
    """Deterministic CPU/ROS clocks for the actual final-source proof path."""
    from alicia_grasp_modes import rgb_noncontact_scene
    clock=[10.]
    volume=NS(vertices_base_m=np.arange(24).reshape(8,3))
    fresh=dict(volume=volume,volumes=(volume,),interest={'observation_set_sha256':'b'*64},
        depth_raw=np.array([[0,250],[251,252]],dtype=np.uint16),depth_scale=.001,
        source_stamp_ns=10**10,source_depth_sha256='c'*64,source_rgb_sha256='d'*64,
        cfg=NS(support_threshold_m=.002,depth_min_m=.03,depth_max_m=2.))
    failure=rgb_noncontact_scene.RgbNoncontactError('RGB_NONCONTACT_LOCAL_OBSTACLE_PROOF_BUDGET_EXHAUSTED',
        {'checked_intervals':17,'certified_minimum_clearance_m':.005})
    def step(stage, elapsed, value):
        def run(*args,**kwargs):
            clock[0]+=elapsed
            if stage==fail_at:
                raise failure
            return value
        return Mock(side_effect=run)
    capture=step('capture',.04,fresh)
    node._rgb_capture_after_plan=capture
    node._rgb_require_owner=step('final_binding',.01,None)
    node._rgb_member_visibility=step('member_visibility',.02,{'all_members_visible':True})
    build=step('scene_build',.03,'new-scene')
    retain=step('retain',.02,'bounded-union')
    proof=step('proof',.10,{'trajectory_sha256':module.wire_digest(trajectory()),'scene_sha256':'new'})
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(rgb_noncontact_scene,'build_rgb_noncontact_scene',build)
    monkeypatch.setattr(rgb_noncontact_scene,'retain_observed_obstacles',retain)
    monkeypatch.setattr(rgb_noncontact_scene,'validate_rgb_noncontact_trajectory',proof)
    return fresh,failure,dict(capture=capture,scene_build=build,retain=retain,proof=proof)


@pytest.mark.parametrize('fail_at',[None,'capture','scene_build','proof','member_visibility','final_binding'])
def test_fresh_proof_records_actual_failed_source_and_scalar_stage_costs(monkeypatch,fail_at):
    node=module.RgbAcquisitionGateway();node._rgb_artifact_frames={}
    fresh,failure,calls=install_fresh_audit_fixture(monkeypatch,node,fail_at)
    prior=dict(proof_scene='old',source_stamp_ns=9*10**9)
    def run():
        return node._rgb_fresh_path_audit(dict(request_id='r'),prior,trajectory(),
            object(),object(),.05,NS(positions=[0.]*6),[.081]*6,artifact_label='submission')
    if fail_at:
        with pytest.raises(ObservationPathError,match='PROOF_BUDGET_EXHAUSTED'):
            run()
        saved=node._rgb_artifact_failure
        assert saved['stage']=='submission' and saved['failed_stage']==fail_at
        assert saved['audit']==failure.audit and saved['motion_submitted'] is False
        assert saved['joint_error_bounds_rad']==[.081]*6
        assert saved['trajectory_sha256']==module.wire_digest(trajectory())
        assert prior==dict(proof_scene='old',source_stamp_ns=9*10**9)
        timing=saved['fresh_source_timing']
        if fail_at=='capture':
            assert saved['source_stamp_ns'] is None
            assert 'submission' not in node._rgb_artifact_frames
        else:
            assert saved['source_stamp_ns']==str(fresh['source_stamp_ns'])
            assert node._rgb_artifact_frames['submission']['depth_raw'] is fresh['depth_raw']
            assert node._rgb_artifact_frames['submission']['source_depth_sha256']=='c'*64
    else:
        audit=run();timing=audit['fresh_source_timing']
        assert node._rgb_artifact_failure is None
        assert prior['proof_scene']=='bounded-union' and prior['source_stamp_ns']==fresh['source_stamp_ns']
        assert timing['scene_build_elapsed_sec']==pytest.approx(.05)
        assert timing['proof_elapsed_sec']==pytest.approx(.1)
        assert timing['member_visibility_elapsed_sec']==pytest.approx(.02)
        assert timing['source_age_at_member_visibility_sec']==pytest.approx(.21)
        assert timing['total_elapsed_sec']==pytest.approx(.22)
    assert timing['capture_elapsed_sec']==pytest.approx(.04)
    assert all(value is None or type(value) is float for value in timing.values())
    assert calls['capture'].call_count==1
    assert calls['proof'].call_count<=1 and node._rgb_member_visibility.call_count<=1


def test_submission_failure_saved_only_by_outer_rpc_separate_from_qualified_audit(monkeypatch):
    from alicia_flexible_grasp.robot import rgb_acquisition_artifacts
    node,planner,params,req=node_fixture(monkeypatch)
    fresh,failure,_=install_fresh_audit_fixture(monkeypatch,node,'proof')
    save=Mock(return_value='/external/fixture')
    monkeypatch.setattr(rgb_acquisition_artifacts,'save_artifacts',save)
    qualified={'trajectory_sha256':module.wire_digest(trajectory()),'ok':True,'scene_sha256':'qualified'}
    def execute(_target):
        planner._last_observation_path_audit=deepcopy(qualified)
        try:
            node._rgb_fresh_path_audit(dict(request_id='r'),dict(proof_scene='old'),trajectory(),
                object(),object(),.05,NS(positions=[0.]*6),[.081]*6,artifact_label='submission')
        except ObservationPathError as exc:
            save.assert_not_called()
            return False,str(exc)
        raise AssertionError('proof unexpectedly accepted')
    planner.execute_cached_strict_pose.side_effect=execute
    result=node.handle_rgb_acquisition_pose_strict(req)
    assert not result.success and params[module.LEDGER_PARAM]['phase']=='planning_rejected'
    save.assert_called_once()
    frames,outcome=save.call_args.args
    assert frames['submission']['depth_raw'] is fresh['depth_raw']
    assert outcome['failure']==str(failure)
    assert outcome['failed_proof']['audit']==failure.audit
    assert outcome['failed_proof']['failed_stage']=='proof'
    assert save.call_args.kwargs['audit']==qualified
    assert node._rgb_artifact_frames=={}


def test_local_proof_budget_rejection_advances_existing_finite_contract_search(monkeypatch):
    from alicia_grasp_modes.rgb_noncontact_scene import RgbNoncontactError
    from alicia_flexible_grasp.robot import observation_preview
    monkeypatch.setattr(observation_preview,'observation_following_support_bound',
        lambda *args:dict(ok=True))
    used=[]
    def validator(*args,**kwargs):
        used.append(kwargs['joint_error_bounds_rad'])
        if len(used)<3:
            raise RgbNoncontactError('RGB_NONCONTACT_LOCAL_OBSTACLE_PROOF_BUDGET_EXHAUSTED')
        return {'trajectory_sha256':module.wire_digest(trajectory())}
    report=module.qualify_contract_path(trajectory(),NS(positions=[0.]*6),NS(normal=[0.,0.,1.],offset=0.),
        NS(names=module.ARM_NAMES),object(),.05,{name:{'trajectory':.12} for name in module.ARM_NAMES},
        .5,path_validator=validator)
    assert report['ok'] and len(used)==3
    assert [entry['ok'] for entry in report['tracking_contract_search']]==[False,False,True]
    assert report['execution_tracking_contract']['path_position_tolerance_rad']==[.05]*6
    assert report['execution_tracking_contract']['joint_error_bounds_rad']==used[-1]


def test_capture_status_trace_exposes_ready_clear_without_images_and_is_bounded(monkeypatch):
    from collections import deque
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock()
    node._rgb_status_trace=deque(maxlen=256);node._rgb_pairs={10:{'rgb':object()}}
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    ready=dict(source_stamp_ns=10,observation_interest=dict(observation_interest_ready=True,
        source_stamp_ns=10,reason='frozen_reference_fresh_scene',reference_id='ref',
        rgb_track=dict(confirmed=True,source_stamps_ns=[8,9,10]),
        source_context={'must_not_copy':'x'*10000},rgb_interest_members=[{}]),
        rgb_acquisition_handoff=dict(reason='frozen_reference_fresh_scene',rgb_consistent_frames=3,
            rgb_associated_stable=True,rgb_interest_member_count=1))
    for _ in range(300): node._rgb_acquisition_status_cb(NS(data=json.dumps(ready)))
    node._rgb_acquisition_status_cb(NS(data=json.dumps(dict(observation_interest=dict(
        observation_interest_ready=False,reason='rgb_observation_rate_limited')))))
    node._rgb_acquisition_status_cb(NS(data=json.dumps(dict(observation_interest=dict(
        observation_interest_ready=False,reason='accepted_feedback_does_not_bracket_source')))))
    assert len(node._rgb_status_trace)==256 and node._rgb_ready_status is None
    rows=list(node._rgb_status_trace)
    assert [row['ready_cache_action'] for row in rows[-3:]]==['stored','retained_rate_limit','cleared']
    assert rows[-3]['track_stamps_ns']==[8,9,10]
    assert rows[-3]['rgb_associated_stable'] and rows[-3]['member_count']==1
    assert rows[-3]['image_pair_rgb'] and not rows[-3]['image_pair_depth']
    assert rows[-2]['cached_ready_stamp_ns']==10 and rows[-1]['cached_ready_stamp_ns'] is None
    text=json.dumps(rows)
    assert 'source_context' not in text and 'must_not_copy' not in text


@pytest.mark.parametrize('ready_present',[False,True])
def test_postplan_capture_diagnostic_distinguishes_absent_ready_and_missing_pair(monkeypatch,ready_present):
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock();node._rgb_pairs={10:{'rgb':object()}}
    ready=dict(source_stamp_ns=10,observation_interest=dict(observation_interest_ready=True,
        source_stamp_ns=10,reason='frozen_reference_fresh_scene'))
    node._rgb_status=ready if ready_present else dict(observation_interest=dict(
        observation_interest_ready=False,reason='accepted_feedback_does_not_bracket_source'))
    node._rgb_ready_status=ready if ready_present else None;node._rgb_reference_ack=None
    node._rgb_require_owner=Mock();clock=[10.]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda value:clock.__setitem__(0,clock[0]+value))
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time.from_sec(clock[0]))
    with pytest.raises(ObservationPathError,match='NO_FRESH_POST_PLAN_SOURCE'):
        node._rgb_capture_after_plan(dict(source_id='source',deadline_monotonic=100.),{})
    record=node._rgb_capture_diagnostics[0]
    assert not record['accepted'] and 3. <= record['elapsed_sec'] < 3.04
    assert record['reasons']=={'RGB_ACQUISITION_FRESH_READY_RGB_D_SOURCE_REQUIRED':record['attempts']}
    snapshot=record['latest_capture']
    assert snapshot['ready_cache_present'] is ready_present
    assert snapshot['image_pair_rgb'] is ready_present and not snapshot['image_pair_depth']
    assert record['latest_status']['reason']==('frozen_reference_fresh_scene' if ready_present else
        'accepted_feedback_does_not_bracket_source')


def test_capture_diagnostic_reason_and_span_budgets_do_not_admit_a_source(monkeypatch):
    node=module.RgbAcquisitionGateway();node._rgb_lock=threading.RLock();node._rgb_pairs={}
    monkeypatch.setattr(module.rospy.Time,'now',lambda:rospy.Time(10))
    for _ in range(20): record=node._rgb_begin_capture_diagnostic('post_plan')
    for i in range(80): node._rgb_capture_diagnostic_sample(record,'failure-'+str(i)+'x'*400)
    assert len(node._rgb_capture_diagnostics)==8
    assert len(record['reasons'])==33 and record['reasons']['<other_reasons>']==48
    assert max(map(len,record['reasons']))==256 and record['accepted'] is False


def test_failed_postplan_diagnostics_and_status_trace_persist_in_outer_rpc(monkeypatch):
    from alicia_flexible_grasp.robot import rgb_acquisition_artifacts
    from collections import deque
    node,planner,params,req=node_fixture(monkeypatch)
    save=Mock(return_value='/external/fixture')
    monkeypatch.setattr(rgb_acquisition_artifacts,'save_artifacts',save)
    def fail(*args):
        node._rgb_capture_diagnostics.append(dict(stage='post_plan',accepted=False,
            reasons={'RGB_ACQUISITION_FRESH_READY_RGB_D_SOURCE_REQUIRED':50}))
        node._rgb_status_trace=deque([dict(ready=False,reason='accepted_feedback_does_not_bracket_source')],maxlen=256)
        save.assert_not_called()
        raise ObservationPathError('NO_FRESH_POST_PLAN_SOURCE')
    node._rgb_capture_after_plan.side_effect=fail
    response=node.handle_rgb_acquisition_pose_strict(req)
    assert not response.success and params[module.LEDGER_PARAM]['phase']=='planning_rejected'
    outcome=save.call_args.args[1]
    assert outcome['capture_diagnostics'][0]['stage']=='post_plan'
    assert outcome['status_trace'][0]['reason']=='accepted_feedback_does_not_bracket_source'
    assert outcome['failure']=='NO_FRESH_POST_PLAN_SOURCE'


@pytest.mark.parametrize('forged_policy',[False,True])
def test_associated_capture_uses_shared_projection_and_retains_exact_policy_binding(monkeypatch,forged_policy):
    node,source_id,_=capture_fixture(monkeypatch)
    initial=node._rgb_capture(source_id)
    interest=node._rgb_ready_status['observation_interest']
    original=asdict(initial['volume']);members=[asdict(v) for v in initial['volumes']]
    node._rgb_reference_volume_json=json.dumps(original)
    node._rgb_reference_volumes_json=json.dumps(members)
    node._rgb_reference_set_sha256=interest['observation_set_sha256']
    node._rgb_install_reference_visibility(dict(reference_id='ref',
        reference_volume_json=node._rgb_reference_volume_json,
        reference_volumes_json=node._rgb_reference_volumes_json,
        reference_observation_set_sha256=node._rgb_reference_set_sha256))
    pixels=np.asarray(module.project_rgb_observation_vertices(original['vertices_base_m'],
        T_base_camera=interest['T_base_camera'],camera_matrix=original['camera_matrix'],
        image_shape=(100,100),working_depth_range_m=original['working_depth_range_m']))
    policy=module.projected_rgb_search_policy(module.CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.25),
        pixels,(100,100))
    interest.update(rgb_search_basis='fixed_reference_volume_projection',reference_id='ref',
        association_reference_volume=original,association_reference_volumes=members,
        association_reference_set_sha256=node._rgb_reference_set_sha256,
        association_sha256='d'*64,association_projected_volume_pixels=pixels.tolist(),rgb_search_policy=asdict(policy))
    node._rgb_reference_ack=dict(status='frozen',reference_id='ref',source_id=source_id)
    projector=Mock(wraps=module.project_rgb_observation_vertices)
    builder=Mock(wraps=module.projected_rgb_search_policy)
    monkeypatch.setattr(module,'project_rgb_observation_vertices',projector)
    monkeypatch.setattr(module,'projected_rgb_search_policy',builder)
    if forged_policy:
        interest['rgb_search_policy']['radius_fraction']=np.nextafter(policy.radius_fraction,np.inf)
        with pytest.raises(ObservationPathError,match='ORIGINAL_REFERENCE_SEARCH_POLICY_CHANGED'):
            node._rgb_capture(source_id,reference_id='ref')
    else:
        result=node._rgb_capture(source_id,reference_id='ref')
        assert result['reference_id']=='ref'
    projector.assert_called_once();builder.assert_called_once()
    assert np.array_equal(projector.call_args.args[0],original['vertices_base_m'])


def recorded_visibility_fixture():
    """Exact archived second observation: fresh mask fits, original rays do not."""
    data=json.loads((Path(__file__).parent/'fixtures'/'rgb_frozen_reference_visibility_20261004.json').read_text())
    plan=RobotTrajectory();plan.deserialize(base64.b64decode(data['trajectory_wire_base64']))
    assert module.wire_digest(plan)==data['trajectory_sha256']
    model=module.SerialUrdfFk(data['robot_description'],plan.joint_trajectory.joint_names)
    assert model.model_sha256==data['robot_description_sha256']
    captured=dict(source_context=data['source_context'],reference_id=data['reference']['reference_id'],
        interest=dict(observation_set_sha256=data['current_set_sha256']),
        volumes=tuple(NS(vertices_base_m=v) for v in data['current_vertices']),
        limits=module.effective_observation_view_limits(data['source_context']['observation_view_parameters']))
    desired=NS(positions=data['desired_positions'],velocities=[0.]*6,accelerations=[0.]*6)
    node=module.RgbAcquisitionGateway();node._rgb_install_reference_visibility(data['reference'])
    return node,captured,plan,model,desired,data


def test_recorded_second_move_cannot_replace_fixed_reference_by_fresh_smaller_volume():
    node,captured,plan,model,desired,data=recorded_visibility_fixture()
    camera=captured['source_context']['camera']
    for volume in captured['volumes']:
        prior=module.validate_observation_target_trajectory(plan,model,
            np.asarray(captured['source_context']['tool_camera']),volume.vertices_base_m,
            [camera[k] for k in ('fx','fy','cx','cy')],camera['width'],camera['height'],desired,**captured['limits'])
        assert prior['ok'] and prior['minimum_sampled_image_edge_distance_px']>56.
    with pytest.raises(ObservationPathError,match='image margin') as rejected:
        node._rgb_member_visibility(captured,plan,model,desired)
    assert rejected.value.audit['target_uv_min_px'][1]==pytest.approx(3.999618,abs=1e-5)


def test_recorded_reference_envelope_is_required_even_when_every_member_fits():
    node,captured,plan,model,desired,data=recorded_visibility_fixture()
    episode=deepcopy(data['reference']);episode.pop('reference_vertices_sha256',None)
    episode['reference_volumes_json']=json.dumps([dict(vertices_base_m=v) for v in data['current_vertices']])
    node._rgb_install_reference_visibility(episode)
    with pytest.raises(ObservationPathError,match='image margin'):
        node._rgb_member_visibility(captured,plan,model,desired)


def test_fitting_frozen_reference_has_separate_bound_member_and_envelope_audits():
    node,captured,plan,model,desired,data=recorded_visibility_fixture()
    episode=deepcopy(data['reference']);episode.pop('reference_vertices_sha256',None)
    episode['reference_volumes_json']=json.dumps([dict(vertices_base_m=v) for v in data['current_vertices']])
    episode['reference_volume_json']=json.dumps(dict(vertices_base_m=data['current_vertices'][0]))
    node._rgb_install_reference_visibility(episode)
    audit=node._rgb_member_visibility(captured,plan,model,desired)
    assert audit['all_original_reference_geometry_visible']
    assert audit['reference_envelope']['ok'] and all(row['ok'] for row in audit['reference_members'])
    for key in ('reference_id','reference_observation_set_sha256','reference_vertices_sha256'):
        assert audit[key]==episode[key]


@pytest.mark.parametrize('changed',['reference_id','set','vertices','digest'])
def test_frozen_visibility_binding_cannot_be_replaced(changed):
    node,captured,_,_,_,_=recorded_visibility_fixture()
    if changed=='reference_id': captured['reference_id']='different'
    elif changed=='set': node._rgb_reference_set_sha256='0'*64
    elif changed=='digest': node._rgb_reference_vertices_sha256='0'*64
    else:
        raw=json.loads(node._rgb_reference_volumes_json);raw[0]['vertices_base_m'][0][0]+=.001
        node._rgb_reference_volumes_json=json.dumps(raw)
    with pytest.raises(ObservationPathError,match='REFERENCE_VISIBILITY'):
        node._rgb_reference_visibility(captured)


def test_final_submission_rejects_audit_for_another_original_reference():
    node,captured,plan,_,_,data=recorded_visibility_fixture()
    node._rgb_require_owner=Mock();node._rgb_fresh_path_audit=Mock();node._rgb_update_ledger=Mock()
    binding=node._rgb_reference_visibility(captured)
    audit=dict(rgb_interest_visibility=dict(binding,all_original_reference_geometry_visible=True))
    audit['rgb_interest_visibility']['reference_vertices_sha256']='0'*64
    with pytest.raises(ObservationPathError,match='FINAL_REFERENCE_VISIBILITY_BINDING_CHANGED'):
        node._rgb_submission(data['reference'],captured,plan,audit,object(),())
    node._rgb_fresh_path_audit.assert_not_called();node._rgb_update_ledger.assert_not_called()


def test_initial_freeze_uses_exact_wire_vertices_without_reconstruction(monkeypatch):
    node,source_id,_=capture_fixture(monkeypatch)
    member=node._rgb_ready_status['observation_interest']['interest_volumes'][0]
    raw=np.asarray(member['vertices_base_m']).tolist();member['vertices_base_m']=raw
    raw[0][0]=float(np.nextafter(raw[0][0],np.inf))
    captured=node._rgb_capture(source_id)
    pending=node._rgb_reference_visibility(captured)
    assert np.array_equal(pending['member_vertices_base_m'][0],raw)
    episode=dict(request_id='r',source_id=source_id,selection=captured['source_context']['selection'],
        deadline_ns=str(120*10**9),deadline_monotonic=time.monotonic()+5.,max_observations=3)
    node._rgb_reference_pub=Mock();node._rgb_require_owner=Mock();node._rgb_update_ledger=Mock()
    node._rgb_reference_ack=dict(version=1,request_id='r',source_id=source_id,
        source_stamp_ns=str(captured['source_stamp_ns']),reference_id='ref',status='frozen',
        observation_set_sha256=captured['interest']['observation_set_sha256'])
    node._rgb_freeze_reference(episode,captured)
    assert np.array_equal(json.loads(episode['reference_volumes_json'])[0]['vertices_base_m'],raw)
    bound=node._rgb_reference_visibility(captured)
    assert bound['reference_id']=='ref' and bound['member_vertices_base_m']==pending['member_vertices_base_m']


@pytest.mark.parametrize('delta,accepted',[(0.,True),(1e-6,True),(1.01e-6,False),(.001,False)])
def test_rpc_independently_requires_original_source_fk_tf_agreement(monkeypatch,delta,accepted):
    node,source_id,_=capture_fixture(monkeypatch)
    captured=node._rgb_capture(source_id);context=captured['source_context']
    transform=np.array(captured['interest']['T_base_camera']);transform[0,3]+=delta
    if accepted:
        node._rgb_require_source_fk(context,captured['q_source'],transform)
    else:
        with pytest.raises(ObservationPathError,match='SOURCE_FK_TF_MISMATCH'):
            node._rgb_require_source_fk(context,captured['q_source'],transform)


def test_source_fk_verification_does_not_use_unbound_robot_model(monkeypatch):
    node,source_id,_=capture_fixture(monkeypatch)
    captured=node._rgb_capture(source_id);context=deepcopy(captured['source_context'])
    context['model_sha256']='0'*64
    with pytest.raises(ObservationPathError,match='SOURCE_ROBOT_MODEL_CHANGED'):
        node._rgb_require_source_fk(context,captured['q_source'],captured['interest']['T_base_camera'])


def test_explicit_new_freeze_after_plan_rejection_replaces_only_reference_binding(monkeypatch):
    node,source_id,_=capture_fixture(monkeypatch);captured=node._rgb_capture(source_id)
    first=reserve();first.update(phase='planning_rejected',reference_id='old',
        reference_vertices_sha256='0'*64)
    req=request();req['request_id']='second'
    episode=reserve(first,req,now=2*10**9,mono=11.)
    monkeypatch.setattr(module.time,'monotonic',lambda:11.)
    budget=(episode['attempts'],episode['deadline_ns'],episode['deadline_monotonic'])
    episode.update(source_id=source_id)
    node._rgb_reference_pub=Mock();node._rgb_require_owner=Mock();node._rgb_update_ledger=Mock()
    node._rgb_reference_ack=dict(version=1,request_id='second',source_id=source_id,
        source_stamp_ns=str(captured['source_stamp_ns']),reference_id='new',status='frozen',
        observation_set_sha256=captured['interest']['observation_set_sha256'])
    node._rgb_freeze_reference(episode,captured)
    assert episode['reference_id']=='new' and episode['reference_vertices_sha256']!='0'*64
    assert (episode['attempts'],episode['deadline_ns'],episode['deadline_monotonic'])==budget
    episode['reference_vertices_sha256']='0'*64
    with pytest.raises(ObservationPathError,match='REFERENCE_VISIBILITY_DIGEST_CHANGED'):
        node._rgb_install_reference_visibility(episode)


@pytest.mark.parametrize('motion_field',['submission_source_stamp_ns','submitted_ns',
    'execution_started_ns','completed_ns','settled_ns','observation_index'])
def test_later_plan_rejection_cannot_reanchor_an_episode_that_already_moved(monkeypatch,motion_field):
    node,source_id,_=capture_fixture(monkeypatch);captured=node._rgb_capture(source_id)
    previous=reserve();previous.update(phase='planning_rejected',reference_id='original')
    previous[motion_field]=2 if motion_field=='observation_index' else str(1_100_000_000)
    req=request();req['request_id']='unreferenced-new-request'
    episode=reserve(previous,req,now=2*10**9,mono=11.)
    node._rgb_reference_pub=Mock();node._rgb_update_ledger=Mock()
    with pytest.raises(ObservationPathError,match='MOVED_EPISODE_REQUIRES_ORIGINAL_REFERENCE'):
        node._rgb_freeze_reference(episode,captured)
    node._rgb_reference_pub.publish.assert_not_called();node._rgb_update_ledger.assert_not_called()
