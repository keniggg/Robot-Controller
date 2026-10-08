from copy import deepcopy
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
import rospy
from genpy.message import fill_message_args
from moveit_msgs.msg import RobotState, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.robot import contact_linear_compensation as module
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, wire_digest, validate_observation_trajectory)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values, compute_plan_id
from test_motion_gateway_controller_start import MotionGateway
from test_pregrasp_task_compensation import task_case, task
from test_pregrasp_gateway import real_context


def recorded(index=1):
    data = json.loads((Path(__file__).parent/'fixtures/contact_following_20260925.json').read_text())
    row = data['rows'][index-1]
    fk = SerialUrdfFk(data['robot_description'], ARM_NAMES)
    plan = Grasp6DPlan(); fill_message_args(plan, [data['plan']])
    nominal = RobotTrajectory(); nominal.joint_trajectory.joint_names = list(ARM_NAMES)
    for n, q in enumerate(row['nominal_joint_path']):
        nominal.joint_trajectory.points.append(JointTrajectoryPoint(positions=q,
            velocities=[0.]*6, accelerations=[0.]*6, time_from_start=rospy.Duration(n*3.)))
    actual = np.array(row['nominal_joint_path'][0])
    offset = np.array(row['hypothetical_constant_bias_rad'])
    sample = dict(model_sha256=fk.model_sha256, joint_names=list(ARM_NAMES),
        sdk_positions_rad=(actual+offset).tolist(), accepted_positions_rad=actual.tolist())
    return data, fk, plan, nominal, sample


@pytest.mark.parametrize('index', [1,2])
def test_recorded_offset_preserves_command_start_and_frozen_visual_goal(index):
    _, fk, plan, nominal, sample = recorded(index)
    original = wire_digest(nominal), wire_digest(plan)
    shifted, report = module.compensate_cartesian_path(nominal, fk, sample, plan.poses[index])
    np.testing.assert_allclose(shifted.joint_trajectory.points[0].positions, sample['sdk_positions_rad'], atol=1e-12)
    offset = np.array(report['sdk_minus_encoder_rad'])
    # Hypothetical constant following bias, not a measured grasp success.
    for before, after in zip(nominal.joint_trajectory.points, shifted.joint_trajectory.points):
        np.testing.assert_allclose(np.array(after.positions)-offset, before.positions, atol=1e-12)
        assert after.velocities == before.velocities
    assert original == (wire_digest(nominal),wire_digest(plan))
    assert report['fixed_goal_pose'] == list(pose_values(plan.poses[index]))
    assert not report['hardware_response_verified'] and not report['real_grasp_success']


def test_recorded_bias_change_is_an_explicit_counterfactual_within_arrival_tolerance():
    data, fk, plan, nominal, sample = recorded()
    shifted, _ = module.compensate_cartesian_path(nominal, fk, sample, plan.poses[1])
    samples = data['approach_samples_before_driver_trim']
    sdk = np.array(samples['/alicia_d/sdk_command']['message']['position'])
    actual = np.array(samples['/alicia_d/accepted_joint_states']['message']['position'][:6])
    goal = pose_matrix(pose_values(plan.poses[1]))[:3,3]
    assert np.linalg.norm(fk(actual)[:3,3]-goal) > .018
    # Replay the observed change in following offset against the NEW path.
    # This tests propagation arithmetic; it is not new physical feedback.
    hypothetical = np.array(shifted.joint_trajectory.points[-1].positions)-(sdk-actual)
    assert np.linalg.norm(fk(hypothetical)[:3,3]-goal) < .006


@pytest.mark.parametrize('fault', ['model','names','start','goal','nan','following','joint_limit'])
def test_invalid_feedforward_cannot_produce_a_command_path(fault):
    _, fk, plan, nominal, sample = recorded()
    if fault == 'model': sample['model_sha256'] = 'other'
    if fault == 'names': nominal.joint_trajectory.joint_names.reverse()
    if fault == 'start': nominal.joint_trajectory.points[0].positions[0] += .01
    if fault == 'goal': plan.poses[1].position.z += .01
    if fault == 'nan': sample['sdk_positions_rad'][0] = float('nan')
    if fault == 'following': sample['sdk_positions_rad'][0] += .1
    if fault == 'joint_limit': nominal.joint_trajectory.points[1].positions[0] = 100.
    with pytest.raises(ValueError):
        module.compensate_cartesian_path(nominal,fk,sample,plan.poses[1])


@pytest.mark.parametrize('index', [1,2])
def test_nominal_and_compensated_continuous_gripper_paths_keep_frozen_geometry(index):
    data,fk,plan,nominal,sample = recorded(index)
    shifted,_ = module.compensate_cartesian_path(nominal,fk,sample,plan.poses[index])
    c=data['gripper_geometry_config']
    g=GripperGeometry(c['max_inner_gap_m'], c['width_safety_margin_per_side_m'],
                     np.asarray(c['finger_box_xyz_m']),np.asarray(c['palm_box_xyz_m']),c.get('support_clearance_m',.003))
    scene=FrozenObservationScene.from_contact_plan(plan)
    for tr in (nominal,shifted):
        audit=validate_observation_trajectory(tr,scene,fk,g,.04975,tr.joint_trajectory.points[0])
        assert audit['certified_minimum_clearance_m'] >= .003-1e-9


def gateway_case(monkeypatch, index=1):
    data,fk,plan,nominal,sample=recorded(index)
    node=MotionGateway.__new__(MotionGateway)
    node._planner_lock=threading.RLock()
    node._observation_context_lock=threading.RLock()
    node._compensation_task_label=('planning linear 6D approach' if index == 1
                                   else 'planning linear 6D grasp pose')
    node._manual_control_active=lambda:False
    context=dict(plan=plan,scene=FrozenObservationScene.from_contact_plan(plan),
                 epoch=1,model=data['robot_description'],pose_index=index,deadline=time.monotonic()+120.)
    def admit_context(*args):
        node._compensation_visual_context = context
        return context
    node._pregrasp_context=mock.Mock(side_effect=admit_context, return_value=context)
    node._validate_pregrasp_context=mock.Mock()
    node._endpoint_following_snapshot=mock.Mock(side_effect=lambda *a,**k:deepcopy(sample))
    node._observation_measured_opening=lambda:.04975
    node._contact_command_collision_check=mock.Mock(return_value=10)
    node._observation_controller_hold=lambda _:JointTrajectoryPoint(
        positions=sample['sdk_positions_rad'],velocities=[0.]*6,accelerations=[0.]*6)
    node._ensure_trajectory_controllers_started=mock.Mock(return_value=(True,'running'))
    state=RobotState();state.joint_state.name=list(ARM_NAMES)+['right_finger']
    state.joint_state.position=[0.]*6+[.04975]
    planner=NS(robot=NS(get_current_state=lambda:deepcopy(state)),
        _plan_cartesian_from_start_state=mock.Mock(return_value=(nominal,'planned')),
        _retime_cartesian_plan=deepcopy,_same_joint_path_geometry=lambda a,b:True,
        _last_pose_plan=None,_pose_xyz_text=lambda _: 'test pose')
    planner._remember_pose_plan=mock.Mock(side_effect=lambda p,tr,kind:
        setattr(planner,'_last_pose_plan',dict(plan=tr)))
    def execute(*args,**kwargs):
        planner.observation_path_validator(planner._last_pose_plan['plan'])
        kwargs['before_execute']()
        return True,'executed'
    planner._execute_cached_pose_plan=mock.Mock(side_effect=execute)
    node._ensure_planner=lambda:planner
    monkeypatch.setattr(module.rospy,'get_param',lambda key,default=None:
        deepcopy(data['gripper_geometry_config']) if key=='/grasp_6d/remote/gripper_geometry' else default)
    return node,planner,plan,sample


def test_planning_has_no_controller_side_effect_and_cache_executes_only_once(monkeypatch):
    node,planner,plan,sample=gateway_case(monkeypatch)
    response=node.handle_contact_approach(NS(plan=plan,execute=False))
    assert response.success,response.message
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner._execute_cached_pose_plan.assert_not_called()
    assert node.handle_contact_approach(NS(plan=plan,execute=True)).success
    assert not node.handle_contact_approach(NS(plan=plan,execute=True)).success
    assert planner._execute_cached_pose_plan.call_count==1
    assert planner._last_pose_plan is None
    assert not hasattr(planner,'observation_path_guard_required')


@pytest.mark.parametrize('fault',['manual','epoch','feedback','changed_plan','wrong_stage','collision'])
def test_contact_cache_cannot_survive_changed_authority_or_feedback(monkeypatch,fault):
    node,planner,plan,sample=gateway_case(monkeypatch)
    assert node.handle_contact_approach(NS(plan=plan,execute=False)).success
    if fault in ('manual','epoch'):
        node._validate_pregrasp_context.side_effect=ValueError(fault+' changed')
    if fault=='feedback':sample['accepted_positions_rad'][0]+=Q
    if fault=='changed_plan':plan.poses[1].position.x+=.01;plan.plan_id=compute_plan_id(plan)
    if fault=='collision':
        node._contact_command_collision_check.side_effect=ValueError('collision')
        assert not node.handle_contact_approach(NS(plan=plan,execute=False)).success
    callback=node.handle_contact_grasp_pose if fault=='wrong_stage' else node.handle_contact_approach
    assert not callback(NS(plan=plan,execute=True)).success
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner._execute_cached_pose_plan.assert_not_called()


@pytest.mark.parametrize('index',[1,2])
def test_task_uses_compensated_line_then_measured_arrival_without_driver_trim(index):
    node,plan,pose,cfg,old_move=task_case()
    plan.model_choice='unknown_tabletop'
    cfg['unknown_contact_cartesian_compensation_enabled']=True
    pose.pose=deepcopy(plan.poses[index])
    label='linear 6D approach' if index==1 else 'linear 6D grasp pose'
    move=mock.Mock(return_value=NS(success=True,message='bounded line'))
    node._contact_linear_service=mock.Mock(return_value=move)
    node._compensate_fixed_endpoint=mock.Mock(return_value=True)
    assert node._plan_and_execute_pose(task.GraspStages.APPROACH_TARGET,label,pose,old_move,label,
                                     execution_plan=plan,gcfg=cfg,execute_pose=old_move)
    old_move.assert_not_called()
    assert [call.args[1] for call in move.call_args_list]==[False,True]
    node._compensate_fixed_endpoint.assert_called_once_with(plan,cfg,index)
    node._set_contact_endpoint_precision.assert_not_called()
    node._record_and_validate_measured_endpoint.assert_called_once()


@pytest.mark.parametrize('index',[1,2])
def test_real_context_requires_matching_unknown_contact_phase(monkeypatch,index):
    node,ctx,params=real_context(monkeypatch)
    ctx['pose_index']=index
    ctx['plan'].model_choice='unknown_tabletop';ctx['plan'].plan_id=compute_plan_id(ctx['plan'])
    ctx['scene']=FrozenObservationScene.from_contact_plan(ctx['plan'])
    params['/grasp']['unknown_contact_cartesian_compensation_enabled']=True
    ctx['config']=deepcopy(params['/grasp'])
    node._pregrasp_task_stage=task.GraspStages.APPROACH_TARGET
    node._compensation_task_label='moving linear 6D approach' if index==1 else 'moving linear 6D grasp pose'
    node._validate_pregrasp_context(ctx)
    node._compensation_task_label='moving linear 6D lift'
    with pytest.raises(ValueError,match='GOAL_NOT_CURRENT'):
        node._validate_pregrasp_context(ctx)


def test_contact_visual_scope_survives_planning_until_successful_execution(monkeypatch):
    node, planner, plan, _ = gateway_case(monkeypatch)
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    context = node._contact_linear_cache['context']
    assert node._compensation_visual_context is context
    original = planner._execute_cached_pose_plan.side_effect
    def execute(*args, **kwargs):
        assert node._compensation_visual_context is context
        return original(*args, **kwargs)
    planner._execute_cached_pose_plan.side_effect = execute
    assert node.handle_contact_approach(NS(plan=plan, execute=True)).success
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None


@pytest.mark.parametrize('failure', ['exception', 'false', 'wrong_index', 'changed_plan', 'feedback'])
def test_consumed_contact_visual_scope_is_retired_on_every_execution_failure(monkeypatch, failure):
    node, planner, plan, sample = gateway_case(monkeypatch)
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    callback = node.handle_contact_approach
    if failure == 'exception': planner._execute_cached_pose_plan.side_effect = ValueError('execution error')
    if failure == 'false': planner._execute_cached_pose_plan.side_effect = lambda *a, **kw: (False, 'rejected')
    if failure == 'wrong_index': callback = node.handle_contact_grasp_pose
    if failure == 'changed_plan': plan.poses[1].position.x += .01
    if failure == 'feedback': sample['accepted_positions_rad'][0] += Q
    assert not callback(NS(plan=plan, execute=True)).success
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None


@pytest.mark.parametrize('replacement', [False, True])
def test_failed_contact_planning_retires_its_scope_including_superseded_cache(monkeypatch, replacement):
    node, _, plan, _ = gateway_case(monkeypatch)
    if replacement:
        assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
        # Reject before admitting a new context: the previous cache's scope
        # must still be retired when that cached episode is discarded.
        node._pregrasp_context.side_effect = ValueError('new context unavailable')
    else:
        node._contact_command_collision_check.side_effect = ValueError('collision')
    assert not node.handle_contact_approach(NS(plan=plan, execute=False)).success
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None


@pytest.mark.parametrize('cached', [False, True])
def test_contact_cleanup_cannot_clear_another_episode_scope(monkeypatch, cached):
    node, _, plan, _ = gateway_case(monkeypatch)
    if cached:
        assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    other_context = {'different_episode': True}
    node._compensation_visual_context = other_context
    assert not node.handle_contact_grasp_pose(NS(plan=plan, execute=True)).success
    assert node._compensation_visual_context is other_context


@pytest.mark.parametrize('index', [1, 2])
@pytest.mark.parametrize('failed_stage', ['compensated_collision', 'nominal_continuous_geometry',
                                         'compensated_continuous_geometry'])
def test_planning_rejection_binds_exact_geometry_stage_and_trajectory(monkeypatch, index, failed_stage):
    node, planner, plan, _ = gateway_case(monkeypatch, index)
    callback = node.handle_contact_approach if index == 1 else node.handle_contact_grasp_pose
    captured = []
    error = module.ObservationPathError('continuous path clearance is unresolved at subdivision limit')
    error.code = 'CONTINUOUS_PATH_UNRESOLVED'
    error.audit = dict(limiting_segment=3, minimum_clearance_m=.0029, certificate_complete=False)
    def collision(trajectory, _start):
        if failed_stage == 'compensated_collision':
            captured.append(wire_digest(trajectory))
            raise error
        return 10
    node._contact_command_collision_check.side_effect = collision
    def retime(trajectory):
        result = deepcopy(trajectory)
        for point in result.joint_trajectory.points:
            point.time_from_start = rospy.Duration(point.time_from_start.to_sec()*2.)
        return result
    planner._retime_cartesian_plan = retime
    def geometry(trajectory, *args):
        captured.append(wire_digest(trajectory))
        if ((failed_stage == 'nominal_continuous_geometry' and len(captured) == 1)
                or failed_stage == 'compensated_continuous_geometry'):
            if failed_stage != 'compensated_continuous_geometry' or len(captured) == 2:
                raise error
        return dict(trajectory_sha256=wire_digest(trajectory))
    monkeypatch.setattr(module, 'validate_observation_trajectory', geometry)
    logger = mock.Mock()
    monkeypatch.setattr(module.rospy, 'logwarn', logger)
    response = callback(NS(plan=plan, execute=False))
    assert not response.success
    report = json.loads(response.message)
    assert report['code'] == 'CONTACT_LINEAR_COMPENSATION_FAILED'
    assert report['reason'] == str(error)
    assert report['exception_code'] == 'CONTINUOUS_PATH_UNRESOLVED'
    assert report['exception_audit'] == error.audit
    assert report['validation_stage'] == failed_stage
    assert report['plan_id'] == plan.plan_id
    assert report['goal_index'] == index and report['execute'] is False
    assert report['trajectory_sha256'] == captured[-1]
    assert len(report['trajectory_sha256']) == 64
    assert json.loads(logger.call_args.args[1]) == report
    node._ensure_trajectory_controllers_started.assert_not_called()
    planner._execute_cached_pose_plan.assert_not_called()
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None


@pytest.mark.parametrize('swallowed', [False, True])
def test_execute_guard_rejection_retains_original_audit_when_planner_converts_to_text(monkeypatch, swallowed):
    node, planner, plan, _ = gateway_case(monkeypatch)
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    cached_sha = wire_digest(node._contact_linear_cache['trajectory'])
    error = module.ObservationPathError('continuous path clearance is unresolved at subdivision limit')
    error.audit = dict(limiting_segment=4, depth=16, certificate_complete=False)
    monkeypatch.setattr(module, 'validate_observation_trajectory', mock.Mock(side_effect=error))
    submitted = mock.Mock()
    def execute(*args, **kwargs):
        try:
            planner.observation_path_validator(planner._last_pose_plan['plan'])
        except Exception as exc:
            if swallowed:
                return False, 'strict cached execute blocked: OBSERVATION_PATH_INVALID: ' + str(exc)
            raise
        submitted()
        return True, 'executed'
    planner._execute_cached_pose_plan.side_effect = execute
    response = node.handle_contact_approach(NS(plan=plan, execute=True))
    assert not response.success
    report = json.loads(response.message)
    assert report['validation_stage'] == 'execution_continuous_geometry'
    assert report['reason'] == str(error)
    assert report['exception_audit'] == error.audit
    assert report['trajectory_sha256'] == cached_sha
    assert report['plan_id'] == plan.plan_id and report['goal_index'] == 1
    assert report['execute'] is True
    if swallowed:
        assert report['planner_reason'].startswith('strict cached execute blocked:')
    submitted.assert_not_called()
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None
    assert planner._last_pose_plan is None
    assert not hasattr(planner, 'observation_path_validator')


def test_repeated_same_trajectory_rejection_has_stable_digest(monkeypatch):
    node, planner, plan, _ = gateway_case(monkeypatch)
    monkeypatch.setattr(module, 'validate_observation_trajectory', mock.Mock(
        side_effect=module.ObservationPathError('subdivision budget exhausted')))
    responses = [node.handle_contact_approach(NS(plan=plan, execute=False)) for _ in range(2)]
    reports = [json.loads(item.message) for item in responses]
    assert all(not item.success for item in responses)
    assert reports[0]['trajectory_sha256'] == reports[1]['trajectory_sha256']
    assert reports[0]['validation_stage'] == reports[1]['validation_stage'] == 'nominal_continuous_geometry'
    assert 'exception_audit' not in reports[0]
    planner._execute_cached_pose_plan.assert_not_called()
