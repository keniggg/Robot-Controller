"""Mode/stage isolation and offline timing of the actually submitted path."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from alicia_flexible_grasp.robot.pregrasp_timing import unknown_pregrasp_velocity_limits,pregrasp_velocity_scope
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.strict_collinear_timing import validate_collinear_refinement
from alicia_flexible_grasp.robot.observation_path_guard import validate_observation_trajectory_velocity
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from test_motion_gateway_controller_start import MotionGateway,motion_gateway_node
from test_strict_collinear_timing import planner_for

DATA=json.loads((Path(__file__).parent/'fixtures/pregrasp_speed_20260928_actual.json').read_text())
BASE={'Joint3':.02,'Joint4':.02,'Joint6':.02}
PROFILE={'enabled':True,'joint_velocity_rad_s':.04}


def recorded_curve(index=0):
    row=DATA['curves'][index];plan=RobotTrajectory();plan.joint_trajectory.joint_names=row['joint_names']
    for p in row['points']:
        plan.joint_trajectory.points.append(JointTrajectoryPoint(positions=p['positions'],
            velocities=p['velocities'],accelerations=p['accelerations'],
            time_from_start=rospy.Duration.from_sec(p['time_from_start_sec'])))
    return plan


@pytest.mark.parametrize('value',[None,True,'0.04',0.,-.01,float('nan'),float('inf'),.081])
def test_invalid_speed_cannot_authorize_a_profile(value):
    with pytest.raises(ValueError):
        unknown_pregrasp_velocity_limits({'mode':'unknown'},dict(PROFILE,joint_velocity_rad_s=value),.08)


def test_speed_respects_global_limit_and_other_modes():
    with pytest.raises(ValueError):unknown_pregrasp_velocity_limits({'mode':'unknown'},PROFILE,.03)
    assert unknown_pregrasp_velocity_limits({'mode':'carton'},PROFILE,.08) is None
    assert unknown_pregrasp_velocity_limits(None,PROFILE,.08) is None
    assert unknown_pregrasp_velocity_limits({'mode':'unknown'},{},.08) is None
    with pytest.raises(ValueError):unknown_pregrasp_velocity_limits({'mode':'unknown'},{'enabled':'true'},.08)


@pytest.mark.parametrize('mode,stage,label,enabled',[
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'planning 6D pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D near-field pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'planning 6D near-field pregrasp',True),
    ('unknown',GraspStages.PLAN_PREGRASP,'moving 6D near-field pregrasp',False),
    ('unknown',GraspStages.APPROACH_TARGET,'moving 6D near-field pregrasp',False),
    ('carton',GraspStages.MOVE_PREGRASP,'moving 6D near-field pregrasp',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D near-field pregrasp compensation',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D near-field pregrasp retreat',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D observation pregrasp',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'bounded final visual correction',False),
    ('unknown',GraspStages.APPROACH_TARGET,'moving linear 6D grasp pose',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'reused contact pregrasp',False),
    ('carton',GraspStages.MOVE_PREGRASP,'moving 6D pregrasp',False)])
def test_gateway_speed_scope_restores_contact_even_on_failure(monkeypatch,mode,stage,label,enabled):
    node=MotionGateway.__new__(MotionGateway);node._pregrasp_task_stage=stage;node._compensation_task_label=label
    baseline=dict(BASE);planner=NS(strict_execution_joint_velocity_limits_rad_s=baseline,
        strict_execution_max_joint_velocity_rad_s=.08)
    params={'/grasp_mode/selection':{'mode':mode},'/robot/unknown_pregrasp_timing':PROFILE}
    monkeypatch.setattr(motion_gateway_node.rospy,'get_param',lambda name,default=None:params.get(name,default))
    with pytest.raises(RuntimeError,match='execution rejected'):
        with node._strict_pregrasp_timing_scope(planner):
            assert planner.strict_execution_joint_velocity_limits_rad_s==({k:.04 for k in BASE} if enabled else BASE)
            raise RuntimeError('execution rejected')
    assert planner.strict_execution_joint_velocity_limits_rad_s is baseline
    assert not hasattr(planner,'strict_collinear_timing_enabled')


def test_sequence_estimate_changes_only_non_cartesian_pregrasp():
    planner=MoveItPlanner.__new__(MoveItPlanner);planner.strict_execution_max_joint_velocity_rad_s=.08
    planner.strict_execution_joint_velocity_limits_rad_s=dict(BASE)
    planner.unknown_pregrasp_joint_velocity_limits_rad_s={k:.04 for k in BASE}
    plan=recorded_curve();before=deepcopy(plan)
    slow=planner._plan_joint_path_metrics(plan)
    fast=planner._sequence_stage_joint_path_metrics(plan,'pregrasp',False)
    assert fast['execution_duration_lower_bound_sec']==pytest.approx(slow['execution_duration_lower_bound_sec']/2.)
    for stage,linear in [('approach',True),('grasp',True),('lift',True),('pregrasp',True),('observation',False)]:
        assert planner._sequence_stage_joint_path_metrics(plan,stage,linear)==slow
    assert planner.strict_execution_joint_velocity_limits_rad_s==BASE
    assert plan==before


def test_recorded_pregrasp_runs_faster_without_changing_path_or_acceleration_contract():
    source=recorded_curve();original=deepcopy(source)
    limits={name:(.04 if name in BASE else .08) for name in source.joint_trajectory.joint_names}
    planner=planner_for(source,limits,True)
    candidate,message=planner._retime_strict_execution_plan(source)
    assert candidate is not None,message
    proof=validate_collinear_refinement(source,candidate)
    assert proof['original_points']==len(source.joint_trajectory.points)
    hold=NS(positions=source.joint_trajectory.points[0].positions,velocities=[0.]*6,accelerations=[0.]*6)
    audit=validate_observation_trajectory_velocity(candidate,hold,limits)
    assert audit['required_time_scale']==1.
    from alicia_flexible_grasp.robot.observation_tracking_contract import validate_command_derivatives
    derivative=validate_command_derivatives(candidate,hold)
    assert max(derivative['maximum_acceleration_rad_s2']) <= .30
    old=source.joint_trajectory.points[-1].time_from_start.to_sec()
    new=candidate.joint_trajectory.points[-1].time_from_start.to_sec()
    assert new < old*.6
    assert source==original
    assert not proof['physical_tracking_verified']


@pytest.mark.parametrize('fail',[False,True])
def test_sequence_gateway_profile_is_temporary_and_uses_same_configuration(monkeypatch,fail):
    node=MotionGateway.__new__(MotionGateway)
    planner=NS(strict_execution_max_joint_velocity_rad_s=.08,
        strict_execution_joint_velocity_limits_rad_s=dict(BASE))
    node._ensure_planner=lambda:planner
    params={'/grasp_mode/selection':{'mode':'unknown'},'/robot/unknown_pregrasp_timing':PROFILE}
    monkeypatch.setattr(motion_gateway_node.rospy,'get_param',lambda name,default=None:params.get(name,default))
    def check(*args,**kwargs):
        assert planner.unknown_pregrasp_joint_velocity_limits_rad_s=={k:.04 for k in BASE}
        assert planner.strict_execution_joint_velocity_limits_rad_s==BASE
        if fail:raise RuntimeError('planning failed')
        return True,'','',{'path_cost':1.,'max_delta':.5},'checked'
    planner.check_pose_sequence=check
    request=NS(targets=[],stage_names=[],linear=[])
    if fail:
        with pytest.raises(RuntimeError,match='planning failed'):node.handle_pose_sequence_strict(request)
    else:assert node.handle_pose_sequence_strict(request).success
    assert not hasattr(planner,'unknown_pregrasp_joint_velocity_limits_rad_s')
    assert planner.strict_execution_joint_velocity_limits_rad_s==BASE


@pytest.mark.parametrize('label', [
    'planning 6D near-field pregrasp', 'moving 6D near-field pregrasp',
])
def test_recorded_near_field_pregrasp_receives_same_profile_as_candidate_estimate(monkeypatch,label):
    """Replay the missed 89 s route, including its distinct stationary SDK hold."""
    from alicia_flexible_grasp.robot.observation_tracking_contract import validate_command_derivatives
    fixture=json.loads((Path(__file__).parent/'fixtures/near_field_pregrasp_timing_20260928.json').read_text())
    source=RobotTrajectory();source.joint_trajectory.joint_names=fixture['joint_names']
    for p in fixture['points']:
        source.joint_trajectory.points.append(JointTrajectoryPoint(
            positions=p['positions'],velocities=p['velocities'],accelerations=p['accelerations'],
            time_from_start=rospy.Duration.from_sec(p['time_from_start_sec'])))
    before=deepcopy(source)
    hold=NS(positions=fixture['hold'],velocities=[0.]*6,accelerations=[0.]*6)
    assert hold.positions != list(source.joint_trajectory.points[0].positions)
    limits={name:BASE.get(name,.08) for name in fixture['joint_names']}
    planner=planner_for(source,limits,False)
    planner.controller_reference_guard_required=True
    planner.controller_reference_provider=lambda names:hold
    planner.observation_tracking_contract_required=True
    node=MotionGateway.__new__(MotionGateway)
    node._pregrasp_task_stage=GraspStages.MOVE_PREGRASP;node._compensation_task_label=label
    params={'/grasp_mode/selection':{'mode':'unknown'},'/robot/unknown_pregrasp_timing':PROFILE}
    monkeypatch.setattr(motion_gateway_node.rospy,'get_param',lambda name,default=None:params.get(name,default))
    with node._strict_pregrasp_timing_scope(planner):
        candidate,message=planner._retime_strict_execution_plan(source)
        assert candidate is not None,message
        candidate,evidence,error=planner._prepare_controller_reference_timing(candidate)
        assert not error,error
        proof=validate_collinear_refinement(source,candidate)
        expected_limits={name:(.04 if name in BASE else .08) for name in fixture['joint_names']}
        assert evidence['limits']==expected_limits
        audit=validate_observation_trajectory_velocity(candidate,hold,expected_limits)
        assert audit['required_time_scale']==1.
        assert planner._last_controller_reference_audit['applied_time_scale']==1.
        derivatives=validate_command_derivatives(candidate,hold)
        assert max(derivatives['maximum_acceleration_rad_s2']) < .03
        assert candidate.joint_trajectory.points[-1].time_from_start.to_sec()==pytest.approx(35.576623642,abs=1e-6)
        assert candidate.joint_trajectory.points[-1].time_from_start.to_sec() < source.joint_trajectory.points[-1].time_from_start.to_sec()*.4
        assert proof['original_waypoint_indices']==list(range(29))
        assert not proof['physical_tracking_verified']
    assert planner.strict_execution_joint_velocity_limits_rad_s is limits
    assert planner.strict_collinear_timing_enabled is False
    assert source==before


def test_near_field_label_cannot_enable_fast_profile_for_cartesian_service(monkeypatch):
    from test_motion_gateway_controller_start import MotionGatewayControllerStartTest
    node=MotionGatewayControllerStartTest().make_gateway()
    node._pregrasp_task_stage=GraspStages.MOVE_PREGRASP
    node._compensation_task_label='moving 6D near-field pregrasp'
    params={'/grasp_mode/selection':{'mode':'unknown'},'/robot/unknown_pregrasp_timing':PROFILE}
    monkeypatch.setattr(motion_gateway_node.rospy,'get_param',lambda name,default=None:params.get(name,default))
    node._strict_pregrasp_timing_scope=lambda planner:pytest.fail('Cartesian path must not enter non-Cartesian profile')
    response=node.handle_pose_linear(NS(target='contact pose',execute=True))
    assert response.success
    assert node.planner.calls==[('linear','contact pose',True)]
