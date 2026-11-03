"""Recorded-path timing proofs; no hardware or ROS service calls."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from genpy.message import fill_message_args
from moveit_msgs.msg import RobotTrajectory

from alicia_flexible_grasp.robot import strict_collinear_timing as timing
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, validate_observation_trajectory_velocity,
)
from alicia_flexible_grasp.robot.observation_tracking_contract import qualify_contract_path
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node


DATA = json.loads((Path(__file__).parent/'fixtures/strict_timing_20260927.json').read_text())


def curve(index):
    row = DATA['curves'][index]
    plan = RobotTrajectory()
    fill_message_args(plan.joint_trajectory, [deepcopy(row['trajectory'])])
    hold = NS(positions=row['hold'], velocities=[0.]*6, accelerations=[0.]*6)
    return plan, hold, row


@pytest.mark.parametrize('index,seconds', [(0,108.807783532),(1,28.419012491),(2,28.419012491)])
def test_recorded_curves_improve_time_under_the_exact_original_speed_limits(index, seconds):
    source, hold, row = curve(index)
    unchanged = deepcopy(source)
    candidate = timing.smooth_strict_collinear(source, row['limits'])
    assert source == unchanged
    proof = timing.validate_collinear_refinement(source, candidate)
    assert proof['original_points'] == len(source.joint_trajectory.points)
    assert proof['refined_points'] == (39 if index == 0 else 4)
    assert candidate.joint_trajectory.points[-1].time_from_start.to_sec() == pytest.approx(seconds)
    assert seconds < row['source_duration']
    audit = validate_observation_trajectory_velocity(candidate, hold, row['limits'])
    assert audit['required_time_scale'] == 1.
    for old_index, new_index in enumerate(proof['original_waypoint_indices']):
        assert candidate.joint_trajectory.points[new_index].positions == source.joint_trajectory.points[old_index].positions
    for point in (candidate.joint_trajectory.points[0], candidate.joint_trajectory.points[-1]):
        assert point.velocities == [0.]*6 and point.accelerations == [0.]*6
    if index:
        for point in candidate.joint_trajectory.points:
            for axis in (1,4):
                assert point.positions[axis] == source.joint_trajectory.points[0].positions[axis]
                assert point.velocities[axis] == point.accelerations[axis] == 0.


@pytest.mark.parametrize('index', [1,2])
def test_faster_compensation_passes_original_scene_following_and_stop_proof(index):
    source, hold, row = curve(index)
    candidate = timing.smooth_strict_collinear(source, row['limits'])
    plan = Grasp6DPlan(); fill_message_args(plan, [DATA['plan']])
    fixture = json.loads((Path(__file__).parent/'fixtures/pregrasp_following_20260919.json').read_text())
    fk = SerialUrdfFk(fixture['robot_description'], tuple(source.joint_trajectory.joint_names))
    assert fk.model_sha256 == DATA['model_sha256']
    cfg = row['gripper']
    gripper = GripperGeometry(cfg['max_inner_gap_m'], cfg['width_safety_margin_per_side_m'],
        np.array(cfg['finger_box_xyz_m']), np.array(cfg['palm_box_xyz_m']), cfg.get('support_clearance_m', .003))
    audit = qualify_contract_path(candidate, hold, FrozenObservationScene.from_contact_plan(plan),
        fk, gripper, row['opening'], row['constraints'], row['stop_duration'], trajectory_stop_reserve=True)
    assert audit['ok'], audit
    contract = audit['execution_tracking_contract']
    assert contract['path_position_tolerance_rad'] == [.035]*6
    assert contract['commanded_stop_reserve_rad'] < row['original_stop_reserve']
    assert not contract['certifies_hardware_tracking_stopping_or_calibration']


@pytest.mark.parametrize('fault', ['curve', 'reverse', 'duplicate', 'stationary'])
def test_ineligible_geometry_retains_the_original_retimer(fault):
    source, _, row = curve(0)
    if fault == 'curve': source.joint_trajectory.points[5].positions[0] += .0001
    if fault == 'reverse': source.joint_trajectory.points[5].positions = list(source.joint_trajectory.points[3].positions)
    if fault == 'duplicate': source.joint_trajectory.points[5].positions = list(source.joint_trajectory.points[4].positions)
    if fault == 'stationary':
        for p in source.joint_trajectory.points: p.positions = list(source.joint_trajectory.points[0].positions)
    assert timing.smooth_strict_collinear(source, row['limits']) is None


@pytest.mark.parametrize('fault', ['endpoint', 'off_line', 'missing_node', 'velocity', 'reverse_spline', 'held_axis'])
def test_independent_refinement_proof_rejects_geometric_or_interpolation_changes(fault):
    source, _, row = curve(0 if fault == 'missing_node' else 1)
    candidate = timing.smooth_strict_collinear(source, row['limits'])
    points = candidate.joint_trajectory.points
    if fault == 'endpoint': points[-1].positions[0] += .00001
    if fault == 'off_line': points[1].positions[0] += .00001
    if fault == 'missing_node': del points[5]
    if fault == 'velocity': points[1].velocities[0] += .001
    if fault == 'reverse_spline':
        points[1].velocities = (-np.array(points[1].velocities)).tolist()
    if fault == 'held_axis': points[1].accelerations[1] = .001
    with pytest.raises(ValueError): timing.validate_collinear_refinement(source, candidate)


def planner_for(source, limits, enabled):
    planner = MoveItPlanner.__new__(MoveItPlanner)
    planner.strict_execution_retime_enabled = True
    planner.strict_execution_velocity_scaling = .2
    planner.strict_execution_acceleration_scaling = .3
    planner.strict_execution_min_duration_sec = 3.
    planner.strict_execution_start_tolerance_rad = .08
    planner.strict_execution_max_joint_velocity_rad_s = max(limits.values())
    planner.strict_execution_joint_velocity_limits_rad_s = limits
    planner.strict_collinear_timing_enabled = enabled
    planner.manipulator = NS(retime_trajectory=lambda _state, plan, **_kwargs:deepcopy(plan),
        get_current_joint_values=lambda:list(source.joint_trajectory.points[0].positions))
    planner.robot = NS(get_current_state=lambda: NS())
    return planner


@pytest.mark.parametrize('enabled', [False, True])
def test_refinement_is_only_enabled_in_the_explicit_new_planner_profile(enabled):
    source, _, row = curve(1)
    planner = planner_for(source, row['limits'], enabled)
    result, message = planner._retime_strict_execution_plan(source)
    assert result is not None, message
    assert len(result.joint_trajectory.points) == (4 if enabled else 2)
    if enabled: timing.validate_collinear_refinement(source, result)
    else: assert MoveItPlanner._same_joint_path_geometry(source, result, tolerance=0.)


def test_profile_does_not_swallow_a_bad_refinement_as_success(monkeypatch):
    source, _, row = curve(1)
    planner = planner_for(source, row['limits'], True)
    invalid = timing.smooth_strict_collinear(source, row['limits'])
    invalid.joint_trajectory.points[1].positions[0] += .001
    monkeypatch.setattr(timing, 'smooth_strict_collinear', lambda *_args,**_kwargs:invalid)
    result, message = planner._retime_strict_execution_plan(source)
    assert result is None and 'strict collinear timing rejected' in message


def test_later_duration_stretch_preserves_the_proven_refinement():
    source, _, row = curve(1)
    planner = planner_for(source, row['limits'], True)
    planner.strict_execution_min_duration_sec = 100.
    result, message = planner._retime_strict_execution_plan(source)
    assert result is not None, message
    assert result.joint_trajectory.points[-1].time_from_start.to_sec() >= 100.-1e-9
    timing.validate_collinear_refinement(source, result)


@pytest.mark.parametrize('mode,stage,label,enabled', [
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'planning 6D pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'planning 6D near-field pregrasp',True),
    ('unknown',GraspStages.MOVE_PREGRASP,'moving 6D near-field pregrasp',True),
    ('carton',GraspStages.MOVE_PREGRASP,'moving 6D pregrasp',False),
    ('unknown',GraspStages.APPROACH_TARGET,'moving linear 6D grasp pose',False),
    ('unknown',GraspStages.MOVE_PREGRASP,'other request',False)])
@pytest.mark.parametrize('had_attribute', [False,True])
def test_gateway_profile_is_scoped_and_restored_even_after_failure(monkeypatch,mode,stage,label,enabled,had_attribute):
    node = MotionGateway.__new__(MotionGateway)
    node._pregrasp_task_stage, node._compensation_task_label = stage, label
    planner = NS()
    if had_attribute: planner.strict_collinear_timing_enabled = 'original'
    monkeypatch.setattr(motion_gateway_node.rospy, 'get_param', lambda _key,_default:dict(mode=mode))
    with pytest.raises(ValueError, match='submission rejected'):
        with node._planner_operation_lock():
            with node._strict_pregrasp_timing_scope(planner):
                assert planner.strict_collinear_timing_enabled is enabled
                raise ValueError('submission rejected')
    assert hasattr(planner,'strict_collinear_timing_enabled') is had_attribute
    if had_attribute: assert planner.strict_collinear_timing_enabled == 'original'
