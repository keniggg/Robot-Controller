"""Recorded endpoint policy plus constructed continuous gripper sequence.

The source saved the failed grasp curve, but only digests and verdicts for
pregrasp/approach. Re-solve the translated pregrasp on the recorded IK branch,
run the production held-axis endpoint policy, and start contact at its actual
predicted approach endpoint. Every constructed curve uses stationary-waypoint
splines. This is neither a MoveIt path replay nor a timing/execution approval.
No planner or collision callback is replaced by a successful test double.
"""
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.contact_lateral_placement import contact_lateral_placement
from alicia_flexible_grasp.grasp.endpoint_tolerance import measured_position_tolerance
from alicia_flexible_grasp.robot.contact_sequence_precheck import (
    contact_precheck_gripper, predict_contact_command_path,
    validate_held_guide_path_bounds)
from alicia_flexible_grasp.robot.held_pregrasp_selection import select_held_pregrasp_endpoint
from alicia_flexible_grasp.robot.finger_cad_separation import load_finger_cad_model
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, ObservationPathError, SerialUrdfFk,
    validate_observation_trajectory, wire_digest)
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from test_contact_lateral_placement import case


def _rotation(matrix):
    return (getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm)(matrix)


def _matrix(rotation):
    return (getattr(rotation, 'as_matrix', None) or rotation.as_dcm)()


def _pose(values):
    result = Pose()
    result.position.x, result.position.y, result.position.z = values[:3]
    (result.orientation.x, result.orientation.y, result.orientation.z,
     result.orientation.w) = values[3:]
    return result


def _path(points):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = list(ARM_NAMES)
    for i, q in enumerate(points):
        result.joint_trajectory.points.append(JointTrajectoryPoint(
            positions=np.asarray(q).tolist(), velocities=[0.]*6, accelerations=[0.]*6,
            time_from_start=rospy.Duration.from_sec(i)))
    return result


def _shifted(curve, delta):
    result = deepcopy(curve)
    for point in result.joint_trajectory.points:
        point.positions = (np.asarray(point.positions)+delta).tolist()
    return result


def _solve_same_branch(fk, target, seed, bias, *, check_compensated_limits):
    lower, upper = fk.limits[:, 0]+1e-12, fk.limits[:, 1]-1e-12
    if check_compensated_limits:
        lower = np.maximum(lower, fk.limits[:, 0]-bias+1e-12)
        upper = np.minimum(upper, fk.limits[:, 1]-bias-1e-12)

    def residual(q):
        actual = fk(q)
        return np.r_[1000*(actual[:3, 3]-target[:3, 3]),
                     100*_rotation(target[:3, :3].T@actual[:3, :3]).as_rotvec()]

    solved = least_squares(residual, np.minimum(upper, np.maximum(lower, seed)),
        bounds=(lower, upper), max_nfev=80, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    assert np.linalg.norm(residual(solved.x)) < 1e-6
    return solved.x


def _source(index, row):
    fixtures = Path(__file__).with_name('fixtures')
    data = json.loads(gzip.decompress(
        (fixtures/'contact_lateral_sequence_20261023.json.gz').read_bytes()))
    assert hashlib.sha256((fixtures/data['terminal_fixture_name']).read_bytes()).hexdigest() == data['terminal_fixture_sha256']
    source = data['rows'][index]
    assert source['source_index'] == row['source_index']
    assert source['plan_id'] == row['failure']['plan_id']
    assert source['source_stamp_ns'] == row['scene']['source_ns']
    sample = json.dumps(row['failure']['source_sample'], sort_keys=True, separators=(',', ':')).encode()
    assert hashlib.sha256(sample).hexdigest() == source['source_sample_sha256']
    return source


def _sequence(row, args, source, *, lateral, geometry_clock=None):
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    sample = args['failure']['source_sample']
    bias = np.asarray(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    goals = deepcopy(row['poses'])
    seed = np.asarray(source['original_nominal_pregrasp_command_rad'])
    proposal = None
    if lateral:
        target, proposal = contact_lateral_placement(**args)
        assert target is not None
        delta = target[:3, 3]-pose_matrix(goals[2])[:3, 3]
        for goal in goals:
            goal[:3] = (np.asarray(goal[:3])+delta).tolist()
        seed = _solve_same_branch(fk, pose_matrix(goals[0]), seed, bias,
                                 check_compensated_limits=False)
    deadline = time.monotonic()+15.

    def remaining():
        assert time.monotonic() < deadline, 'original candidate budget consumed'

    config = source['grasp_config']
    command, selection = select_held_pregrasp_endpoint(fk, sample, goals[0], goals[1], seed,
        position_tolerance_m=measured_position_tolerance(config, 'unknown_tabletop'),
        orientation_tolerance_rad=math.radians(config['measured_endpoint_orientation_tolerance_deg']),
        deadline_check=remaining)
    assert selection['success'] and not selection['execution_authorized']
    pre, approach = selection['predicted_pregrasp'], selection['predicted_approach']
    assert approach['held_joints'] == ['Joint2', 'Joint5']
    assert approach['maximum_total_counts'] == 32
    curves = {}
    curves['observation_to_pregrasp_command'] = _path([sample['sdk_positions_rad'], command])
    curves['observation_to_pregrasp_predicted_actual'] = _shifted(
        curves['observation_to_pregrasp_command'], -bias)
    bounds = {}
    for name, begin, end in (
        ('pregrasp_correction', command, pre['predicted_command_positions_rad']),
        ('approach', pre['predicted_command_positions_rad'], approach['predicted_command_positions_rad'])):
        curves[name+'_command'] = _path([begin, end])
        bounds[name] = validate_held_guide_path_bounds(curves[name+'_command'], begin, end)
        curves[name+'_predicted_actual'] = _shifted(curves[name+'_command'], -bias)
    # Recompute the grasp stroke from this selection's actual predicted start;
    # translating the old stroke would hide a new approach/contact mismatch.
    q = np.asarray(approach['predicted_positions_rad'])
    start, goal = fk(q), pose_matrix(goals[2])
    rotation = Slerp([0., 1.], _rotation(np.array([start[:3, :3], goal[:3, :3]])))
    points = [q.copy()]
    for fraction in np.linspace(0., 1., source['grasp_waypoint_count'])[1:]:
        target = np.eye(4)
        target[:3, :3] = _matrix(rotation([fraction]))[0]
        target[:3, 3] = (1-fraction)*start[:3, 3]+fraction*goal[:3, 3]
        q = _solve_same_branch(fk, target, q, bias, check_compensated_limits=True)
        points.append(q.copy())
    curves['grasp_nominal'] = _path(points)
    curves['grasp_predicted_command'], prediction = predict_contact_command_path(
        curves['grasp_nominal'], fk, sample, _pose(goals[2]))
    np.testing.assert_array_equal(points[0], approach['predicted_positions_rad'])
    np.testing.assert_allclose(curves['grasp_predicted_command'].joint_trajectory.points[0].positions,
        approach['predicted_command_positions_rad'], rtol=0., atol=1e-12)
    scene = FrozenObservationScene(**{
        k: np.asarray(v) if k in ('center', 'size', 'normal', 'rotation') else v
        for k, v in row['scene'].items()})
    gripper = contact_precheck_gripper(row['gripper_geometry'])
    reports = {}
    for name, curve in curves.items():
        started = time.monotonic()
        try:
            proof = validate_observation_trajectory(curve, scene, fk, gripper, row['opening_m'],
                # Separate geometric truth from shared-worker wall time. Keep
                # the production count/depth limits and the actual kernel;
                # the independent elapsed-budget case below exercises time.
                deepcopy(curve.joint_trajectory.points[0]), max_seconds=5., max_checks=8192,
                clock=geometry_clock if geometry_clock is not None else lambda: 0.)
            reports[name] = dict(passed=True, evidence=proof)
        except ObservationPathError as exc:
            reports[name] = dict(passed=False, reason=str(exc))
        reports[name]['actual_wall_sec'] = time.monotonic()-started
    return dict(proposal=proposal, selection=selection, held_path_bounds=bounds,
        contact_prediction=prediction, geometry=reports,
        curve_sha256={name: wire_digest(curve) for name, curve in curves.items()},
        curves_are_constructed_diagnostics=True, original_complete_paths_replayed=False,
        geometry_clock_mode='fixed' if geometry_clock is None else 'injected',
        wall_clock_limit_exercised=False,
        original_timing_reproduced=False, whole_arm_moveit_checked=False,
        future_following_verified=False, execution_authorized=False)


@pytest.mark.parametrize('index', range(4))
def test_lateral_sequence_preserves_handoff_and_original_command_rejections(index, tmp_path):
    row, args = case(index)
    before = deepcopy(args)
    source = _source(index, row)
    original = _sequence(row, args, source, lateral=False)
    lateral = _sequence(row, args, source, lateral=True)
    (tmp_path/'constructed_sequence_report.json').write_text(json.dumps(
        dict(source_index=row['source_index'], source=source, original=original, lateral=lateral), indent=2))
    assert args == before
    assert len(original['geometry']) == len(lateral['geometry']) == 8
    for evidence in (original, lateral):
        assert all(result['passed'] for name, result in evidence['geometry'].items()
                   if not name.startswith('grasp_'))
    assert original['geometry']['grasp_nominal']['passed']
    rejection = original['geometry']['grasp_predicted_command']
    assert not rejection['passed'] and 'right_finger' in rejection['reason']
    # The former index-1 rejection was an inconclusive 15-axis projection.
    # Full original-vertex support proves a 1.0023 mm gap at its first refusal.
    assert all(result['passed'] for result in lateral['geometry'].values())
    assert all(0 < result['evidence']['fk_checks'] <= 8192
               and 0 < result['evidence']['interval_checks'] <= 8192
               for result in lateral['geometry'].values())
    assert lateral['geometry']['grasp_predicted_command']['passed']


def test_constructed_sequence_elapsed_budget_still_rejects():
    row, args = case(1)
    source = _source(1, row)
    ticks = iter(range(0, 1000, 6))
    result = _sequence(row, args, source, lateral=True, geometry_clock=lambda: next(ticks))
    assert len(result['geometry']) == 8
    assert all(not proof['passed'] and 'computation budget exhausted' in proof['reason']
               for proof in result['geometry'].values())
    assert not result['execution_authorized']


def test_narrow_opening_with_verified_cad_union_intersection_still_rejects():
    row, args = case(1)
    source = _source(1, row)
    narrow = deepcopy(row)
    narrow['opening_m'] = .025
    target, proposal = contact_lateral_placement(**args)
    assert target is not None and not proposal['motion_authorized']
    model = load_finger_cad_model(args['model_xml'])
    assert model is not None
    scene = row['scene']
    # An unpadded original mesh vertex strictly inside the target OBB is an
    # independent intersection witness for the validated active/reference
    # union. It does not establish active-hardware intersection on its own.
    vertices = np.concatenate([
        part.vertices_at_zero+narrow['opening_m']*part.velocity_per_opening
        for part in model._body_components('left_finger')])
    world = vertices @ target[:3, :3].T+target[:3, 3]
    target_local = (world-np.asarray(scene['center'])) @ np.asarray(scene['rotation'])
    interior = np.asarray(scene['size'])/2.-np.abs(target_local)
    assert np.any(np.all(interior > 1e-6, axis=1))
    result = _sequence(narrow, args, source, lateral=True)
    rejection = result['geometry']['grasp_nominal']
    assert not rejection['passed'] and 'left_finger' in rejection['reason']
    assert not result['execution_authorized']
