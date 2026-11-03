"""Frozen grasp-stroke geometry; no claim of whole-arm planning or execution.

Re-solve each saved Cartesian waypoint after the generic lateral proposal and
validate both complete continuous curves with the production geometry checker.
The offline spline has stationary waypoints; it is not a MoveIt timing replay.
"""
from copy import deepcopy
import gzip
import json
from pathlib import Path
import time

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.grasp.contact_lateral_placement import contact_lateral_placement
from alicia_flexible_grasp.robot.contact_sequence_precheck import contact_precheck_gripper
from alicia_flexible_grasp.robot.observation_path_guard import (
    FrozenObservationScene, SerialUrdfFk, validate_observation_trajectory, ObservationPathError)
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
from test_contact_lateral_placement import case


@pytest.fixture(scope='module', params=range(4))
def frozen_curves(request):
    index = request.param
    row, args = case(index)
    data = json.loads(gzip.decompress((Path(__file__).parent/
        'fixtures/compensated_contact_curves_20261023.json.gz').read_bytes()))['rows'][index]
    fk = SerialUrdfFk(args['model_xml'], ARM_NAMES)
    scene = FrozenObservationScene(**{k: np.array(v) if k in ('center','size','normal','rotation') else v
                                     for k, v in row['scene'].items()})
    sample = args['failure']['source_sample']
    bias = np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    proposed, proposal = contact_lateral_placement(**args)
    delta = proposed[:3, 3]-pose_matrix(args['goal_pose'])[:3, 3]
    lower = np.maximum(fk.limits[:, 0], fk.limits[:, 0]-bias)+1e-12
    upper = np.minimum(fk.limits[:, 1], fk.limits[:, 1]-bias)-1e-12
    nominal = RobotTrajectory()
    nominal.joint_trajectory.joint_names = list(ARM_NAMES)
    make = getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm
    for i, point in enumerate(data['curve']['points']):
        original = np.array(point['positions'])-bias
        target = fk(original)
        target[:3, 3] += delta
        def residual(q):
            actual = fk(q)
            return np.r_[1000*(actual[:3, 3]-target[:3, 3]),
                         100*make(target[:3, :3].T@actual[:3, :3]).as_rotvec()]
        result = least_squares(residual, original, bounds=(lower, upper), max_nfev=80,
                               ftol=1e-12, xtol=1e-12, gtol=1e-12)
        assert np.linalg.norm(residual(result.x)) < 1e-6
        nominal.joint_trajectory.points.append(JointTrajectoryPoint(positions=result.x.tolist(),
            velocities=[0.]*6, accelerations=[0.]*6, time_from_start=rospy.Duration.from_sec(i)))
    command = deepcopy(nominal)
    for point in command.joint_trajectory.points:
        point.positions = (np.array(point.positions)+bias).tolist()
    original = deepcopy(command)
    for point, saved in zip(original.joint_trajectory.points, data['curve']['points']):
        point.positions = list(saved['positions'])
    return row, scene, fk, nominal, command, original, proposal


def test_frozen_lateral_grasp_stroke_continuous_geometry(frozen_curves, tmp_path):
    row, scene, fk, nominal, command, _, proposal = frozen_curves
    reports = {}
    for label, trajectory in [('nominal', nominal), ('predicted_command', command)]:
        hold = deepcopy(trajectory.joint_trajectory.points[0])
        started = time.monotonic()
        try:
            # Geometry must not depend on host load. In the old index-1
            # expectation, False meant a wall-clock timeout, not a collision.
            # Keep the finite subdivision/check limits; test timeouts below.
            reports[label] = dict(passed=True, evidence=validate_observation_trajectory(
                trajectory, scene, fk, contact_precheck_gripper(row['gripper_geometry']),
                row['opening_m'], hold, max_seconds=5., clock=lambda: 0.))
        except ObservationPathError as exc:
            reports[label] = dict(passed=False, reason=str(exc))
        reports[label]['actual_wall_sec'] = time.monotonic()-started
    (tmp_path/'full_grasp_stroke.json').write_text(json.dumps(dict(proposal=proposal, curves=reports,
        whole_arm_moveit_checked=False, original_timing_reproduced=False,
        wall_clock_limit_exercised=False, execution_authorized=False), indent=2))
    assert all(r['passed'] for r in reports.values()), reports
    assert all(r['evidence']['certified_minimum_clearance_m'] >=
               contact_precheck_gripper(row['gripper_geometry']).support_clearance_m
               for r in reports.values())


@pytest.mark.parametrize('label', ['nominal', 'predicted_command'])
def test_frozen_grasp_stroke_elapsed_budget_still_rejects(frozen_curves, label):
    row, scene, fk, nominal, command, _, _ = frozen_curves
    trajectory = nominal if label == 'nominal' else command
    ticks = iter([0., 6.])
    with pytest.raises(ObservationPathError, match='computation budget exhausted'):
        validate_observation_trajectory(
            trajectory, scene, fk, contact_precheck_gripper(row['gripper_geometry']),
            row['opening_m'], deepcopy(trajectory.joint_trajectory.points[0]),
            max_seconds=5., clock=lambda: next(ticks, 6.))


def test_original_unshifted_contact_curve_still_collides(frozen_curves):
    row, scene, fk, _, _, original, _ = frozen_curves
    with pytest.raises(ObservationPathError, match='target OBB collision'):
        validate_observation_trajectory(
            original, scene, fk, contact_precheck_gripper(row['gripper_geometry']),
            row['opening_m'], deepcopy(original.joint_trajectory.points[0]),
            max_seconds=5., clock=lambda: 0.)
