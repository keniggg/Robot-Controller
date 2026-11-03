"""Frozen GEN54 paths through the production prediction and continuous gates.

Only the original contact curves were recorded. The nominal curve is the
explicit constant-bias reconstruction, not a new measured hardware path.
The whole-arm service is an isolated provider; no physical success is claimed.
"""
from copy import deepcopy
import hashlib
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

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _obb_overlap)
from alicia_flexible_grasp.robot import contact_sequence_precheck as precheck
from alicia_flexible_grasp.robot import contact_linear_compensation as execution
from alicia_flexible_grasp.robot import observation_path_guard as guard


FIXTURE = Path(__file__).parent/'fixtures/gen55_gen54_contact_curves.json'
FIXTURE_SHA = '526b60d8fe8e43d736f33f75d1c972c07245dca75ebcc9a603b2d99e5e68ddbf'


def recorded(index):
    raw = FIXTURE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    fixture = json.loads(raw)
    receipt = fixture['cases'][index]['receipt']
    model = fixture['model']
    fk = guard.SerialUrdfFk(model['xml'], receipt['checked_curve']['joint_names'])
    assert fk.model_sha256 == model['sha256']
    assert fk.finger_cad_model is not None
    sample = deepcopy(receipt['following_sample'])
    bias = np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    curves = []
    for offset in (-bias, np.zeros(6)):
        curve = RobotTrajectory()
        header = receipt['checked_curve']['header']
        curve.joint_trajectory.header.seq = header['seq']
        curve.joint_trajectory.header.frame_id = header['frame_id']
        ns = header['stamp_ns']
        curve.joint_trajectory.header.stamp = rospy.Time(ns//10**9, ns%10**9)
        curve.joint_trajectory.joint_names = receipt['checked_curve']['joint_names']
        for point in receipt['checked_curve']['points']:
            ns = point['time_from_start_ns']
            curve.joint_trajectory.points.append(JointTrajectoryPoint(
                positions=(np.array(point['positions'])+offset).tolist(),
                velocities=point['velocities'], accelerations=point['accelerations'],
                effort=point['effort'], time_from_start=rospy.Duration(ns//10**9, ns%10**9)))
        curves.append(curve)
    # Header bytes participate in ROS serialization. Restoring only the joint
    # values is insufficient to call the command an exact historical replay.
    assert guard.wire_digest(curves[1]) == receipt['checked_curve']['wire_sha256']
    scene = guard.FrozenObservationScene(**{
        key: np.array(value) if key in ('normal', 'center', 'rotation', 'size') else value
        for key, value in receipt['scene'].items()})
    geometry = GripperGeometry(**receipt['config']['evaluated_gripper'])
    values = receipt['plan_data']['poses_xyz_xyzw'][2]
    goal = Pose()
    for name, value in zip('xyz', values[:3]):
        setattr(goal.position, name, value)
    for name, value in zip('xyzw', values[3:]):
        setattr(goal.orientation, name, value)
    return receipt, fk, sample, curves, scene, geometry, goal


@pytest.mark.parametrize('index', range(8))
def test_exact_wire_command_uses_same_continuous_kernel_at_execution(index, tmp_path):
    d, fk, sample, (nominal, command), scene, g, goal = recorded(index)
    assert precheck.validate_observation_trajectory is guard.validate_observation_trajectory
    assert execution.validate_observation_trajectory is guard.validate_observation_trajectory
    header = command.joint_trajectory.header
    assert dict(seq=header.seq, frame_id=header.frame_id,
                stamp_ns=header.stamp.to_nsec()) == d['checked_curve']['header']
    rebuilt_command, _ = precheck.predict_contact_command_path(nominal, fk, sample, goal)
    roundtrip_delta = np.abs(
        np.array([p.positions for p in rebuilt_command.joint_trajectory.points])-
        np.array([p.positions for p in command.joint_trajectory.points]))
    # The reconstructed nominal plus the frozen bias can round by one ULP.
    # Its numerical equivalence must not be misreported as wire identity.
    assert float(roundtrip_delta.max()) <= 1.e-15
    rows = {}
    bias = np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
    # The execution kernel receives the recorded controller hold, not an
    # invented zero-velocity start. Only the nominal hold is reconstructed.
    for name, curve, offset in (('nominal_reconstructed', nominal, -bias),
                                ('recorded_command', command, np.zeros(6))):
        hold = JointTrajectoryPoint(
            positions=(np.array(d['controller_desired']['positions'])+offset).tolist(),
            velocities=d['controller_desired']['velocities'],
            accelerations=d['controller_desired']['accelerations'],
            effort=d['controller_desired']['effort'],
            time_from_start=rospy.Duration(
                d['controller_desired']['time_from_start_ns']//10**9,
                d['controller_desired']['time_from_start_ns']%10**9))
        before = guard.wire_digest(curve)
        try:
            report = execution.validate_observation_trajectory(curve, scene, fk, g,
                d['opening_m'], hold)
        except guard.ObservationPathError as exc:
            assert 'target OBB collision' in str(exc)
            rows[name] = dict(passed=False, reason=str(exc), wire_sha256=before)
        else:
            assert report['segments'] == 14
            assert report['target_cad_projection']['sufficient_separation_proofs'] > 0
            assert report['minimum_sampled_clearance_m'] >= .003
            rows[name] = dict(passed=True, geometry=report, wire_sha256=before)
        assert guard.wire_digest(curve) == before
        expected_indices = (1, 4, 7) if name == 'nominal_reconstructed' else (2, 4, 7)
        assert rows[name]['passed'] == (index in expected_indices)
    assert all(row['passed'] for row in rows.values()) == (index in (4, 7))
    (tmp_path/'exact_curve_replay.json').write_text(json.dumps(dict(
        index=index, plan_id=d['binding']['plan_id'],
        header=d['checked_curve']['header'],
        original_command_wire_sha256=d['checked_curve']['wire_sha256'],
        reconstruction_roundtrip_command_wire_sha256=guard.wire_digest(rebuilt_command),
        reconstruction_roundtrip_max_delta_rad=float(roundtrip_delta.max()),
        reconstruction_roundtrip_wire_exact=(guard.wire_digest(rebuilt_command) ==
                                            d['checked_curve']['wire_sha256']),
        curves=rows, original_result='failure', diagnostic_only=True,
        execution_authorized=False, physical_success=False), sort_keys=True, allow_nan=False))


def planner():
    # The receipt curve already has its original final timing. This provider
    # preserves it exactly, so production's unchanged-geometry gate still runs.
    return NS(_retime_cartesian_plan=mock.Mock(side_effect=deepcopy),
              _same_joint_path_geometry=lambda a, b: guard.wire_digest(a) == guard.wire_digest(b))


@pytest.mark.parametrize('index', range(8))
def test_exact_failed_endpoints_distinguish_rectangular_empty_space_from_CAD(index):
    d, fk, sample, (nominal, command), scene, g, goal = recorded(index)
    transform = fk(command.joint_trajectory.points[-1].positions)
    np.testing.assert_array_equal(transform, d['first_failed_sample']['T_base_tool0'])
    old_overlap = [body for body, center, size in _stage_boxes(transform, g, d['opening_m'], 'y', 'z')
        if _obb_overlap(center, transform[:3, :3], size, scene.center, scene.rotation, scene.size)]
    assert 'right_finger' in old_overlap
    if index in (2, 4, 7):
        report = precheck.contact_curve_terminal_collision(command, fk, scene, g, d['opening_m'])
        assert report['path_verified'] is False  # A terminal is never a path certificate.
        assert report['cad_target_projection_proofs']
        assert all(row['proof']['separated'] for row in report['cad_target_projection_proofs'])
    else:
        with pytest.raises(guard.ObservationPathError, match='target OBB collision'):
            precheck.contact_curve_terminal_collision(command, fk, scene, g, d['opening_m'])


@pytest.mark.parametrize('index', range(8))
def test_actual_rejected_curves_keep_full_path_checks_and_original_remaining_failures(index):
    d, fk, sample, (nominal, command), scene, g, goal = recorded(index)
    before = (guard.wire_digest(nominal), guard.wire_digest(command), deepcopy(sample))
    whole_arm = mock.Mock(return_value=23)
    p = planner()
    if index in (4, 7):
        report = precheck.validate_contact_prediction(nominal, fk, sample, goal, scene, g,
            d['opening_m'], p, RobotState(), whole_arm, lambda: 5.)
        assert report['future_following_verified'] is False
        assert report['execution_authorized'] is False
        for key in ('nominal_geometry', 'predicted_command_geometry'):
            assert report[key]['segments'] == 14
            assert report[key]['target_cad_projection']['sufficient_separation_proofs'] > 0
            assert report[key]['minimum_sampled_clearance_m'] >= .003
            assert report[key]['target_cad_projection']['motion_radius_m'] >= .1820099505521607-1e-12
        assert p._retime_cartesian_plan.call_count == 2
        whole_arm.assert_called_once()
    else:
        with pytest.raises(precheck.ContactPrecheckError, match='continuous geometry: target OBB collision'):
            precheck.validate_contact_prediction(nominal, fk, sample, goal, scene, g,
                d['opening_m'], p, RobotState(), whole_arm, lambda: 5.)
        whole_arm.assert_not_called()
    assert before == (guard.wire_digest(nominal), guard.wire_digest(command), sample)


def test_clear_terminal_cannot_hide_original_recorded_interior_collision():
    d, fk, sample, (nominal, command), scene, g, goal = recorded(2)
    for curve in (nominal, command):
        assert precheck.contact_curve_terminal_collision(curve, fk, scene, g, d['opening_m'])['checked']
    whole_arm = mock.Mock()
    with pytest.raises(precheck.ContactPrecheckError, match='nominal continuous geometry: target OBB collision'):
        precheck.validate_contact_prediction(nominal, fk, sample, goal, scene, g,
            d['opening_m'], planner(), RobotState(), whole_arm, lambda: 5.)
    whole_arm.assert_not_called()


@pytest.mark.parametrize('fault', ('missing_model', 'noncanonical_finger'))
def test_unsupported_shape_keeps_original_terminal_rejection(fault):
    d, fk, _, (_, command), scene, g, _ = recorded(4)
    if fault == 'missing_model':
        fk.finger_cad_model = None
    else:
        g = GripperGeometry(g.max_inner_gap_m, g.jaw_clearance_each_side_m,
            g.finger_size_xyz_m+1e-6, g.palm_size_xyz_m, g.support_clearance_m)
    with pytest.raises(guard.ObservationPathError, match='target OBB collision.*right_finger'):
        precheck.contact_curve_terminal_collision(command, fk, scene, g, d['opening_m'])


@pytest.mark.parametrize('value', (-.001, float('nan'), float('inf'), True))
def test_invalid_displacement_cannot_escape_through_broad_phase(value):
    d, fk, _, (_, curve), scene, g, _ = recorded(4)
    transform = fk(curve.joint_trajectory.points[-1].positions)
    body, center, size = _stage_boxes(transform, g, d['opening_m'], 'y', 'z')[0]
    with pytest.raises(guard.ObservationPathError, match='invalid target displacement'):
        guard.gripper_target_overlap(fk, g, d['opening_m'], body, transform, center, size,
            scene, displacement=value)


@pytest.mark.parametrize('fault', ('deadline', 'control_revoked', 'whole_arm_collision'))
def test_CAD_separation_cannot_authorize_motion_or_hide_independent_rejection(fault):
    d, fk, sample, (nominal, _), scene, g, goal = recorded(4)
    p = planner()
    error = precheck.ContactPrecheckError(fault, 'MOVEIT_TIMEOUT' if fault == 'deadline' else 'MOVEIT_CHECK_ERROR')
    def fail(*args, **kwargs):
        raise error
    remaining = fail if fault in ('deadline', 'control_revoked') else lambda: 5.
    whole_arm = mock.Mock(side_effect=fail)
    with pytest.raises(precheck.ContactPrecheckError) as caught:
        precheck.validate_contact_prediction(nominal, fk, sample, goal, scene, g,
            d['opening_m'], p, RobotState(), whole_arm, remaining)
    assert caught.value is error
    if fault != 'whole_arm_collision':
        p._retime_cartesian_plan.assert_not_called()
        whole_arm.assert_not_called()
    else:
        whole_arm.assert_called_once()


def test_fixture_retains_failure_classification_and_all_receipts():
    data = json.loads(FIXTURE.read_bytes())
    assert data['original_result'] == 'failure'
    assert data['execution_authorized'] is False and data['diagnostic_only'] is True
    assert len(data['cases']) == 8
    for row in data['cases']:
        receipt = dict(row['receipt'], model=data['model'])
        # The source hash remains available for exact archival lookup. These
        # reconstructed JSON objects do not pretend to preserve text ordering.
        assert len(row['reference']['object_sha256']) == 64
        assert receipt['failure']['message'].startswith('predicted_command continuous geometry: target OBB collision')
