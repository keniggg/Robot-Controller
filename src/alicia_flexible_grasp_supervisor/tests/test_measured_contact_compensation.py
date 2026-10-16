"""Frozen real failures, followed only by explicitly hypothetical responses."""
from dataclasses import asdict
import json
import math
from pathlib import Path

import numpy as np
import pytest

from test_endpoint_correction import sample
from alicia_flexible_grasp.grasp.gripper_geometry import (
    ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M, GripperGeometry,
)
from alicia_flexible_grasp.grasp.measured_contact import evaluate_measured_contact
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def recorded(name):
    data = json.loads((Path(__file__).parent/'fixtures/measured_contact_20260926.json').read_text())
    record = data['records'][name]
    fk = SerialUrdfFk(data['robot_description'], ARM_NAMES)
    assert fk.model_sha256 == data['model_sha256']
    gripper = GripperGeometry(.05, .002, ANALYTICAL_FINGER_SIZE_XYZ_M,
                              ANALYTICAL_PALM_SIZE_XYZ_M, .003)
    def acceptance(transform):
        return asdict(evaluate_measured_contact(gripper=gripper,
            actual_T_base_tool0=transform, **record['frozen_contact']))
    sdk = (np.asarray(record['sdk_counts'])-2048)*Q
    actual = (np.asarray(record['measured_counts'])-2048)*Q
    goal = record['planned_position_m']+record['planned_quaternion_xyzw']
    return record, fk, sdk, actual, goal, acceptance


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_old_pose_only_gate_still_passes_without_contact_callback(name):
    _record, fk, sdk, actual, goal, _accept = recorded(name)
    correction = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
                                     position_tolerance_m=.010)
    code, step, report = correction.evaluate(sample(fk, sdk, actual))
    assert code == 'PREGRASP_MEASURED_CONVERGED' and step is None
    assert correction.steps == 0
    assert 'measured_contact' not in report
    assert not report['real_grasp_success']


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_contact_convergence_requires_new_hypothetical_responses_inside_original_budgets(name):
    record, fk, sdk, actual, goal, accept = recorded(name)
    correction = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
        position_tolerance_m=.010, endpoint_acceptance=accept)
    frozen_goal = correction.goal.copy()
    original_sdk = sdk.copy()
    merits, errors, reports = [], [], []
    for i in range(correction.MAX_STEPS+1):
        code, step, report = correction.evaluate(sample(fk, sdk, actual, 10.+4*i))
        merits.append(report['contact_merit'])
        errors.append(report['position_error_m'])
        reports.append(report)
        assert report['real_grasp_success'] is False
        assert report['position_error_m'] <= .010
        assert report['orientation_error_rad'] <= math.radians(5)
        assert report['measured_contact']['minimum_support_clearance_m'] >= .003-1e-9
        if step is None:
            break
        assert code == 'PREGRASP_STEP_PROPOSED'
        assert report['measured_contact']['actual_overlap_m'] < .002
        assert report['predicted_contact']['minimum_support_clearance_m'] >= .003-1e-9
        assert report['predicted_contact_merit'] < report['contact_merit']-1e-4
        delta = np.asarray(step.target_counts)-step.baseline_counts
        assert np.max(abs(delta)) <= 4
        assert np.max(abs(np.asarray(step.target_counts)-correction.initial_counts)) <= 32
        assert delta[1] == delta[4] == 0
        assert not report['joint2_response_probe_used']
        correction.committed(step, 12.+4*i)
        sdk = np.asarray(step.positions)
        actual += delta*Q  # Hypothesis only: none of these encoder responses occurred on hardware.
        np.testing.assert_array_equal(sdk[[1, 4]], original_sdk[[1, 4]])
    assert code == 'PREGRASP_MEASURED_CONVERGED'
    assert 0 < correction.steps <= 12
    assert report['measured_contact']['actual_overlap_m'] >= .002-1e-9
    assert all(first > second for first, second in zip(merits, merits[1:]))
    # A small, still-admissible position increase can restore the actual patch.
    assert any(second > first for first, second in zip(errors, errors[1:]))
    np.testing.assert_array_equal(correction.goal, frozen_goal)
    assert tuple(goal) == correction.goal_values
    assert record['operator_outcome'] == 'empty_close_above_target'


@pytest.mark.parametrize('name', ['gen11', 'gen18'])
def test_no_encoder_response_still_fails_without_another_command(name):
    _record, fk, sdk, actual, goal, accept = recorded(name)
    correction = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
        position_tolerance_m=.010, endpoint_acceptance=accept)
    _code, step, report = correction.evaluate(sample(fk, sdk, actual))
    assert report['steps_completed'] == 0 and step is not None
    correction.committed(step, 12.)
    for _ in range(2):
        with pytest.raises(ValueError, match='PREGRASP_NO_BOUNDED_DIRECTIONAL_RESPONSE'):
            correction.evaluate(sample(fk, step.positions, actual, 14.))
    assert correction.steps == 1
    assert not correction.last_evidence['real_grasp_success']


@pytest.mark.parametrize('field,value', [
    ('minimum_support_clearance_m', .0029),
    ('minimum_contact_overlap_m', 0.),
    ('actual_overlap_m', float('nan')),
    ('code', 'FROZEN_CONTACT_EVIDENCE_INVALID'),
])
def test_contact_merit_cannot_trade_away_support_or_invalid_evidence(field, value):
    _record, fk, sdk, actual, goal, accept = recorded('gen18')
    def invalid(transform):
        result = accept(transform)
        result[field] = value
        return result
    correction = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
        position_tolerance_m=.010, endpoint_acceptance=invalid)
    with pytest.raises(ValueError, match='PREGRASP_ENDPOINT_ACCEPTANCE_INVALID_OR_UNSAFE'):
        correction.evaluate(sample(fk, sdk, actual))
    assert correction.steps == 0 and correction.proposed is None


def test_contact_search_cannot_rewrite_fixed_goal_or_relax_timeout():
    _record, fk, sdk, actual, goal, accept = recorded('gen18')
    correction = PregraspCompensation(fk, sample(fk, sdk, actual), goal,
        position_tolerance_m=.010, endpoint_acceptance=accept)
    with pytest.raises(ValueError):
        correction.goal[2, 3] -= .010
    with pytest.raises(ValueError, match='PREGRASP_STALE_OR_EXPIRED'):
        correction.evaluate(sample(fk, sdk, actual, 71.))
    assert correction.steps == 0
