"""Independent offline contract checks; all encoder responses are synthetic.

The historical gen18 fixture supplies signed geometry and the robot model.
The passing nominal counts below are a hypothetical FK pose, never a physical
grasp claim. No live ROS calls or external experiment files are required.
"""
from copy import deepcopy
import math

import numpy as np
import pytest

from test_contact_execution import FIXTURE, plan_from_record
from test_endpoint_correction import sample
from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
from alicia_flexible_grasp.grasp.contact_height_margin import (
    contact_height_evidence_limit, plan_contact_height_margin,
)
from alicia_flexible_grasp.grasp.rich_plan_integrity import pose_values
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.stationary_following import (
    ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q,
)


POLICY = dict(contact_height_registration_margin_enabled=True,
              handeye_translation_max_error_m=.00460616416406)


def nominal_fixture(limit=POLICY['handeye_translation_max_error_m']):
    plan, _ = plan_from_record('gen18')
    fk = SerialUrdfFk(FIXTURE['robot_description'], ARM_NAMES)
    actual = (np.asarray([816, 1583, 2881, 2020, 1237, 2047])-2048)*Q
    accept = plan_contact_evaluator(plan, {}, .04975,
                                   translation_evidence_limit_m=limit)
    initial = sample(fk, actual, actual)
    correction = PregraspCompensation(fk, initial, pose_values(plan.poses[2]),
        position_tolerance_m=.010, endpoint_acceptance=accept)
    return plan, fk, actual, accept, initial, correction


def test_nominal_contact_cannot_converge_without_required_height_margin():
    plan, fk, actual, accept, initial, correction = nominal_fixture()
    report = accept(fk(actual))
    assert report['ok'] is True
    assert report['code'] == 'MEASURED_CONTACT_GEOMETRY_OK'
    assert report['height_registration_margin']['ok'] is False
    assert correction.residual(actual)[0] < .010
    # No proposed trajectory is needed to test the acceptance boundary. Even
    # after an episode is spent, a bare-contact pass must not return success.
    correction.steps = correction.MAX_STEPS
    with pytest.raises(ValueError, match='PREGRASP_STEP_BUDGET'):
        correction.evaluate(initial)
    assert correction.proposed is None
    assert correction.last_evidence['measured_contact']['height_registration_margin']['ok'] is False
    assert correction.last_evidence['real_grasp_success'] is False


def test_height_deficit_changes_merit_without_changing_bare_contact_requirement():
    _, _, actual, _, _, enabled = nominal_fixture()
    _, _, _, _, _, disabled = nominal_fixture(None)
    measured = enabled._contact_report(actual)
    height = measured['height_registration_margin']
    assert height['maximum_symmetric_normal_translation_m'] < height['translation_evidence_limit_m']
    assert enabled._cost(actual) > 2.
    assert disabled._cost(actual) <= 1.
    assert measured['minimum_contact_overlap_m'] == .002
    assert measured['minimum_support_clearance_m'] >= .003


def test_better_height_margin_is_not_defeated_by_an_already_admissible_pose_norm():
    _, _, actual, _, _, correction = nominal_fixture()
    predicted = actual.copy()
    predicted[2] += 2*Q  # Hypothetical response; no hardware command.
    before, after = (correction._contact_report(q) for q in (actual, predicted))
    assert before['ok'] and after['ok']
    assert not before['height_registration_margin']['ok']
    assert not after['height_registration_margin']['ok']
    assert after['height_registration_margin']['maximum_symmetric_normal_translation_m'] > before['height_registration_margin']['maximum_symmetric_normal_translation_m']
    p0, _ = correction.residual(actual)
    p1, angle = correction.residual(predicted)
    assert p0 < p1 < correction.tolerances[0] and angle < correction.tolerances[1]
    assert correction._cost(predicted) < correction._cost(actual)
    assert correction._improves(predicted, p0, correction._cost(actual))


def test_satisfying_height_contract_has_priority_within_unchanged_pose_limits():
    # This synthetic requirement straddles two fixture FK samples. It does not
    # replace the empirical limit in any production or recorded experiment.
    _, _, actual, _, _, correction = nominal_fixture(.00035)
    predicted = actual.copy()
    predicted[2] += 2*Q
    assert correction._contact_report(actual)['height_registration_margin']['ok'] is False
    assert correction._contact_report(predicted)['height_registration_margin']['ok'] is True
    p0, _ = correction.residual(actual)
    p1, angle = correction.residual(predicted)
    assert p0 < p1 < correction.tolerances[0] and angle < correction.tolerances[1]
    assert 0. <= correction._cost(predicted) <= 1. < correction._cost(actual)
    assert correction._improves(predicted, p0, correction._cost(actual))


def test_initial_margin_failure_proposes_only_an_original_budget_correction():
    _, _, _, _, initial, correction = nominal_fixture()
    frozen_goal = correction.goal.copy()
    code, step, report = correction.evaluate(initial)
    assert code == 'PREGRASP_STEP_PROPOSED' and step is not None
    assert report['steps_completed'] == 0
    assert report['measured_contact']['ok'] is True
    assert report['measured_contact']['height_registration_margin']['ok'] is False
    assert report['predicted_contact_merit'] < report['contact_merit']
    delta = np.asarray(step.target_counts)-step.baseline_counts
    assert max(abs(delta)) <= correction.MAX_STEP_COUNTS == 4
    assert max(abs(np.asarray(step.target_counts)-correction.initial_counts)) <= correction.MAX_TOTAL_COUNTS == 32
    assert delta[1] == delta[4] == 0
    np.testing.assert_array_equal(correction.goal, frozen_goal)
    assert correction.MAX_SECONDS == 60. and correction.MAX_STEPS == 12
    assert report['real_grasp_success'] is False
    # A proposal is never executed by this pure numerical contract test.
    assert correction.steps == 0 and correction.pending is None


def test_disabled_policy_preserves_previous_zero_step_contract():
    assert contact_height_evidence_limit({}) is None
    assert contact_height_evidence_limit(dict(POLICY,
        contact_height_registration_margin_enabled=False)) is None
    _, fk, actual, accept, initial, correction = nominal_fixture(None)
    assert 'height_registration_margin' not in accept(fk(actual))
    code, step, report = correction.evaluate(initial)
    assert code == 'PREGRASP_MEASURED_CONVERGED' and step is None
    assert report['steps_completed'] == 0 and report['measured_contact']['ok'] is True


def test_factory_freezes_plan_and_gripper_configuration_and_reports_are_independent():
    plan, fk, actual, _, _, _ = nominal_fixture()
    config = {'finger_box_xyz_m': [.0434, .0286, .06], 'support_clearance_m': .003}
    limit = contact_height_evidence_limit(POLICY)
    evaluate = plan_contact_evaluator(plan, config, .04975,
                                      translation_evidence_limit_m=limit)
    before = evaluate(fk(actual))
    plan.poses[2].position.z += .04
    plan.object_geometry.support_normal_base.z = -.5
    plan.object_geometry.support_offset_m += .10
    plan.contact_geometry_evidence = ''
    config['finger_box_xyz_m'][0] = .4
    config['support_clearance_m'] = 0.
    changed_report = evaluate(fk(actual))
    changed_report['height_registration_margin']['allowed_normal_translation_intervals_m'][0][0] = -10.
    assert evaluate(fk(actual)) == before
    assert before['height_registration_margin']['translation_evidence_limit_m'] == limit


def test_compensation_factory_matches_independent_existing_close_adapter():
    plan, fk, actual, accept, _, _ = nominal_fixture()
    expected = plan_contact_height_margin(plan, {}, .04975,
        POLICY['handeye_translation_max_error_m'], actual_T_base_tool0=fk(actual))
    assert accept(fk(actual))['height_registration_margin'] == expected


@pytest.mark.parametrize('value', [None, True, False, 0., -1., float('nan'), float('inf'), '0.004'])
def test_enabled_policy_never_defaults_or_relaxes_invalid_error_evidence(value):
    with pytest.raises(ValueError, match='CONTACT_HEIGHT_MARGIN_CONFIG_INVALID'):
        contact_height_evidence_limit(dict(POLICY, handeye_translation_max_error_m=value))


@pytest.mark.parametrize('value', [None, 0, 1, 'true', 'false'])
def test_nonboolean_policy_is_not_implicit_disable(value):
    with pytest.raises(ValueError, match='CONTACT_HEIGHT_MARGIN_CONFIG_INVALID'):
        contact_height_evidence_limit(dict(POLICY, contact_height_registration_margin_enabled=value))


@pytest.mark.parametrize('value', [True, False, 0., -1., float('nan'), float('inf'), '0.004'])
def test_factory_rejects_invalid_enabled_limit(value):
    plan, _ = plan_from_record('gen18')
    with pytest.raises(ValueError, match='CONTACT_HEIGHT_MARGIN_CONFIG_INVALID'):
        plan_contact_evaluator(plan, {}, .04975, translation_evidence_limit_m=value)


@pytest.mark.parametrize('corruption', [
    'null', 'unevaluated', 'nan_margin', 'boolean_limit', 'negative_margin',
    'false_success', 'false_code', 'false_interval_coverage',
])
def test_corrupt_margin_evidence_cannot_be_treated_as_disabled_or_converged(corruption):
    _, fk, actual, accept, initial, correction = nominal_fixture()
    valid = accept(fk(actual))
    def corrupt(_):
        result = deepcopy(valid)
        height = result['height_registration_margin']
        if corruption == 'null': result['height_registration_margin'] = None
        elif corruption == 'unevaluated': height['evaluated'] = False
        elif corruption == 'nan_margin': height['maximum_symmetric_normal_translation_m'] = math.nan
        elif corruption == 'boolean_limit': height['translation_evidence_limit_m'] = True
        elif corruption == 'negative_margin': height['maximum_symmetric_normal_translation_m'] = -.001
        elif corruption == 'false_success': height['ok'] = True
        elif corruption == 'false_code': height['code'] = 'CONTACT_HEIGHT_NORMAL_MARGIN_OK'
        elif corruption == 'false_interval_coverage': height['covers_entire_normal_translation_interval'] = True
        return result
    correction.endpoint_acceptance = corrupt
    with pytest.raises(ValueError, match='PREGRASP_CONTACT_HEIGHT_EVIDENCE_INVALID'):
        correction.evaluate(initial)
    assert correction.steps == 0 and correction.proposed is None


def test_height_success_cannot_trade_away_nominal_support_clearance():
    _, fk, actual, accept, initial, correction = nominal_fixture(.0001)
    report = accept(fk(actual))
    assert report['height_registration_margin']['ok'] is True
    report['minimum_support_clearance_m'] = .0029
    correction.endpoint_acceptance = lambda _: deepcopy(report)
    with pytest.raises(ValueError, match='PREGRASP_ENDPOINT_ACCEPTANCE_INVALID_OR_UNSAFE'):
        correction.evaluate(initial)
    assert correction.steps == 0 and correction.proposed is None
