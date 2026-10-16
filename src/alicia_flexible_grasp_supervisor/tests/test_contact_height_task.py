"""A measured calibration margin must stop contact before motion and closure."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

import test_grasp_task_sequence as fixtures
from test_contact_execution import make_task, plan_from_record
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id

module = fixtures.grasp_task_node
POLICY = {'contact_height_registration_margin_enabled': True,
          'handeye_translation_max_error_m': .00460616416406,
          'calibration_centering_margin_override': True}


@pytest.mark.parametrize('override', [True, False])
def test_height_margin_independent_of_legacy_jaw_override(override):
    plan, _ = plan_from_record('gen18')
    config = dict(POLICY, calibration_centering_margin_override=override)
    before = deepcopy(plan)
    result, report = module.validate_contact_height_registration_margin(plan, config, {})
    assert not result.ok and result.code == 'CONTACT_HEIGHT_REGISTRATION_MARGIN'
    assert report['physical_grip_verified'] is False
    assert plan == before  # No changed target, normal or downward correction.


@pytest.mark.parametrize('value', [None, True, False, 0., -1., float('nan'), float('inf'), '0.004'])
def test_required_empirical_evidence_cannot_be_missing_or_faked(value):
    plan, _ = plan_from_record('gen18')
    config = dict(POLICY, handeye_translation_max_error_m=value)
    result, _ = module.validate_contact_height_registration_margin(plan, config, {})
    assert not result.ok and result.code == 'CONTACT_HEIGHT_MARGIN_CONFIG_INVALID'


@pytest.mark.parametrize('model,phase', [('carton_segment', 'CONTACT_EXECUTION_PLAN'),
                                       ('unknown_tabletop', 'FAR_FIELD_OBSERVATION_PLAN')])
def test_height_policy_does_not_change_unrelated_motion_contracts(model, phase):
    plan, _ = plan_from_record('gen18')
    plan.model_choice, plan.diagnostic = model, phase
    result, report = module.validate_contact_height_registration_margin(plan, POLICY, {})
    assert result.ok and report == {}


def bound_task(monkeypatch):
    plan, _ = plan_from_record('gen18')
    node = module.GraspTaskNode.__new__(module.GraspTaskNode)
    node.active = True
    node.set_state = Mock()
    node._bound_target_drift_result = Mock(return_value=module.PlanValidationResult(True))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time.from_sec(20.)))
    monkeypatch.setattr(module.rospy, 'get_param', lambda key, default=None: default)
    node._freeze_execution_plan(plan)
    return node, plan


def test_real_task_entry_refuses_before_simulation_or_gripper_or_arm_actions(monkeypatch):
    node, plan = bound_task(monkeypatch)
    actions = [Mock() for _ in range(5)]
    node._simulate_grasp6d_plan_if_required = Mock()
    assert not node._execute_grasp6d_plan(
        POLICY, {}, .05, actions[0], actions[1], actions[2], actions[3], plan,
        strict_execute_pose=actions[4])
    assert 'CONTACT_HEIGHT_REGISTRATION_MARGIN' in node.set_state.call_args.args[1]
    assert all(not call.called for call in actions)
    node._simulate_grasp6d_plan_if_required.assert_not_called()


def test_atomic_action_commit_cannot_bypass_height_margin(monkeypatch):
    node, plan = bound_task(monkeypatch)
    action = Mock()
    validation, response = node._invoke_plan_bound_action(plan, POLICY, 'close', action)
    assert not validation.ok and response is None
    action.assert_not_called()


def test_rebound_plan_is_rechecked_instead_of_reusing_a_prior_margin_report(monkeypatch):
    node, plan = bound_task(monkeypatch)
    node._last_contact_height_margin_report = {'ok': True, 'plan_id': 'old'}
    assert not node._execution_checkpoint(plan, POLICY, 'rebound pregrasp')
    assert node._last_contact_height_margin_report['ok'] is False


def test_fresh_nominal_contact_does_not_override_empirical_height_stress(monkeypatch):
    node, plan, _, _ = make_task(monkeypatch, hypothetical_contact=True)
    assert node._validate_measured_contact_before_close(plan, {})  # Old nominal gate passes.
    assert not node._validate_measured_contact_before_close(plan, POLICY)
    report = node._last_measured_contact_sample
    assert report['code'] == 'CONTACT_HEIGHT_REGISTRATION_MARGIN'
    assert report['height_registration_margin']['ok'] is False


def test_invalid_explicit_policy_is_rejected():
    plan, _ = plan_from_record('gen18')
    result, _ = module.validate_contact_height_registration_margin(
        plan, dict(POLICY, contact_height_registration_margin_enabled='true'), {})
    assert not result.ok and result.code == 'CONTACT_HEIGHT_MARGIN_CONFIG_INVALID'
