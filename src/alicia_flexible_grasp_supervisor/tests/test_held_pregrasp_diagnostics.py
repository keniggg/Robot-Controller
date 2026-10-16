"""Typed held-guide failures survive the real read-only service/planner boundary."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import pytest

from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot import held_pregrasp_selection as selection
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from test_contact_sequence_precheck import gateway_case
from test_gen14_held_approach_precheck import config, pose, path
from test_held_pregrasp_selection import recorded_case, select


CODE = 'PREGRASP_HELD_AXES_GUIDE_DOES_NOT_REACH_FIXED_GOAL'


def unchanged_seed(fun, seed, **kwargs):
    # Exhaust the bounded original and fallback starts deterministically; the actual shared FK and
    # quantized predictor decide failure. This is not the lost GEN11 nominal.
    return NS(success=False, message='unchanged local seed', nit=1, x=seed.copy())


@pytest.mark.parametrize('index', range(3))
def test_exhausted_real_selector_has_json_safe_original_and_trial_snapshots(monkeypatch, index):
    fk, case = recorded_case(index)
    before = deepcopy(case)
    monkeypatch.setattr(selection, 'minimize', unchanged_seed)
    with pytest.raises(HeldAxesPredictionError, match=CODE) as caught:
        select(fk, case)
    # Old production closes a cycle through original_failure.evidence here.
    report = json.loads(json.dumps(caught.value.evidence, allow_nan=False))
    diagnostics = report['pregrasp_endpoint_selection']
    original = diagnostics['original_failure']['evidence']
    assert diagnostics['original_failure']['code'] == CODE
    assert 'pregrasp_endpoint_selection' not in original
    assert original['predicted_position_error_m'] == report['predicted_position_error_m']
    assert diagnostics['original_nominal_command_rad'] == case['nominal_command']
    assert diagnostics['fixed_pregrasp_goal'] == case['poses'][0]
    assert diagnostics['fixed_approach_goal'] == case['poses'][1]
    assert len(diagnostics['trials']) == 3
    for trial in diagnostics['trials']:
        assert trial['rejection'] == CODE
        assert len(trial['quantized_command_rad']) == 6
        assert trial['rejection_evidence']['code'] == CODE
        assert trial['rejection_evidence']['predicted_position_error_m'] > .006
        assert trial['rejection_evidence']['position_tolerance_m'] == .006
        assert trial['rejection_evidence']['maximum_total_counts'] == 32
        assert trial['rejection_evidence']['held_joints'] == ['Joint2', 'Joint5']
        assert trial['rejection_evidence']['execution_authorized'] is False
    assert case == before and diagnostics['execution_authorized'] is False
    # The report must not mutate its original-failure snapshot retroactively.
    caught.value.evidence['predicted_position_error_m'] = 123.
    assert caught.value.evidence['pregrasp_endpoint_selection']['original_failure']['evidence'] == original


def real_gateway(monkeypatch):
    from test_moveit_planner_pose_feedback import MoveItPlannerPoseFeedbackTest, FakeManipulator
    node, fake, req, params, sample = gateway_case(monkeypatch)
    fk, case = recorded_case()
    params['/robot_description'] = (Path(__file__).resolve().parents[3]/
        'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/'
        'alicia_expanded_metric_candidate.urdf').read_text()
    params['/grasp'].update(config())
    sample.clear(); sample.update(deepcopy(case['sample']), epoch_ns=1)
    req.plan.poses = [pose(v) for v in case['poses']]
    req.plan.plan_id = compute_plan_id(req.plan)
    state = fake.robot.get_current_state()
    state.joint_state.position = list(sample['accepted_positions_rad'])+[.04975]
    arm = FakeManipulator()
    planner = MoveItPlannerPoseFeedbackTest().make_planner(arm)
    planner.robot = NS(get_current_state=lambda: deepcopy(state))
    planner._planning_now_sec = lambda: 100.
    planner._plan_pose_from_start_state = mock.Mock(return_value=(path(case['nominal_command']), 'frozen GEN20 nominal'))
    planner._plan_joint_from_start_state = mock.Mock(side_effect=AssertionError('failed endpoint must not produce a joint path'))
    planner._plan_cartesian_from_start_state = mock.Mock(side_effect=AssertionError('failed endpoint must stop subsequent stages'))
    planner._last_pose_plan = object()
    node._ensure_planner = lambda: planner
    return node, planner, arm, req, case


@pytest.mark.parametrize('fault', ['held_guide', 'outside_capture', 'solver_error', 'nominal_error'])
def test_actual_service_and_moveit_sequence_keep_diagnostics_and_legacy_rejection(monkeypatch, fault):
    node, planner, arm, req, case = real_gateway(monkeypatch)
    original = deepcopy(req.plan)
    monkeypatch.setattr(selection, 'minimize', unchanged_seed)
    if fault == 'solver_error':
        monkeypatch.setattr(selection, 'minimize', mock.Mock(side_effect=RuntimeError('unrelated solver failure')))
    elif fault == 'nominal_error':
        planner._plan_pose_from_start_state.side_effect = RuntimeError('unrelated planner failure')
    elif fault == 'outside_capture':
        req.plan.poses[0].position.z += .1
        req.plan.plan_id = compute_plan_id(req.plan)
        original = deepcopy(req.plan)
    result = node.handle_contact_sequence_precheck(req)
    assert not result.success
    # Preserve the caller's existing retry classification; the exact typed
    # cause and callback stage belong in the structured audit beneath it.
    assert result.failure_code == 'MOVEIT_CHECK_ERROR'
    assert result.failed_stage == ''
    report = json.loads(result.evidence_json)
    assert report['success'] is False and report['execution_authorized'] is False
    if fault in ('held_guide', 'outside_capture'):
        expected = CODE if fault == 'held_guide' else 'PREGRASP_OUTSIDE_LOCAL_CAPTURE_RANGE'
        assert expected in result.message
        failure = report['held_pregrasp_selection_failure']
        assert failure['code'] == expected and failure['callback_stage'] == 'pregrasp'
        assert failure['nominal_pregrasp_command_rad'] == case['nominal_command']
        assert failure['fixed_approach_goal'] == case['poses'][1]
        assert failure['evidence']['code'] == expected
        assert failure['evidence']['execution_authorized'] is False
        if fault == 'held_guide':
            assert report['held_pregrasp_selection'] == failure['evidence']['pregrasp_endpoint_selection']
            assert len(report['held_pregrasp_selection']['trials']) == 3
    else:
        assert 'unrelated' in result.message
        assert 'held_pregrasp_selection_failure' not in report
    assert req.plan == original and not getattr(node, '_contact_precheck_cache', {})
    assert not arm.executed_plans and not arm.go_calls and not arm.stopped
    assert planner._last_pose_plan is None
    planner._plan_joint_from_start_state.assert_not_called()
    planner._plan_cartesian_from_start_state.assert_not_called()
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._synchronize_trajectory_controller_to_feedback.assert_not_called()
