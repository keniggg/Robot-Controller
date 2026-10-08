"""Close-range final refinement is distinct from observation acquisition."""
from copy import deepcopy
from unittest.mock import Mock
import math

import pytest
import test_grasp_task_sequence as sequence
from test_final_refine_reference import setup_node

m = sequence.grasp_task_node


def contact(helper, stamp):
    plan = helper._mark_valid_3d(helper._rich_plan(stamp_sec=stamp))
    plan.diagnostic = m._CONTACT_EXECUTION_PLAN
    plan.plan_id = sequence.compute_plan_id(plan)
    return plan


def preview_fixture(monkeypatch, distance=.1564):
    node, helper, clock = setup_node(monkeypatch, now=10.)
    bound, preview = contact(helper, 9.), contact(helper, 9.95)
    node.latest_grasp6d_preview_plan = preview
    node._target_drift_result = Mock(return_value=m.PlanValidationResult(True))
    center = node._plan_geometry_center_xyz(preview)
    node._current_camera_pose_base = lambda: helper._pose(center[0], center[1], center[2]+distance)
    cfg = dict(near_field_strategy='single_snapshot_direct',
               observation_camera_target_range_check_enabled=True,
               observation_camera_target_min_distance_m=.22,
               observation_camera_target_max_distance_m=.30)
    return node, helper, bound, preview, cfg


@pytest.mark.parametrize('distance,observation_ok', [(.1564, False), (.26, True)])
def test_same_preview_keeps_observation_range_but_accepts_registered_final_view(monkeypatch, distance, observation_ok):
    node, _, bound, preview, cfg = preview_fixture(monkeypatch, distance)
    originals = deepcopy((bound, preview, cfg))
    observation, _ = node._copy_near_field_preview_candidate(bound, 9_500_000_000, cfg)
    final, candidate = node._copy_near_field_preview_candidate(
        bound, 9_500_000_000, cfg, preview_stage='final_refinement')
    assert observation.ok is observation_ok
    if not observation_ok: assert observation.code == 'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE'
    assert final.ok and candidate.plan_id == preview.plan_id
    assert node._target_drift_result.call_count == 2
    assert (bound, preview, cfg) == originals


@pytest.mark.parametrize('stage,phase', [('misspelled', m._CONTACT_EXECUTION_PLAN),
    ('final_refinement', m._FAR_FIELD_OBSERVATION_PLAN), ('final_refinement', '')])
def test_final_stage_cannot_authorize_an_observation_or_unknown_phase(monkeypatch, stage, phase):
    node, _, bound, _, cfg = preview_fixture(monkeypatch)
    bound.diagnostic = phase; bound.plan_id = sequence.compute_plan_id(bound)
    result, candidate = node._copy_near_field_preview_candidate(
        bound, 9_500_000_000, cfg, preview_stage=stage)
    assert not result.ok and result.code == 'FINAL_REFINE_PHASE_INVALID'
    assert result.terminal_for_request and candidate is None


@pytest.mark.parametrize('fault', ['old', 'id', 'track', 'registration', 'support', 'drift'])
def test_final_stage_retains_freshness_identity_registration_support_and_drift(monkeypatch, fault):
    node, _, bound, preview, cfg = preview_fixture(monkeypatch)
    if fault == 'old':
        preview.header.stamp = preview.object_geometry.header.stamp = m.rospy.Time.from_sec(9.)
    if fault == 'track': preview.target_track_id = preview.object_geometry.target_track_id = 'g0-t2'
    if fault == 'registration': preview.refinement_status = 'INVALID_3D'
    if fault == 'support': preview.object_geometry.support_offset_m = .0041
    if fault == 'drift': node._target_drift_result.return_value = m.PlanValidationResult(False, 'TARGET_DRIFT', 'actual target moved')
    preview.plan_id = sequence.compute_plan_id(preview)
    if fault == 'id': preview.plan_id = 'corrupted'
    result, candidate = node._copy_near_field_preview_candidate(
        bound, 9_500_000_000, cfg, preview_stage='final_refinement')
    assert not result.ok and candidate is None
    assert result.terminal_for_request is (fault != 'old')
    assert result.code == ('TARGET_DRIFT' if fault == 'drift' else 'FINAL_REFINE_3D_INVALID')


def waiting_node(monkeypatch, result):
    node, helper, clock = setup_node(monkeypatch)
    bound = contact(helper, 10.)
    node.set_state = Mock()
    node._wait_for_final_refine_reference = lambda *_: helper._object(stamp_sec=12.)
    node._request_near_field_preview_stream = lambda *_: True
    node._set_near_field_preview_stream = Mock(return_value=True)
    stages = []
    def preview(*args, **kwargs):
        stages.append(kwargs['preview_stage'])
        return result, None
    node._copy_near_field_preview_candidate = preview
    node._freeze_execution_plan = Mock(side_effect=AssertionError('failed preview rebound'))
    return node, bound, clock, stages


@pytest.mark.parametrize('required', [True, False])
@pytest.mark.parametrize('code,flag', [('FINAL_REFINE_3D_INVALID', True), ('WSL_UNAVAILABLE', False)])
def test_fresh_request_terminal_finishes_without_twenty_second_wait(monkeypatch, required, code, flag):
    node, bound, clock, stages = waiting_node(monkeypatch,
        m.PlanValidationResult(False, code, 'current request rejected', terminal_for_request=flag))
    result = node._maybe_final_refine_grasp6d_plan(dict(
        final_visual_refine_enabled=True, final_visual_refine_required=required,
        final_visual_refine_timeout_sec=20., near_field_strategy='single_snapshot_direct'), {}, bound)
    assert clock[0] == 12. and stages == ['final_refinement']
    node._set_near_field_preview_stream.assert_called_once()
    assert node._set_near_field_preview_stream.call_args.args[-1] is False
    if required:
        assert result is None
        assert node.set_state.call_args.args == (m.GraspStages.FAILED, code+': current request rejected')
    else: assert result is bound
    node._freeze_execution_plan.assert_not_called()


def test_cached_invalid_preview_waits_for_new_request_until_original_deadline(monkeypatch):
    node, bound, clock, stages = waiting_node(monkeypatch,
        m.PlanValidationResult(False, 'FINAL_REFINE_3D_INVALID', 'old cached preview'))
    result = node._maybe_final_refine_grasp6d_plan(dict(
        final_visual_refine_enabled=True, final_visual_refine_timeout_sec=.2,
        near_field_strategy='single_snapshot_direct'), {}, bound)
    assert result is None and clock[0] == pytest.approx(12.2)
    assert len(stages) > 1
    assert node.set_state.call_args.args[1].startswith('FINAL_REFINE_TIMEOUT:')


def test_wrong_stage_is_never_an_optional_refinement_fallback(monkeypatch):
    node, bound, clock, _ = waiting_node(monkeypatch,
        m.PlanValidationResult(False, 'FINAL_REFINE_PHASE_INVALID', 'wrong stage', terminal_for_request=True))
    result = node._maybe_final_refine_grasp6d_plan(dict(
        final_visual_refine_enabled=True, final_visual_refine_required=False,
        final_visual_refine_timeout_sec=20., near_field_strategy='single_snapshot_direct'), {}, bound)
    assert result is None and clock[0] == 12.
    assert node.set_state.call_args.args[0] == m.GraspStages.FAILED


@pytest.mark.parametrize('failure_stage', ['construction', 'sequence'])
@pytest.mark.parametrize('required', [True, False])
def test_completed_single_snapshot_does_not_repeat_failed_build_or_sequence(monkeypatch, failure_stage, required):
    node, helper, clock = setup_node(monkeypatch)
    bound, preview = contact(helper, 10.), contact(helper, 12.01)
    node.set_state = Mock()
    node._wait_for_final_refine_reference = lambda *_: helper._object(stamp_sec=12.)
    node._request_near_field_preview_stream = lambda *_: True
    node._set_near_field_preview_stream = Mock(return_value=True)
    node._copy_near_field_preview_candidate = Mock(return_value=(m.PlanValidationResult(True), preview))
    failure = m.PlanValidationResult(False,
        'CONTACT_EVIDENCE_INVALID' if failure_stage == 'construction' else 'MOVEIT_UNREACHABLE',
        'current snapshot cannot pass')
    builder = Mock(return_value=(failure, None, {}) if failure_stage == 'construction' else
        (m.PlanValidationResult(True), preview, {}))
    monkeypatch.setattr(m, 'build_bounded_final_visual_refinement', builder)
    monkeypatch.setattr(m, 'validate_calibration_centering_margin', lambda *_: m.PlanValidationResult(True))
    node._execution_checkpoint = lambda *_: True
    node._check_final_refine_sequence = Mock(return_value=failure)
    node._freeze_execution_plan = Mock(side_effect=AssertionError('invalid candidate rebound'))
    result = node._maybe_final_refine_grasp6d_plan(dict(
        final_visual_refine_enabled=True, final_visual_refine_required=required,
        final_visual_refine_timeout_sec=20., near_field_strategy='single_snapshot_direct'), {}, bound)
    assert clock[0] == 12. and builder.call_count == 1
    assert node._copy_near_field_preview_candidate.call_count == 1
    assert node._copy_near_field_preview_candidate.call_args.kwargs['preview_stage'] == 'final_refinement'
    assert node._check_final_refine_sequence.call_count == (failure_stage == 'sequence')
    assert result is (None if required else bound)
    if required: assert node.set_state.call_args.args == (m.GraspStages.FAILED, failure.code+': '+failure.reason)
    node._freeze_execution_plan.assert_not_called()
