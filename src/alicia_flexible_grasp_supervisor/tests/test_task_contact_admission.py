"""Reject unmeasured unknown contact plans before any task movement."""
from copy import deepcopy
import json
import math

import pytest

from alicia_flexible_grasp.grasp.contact_evidence import decode_contact_evidence
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
import test_grasp_task_sequence as task_fixtures


task = task_fixtures.grasp_task_node
fixtures = task_fixtures.GraspTaskSequenceTest()


def contact_plan(stamp_sec=9.):
    plan = fixtures._rich_plan(stamp_sec=stamp_sec)
    plan.model_choice = 'unknown_tabletop'
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    return fixtures._mark_contact_geometry(plan)


@pytest.mark.parametrize('fault', ['missing', 'malformed', 'wrong_pose', 'wrong_axis', 'wrong_model'])
@pytest.mark.parametrize('freshness', [True, False])
def test_unknown_contact_admission_rejects_rehashed_bad_evidence_before_execution(fault, freshness):
    plan = contact_plan()
    if fault == 'missing':
        plan.contact_geometry_evidence = ''
    elif fault == 'malformed':
        plan.contact_geometry_evidence = '{'
    elif fault == 'wrong_pose':
        plan.poses[2].position.z += .001
    else:
        evidence = json.loads(plan.contact_geometry_evidence)
        if fault == 'wrong_axis':
            evidence['contact_height_axis_base'] = [0., 0., -1.]
        else:
            evidence['gripper_model'] = 'another gripper'
        plan.contact_geometry_evidence = json.dumps(evidence, sort_keys=True, separators=(',', ':'))
    plan.plan_id = compute_plan_id(plan)
    result = task.validate_execution_plan(plan, 10., 2., enforce_freshness=freshness)
    assert not result.ok
    assert result.code == 'CONTACT_EVIDENCE_INVALID'


@pytest.mark.parametrize('model,phase,evidence', [
    ('unknown_tabletop', 'CONTACT_EXECUTION_PLAN', True),
    ('unknown_tabletop', 'FAR_FIELD_OBSERVATION_PLAN', False),
    ('carton_segment', 'CONTACT_EXECUTION_PLAN', False),
])
def test_execution_admission_retains_valid_contact_and_legacy_observation_contracts(model, phase, evidence):
    plan = contact_plan()
    plan.model_choice, plan.diagnostic = model, phase
    if not evidence:
        plan.contact_geometry_evidence = ''
    plan.plan_id = compute_plan_id(plan)
    result = task.validate_execution_plan(plan, 10., 2.)
    assert result.ok, result.reason


def refine(current, observed):
    return task.build_bounded_final_visual_refinement(
        current, observed, .012, math.radians(10.), math.radians(4.))


def test_measured_refinement_preserves_exact_fresh_family_within_original_tilt_limit():
    current, observed = contact_plan(), contact_plan(9.5)
    for pose in observed.poses:
        pose.position.x += .002
        pose.orientation.x = math.sin(math.radians(2.) / 2.)
        pose.orientation.w = math.cos(math.radians(2.) / 2.)
    fixtures._mark_contact_geometry(observed)
    # The observed pose is measured. Keep all its DOFs together with its
    # evidence; preserving the old tilt would invalidate that evidence.
    assert task.validate_execution_plan(observed, 10., 2.).ok
    before_current, before_observed = deepcopy(current), deepcopy(observed)
    result, refined, metrics = refine(current, observed)
    assert result.ok, result.reason
    assert refined == observed and refined is not observed
    assert task._plan_phase(refined) == 'CONTACT_EXECUTION_PLAN'
    assert math.degrees(metrics['observed_roll_pitch_change_rad']) == pytest.approx(2.)
    assert decode_contact_evidence(refined, required=True) == decode_contact_evidence(observed)
    assert task.validate_execution_plan(refined, 10., 2.).ok
    assert current == before_current and observed == before_observed


def test_measured_refinement_preserves_translation_family_and_bound_evidence():
    current, observed = contact_plan(), contact_plan(9.5)
    for pose in observed.poses:
        pose.position.x += .002
    fixtures._mark_contact_geometry(observed)
    result, refined, _ = refine(current, observed)
    assert result.ok, result.reason
    assert decode_contact_evidence(refined, required=True) == decode_contact_evidence(observed)
    assert refined.contact_geometry_evidence == observed.contact_geometry_evidence


def test_old_visual_refine_cannot_launder_missing_unknown_evidence_through_diagnostic_change():
    current, observed = contact_plan(), contact_plan(9.5)
    observed.contact_geometry_evidence = ''
    observed.diagnostic = 'FINAL_VISUAL_REFINE: legacy'
    observed.plan_id = compute_plan_id(observed)
    result, refined, _ = refine(current, observed)
    assert not result.ok
    assert result.code == 'CONTACT_EVIDENCE_INVALID'
    assert refined is None


@pytest.mark.parametrize('fault,code', [
    ('tilt', 'FINAL_REFINE_ROLL_PITCH_LIMIT'),
    ('yaw', 'FINAL_REFINE_YAW_LIMIT'),
    ('translation', 'FINAL_REFINE_TRANSLATION_LIMIT'),
    ('source', 'FINAL_REFINE_CANDIDATE_SWITCH'),
    ('hash', 'FINAL_REFINE_PLAN_MALFORMED'),
    ('phase', 'FINAL_REFINE_PLAN_MALFORMED'),
    ('evidence', 'CONTACT_EVIDENCE_INVALID'),
])
def test_measured_refinement_cannot_bypass_original_bounds_or_contact_contract(fault, code):
    current, observed = contact_plan(), contact_plan(9.5)
    if fault in ('tilt', 'yaw'):
        for pose in observed.poses:
            setattr(pose.orientation, 'x' if fault == 'tilt' else 'z',
                    math.sin(math.radians(5. if fault == 'tilt' else 12.) / 2.))
            pose.orientation.w = math.cos(math.radians(5. if fault == 'tilt' else 12.) / 2.)
    if fault == 'translation':
        for pose in observed.poses: pose.position.x += .020
    if fault == 'source':
        observed.candidate_source = 'tabletop_geometry'
        observed.candidate_source_lineage = ['tabletop_geometry']
        observed.has_candidate_model_width = False
        observed.candidate_width_m = 0.
    if fault == 'phase': observed.diagnostic = 'FINAL_VISUAL_REFINE: legacy'
    fixtures._mark_contact_geometry(observed)
    if fault == 'hash': observed.plan_id = 'wrong'
    if fault == 'evidence':
        observed.poses[2].position.z += .001
        observed.plan_id = compute_plan_id(observed)
    result, refined, _ = refine(current, observed)
    assert not result.ok and result.code == code
    assert refined is None
