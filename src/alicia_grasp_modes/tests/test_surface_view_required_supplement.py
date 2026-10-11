"""Keep the surface-view literal and current registered frame through handoff."""
from copy import deepcopy
from dataclasses import replace

import numpy as np
import pytest

from alicia_grasp_modes import observe_return_remote as remote
from alicia_grasp_modes.active_observation_policy import evidence_digest
from alicia_grasp_modes.runner_contract import surface_view_required_search_proof_valid
from test_observe_return_supplement_remote import continuation, configured
from test_observe_return_supplement_task import context, begin, supplement


SURFACE = 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'


def surface_request(configured, monkeypatch):
    node, prepared = continuation(configured, monkeypatch)
    node._active_gate_audit_report['outcome'] = {'code': SURFACE}
    return node, prepared


@pytest.mark.parametrize('code', ['NEAR_FIELD_NO_HARD_SAFE_CANDIDATE',
                                'NEAR_FIELD_NO_REACHABLE_CANDIDATE', SURFACE])
def test_original_and_surface_codes_preserve_completed_search_literal(configured, monkeypatch, code):
    node, prepared = surface_request(configured, monkeypatch)
    if code == 'NEAR_FIELD_NO_REACHABLE_CANDIDATE':
        report = node._active_gate_audit_report
        report['moveit_selection'].update(checked_count=2, shortlist_count=2)
        report['pipeline_funnel']['stage_counts']['locally_valid'] = dict(entered=3, passed=2, rejected=1)
    assert node._observe_return_prepare_supplement(prepared, code)
    request = node._observe_return_pending_supplement
    assert request['failed_search']['contact_search_code'] == code
    assert request['visual_opportunity']['contact_search_code'] == code
    assert request['contact_or_motion_authorized'] is False
    if code == SURFACE:
        assert request['visual_opportunity']['completed_contact_search'] == request['failed_search']
        assert surface_view_required_search_proof_valid(request['failed_search'])


@pytest.mark.parametrize('fault', ['nonzero_search', 'local_pass', 'incomplete', 'unchecked',
    'local_incomplete', 'bool_count', 'stale_audit', 'wrong_outcome', 'selected',
    'unregistered', 'registration_source', 'registration_identity', 'no_gain',
    'no_current_observation', 'observation_source', 'no_gap', 'max_views', 'expired'])
def test_new_literal_never_stands_in_for_missing_original_evidence(configured, monkeypatch, fault):
    node, p = surface_request(configured, monkeypatch)
    report = node._active_gate_audit_report
    search = report['moveit_selection']
    local = report['pipeline_funnel']['stage_counts']['locally_valid']
    evidence = node._latest_registration_evidence
    if fault == 'nonzero_search': search.update(checked_count=1, shortlist_count=1)
    elif fault == 'local_pass': local.update(entered=1, passed=1, rejected=0)
    elif fault == 'incomplete': search['terminated_early'] = True
    elif fault == 'unchecked': search['shortlist_count'] = 1
    elif fault == 'local_incomplete': local['entered'] += 1
    elif fault == 'bool_count': search['checked_count'] = False
    elif fault == 'stale_audit': report['snapshot_stamp_ns'] -= 1
    elif fault == 'wrong_outcome': report['outcome']['code'] = 'NEAR_FIELD_DIRECT_TIMEOUT'
    elif fault == 'selected': report['selected'] = {'plan_id': 'selected'}
    elif fault == 'unregistered': node._latest_registration_evidence = replace(evidence, result=replace(evidence.result, ok=False))
    elif fault == 'registration_source': node._latest_registration_evidence = replace(evidence, stamp_ns=evidence.stamp_ns - 1)
    elif fault == 'registration_identity': node._latest_registration_evidence = replace(evidence, identity=replace(evidence.identity, epoch=evidence.identity.epoch + 1))
    elif fault == 'no_gain': node._observe_return_step_gain['ok'] = False
    elif fault == 'no_current_observation': node._latest_target_observation = None
    elif fault == 'observation_source': node._latest_target_observation = replace(node._latest_target_observation, stamp_ns=p.snapshot.stamp_ns - 1)
    elif fault == 'no_gap': p.remote_diagnostics['tabletop_geometry']['surface_evidence_rejections'] = []
    elif fault == 'max_views': node._observe_return_session['view_index'] = 3
    elif fault == 'expired': node._observe_return_session['deadline_ns'] = p.snapshot.stamp_ns
    assert not node._observe_return_prepare_supplement(p, SURFACE)
    assert node._observe_return_pending_supplement is None
    assert node._observe_return_phase == 'observing_contact'


def test_supplement_uses_inverse_registered_points_without_mutating_phase_reference(configured, monkeypatch):
    node, p = surface_request(configured, monkeypatch)
    surface, reference = node._active_multiview_surface()
    angle = .12
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [.011, -.007, .003]
    normal = rotation @ reference.support_normal_base
    saved_surface = replace(surface, points_base=surface.points_base @ rotation.T + transform[:3, 3])
    saved_reference = replace(reference,
        points_base=reference.points_base @ rotation.T + transform[:3, 3],
        support_normal_base=normal,
        support_offset_m=reference.support_offset_m - normal @ transform[:3, 3])
    node._near_field_fused_surface = saved_surface
    node._near_field_reference_view = saved_reference
    node._latest_registration_evidence = replace(node._latest_registration_evidence,
        result=replace(node._latest_registration_evidence.result, transform_base=transform))
    before = saved_surface.points_base.tobytes(), saved_surface.view_indices.tobytes(), saved_surface.view_sources
    original = remote.measured_bilateral_view_gaps
    checked = []

    def measured(diagnostics, geometry, config, current_surface, gripper, camera, **kwargs):
        assert geometry is p.geometry
        np.testing.assert_allclose(current_surface.points_base, surface.points_base, rtol=0, atol=1e-15)
        assert current_surface.view_sources == saved_surface.view_sources
        assert current_surface.view_stamps_ns == saved_surface.view_stamps_ns
        checked.append(True)
        assert kwargs == {"include_thin": True}
        return original(diagnostics, geometry, config, current_surface, gripper, camera, **kwargs)

    monkeypatch.setattr(remote, 'measured_bilateral_view_gaps', measured)
    assert node._observe_return_prepare_supplement(p, SURFACE)
    assert checked == [True]
    assert node._observe_return_saved[0] is saved_surface
    assert before == (saved_surface.points_base.tobytes(), saved_surface.view_indices.tobytes(), saved_surface.view_sources)


@pytest.mark.parametrize('fault', ['', 'nonzero', 'passed', 'incomplete', 'bool_count', 'proof_changed'])
def test_task_requires_same_zero_candidate_proof_for_new_wire_code(context, fault):
    c = context
    begin(c)
    c.now[0] += 2_000_000_000
    plan, report = supplement(c, 2)
    metadata = report['observe_return']
    failure = metadata['failed_search']
    failure.update(contact_search_code=SURFACE, checked_count=0, shortlist_count=0,
                   locally_valid=dict(entered=12, passed=0, rejected=12))
    opportunity = metadata['visual_opportunity']
    opportunity['contact_search_code'] = SURFACE
    if fault == 'nonzero': failure.update(checked_count=1, shortlist_count=1)
    elif fault == 'passed': failure['locally_valid'] = dict(entered=1, passed=1, rejected=0)
    elif fault == 'incomplete': failure['locally_valid']['rejected'] = 11
    elif fault == 'bool_count': failure['checked_count'] = False
    opportunity['completed_contact_search'] = deepcopy(failure)
    if fault == 'proof_changed': opportunity['completed_contact_search']['snapshot_stamp_ns'] += 1
    fields = ('session_id', 'view_index', 'previous_view_index', 'origin_plan_id',
        'origin_snapshot_stamp_ns', 'target_epoch', 'target_track_id', 'deadline_ns',
        'max_observations', 'failed_search', 'visual_opportunity', 'previous_gain',
        'previous_gain_sha256', 'contact_or_motion_authorized')
    metadata['view_request_id'] = evidence_digest({key: metadata[key] for key in fields})
    if fault:
        with pytest.raises(ValueError, match='SUPPLEMENTAL_EVIDENCE_INVALID'):
            c.node._observe_return_supplement_report(plan, report)
    else:
        assert c.node._observe_return_supplement_report(plan, report) is metadata


@pytest.mark.parametrize('policy,fault', [
    ('mixed', ''), ('completed', ''), ('mixed', 'missing'),
    ('mixed', 'nonzero'), ('mixed', 'source'), ('completed', 'incomplete')])
def test_measured_runner_preserves_new_literal_and_requires_bound_zero_search(
        configured, monkeypatch, tmp_path, policy, fault):
    from test_measured_observation_planning import independent_context
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    report = deepcopy(node._active_execution_audit_report)
    trigger = report['observe_return']['trigger_evidence']
    proof = {key: trigger[key] for key in
             ('request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns')}
    proof.update(contact_search_code=SURFACE, selected=False, terminated_early=False,
        contact_or_motion_authorized=False, checked_count=0, shortlist_count=0,
        reachable_count=0, locally_valid=dict(entered=12, passed=0, rejected=12))
    if fault == 'nonzero': proof.update(checked_count=1, shortlist_count=1)
    elif fault == 'source': proof['snapshot_stamp_ns'] -= 1
    elif fault == 'incomplete': proof['locally_valid']['rejected'] = 11
    if policy == 'mixed':
        trigger.update(policy='current_request_independent_bilateral_view_gap',
            contact_search_code=SURFACE, checked_count=0, completed_contact_search=proof)
        if fault == 'missing': del trigger['completed_contact_search']
    else:
        trigger.update(policy='current_request_completed_contact_view_deficit',
            contact_search_completed=True, initial_contact_search=proof)
        trigger.pop('evidence_sha256')
        trigger['evidence_sha256'] = evidence_digest(trigger)
    assert bool(observe_return_audit_error(report)) is bool(fault)
