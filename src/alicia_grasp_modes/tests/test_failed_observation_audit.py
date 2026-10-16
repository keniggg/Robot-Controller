"""Rejected view searches retain evidence without creating plan authority."""
from copy import deepcopy
import json

import pytest

from test_remote_modes import configured, module, base
from test_silhouette_observation_planning import context


def failed_context(configured, monkeypatch, tmp_path, ordinary=True):
    node, prepared, checks, variant = context(
        configured, monkeypatch, tmp_path, ordinary=ordinary)
    binding = dict(request_id=prepared.ticket.request_id,
        generation=prepared.ticket.generation, target_epoch=prepared.ticket.target_epoch,
        target_track_id=prepared.snapshot.target_identity.track_id,
        snapshot_stamp_ns=prepared.snapshot.stamp_ns)
    source = dict(binding=binding, audit=dict(support_points_sha256='frozen-cloud',
        source_stamp_ns=prepared.snapshot.stamp_ns))
    node._diagnostic_observation_view_source = source
    node._diagnostic_observation_view_prepared = prepared
    node._joint_space_observation_cache = dict(prepared=prepared, source=source, key='validated',
        candidates=({'binding': deepcopy(binding), 'execution_authority': False},),
        audit=dict(status='complete', candidate_count=1, directions=[dict(index=0)]), used_sec=.3)
    variant['search_family'] = 'joint_space_fk'
    variant['camera_target_distance_m'] = .29
    variant['observation_view']['check_scheduling_index'] = 0
    variant['observation_view']['direction'] = dict(missing_side_sign=-1)
    variant['sequence'].observation_view_audit = deepcopy(variant['observation_view'])
    # A gateway success followed by a failed view qualification is precisely
    # the previously lost evidence. The real accept/audit flow remains active.
    def reject_view(*_args):
        raise ValueError('original path FOV qualification failed')
    monkeypatch.setattr(node, '_diagnostic_observation_view_evaluation', reject_view)
    return node, prepared, checks, variant, source


@pytest.mark.parametrize('ordinary', [True, False])
def test_failed_strict_success_retains_view_reason_candidates_and_source(
        configured, monkeypatch, tmp_path, ordinary):
    node, prepared, checks, variant, source = failed_context(
        configured, monkeypatch, tmp_path, ordinary)
    before = len(node.rich_plan_pub.messages)
    result = node._accept_prediction(prepared)
    assert result['status'].endswith('OBSERVATION_NO_SAFE_VIEW')
    assert len(checks) == 1 and checks[0][1]
    report = json.loads((tmp_path / 'planning.json').read_text())
    assert report['selected'] is None and report['stable_evaluations'] == []
    assert report['plan_id'] == '' and not report['outcome']['valid_plan']
    assert len(node.rich_plan_pub.messages) == before
    diagnostics = report['observation_failure_diagnostics']
    assert diagnostics['execution_authority'] is False
    rows = diagnostics['observation_orientation_search']['variant_results']
    assert len(rows) == 1 and rows[0]['checked'] and not rows[0]['reachable']
    assert rows[0]['failure_code'].endswith('PATH_VIEW_INVALID')
    assert rows[0]['following_support']['failure_location'] == 'path_view'
    assert 'original path FOV' in rows[0]['reason']
    candidate = diagnostics['observation_candidates'][0]
    assert candidate['search_family'] == 'joint_space_fk'
    assert candidate['camera_target_distance_m'] == .29
    assert candidate['frame_id'] == 'base_link'
    assert candidate['source_stamp_ns'] == prepared.snapshot.stamp_ns
    assert candidate['observation_view']['check_scheduling_index'] == 0
    assert diagnostics['source']['binding_matches_current_request']
    assert diagnostics['source']['prepared_identity_matches']
    assert diagnostics['source']['source_validation_repeated'] is False
    assert diagnostics['joint_space_fk_search']['available']
    assert diagnostics['joint_space_fk_search']['audit']['candidate_count'] == 1
    # Audit serialization must not mutate or consume proposal/path evidence.
    assert variant['observation_view']['direction'] == dict(missing_side_sign=-1)
    assert node._joint_space_observation_cache['candidates'][0]['execution_authority'] is False
    assert source['audit']['support_points_sha256'] == 'frozen-cloud'


@pytest.mark.parametrize('mismatch', ['request_binding', 'prepared', 'cache_prepared', 'cache_source'])
def test_failed_audit_does_not_claim_other_source_cache_matches(
        configured, monkeypatch, tmp_path, mismatch):
    node, prepared, _, _, source = failed_context(configured, monkeypatch, tmp_path)
    if mismatch == 'request_binding':
        source['binding']['request_id'] += 1
    elif mismatch == 'prepared':
        node._diagnostic_observation_view_prepared = object()
    elif mismatch == 'cache_prepared':
        node._joint_space_observation_cache['prepared'] = object()
    else:
        node._joint_space_observation_cache['source'] = deepcopy(source)
    node._accept_prediction(prepared)
    report = json.loads((tmp_path / 'planning.json').read_text())
    details = report['observation_failure_diagnostics']
    assert details['joint_space_fk_search']['available'] is False
    assert 'audit' not in details['joint_space_fk_search']
    assert 'candidates' not in details['joint_space_fk_search']
    if mismatch == 'request_binding':
        assert details['source']['binding_matches_current_request'] is False
    if mismatch == 'prepared':
        assert details['source']['prepared_identity_matches'] is False
    assert report['selected'] is None and not report['outcome']['valid_plan']


def test_no_endpoint_preserves_rejections_without_consuming_strict_budget(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, _, _ = failed_context(configured, monkeypatch, tmp_path)
    rejection = dict(search_family='joint_space_fk', failure_code='OBSERVATION_ENVELOPE_INVALID',
        minimum_support_clearance_m=-.001)
    monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: ((), (rejection,)))
    node._accept_prediction(prepared)
    report = json.loads((tmp_path / 'planning.json').read_text())
    details = report['observation_failure_diagnostics']
    assert checks == []
    assert details['observation_candidates'] == []
    assert details['observation_endpoint_rejections'] == [rejection]
    assert details['joint_space_fk_search']['audit']['status'] == 'complete'
    assert not report['outcome']['valid_plan']


def test_failed_audit_copies_nested_runtime_evidence(configured, monkeypatch, tmp_path):
    node, prepared, _, _, _ = failed_context(configured, monkeypatch, tmp_path)
    captured = []
    build = node._build_diagnostic_observation_gate_audit
    def capture(*args, **kwargs):
        result = build(*args, **kwargs)
        captured.append(result[0])
        return result
    monkeypatch.setattr(node, '_build_diagnostic_observation_gate_audit', capture)
    node._accept_prediction(prepared)
    runtime = node._stable_variant_runtime[(0, 0)]
    report = captured[-1]
    details = deepcopy(report['observation_failure_diagnostics'])
    runtime['observation_orientation_search']['variant_results'][0]['reason'] = 'mutated after audit'
    runtime['observation_variants'][0]['observation_view']['direction']['missing_side_sign'] = 1
    node._joint_space_observation_cache['audit']['directions'][0]['index'] = 99
    node._diagnostic_observation_view_source['audit']['support_points_sha256'] = 'changed'
    assert report['observation_failure_diagnostics'] == details
