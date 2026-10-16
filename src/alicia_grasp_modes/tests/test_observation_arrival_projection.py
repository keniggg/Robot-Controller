"""Production source -> actual strict endpoint, plus bounded FK constraints."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_unknown_observation_view_source import generic_context
from test_silhouette_observation_view_source import encoded_result
from test_reachable_observation_views import recorded_inputs
from test_reachable_observation_integration import recorded_proposal
from alicia_grasp_modes import observation_arrival_projection as projection
from alicia_grasp_modes import reachable_observation_views as reachable
from alicia_grasp_modes.silhouette_observation_view_source import array_fingerprint


def test_real_source_capture_and_original_strict_path_publish_endpoint_audit(configured, monkeypatch, tmp_path):
    node, prepared, sample, camera, fk = generic_context(configured, monkeypatch, tmp_path)
    captured = node._capture_diagnostic_observation_view_source(prepared.ticket)
    node._bind_diagnostic_observation_view_source(prepared, captured)
    result, target = encoded_result(prepared, fk)
    report = node._diagnostic_observation_view_evaluation(prepared, result, target)
    assert report['ok'] and report['target_trajectory']['sample_count'] > 3
    endpoint = report['arrival_projection_endpoint']
    assert endpoint['ok'] and endpoint['endpoint_only']
    assert not endpoint['certifies_continuous_visibility']
    assert endpoint['source']['source_stamp_ns'] == sample.stamp_ns
    assert report['source']['arrival_projection'] == endpoint['source']


def test_actual_strict_endpoint_prism_failure_precedes_following_and_publication(configured, monkeypatch, tmp_path):
    node, prepared, _, _, fk = generic_context(configured, monkeypatch, tmp_path)
    source = base._thaw_request_evidence(node._diagnostic_observation_view_source)
    contract = source['arrival_projection']
    contract['prism_base'][:, 1] += .17
    contract['audit']['prism_sha256'] = array_fingerprint(contract['prism_base'])
    source['audit']['arrival_projection'] = deepcopy(contract['audit'])
    node._diagnostic_observation_view_source = base._freeze_request_evidence(source)
    result, target = encoded_result(prepared, fk)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_observation_following_support_evaluation',
        lambda *a, **k: pytest.fail('following entered after invalid arrival endpoint'))
    outcome, report = node._observation_following_support_evaluation(prepared, result, target)
    assert not outcome.reachable
    assert outcome.failure_code == 'UNKNOWN_OBSERVATION_PATH_VIEW_INVALID'
    assert 'arrival prism' in outcome.reason
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_fk_original_solver_constraints_include_prism_and_success_cannot_override_gate(
        recorded_inputs, recorded_proposal, monkeypatch):
    from test_observation_frozen_prism_guards import capture
    args = deepcopy(recorded_inputs)
    q = np.asarray(recorded_proposal['q'])
    camera = np.asarray(recorded_proposal['T_base_camera_link'])
    contract = capture()
    # Move a genuine prism far outside this measured camera. Original OBB,
    # support, gain and motion remain the previously passing recorded endpoint.
    prism = np.asarray(contract['prism_base']).copy()
    prism = prism @ camera[:3, :3].T + camera[:3, 3]
    prism += camera[:3, 1] * 1.
    contract['prism_base'] = prism
    contract['audit']['prism_sha256'] = array_fingerprint(prism)
    width, height = args['source']['camera']['width'], args['source']['camera']['height']
    valid = np.ones((height, width), bool)
    integral = np.zeros((height+1, width+1), np.int64)
    contract.update(rgb_valid=valid, invalid_integral=integral)
    contract['audit'].update(rgb_valid_sha256=array_fingerprint(valid),
        invalid_integral_sha256=array_fingerprint(integral))
    residuals = []
    def solver(objective, initial, **kwargs):
        residuals.append(kwargs['constraints'][0]['fun'](q))
        return NS(x=q.copy(), nit=1, success=True, message='injected successful solver')
    monkeypatch.setattr(reachable, '_backend', lambda: solver)
    old, _ = reachable.generate_reachable_observation_views(**args)
    assert old
    old_size = len(residuals[-1])
    args['source']['arrival_projection'] = contract
    rows, audit = reachable.generate_reachable_observation_views(**args)
    assert not rows
    assert len(residuals[-1]) == old_size + len(prism)*6
    assert np.min(residuals[-1][-len(prism)*6:]) < 0.
    assert 'arrival prism' in audit['directions'][0]['reason']
    assert audit['max_seconds'] == recorded_inputs['max_seconds']


def test_missing_new_metadata_only_blocks_observation_capture(configured, monkeypatch, tmp_path):
    node, prepared, _, _, _ = generic_context(configured, monkeypatch, tmp_path)
    del node._unknown_identity_status[prepared.snapshot.stamp_ns]['reference_object_height_m']
    with pytest.raises(base.CandidateContractError) as error:
        node._capture_diagnostic_observation_view_source(prepared.ticket)
    assert error.value.code == 'UNKNOWN_OBSERVATION_VIEW_SOURCE_INVALID'
    node._observe_return_diagnostic_observation_only = True
    captured = node._capture_diagnostic_observation_view_source(prepared.ticket)
    assert 'arrival_projection' not in captured
