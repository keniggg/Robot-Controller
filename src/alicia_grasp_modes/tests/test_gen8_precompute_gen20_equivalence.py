"""Current GEN20 replay with exact cached/uncached equations and controlled clock.

This does not assert a fixed number of wall-clock candidates or apply one
historical arrival error transform as a prediction for every new proposal.
Real 2-second deadline behavior has separate tests and recorded benchmarks.
The immutable GEN8 algorithm remains a historical comparison: its old seeds,
joint bounds and stronger-tier objective no longer define current behavior.
"""
import hashlib
import importlib.util
from importlib.machinery import SourceFileLoader
import json
from pathlib import Path

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as candidate
from test_observation_prism_centre_preference import gen20_source


def baseline():
    path = Path(__file__).parent/'fixtures/gen8_reachable_before_precompute.py.txt'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == '8e7afbd059c028e15d0b6ce98f0ccfa22717c5e61714062a440e942b2572208d'
    name = 'alicia_grasp_modes._gen8_before_precompute_equivalence'
    spec = importlib.util.spec_from_loader(name, SourceFileLoader(name, str(path)))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def serialized(result):
    return json.dumps(result, sort_keys=True, default=lambda v:
        v.tolist() if isinstance(v, np.ndarray) else v.item())


@pytest.mark.parametrize('objective', ['centered', 'joint_only'])
def test_frozen_gen20_current_precomputation_preserves_complete_result(
        gen20_source, objective, monkeypatch, tmp_path):
    args, _historical_relative_error = gen20_source
    results = []
    historical = baseline()
    for module in [historical, candidate]:
        preference = module._arrival_center_preference
        if objective == 'joint_only':
            monkeypatch.setattr(module, '_arrival_center_preference',
                lambda *a, _p=preference, **kw: dict(_p(*a, **kw), cost=0.))
        rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
        assert rows and report['status'] == 'complete'
        assert report['tier_attempt_count'] == 4
        for row in rows:
            assert not row['execution_authority'] and not row['complete_path_checked']
            assert row['actual_arrival_measurement_required']
            for key in ['working_volume', 'observation_envelope', 'conditional_endpoint_support',
                        'measured_support_projection', 'optical_side_evidence', 'arrival_projection_endpoint']:
                assert row['endpoint_checks'][key]['ok']
        results.append((rows, report))
    # Isolate the optimization actually under test: both runs retain the same
    # production seeds, bounds, objectives and independent final admission.
    # Only proposal evaluation replaces the cache with the original formula.
    prepared = []
    evaluated = []
    def independent_support_bound(*contract):
        prepared.append(True)
        fk, errors, normal, offset, gripper, opening = contract
        def evaluate(q):
            evaluated.append(True)
            return candidate.observation_endpoint_support_bound(
                fk, q, errors, normal, offset, gripper, opening)
        return evaluate
    with monkeypatch.context() as uncached:
        uncached.setattr(candidate, 'PreparedEndpointSupportBound', independent_support_bound)
        uncached_result = candidate.generate_reachable_observation_views(**args, clock=lambda: 0.)

    # Preserve the complete old result and actual algorithm differences in the
    # run's isolated temporary directory, including on an equivalence failure.
    old_json, current_json = (json.loads(serialized(value)) for value in results)
    old_report, current_report = old_json[1], current_json[1]
    history = dict(historical_source_sha256=hashlib.sha256(
        Path(historical.__file__).read_bytes()).hexdigest(),
        historical_result=old_json, current_result=current_json,
        same_current_algorithm_uncached_result=json.loads(serialized(uncached_result)),
        historical_complete_result_equal=serialized(results[0]) == serialized(results[1]),
        changed_report_fields={key: dict(historical=old_report.get(key), current=current_report.get(key))
            for key in sorted(set(old_report) | set(current_report))
            if old_report.get(key) != current_report.get(key)},
        historical_algorithm_is_current_oracle=False, controlled_clock=True,
        certifies_arrival_error_box=False, certifies_hardware_or_full_path=False)
    (tmp_path/'gen8-history.json').write_text(json.dumps(history, sort_keys=True))
    assert prepared == [True] and evaluated
    assert serialized(results[1]) == serialized(uncached_result)
    assert 'endpoint_error_bounds_rad' not in args
    assert results[1][1]['arrival_error_box_required'] is False
    assert all(row['endpoint_checks']['endpoint_joint_reserve']['ok'] for row in results[1][0])

    # The new per-direction audit labels the old common-height schema. Retain
    # its complete contract independently of the cached/uncached comparison.
    assert all('observation_span' not in row for row in results[0][1]['directions'])
    for direction in results[1][1]['directions']:
        span_audit = direction['observation_span']
        gap = args['gaps'][direction['gap_index']]
        assert gap['missing_side_policy'] == 'insufficient_samples_or_weakest_common_height'
        assert 'side_measurement_basis' not in gap
        assert span_audit == dict(
            observation_span_policy='legacy_common_height_unpaired_observation',
            observation_span_m=gap['object_height_m'], common_contact_band_predicted=False,
            measured_registration_required=True, contact_or_motion_authorized=False,
            legacy_measurement_basis='common_height_statistics_not_independent_face_span')
