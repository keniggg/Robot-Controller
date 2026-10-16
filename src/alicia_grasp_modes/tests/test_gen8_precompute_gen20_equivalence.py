"""Full historical GEN20 equations/candidates, with controlled clock only.

This does not assert a fixed number of wall-clock candidates or apply one
historical arrival error transform as a prediction for every new proposal.
Real 2-second deadline behavior has separate tests and recorded benchmarks.
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
def test_frozen_gen20_complete_result_equals_before_precomputation(gen20_source, objective, monkeypatch):
    args, _historical_relative_error = gen20_source
    results = []
    for module in [baseline(), candidate]:
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
    assert serialized(results[0]) == serialized(results[1])
