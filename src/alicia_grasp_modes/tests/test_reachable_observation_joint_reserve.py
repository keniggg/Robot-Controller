"""Actual source plus solver faults; no robot or remote service access."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as module
from test_reachable_observation_seed_replay import load_recorded_scene
from test_reachable_observation_exposure_tiers import synthetic


def test_recorded_boundary_failure_has_stronger_interior_views():
    args = load_recorded_scene('supplement_view_joint_reserve_20261023')
    args['max_seconds'] = 3.  # Original two-second timing is recorded separately.
    before = deepcopy(args)
    rows, report = module.generate_reachable_observation_views(**args)
    assert report['status'] == 'complete' and len(rows) == 4, report
    # Finite-side gain is smaller than the historical centre-ray proxy; retain
    # actual resolution and all physical gates rather than the old 5 mm label.
    assert any(row['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'] > args['depth_uncertainty_m']
        for row in rows)
    for row in rows:
        for gate in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                'conditional_endpoint_support', 'measured_support_projection',
                'optical_side_evidence', 'arrival_projection_endpoint', 'endpoint_joint_reserve'):
            assert row['endpoint_checks'][gate]['ok'], (gate, row['endpoint_checks'][gate])
        q = np.array(row['q']); limits = args['fk'].limits; errors = np.array(args['joint_error_bounds_rad'])
        assert np.all(q >= limits[:, 0]+errors) and np.all(q <= limits[:, 1]-errors)
        np.testing.assert_allclose(args['fk'](q), row['T_base_tool0'], atol=1e-12, rtol=0.)
    np.testing.assert_array_equal(args['current_joint_positions'], before['current_joint_positions'])
    np.testing.assert_array_equal(args['fk'].limits, before['fk'].limits)


@pytest.mark.parametrize('endpoint', [-1., 1.])
def test_solver_success_at_original_limit_cannot_bypass_reserve(synthetic, monkeypatch, endpoint):
    args, _, _ = synthetic
    monkeypatch.setattr(module, '_backend', lambda: lambda *a, **k:
        SimpleNamespace(x=np.array([endpoint]), nit=1, success=True, message='untrusted'))
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and all(not d['accepted'] for d in report['directions'])
    assert all(not d['endpoint_checks']['endpoint_joint_reserve']['ok'] for d in report['directions'])


def test_projected_numerical_seed_does_not_replace_measured_motion_origin(synthetic):
    args, observed, _ = synthetic
    args['current_joint_positions'] = [1.]
    rows, report = module.generate_reachable_observation_views(**args)
    assert rows and report['status'] == 'complete'
    assert observed['calls'][0][0][0] < 1.-args['joint_error_bounds_rad'][0]
    assert args['current_joint_positions'] == [1.]
    assert all(row['joint_max_delta_rad'] == pytest.approx(.7) for row in rows)

