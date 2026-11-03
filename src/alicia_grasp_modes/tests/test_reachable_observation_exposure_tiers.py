"""Self-contained tier scheduling faults; real endpoint physics has other tests."""
from copy import deepcopy
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as module
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


@dataclass
class Envelope:
    ok: bool = True


@pytest.fixture
def synthetic(monkeypatch):
    # A real one-joint URDF and same-source scene, independent of external logs.
    # Endpoint evaluators are controlled spies here to isolate budget/dispatch;
    # the sealed replay and existing physical tests exercise the actual gates.
    model = ('<robot name="tier"><joint name="joint" type="revolute">'
        '<parent link="base_link"/><child link="one"/><origin xyz="0 0 .3"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>'
        '</joint><joint name="tip" type="fixed"><parent link="one"/>'
        '<child link="tool0"/><origin xyz=".2 0 0"/></joint></robot>')
    fk = SerialUrdfFk(model, ['joint'])
    gaps = []
    for index, angle in enumerate((0., .2)):
        gaps.append(dict(target_center_base_m=[0., 0., .02],
            jaw_axis_base=[np.cos(angle), np.sin(angle), 0.],
            initial_camera_origin_base_m=[-.2, 0., .3], object_height_m=.03,
            whole_obb_width_m=.03, missing_side_signs=[1], target_epoch=2,
            target_track_id='fixed', source_view_stamps_ns=[100], angle_deg=index))
    arguments = dict(source=dict(binding=dict(snapshot_stamp_ns=200,
        target_epoch=2, target_track_id='fixed'), tool_from_camera=np.eye(4),
        camera=dict(fx=400., fy=400., cx=320., cy=240., width=640, height=480),
        view_limits={'effective_visibility_limits': dict(roi_margin_px=36.,
            minimum_depth_m=.07, maximum_depth_m=.5)}, corners_base=np.ones((8, 3))*.01,
        center_base=np.array([0., 0., .02]), points_base=np.zeros((20, 3)),
        geometry=dict(support_normal_base=[0., 0., 1.], support_offset_m=0.,
            R_base_obb=np.eye(3), obb_size_xyz_m=[.04, .03, .03])),
        current_joint_positions=[0.], fk=fk,
        gripper=GripperGeometry(.05, .002, np.array([.04, .03, .06]), np.array([.1, .1, .07]), .003),
        opening_width_m=.05, gaps=gaps, depth_uncertainty_m=.003,
        side_depth_uncertainty_m=.004, joint_error_bounds_rad=[.01],
        max_directions=2, max_iterations=96, max_seconds=2.)
    observed = dict(gain_noise=[], side_noise=[], calls=[])
    def gain(_gaps, origin, noise):
        observed['gain_noise'].append(noise)
        return dict(ok=origin[1] >= 0., depth_uncertainty_m=noise,
            predicted_new_side_gain_m=.007)
    def side(_camera, _normal, _rotation, _size, noise):
        observed['side_noise'].append(noise)
        return dict(ok=True, observation_projected_side_evidence_m=.01)
    monkeypatch.setattr(module, 'missing_side_view_gain', gain)
    monkeypatch.setattr(module, '_optical_side_evidence', side)
    monkeypatch.setattr(module, 'validate_planned_observation_working_volume', lambda *_a, **_k: {'ok': True})
    monkeypatch.setattr(module, 'evaluate_open_gripper_observation_envelope', lambda **_k: Envelope())
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', lambda *_a: {'ok': True,
        'minimum_support_clearance_lower_bound_m': .01})
    monkeypatch.setattr(module, 'validate_observation_support_projection', lambda *_a, **_k: {'ok': True})
    def solver(_objective, seed, **kwargs):
        observed['calls'].append((seed.copy(), kwargs))
        return SimpleNamespace(x=np.array([.3]), nit=1, success=True, message='test endpoint')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    return arguments, observed, solver


def test_baseline_coverage_precedes_extra_tiers_from_qualified_endpoints(synthetic):
    args, observed, _ = synthetic
    before = deepcopy(args['source'])
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == 4 and report['status'] == 'complete'
    assert [r['exposure_target_noise_multiplier'] for r in report['directions']] == [1., 1., 2., 2.]
    indices = [r['gap_index'] for r in report['directions']]
    assert indices[0] == indices[2] and indices[1] == indices[3] and indices[0] != indices[1]
    assert report['selected_direction_count'] == 2 and report['tier_attempt_count'] == 4
    for index, (seed, kwargs) in enumerate(observed['calls']):
        np.testing.assert_array_equal(seed, [.3] if index >= 2 else args['current_joint_positions'])
        assert kwargs['options']['maxiter'] == (8 if index < 2 else 95)
    assert [r['numerical_seed_kind'] for r in report['directions']] == [
        'current_measured_start','current_measured_start',
        'same_direction_qualified_endpoint','same_direction_qualified_endpoint']
    assert all(r['binding'] == args['source']['binding'] for r in rows)
    assert all(r['execution_authority'] is False and r['proves_common_contact_height'] is False for r in rows)
    np.testing.assert_array_equal(args['source']['points_base'], before['points_base'])


def test_each_direction_reuses_only_its_own_admitted_seed_and_keeps_measured_motion_origin(synthetic, monkeypatch):
    args, observed, _ = synthetic
    calls=[]
    def numerical(objective, seed, **kwargs):
        calls.append(seed.copy())
        # Baseline motion remains centred on the measurement. Refinements
        # optimize exposure; reported displacement still uses that measurement.
        if len(calls) <= 2:
            assert objective(np.array(args['current_joint_positions'])) == 0.
        return SimpleNamespace(x=np.array([.2+.1*len(calls)]),nit=1,success=True,message='distinct endpoints')
    monkeypatch.setattr(module,'_backend',lambda:numerical)
    rows,report=module.generate_reachable_observation_views(**args)
    assert report['status']=='complete' and len(rows)==4
    np.testing.assert_allclose(calls,[[0.],[0.],[.3],[.4]],rtol=0.,atol=1e-15)
    assert all(row['joint_max_delta_rad']==pytest.approx(abs(row['q'][0])) for row in rows)
    assert args['current_joint_positions']==[0.]


def test_refinement_prefers_more_exposure_over_shorter_joint_motion(synthetic, monkeypatch):
    args, observed, solver = synthetic
    args['current_joint_positions'] = [-.8]
    args['gaps'] = [args['gaps'][1]]
    args['depth_uncertainty_m'] = .007
    costs = []
    def compare(objective, seed, **kwargs):
        costs.append((objective(np.array([-.7])), objective(np.array([.2]))))
        return solver(objective, seed, **kwargs)
    monkeypatch.setattr(module, '_backend', lambda: compare)
    rows, report = module.generate_reachable_observation_views(**args)
    assert report['status'] == 'complete' and len(rows) == 2
    assert costs[0][0] < costs[0][1]  # baseline minimizes actual movement
    assert costs[1][1] < costs[1][0]  # refinement exposes the requested face
    assert report['directions'][1]['objective'] == 'bounded_missing_side_exposure'


def test_rejected_baseline_only_seeds_its_own_unqualified_continuation_and_request_drops_seeds(synthetic,monkeypatch):
    args,observed,_=synthetic
    def numerical(_objective,seed,**kwargs):
        observed['calls'].append((seed.copy(),kwargs))
        count=len(observed['calls'])
        return SimpleNamespace(x=np.array([-.3 if count==1 else -.2 if count==4 else .3]),
            nit=1,success=True,message='rejected measured and centre seeds')
    monkeypatch.setattr(module,'_backend',lambda:numerical)
    _,report=module.generate_reachable_observation_views(**args)
    assert [row['accepted'] for row in report['directions']]==[False,True,True,False,True,True]
    np.testing.assert_array_equal([x[0] for x in observed['calls']],[[0.],[0.],[.3],[0.],[-.2],[.3]])
    assert report['directions'][4]['gap_index'] == report['directions'][0]['gap_index']
    assert report['directions'][3]['numerical_seed_kind'] == 'joint_range_centre_interpolation'
    assert report['directions'][4]['numerical_seed_kind'] == 'same_direction_unqualified_numerical_endpoint'
    assert all(row['measured_start_unchanged'] for row in report['directions'])
    observed['calls'].clear()
    _,again=module.generate_reachable_observation_views(**args)
    np.testing.assert_array_equal(observed['calls'][0][0],[0.])
    assert again['directions'][0]['numerical_seed_kind']=='current_measured_start'


def test_extra_target_does_not_change_gain_or_current_optical_acceptance_noise(synthetic):
    args, observed, _ = synthetic
    rows, report = module.generate_reachable_observation_views(**args)
    assert [r['requested_new_side_gain_m'] for r in report['directions']] == pytest.approx([.0032, .0032, .0062, .0062])
    assert observed['gain_noise'] == [.003]*4
    assert observed['side_noise'] == [.004]*4
    assert all(r['endpoint_checks']['missing_side_view_gain']['depth_uncertainty_m'] == .003 for r in rows)


def test_extra_tier_late_result_preserves_completed_baseline_without_budget_reset(synthetic, monkeypatch):
    args, observed, solver = synthetic
    now = [0.]
    def limited(*a, **kw):
        result = solver(*a, **kw)
        now[0] += .4 if len(observed['calls']) == 1 else 1.7
        return result
    monkeypatch.setattr(module, '_backend', lambda: limited)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert report['status'] == 'budget_exhausted' and report['elapsed_sec'] == pytest.approx(2.1)
    assert len(observed['calls']) == 2 and len(rows) == 1
    assert all(r['exposure_target_noise_multiplier'] == 1. for r in rows)
    assert report['directions'][-1]['reason'] == 'shared wall-clock budget exhausted'


@pytest.mark.parametrize('failure', ['solver_exception', 'original_gain_reject'])
def test_extra_tier_failure_keeps_original_candidates_and_original_gate(failure, synthetic, monkeypatch):
    args, observed, solver = synthetic
    def failing(*a, **kw):
        result = solver(*a, **kw)
        if len(observed['calls']) in (3, 4):
            if failure == 'solver_exception':
                raise RuntimeError('no stronger proposal')
            result.x = np.array([-.3])
        return result
    monkeypatch.setattr(module, '_backend', lambda: failing)
    rows, report = module.generate_reachable_observation_views(**args)
    assert report['status'] == 'complete' and len(rows) == 2
    assert all(r['exposure_target_noise_multiplier'] == 1. for r in rows)
    extra = [r for r in report['directions'] if r['exposure_target_noise_multiplier'] == 2.]
    assert all(r['accepted'] is False for r in extra)
    if failure == 'original_gain_reject':
        assert all(not r['endpoint_checks']['missing_side_view_gain']['ok'] for r in extra)


def test_no_budget_left_after_baseline_never_starts_extra_solver(synthetic, monkeypatch):
    args, observed, solver = synthetic
    now = [0.]
    copied_bindings = []
    def materialize_binding(value):
        result = deepcopy(value)
        if isinstance(value, dict) and value == args['source']['binding']:
            # Candidate binding materialization follows the complete endpoint
            # audit and admission guard. Charge that work once; reading the
            # clock itself never advances time. No next solver may start.
            copied_bindings.append(result)
            now[0] = 2.1
        return result
    monkeypatch.setattr(module, 'deepcopy', materialize_binding)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert len(copied_bindings) == len(observed['calls']) == len(rows) == 1
    assert rows[0]['binding'] == args['source']['binding']
    assert report['directions'][0]['accepted'] is True
    assert report['status'] == 'budget_exhausted'
    assert report['elapsed_sec'] == pytest.approx(2.1)


def test_zero_noise_has_no_duplicate_exposure_tier(synthetic):
    args, observed, _ = synthetic
    args['depth_uncertainty_m'] = 0.
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == len(observed['calls']) == 2
    assert report['exposure_target_noise_multipliers'] == [1.]
    assert report['maximum_candidate_count'] == 2 and report['tier_attempt_count'] == 2


def test_unachievable_extra_target_does_not_prune_baseline(synthetic):
    args, observed, _ = synthetic
    for gap in args['gaps']:
        gap['object_height_m'] = .005
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == len(observed['calls']) == 2
    assert all(r['exposure_target_noise_multiplier'] == 1. for r in rows)
    assert all(r['reason'] == 'requested exposure exceeds object-height projection'
        for r in report['directions'] if r['exposure_target_noise_multiplier'] == 2.)


def test_late_refinement_keeps_both_baselines_without_renewing_shared_deadline(synthetic, monkeypatch):
    args, observed, solver = synthetic
    now = [0.]
    def limited(*a, **kw):
        result = solver(*a, **kw)
        now[0] += .25 if len(observed['calls']) <= 2 else 1.6
        return result
    monkeypatch.setattr(module, '_backend', lambda: limited)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert report['status'] == 'budget_exhausted' and report['elapsed_sec'] == pytest.approx(2.1)
    assert len(observed['calls']) == 3 and len(rows) == 2
    assert [r['exposure_target_noise_multiplier'] for r in rows] == [1., 1.]
    assert rows[0]['direction_gap_index'] != rows[1]['direction_gap_index']
    assert report['directions'][-1]['reason'] == 'shared wall-clock budget exhausted'


def test_failed_endpoint_does_not_get_priority_even_if_solver_succeeded(synthetic, monkeypatch):
    args, observed, solver = synthetic
    def first_rejected(*a, **kw):
        result = solver(*a, **kw)
        if len(observed['calls']) == 1:
            result.x = np.array([-.3])
        return result
    monkeypatch.setattr(module, '_backend', lambda: first_rejected)
    rows, report = module.generate_reachable_observation_views(**args)
    audit = report['directions']
    assert len(rows) == 4 and len(observed['calls']) == 5
    assert [r['exposure_target_noise_multiplier'] for r in audit] == [1., 1., 2., 1., 2.]
    assert audit[0]['solver_success'] and not audit[0]['accepted']
    assert audit[1]['gap_index'] == audit[2]['gap_index']
    assert audit[0]['gap_index'] == audit[3]['gap_index'] == audit[4]['gap_index']
