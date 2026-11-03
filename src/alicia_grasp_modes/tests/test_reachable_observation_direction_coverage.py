"""Frozen failure replay and bounded scheduling; no ROS or motion authority."""
from collections import defaultdict
from copy import deepcopy
import hashlib
import itertools
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_grasp_modes import reachable_observation_views as module
from test_reachable_observation_exposure_tiers import synthetic

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = Path(__file__).with_name('fixtures')


def fingerprint(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def recorded():
    meta = json.loads((FIXTURES/'supplement_direction_coverage_20261029.json').read_text())
    path = FIXTURES/'supplement_direction_coverage_20261029.npz'
    assert hashlib.sha256(path.read_bytes()).hexdigest() == meta['arrays_sha256']
    assert all(v['matches'] for v in meta['provenance']['source_reconstruction_checks'].values())
    source = deepcopy(meta['source'])
    with np.load(path) as arrays:
        source.update({k: arrays[k].copy() for k in ('points_base', 'corners_base', 'tool_from_camera')})
        source['arrival_projection'] = {k: arrays[k].copy() for k in ('prism_base', 'rgb_valid', 'invalid_integral')}
    source['arrival_projection']['audit'] = deepcopy(meta['arrival_projection_audit'])
    for key, audit in [('points_base', 'support_points_sha256'), ('corners_base', 'conservative_target_corners_sha256'),
                       ('tool_from_camera', 'tool_camera_sha256')]:
        assert fingerprint(source[key]) == source['audit'][audit]
    fk = SerialUrdfFk((ROOT/meta['model_relative_path']).read_text(), ['Joint%d'%i for i in range(1, 7)])
    assert fk.model_sha256 == meta['robot_description_sha256']
    args = deepcopy(meta['arguments'])
    # Shared regression workers may be slower than the single-process live
    # two-second measurement retained with this fixture's provenance.
    args.update(source=source, fk=fk, max_seconds=3., gripper=GripperGeometry(.05, .002,
        np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003))
    return args


def assert_original_gates(rows, args, report):
    assert rows, report
    assert len(rows) <= 2*args.get('max_directions', 2)
    for row in rows:
        np.testing.assert_allclose(args['fk'](row['q']), row['T_base_tool0'], atol=1e-12, rtol=0.)
        assert row['binding'] == args['source']['binding']
        assert row['execution_authority'] is False
        assert row['complete_path_checked'] is False and row['whole_arm_collision_checked'] is False
        assert row['actual_arrival_measurement_required'] is True
        assert row['joint_max_delta_rad'] == pytest.approx(max(abs(np.asarray(row['q'])-args['current_joint_positions'])))
        for gate in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                     'conditional_endpoint_support', 'measured_support_projection', 'optical_side_evidence',
                     'endpoint_joint_reserve', 'arrival_projection_endpoint', 'arrival_error_box_visibility'):
            assert row['endpoint_checks'][gate]['ok'], (gate, row['endpoint_checks'][gate])


def test_recorded_third_ranked_direction_is_recovered_without_relaxing_endpoint_or_budget():
    args = recorded(); before = deepcopy(args)
    rows, report = module.generate_reachable_observation_views(**args)
    assert_original_gates(rows, args, report)
    assert report['available_directions'] == 9 and report['selected_direction_count'] == 6
    assert report['coverage_limit_includes_all_directions'] is False
    assert [r['gap_index'] for r in report['directions'][:3]] == [8, 6, 0]
    assert report['directions'][2]['accepted']
    assert report['max_seconds'] == args['max_seconds'] and report['max_iterations'] == 96
    assert len({r['direction_gap_index'] for r in rows}) <= args['max_directions']
    charged = defaultdict(int)
    for row in report['directions']:
        charged[row['gap_index'], row['side_sign']] += row.get('charged_iterations', 0)
    assert max(charged.values()) <= 96
    assert args['current_joint_positions'] == before['current_joint_positions']
    assert args['gaps'] == before['gaps']
    for key in ('points_base', 'corners_base', 'tool_from_camera'):
        np.testing.assert_array_equal(args['source'][key], before['source'][key])


def transformed_scene(args, *, mirror_base_yaw=False, yaw_delta=0., long_size_scale=1.):
    """Synthetic related geometry, never relabelled as another measured scene.

    Joint1 rotates about the base z axis. Rotating all frozen world geometry
    and changing that measured joint by the same angle preserves the exact
    URDF/camera relationship. The opposite-sign base joint exercises another
    reachable arm posture. Object length changes retain the actual gripper.
    """
    result = deepcopy(args); source = result['source']; geometry = source['geometry']
    angle = -2.*result['current_joint_positions'][0] if mirror_base_yaw else yaw_delta
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.], [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    result['current_joint_positions'][0] += angle
    source['points_base'] = source['points_base']@rotation.T
    source['center_base'] = (rotation@np.asarray(source['center_base'])).tolist()
    geometry['support_normal_base'] = (rotation@np.asarray(geometry['support_normal_base'])).tolist()
    geometry['R_base_obb'] = (rotation@np.asarray(geometry['R_base_obb'])).tolist()
    geometry['obb_size_xyz_m'][0] *= long_size_scale
    axes = np.asarray(geometry['R_base_obb']); size = np.asarray(geometry['obb_size_xyz_m'])
    center = np.asarray(source['center_base'])
    source['corners_base'] = center+(np.asarray(list(itertools.product((-.5, .5), repeat=3)))*size)@axes.T
    transform = np.eye(4); transform[:3, :3] = rotation
    source['base_from_tool'] = (transform@np.asarray(source['base_from_tool'])).tolist()
    for gap in result['gaps']:
        for key in ('target_center_base_m', 'jaw_axis_base', 'initial_camera_origin_base_m'):
            gap[key] = (rotation@np.asarray(gap[key])).tolist()
        if long_size_scale != 1.:
            gap['whole_obb_width_m'] = float(np.sum(np.abs(axes.T@np.asarray(gap['jaw_axis_base']))*size))
    arrival = source['arrival_projection']
    prism = arrival['prism_base']@rotation.T
    if long_size_scale != 1.:
        # Expand the original measured footprint along the same target axis;
        # this synthetic case remains explicit and keeps its original plane.
        axis = axes[:, 0]
        prism += (long_size_scale-1.)*((prism-center)@axis)[:, None]*axis
    arrival['prism_base'] = prism
    arrival['audit']['prism_sha256'] = fingerprint(prism)
    return result


@pytest.mark.parametrize('variation', [dict(yaw_delta=.15), dict(mirror_base_yaw=True),
    dict(long_size_scale=.9), dict(long_size_scale=1.1), dict(yaw_delta=-.03)])
def test_related_postures_and_object_lengths_keep_real_model_and_all_endpoint_gates(variation):
    """Finite-iteration geometry semantics, not a two/three-second success promise."""
    args = transformed_scene(recorded(), **variation)
    # Exhaust the same bounded numerical schedule independently of host timing.
    # Real-clock deadline tests and the separately sealed unwrapped two-second
    # receipts remain authoritative for timing; the -0.03 yaw receipt has no view.
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: 0.)
    assert_original_gates(rows, args, report)
    assert report['status'] == 'complete' and report['elapsed_sec'] == 0.
    assert report['max_qualified_directions'] == 2 and report['max_search_directions'] == 6
    assert report['selected_direction_count'] <= 6
    assert report['qualified_direction_count'] <= 2
    assert report['candidate_count'] == len(rows) <= 4
    assert report['max_iterations'] == args['max_iterations'] == 96
    charged = defaultdict(int)
    for attempt in report['directions']:
        charged[attempt['gap_index'], attempt['side_sign']] += attempt.get('charged_iterations', 0)
    assert report['attempted_direction_count'] == len(charged) <= 6
    assert charged and max(charged.values()) <= args['max_iterations']


def extra_gap(args):
    gap = deepcopy(args['gaps'][0]); gap['jaw_axis_base'] = [np.cos(.4), np.sin(.4), 0.]
    args['gaps'].append(gap)


def test_first_two_numerical_failures_do_not_exclude_a_later_direction(synthetic, monkeypatch):
    args, observed, _ = synthetic; extra_gap(args); args['max_directions'] = 1
    def solver(_objective, seed, **kwargs):
        observed['calls'].append((seed.copy(), kwargs))
        return SimpleNamespace(x=np.array([-.3 if len(observed['calls']) <= 2 else .3]),
            nit=1, success=True, message='controlled endpoint')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == 2 and report['qualified_direction_count'] == 1
    assert [r['accepted'] for r in report['directions']] == [False, False, True, True]
    assert len({r['gap_index'] for r in report['directions'][:3]}) == 3
    assert all(np.array_equal(seed, args['current_joint_positions']) for seed, _ in observed['calls'][:3])
    assert all(kw['options']['maxiter'] <= 8 for _, kw in observed['calls'][:3])


def test_direction_slice_yields_to_next_direction_without_resetting_shared_clock(synthetic, monkeypatch):
    args, observed, solver = synthetic; extra_gap(args); args['max_directions'] = 1
    now = [0.]
    def delayed(*a, **kw):
        result = solver(*a, **kw)
        now[0] += .6 if len(observed['calls']) == 1 else .01
        return result
    monkeypatch.setattr(module, '_backend', lambda: delayed)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert rows and report['elapsed_sec'] < 2.
    assert report['directions'][0]['reason'] == 'direction wall-clock slice exhausted'
    assert report['directions'][1]['gap_index'] != report['directions'][0]['gap_index']
    assert all(r['direction_gap_index'] != report['directions'][0]['gap_index'] for r in rows)


@pytest.mark.parametrize('bad', [np.array([np.nan]), np.array([np.inf]), np.array([2.]), np.array([])])
def test_malformed_solver_endpoint_cannot_seed_another_attempt_or_authorize(synthetic, monkeypatch, bad):
    args, observed, _ = synthetic
    def solver(_objective, seed, **kwargs):
        observed['calls'].append((seed.copy(), kwargs))
        return SimpleNamespace(x=bad, nit=1, success=True, message='untrusted solver')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and report['execution_authority'] is False
    assert all(np.array_equal(seed, args['current_joint_positions']) for seed, _ in observed['calls'])


@pytest.mark.parametrize('coverage', [True, 0, 7, 1.5])
def test_invalid_coverage_cap_is_rejected_before_solver(synthetic, monkeypatch, coverage):
    args, _, _ = synthetic
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('invalid coverage entered solver'))
    rows, report = module.generate_reachable_observation_views(**args, max_search_directions=coverage)
    assert not rows and report['status'] == 'invalid_input'


def test_all_rejected_directions_remain_unqualified_and_do_not_grant_authority(synthetic, monkeypatch):
    args, _, _ = synthetic; extra_gap(args)
    monkeypatch.setattr(module, 'observation_endpoint_support_bound', lambda *_a: {'ok': False})
    monkeypatch.setattr(module, 'observation_following_support_bound', lambda *_a, **_kw: {'ok': False})
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and report['qualified_direction_count'] == 0
    assert report['attempted_direction_count'] == 3
    assert all(not r['accepted'] for r in report['directions'])
    assert report['execution_authority'] is False and report['original_candidates_removed'] == 0


def test_slsqp_terminal_callback_reaches_original_final_audit_with_total_iteration_cap(synthetic, monkeypatch):
    args, _, _ = synthetic; args['gaps'] = args['gaps'][:1]
    calls = []
    def solver(_objective, seed, **kwargs):
        quota = kwargs['options']['maxiter']; calls.append(quota)
        for _ in range(quota+1):
            kwargs['callback'](np.array([.3]))
        return SimpleNamespace(x=np.array([.3]), nit=quota+1, success=False, message='Iteration limit exceeded')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args)
    assert len(rows) == 2 and calls == [8, 88]
    assert sum(r['charged_iterations'] for r in report['directions']) == 96
    assert [r['solver_callback_notifications'] for r in report['directions']] == [9, 89]
    assert all(r['accepted'] and not r['solver_success'] for r in report['directions'])


def test_backend_exceeding_iteration_quota_plus_terminal_notification_is_rejected(synthetic, monkeypatch):
    args, _, _ = synthetic; args['gaps'] = args['gaps'][:1]
    def solver(_objective, seed, **kwargs):
        for _ in range(kwargs['options']['maxiter']+2):
            kwargs['callback'](np.array([.3]))
        pytest.fail('excess backend iteration was admitted')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows
    assert sum(r['charged_iterations'] for r in report['directions']) == 96
    assert all(r['reason'] == 'direction solver iteration quota exhausted' for r in report['directions'])


@pytest.mark.parametrize('with_callback', [False, True])
def test_solver_exception_charges_work_and_cannot_reset_direction_iteration_budget(synthetic, monkeypatch, with_callback):
    args, _, _ = synthetic; args['gaps'] = args['gaps'][:1]
    allocations = []
    def solver(_objective, seed, **kwargs):
        allocations.append(kwargs['options']['maxiter'])
        if with_callback:
            kwargs['callback'](np.array([.3]))
        raise RuntimeError('interrupted numerical backend')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args)
    assert not rows and report['execution_authority'] is False
    assert allocations == [8]*12
    assert sum(r['charged_iterations'] for r in report['directions']) == (12 if with_callback else 96)
    assert sum(r['charged_iterations'] for r in report['directions']) <= 96


def test_interrupted_finite_difference_probe_cannot_replace_last_major_iterate_seed(synthetic, monkeypatch):
    args, _, _ = synthetic; extra_gap(args); now = [0.]; seeds = []
    def solver(objective, seed, **kwargs):
        seeds.append(seed.copy()); call = len(seeds)
        if call in (1, 4):
            kwargs['callback'](np.array([.2 if call == 1 else -.2]))
            objective(np.array([.400001]))  # unaccepted finite-difference/line-search probe
            now[0] = .51 if call == 1 else 1.02
            kwargs['constraints'][0]['fun'](np.array([.400002]))
            pytest.fail('expired direction slice accepted a probe')
        if call == 7:
            np.testing.assert_array_equal(seed, [-.2])
            now[0] = 2.1
            objective(seed)
            pytest.fail('shared deadline was reset')
        return SimpleNamespace(x=np.array([-.3]), nit=1, success=False, message='rejected endpoint')
    monkeypatch.setattr(module, '_backend', lambda: solver)
    rows, report = module.generate_reachable_observation_views(**args, clock=lambda: now[0])
    assert not rows and len(seeds) == 7
    assert report['status'] == 'budget_exhausted'
    assert report['directions'][6]['numerical_seed_kind'] == 'same_direction_unqualified_numerical_endpoint'
    assert all(not np.array_equal(seed, [.400001]) for seed in seeds)
    assert args['current_joint_positions'] == [0.]
