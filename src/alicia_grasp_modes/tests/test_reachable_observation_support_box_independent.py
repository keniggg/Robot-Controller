"""Actual R2 error-box regression; frozen clocks are semantic, not timing evidence.

Only numerical proposal endpoints are substituted in generator wiring tests.
The model, full measured support arrays, CAD, visibility, arrival, gain and
independent whole-box proof remain the production implementations.
"""
from copy import deepcopy
from itertools import product
from pathlib import Path
from types import SimpleNamespace
import hashlib
import json

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _box_corners, _stage_boxes)
from alicia_flexible_grasp.robot.observation_path_guard import (
    SerialUrdfFk, ObservationPathError, ObservationSupportBox,
    observation_endpoint_support_bound, observation_following_support_bound)
from alicia_flexible_grasp.robot.observation_arrival_reserve import (
    PreparedObservationArrivalReserve, arrival_reserve_from_source)
from alicia_grasp_modes.observation_endpoint_precompute import PreparedEndpointSupportBound
from alicia_grasp_modes import reachable_observation_views as module


def load_recorded_support_scene():
    """Self-contained exact consumed subset; no dependency on the external SSD."""
    fixtures = Path(__file__).with_name('fixtures')
    metadata = json.loads((fixtures/'supplement_support_box_20261103.json').read_text())
    archive = fixtures/'supplement_support_box_20261103.npz'
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == metadata['arrays_sha256']
    with np.load(archive, allow_pickle=False) as loaded:
        assert set(loaded.files) == set(metadata['arrays'])
        arrays = {name: loaded[name].copy() for name in loaded.files}
    for name, array in arrays.items():
        expected = metadata['arrays'][name]
        assert list(array.shape) == expected['shape']
        assert str(array.dtype) == expected['dtype']
        assert hashlib.sha256(array.tobytes()).hexdigest() == expected['sha256']
    work = Path(__file__).resolve().parents[3]
    model = work/metadata['model_relative_path']
    assert hashlib.sha256(model.read_bytes()).hexdigest() == metadata['robot_description_sha256']
    fk = SerialUrdfFk(model.read_text(), ['Joint%d' % i for i in range(1, 7)])
    source = deepcopy(metadata['source'])
    source.update({k: arrays[k] for k in ('points_base', 'corners_base', 'center_base', 'tool_from_camera')})
    source['arrival_projection'] = {k: arrays[k] for k in ('prism_base', 'rgb_valid', 'invalid_integral')}
    source['arrival_projection']['audit'] = deepcopy(metadata['arrival_projection_audit'])
    gripper = dict(metadata['gripper'])
    for key in ('finger_size_xyz_m', 'palm_size_xyz_m'):
        gripper[key] = np.array(gripper[key])
    arguments = deepcopy(metadata['arguments'])
    arguments.update(source=source, fk=fk, gripper=GripperGeometry(**gripper),
        max_seconds=5., max_iterations=96, max_directions=2, max_search_directions=6,
        proposal_image_reserve_px=4.)
    return arguments, metadata


@pytest.fixture
def recorded_support():
    return load_recorded_support_scene()


def support_arguments(arguments, q):
    geometry = arguments['source']['geometry']
    return (arguments['fk'], q, arguments['joint_error_bounds_rad'],
            geometry['support_normal_base'], geometry['support_offset_m'],
            arguments['gripper'], arguments['opening_width_m'])


def prove(arguments, q):
    # No wall-time claim: real 5-second performance is tested by the sealed
    # standalone replay; another file covers clock and late-result boundaries.
    return observation_following_support_bound(*support_arguments(arguments, q), clock=lambda: 0.)


def direct_endpoint(arguments, q, monkeypatch):
    calls = []

    def solver(_objective, _seed, **_kwargs):
        calls.append(True)
        return SimpleNamespace(x=np.asarray(q).copy(), nit=1, success=False,
            message='frozen numerical proposal; every real endpoint gate remains required')

    monkeypatch.setattr(module, '_backend', lambda: solver)
    # A single returned iterate avoids doing an optimization in this semantic
    # regression. It cannot skip complete_endpoint or the independent proof.
    rows, report = module.generate_reachable_observation_views(**dict(arguments,
        max_directions=1, max_search_directions=1, max_iterations=1, clock=lambda: 0.))
    assert calls
    return rows, report


def test_fixture_preserves_full_actual_consumed_arrays_and_error_domains(recorded_support):
    arguments, metadata = recorded_support
    assert metadata['provenance']['source_stamp_ns'] == 1793698552367479085
    assert metadata['provenance']['all_original_recovery_array_checks_exact']
    assert metadata['provenance']['arrival_audit_exact']
    assert len(arguments['source']['points_base']) == 98983
    assert set(metadata['arrays']) == {'points_base', 'corners_base', 'center_base',
        'tool_from_camera', 'prism_base', 'rgb_valid', 'invalid_integral'}
    np.testing.assert_array_equal(arguments['joint_error_bounds_rad'], np.full(6, .06601699039494283))
    np.testing.assert_array_equal(arguments['endpoint_error_bounds_rad'], np.full(6, .03576699039394282))
    assert arguments['gripper'].support_clearance_m == .003
    assert arguments['opening_width_m'] == arguments['gripper'].max_inner_gap_m == .05
    assert not metadata['provenance']['execution_authority']
    assert not metadata['provenance']['current_freshness_restored']


def test_actual_coarse_rejection_and_independent_complete_same_box_proof(recorded_support):
    arguments, metadata = recorded_support
    case = metadata['recorded_cases']['coarse_support_only_rejected']
    coarse = observation_endpoint_support_bound(*support_arguments(arguments, case['q']))
    assert coarse == case['endpoint_checks']['conditional_endpoint_support']
    assert not coarse['ok'] and coarse['minimum_support_clearance_lower_bound_m'] < .003
    result = prove(arguments, case['q'])
    assert result['ok'] and result['box_checks'] >= 1
    assert result['minimum_support_clearance_lower_bound_m'] > .003
    assert result['required_support_clearance_m'] == coarse['required_support_clearance_m'] == .003
    assert result['joint_error_bounds_rad'] == coarse['joint_error_bounds_rad'] == arguments['joint_error_bounds_rad']
    assert result['goal_joint_positions_rad'] == case['q']
    assert result['robot_description_sha256'] == arguments['fk'].model_sha256
    assert result['scope'] == 'conditional_joint_error_box_Link6_Link7_Link8_support_only'
    assert not result['certifies_hardware_tracking_stopping_or_calibration']


def rotation(axis, angle):
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*cross + (1.-np.cos(angle))*(cross@cross)


@pytest.mark.parametrize('scale', [.65, 1., 1.4])
@pytest.mark.parametrize('yaw', [-.35, 0., .35])
@pytest.mark.parametrize('opening', [0., .025, .05])
def test_box_enclosures_cover_different_object_sizes_poses_and_openings(recorded_support, scale, yaw, opening):
    arguments, metadata = recorded_support
    q = np.array(metadata['recorded_cases']['coarse_support_only_rejected']['q'])
    arguments['opening_width_m'] = opening
    geometry = arguments['source']['geometry']
    normal = np.array(geometry['support_normal_base'])
    result = prove(arguments, q)
    assert result['ok']
    arrival = arrival_reserve_from_source(arguments['fk'], arguments['endpoint_error_bounds_rad'], arguments['source'])
    centre = arguments['source']['center_base']
    points = centre+scale*(arrival.points-centre)@rotation(normal, yaw).T
    bound = PreparedObservationArrivalReserve(arguments['fk'], arrival.errors,
        arrival.mount, points, arrival.camera, arrival.limits)
    lower = bound.lower_bounds(q, extra_margin_px=4.)
    k = arrival.camera
    for signs in product((-1., 1.), repeat=6):
        physical_q = q+np.array(signs)*arguments['joint_error_bounds_rad']
        pose = arguments['fk'](physical_q)
        physical = np.concatenate([_box_corners(c, pose[:3, :3], size)
            for _, c, size in _stage_boxes(pose, arguments['gripper'], opening, 'y', 'z')])
        actual_support = float(np.min(physical@normal+geometry['support_offset_m']))
        assert actual_support >= result['minimum_support_clearance_lower_bound_m']-1.e-12
        camera_q = q+np.array(signs)*arrival.errors
        pose = arguments['fk'](camera_q)@arrival.mount
        xyz = (points-pose[:3, 3])@pose[:3, :3]
        uv = np.array([k['cx'], k['cy']])-np.array([k['fx'], k['fy']])*xyz[:, 1:]/xyz[:, :1]
        if np.all(lower >= 0.):
            # Samples corroborate the original full-box enclosure; the samples
            # themselves never replace it or authorize a path.
            assert np.min(xyz[:, 0]) >= arrival.limits['minimum_depth_m']
            assert np.max(xyz[:, 0]) <= arrival.limits['maximum_depth_m']
            assert np.min(np.c_[uv[:, 0], k['width']-1-uv[:, 0], uv[:, 1], k['height']-1-uv[:, 1]]) >= 28.


def test_actual_model_collision_still_has_a_counterexample(recorded_support):
    arguments, metadata = recorded_support
    q = metadata['recorded_cases']['coarse_support_only_rejected']['q']
    coarse = observation_endpoint_support_bound(*support_arguments(arguments, q))
    geometry = arguments['source']['geometry']
    # This deliberately moves the model support plane to leave only 2mm.
    # It is a synthetic collision input, not a modified measured source.
    geometry['support_offset_m'] -= coarse['nominal_minimum_support_clearance_m']-.002
    result = prove(arguments, q)
    assert not result['ok']
    assert result['counterexample_model_clearance_m'] < .003
    witness = result['counterexample_joint_positions_rad']
    independent = observation_endpoint_support_bound(arguments['fk'], witness, np.zeros(6),
        geometry['support_normal_base'], geometry['support_offset_m'], arguments['gripper'], arguments['opening_width_m'])
    assert independent['nominal_minimum_support_clearance_m'] < .003
    assert result['joint_error_bounds_rad'] == arguments['joint_error_bounds_rad']


@pytest.mark.parametrize('fault', ['model', 'gripper', 'normal', 'opening', 'clearance'])
def test_unsupported_or_invalid_whole_box_contract_is_rejected(recorded_support, fault):
    arguments, _ = recorded_support
    geometry = arguments['source']['geometry']
    fk, gripper, opening = arguments['fk'], arguments['gripper'], arguments['opening_width_m']
    normal, offset = geometry['support_normal_base'], geometry['support_offset_m']
    if fault == 'model': fk = object()
    elif fault == 'gripper': gripper = object()
    elif fault == 'normal': normal = np.array(normal)*2.
    elif fault == 'opening': opening = .050001
    else:
        from dataclasses import replace
        gripper = replace(gripper, support_clearance_m=.002999)
    with pytest.raises(ObservationPathError):
        ObservationSupportBox(fk, gripper, opening, normal, offset)


def test_prepared_coarse_snapshot_still_owns_its_caller_inputs(recorded_support):
    arguments, metadata = recorded_support
    q = metadata['recorded_cases']['coarse_support_only_rejected']['q']
    fk, _, errors, normal, offset, gripper, opening = support_arguments(arguments, q)
    errors, normal = np.array(errors), np.array(normal)
    prepared = PreparedEndpointSupportBound(fk, errors, normal, offset, gripper, opening)
    expected = prepared(q)
    errors[:] *= .5
    normal[:] = [0., 0., 1.]
    fk.chain[0][0][0, 3] += .01
    gripper.finger_size_xyz_m.setflags(write=True)
    gripper.finger_size_xyz_m[0] += .01
    assert prepared(q) == expected
    prepared(q)['joint_error_bounds_rad'][0] = 0.
    assert prepared(q) == expected


@pytest.mark.parametrize('case_name,gate', [('arrival_rejected', 'arrival_error_box_visibility'),
                                         ('gain_rejected', 'missing_side_view_gain')])
def test_support_success_never_overrides_original_arrival_or_gain(recorded_support, monkeypatch, case_name, gate):
    arguments, metadata = recorded_support
    q = metadata['recorded_cases'][case_name]['q']
    assert prove(arguments, q)['ok']
    coarse = observation_endpoint_support_bound(*support_arguments(arguments, q))
    assert coarse['ok']
    monkeypatch.setattr(module, 'observation_following_support_bound',
        lambda *_a, **_kw: pytest.fail('a sufficient original coarse bound must remain unchanged'))
    rows, report = direct_endpoint(arguments, q, monkeypatch)
    assert not rows and report['candidate_count'] == 0
    checks = [row['endpoint_checks'] for row in report['directions'] if 'endpoint_checks' in row]
    assert checks
    assert all(not check[gate]['ok'] for check in checks)
    assert all(check['conditional_endpoint_support'] == coarse for check in checks)
    assert not report['execution_authority'] and not report['proves_complete_path']


def source_fingerprints(source):
    arrays = {key: source[key] for key in ('points_base', 'corners_base', 'center_base', 'tool_from_camera')}
    arrays.update({key: source['arrival_projection'][key] for key in ('prism_base', 'rgb_valid', 'invalid_integral')})
    return {key: hashlib.sha256(value.tobytes()).hexdigest() for key, value in arrays.items()}


def test_actual_generator_requires_full_same_box_proof_and_keeps_coarse_rejection(recorded_support, monkeypatch):
    arguments, metadata = recorded_support
    case = metadata['recorded_cases']['coarse_support_only_rejected']
    original = module.observation_following_support_bound
    proof_calls = []
    before = source_fingerprints(arguments['source'])

    def independent(*args, **kwargs):
        result = original(*args, **kwargs)
        proof_calls.append((deepcopy(result), np.array(args[2]), args[6]))
        return result

    monkeypatch.setattr(module, 'observation_following_support_bound', independent)
    rows, report = direct_endpoint(arguments, case['q'], monkeypatch)
    assert len(rows) == len(proof_calls) == 1
    row = rows[0]
    checks = row['endpoint_checks']
    assert all(value['ok'] for value in checks.values() if isinstance(value, dict) and 'ok' in value)
    refined = checks['conditional_endpoint_support']
    assert refined['coarse_endpoint_bound'] == case['endpoint_checks']['conditional_endpoint_support']
    assert not refined['coarse_endpoint_bound']['ok'] and refined['ok']
    assert refined['support_bound_method'] == 'existing_full_joint_error_box_after_inconclusive_coarse_bound'
    assert refined['minimum_support_clearance_lower_bound_m'] == proof_calls[0][0]['minimum_support_clearance_lower_bound_m']
    assert refined['required_support_clearance_m'] == .003
    np.testing.assert_array_equal(proof_calls[0][1], arguments['joint_error_bounds_rad'])
    assert proof_calls[0][2] == arguments['opening_width_m'] == .05
    np.testing.assert_array_equal(refined['goal_joint_positions_rad'], case['q'])
    for name in ('arrival_error_box_visibility', 'missing_side_view_gain', 'arrival_projection_endpoint',
                 'working_volume', 'measured_support_projection', 'observation_envelope', 'endpoint_joint_reserve'):
        assert checks[name] == case['endpoint_checks'][name]
    receipt = report['directions'][0]['support_bound_refinements'][0]
    assert receipt['proof_returned'] and receipt['checked_within_remaining_budget']
    assert receipt['coarse_endpoint_bound']['ok'] is False
    assert receipt['proof'] == proof_calls[0][0]
    assert source_fingerprints(arguments['source']) == before
    assert not row['execution_authority'] and not row['complete_path_checked']
    assert not row['whole_arm_collision_checked'] and row['actual_arrival_measurement_required']
    saved = deepcopy(refined)
    # Later caller changes must not rewrite the completed evidence object.
    arguments['joint_error_bounds_rad'][0] = 0.
    arguments['source']['geometry']['support_normal_base'][0] += .1
    arguments['source']['points_base'][0] += 1.
    assert refined == saved


def test_full_support_proof_cannot_replace_unobserved_arrival_rgb(recorded_support, monkeypatch):
    arguments, metadata = recorded_support
    case = metadata['recorded_cases']['coarse_support_only_rejected']
    q = case['q']
    assert prove(arguments, q)['ok']
    arrival = arguments['source']['arrival_projection']
    # The real reserved rectangle exceeds the nominal prism rectangle. Add a
    # hole in that extra required region, leaving nominal prism RGB intact.
    x, y, _, _ = case['endpoint_checks']['arrival_error_box_visibility']['required_observed_box_xyxy']
    x0, y0, x1, y1 = case['endpoint_checks']['arrival_projection_endpoint']['required_observed_box_xyxy']
    assert not (x0 <= x < x1 and y0 <= y < y1)
    assert arrival['rgb_valid'][y, x]
    arrival['rgb_valid'][y, x] = False
    integral = np.zeros((arrival['rgb_valid'].shape[0]+1, arrival['rgb_valid'].shape[1]+1), dtype=np.int64)
    integral[1:, 1:] = (~arrival['rgb_valid']).astype(np.int64).cumsum(axis=0).cumsum(axis=1)
    arrival['invalid_integral'] = integral
    for key in ('rgb_valid', 'invalid_integral'):
        arrival['audit'][key+'_sha256'] = hashlib.sha256(arrival[key].tobytes()).hexdigest()
    rows, report = direct_endpoint(arguments, q, monkeypatch)
    assert not rows
    checks = [row['endpoint_checks'] for row in report['directions'] if 'endpoint_checks' in row]
    assert checks
    for check in checks:
        assert check['conditional_endpoint_support']['ok']
        assert check['arrival_projection_endpoint']['ok']
        assert not check['arrival_error_box_visibility']['ok']
        assert check['arrival_error_box_visibility']['rectification_domain_observed'] is False


def test_model_collision_is_rejected_by_real_endpoint_path(recorded_support, monkeypatch):
    arguments, metadata = recorded_support
    q = metadata['recorded_cases']['coarse_support_only_rejected']['q']
    coarse = observation_endpoint_support_bound(*support_arguments(arguments, q))
    arguments['source']['geometry']['support_offset_m'] -= coarse['nominal_minimum_support_clearance_m']-.002
    rows, report = direct_endpoint(arguments, q, monkeypatch)
    assert not rows
    checks = [row['endpoint_checks'] for row in report['directions'] if 'endpoint_checks' in row]
    assert checks
    for check in checks:
        proof = check['conditional_endpoint_support']
        assert not proof['ok'] and not proof['coarse_endpoint_bound']['ok']
        assert proof['counterexample_model_clearance_m'] < .003
        assert proof['joint_error_bounds_rad'] == arguments['joint_error_bounds_rad']


def test_invalid_model_never_reaches_solver_or_proof(recorded_support, monkeypatch):
    arguments, _ = recorded_support
    arguments['fk'] = object()
    monkeypatch.setattr(module, '_backend', lambda: pytest.fail('unsupported model reached solver'))
    monkeypatch.setattr(module, 'observation_following_support_bound',
        lambda *_a, **_kw: pytest.fail('unsupported model reached proof'))
    rows, report = module.generate_reachable_observation_views(**arguments, clock=lambda: 0.)
    assert not rows and report['status'] == 'invalid_input'
    assert not report['execution_authority']
