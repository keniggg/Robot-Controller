"""GEN58 recorded view and generic opposite-height information contracts.

These tests exercise view proposals only, never IK, visibility, contact or motion.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from alicia_grasp_modes.observe_return_visual import (
    directed_missing_side_rotations, distance_bound_missing_side_rotation,
    missing_side_observation_span, missing_side_view_gain,
)


def task_scale_gap(*, width=.03, height=.02, sign=-1, scale=1., transformed=False):
    """Independent measured face spans; neither is a claimed common band."""
    R = rotation() if transformed else np.eye(3)
    shift = np.array([.37, -.22, .09]) if transformed else np.zeros(3)
    gap = dict(failure_code='BILATERAL_SURFACE_EVIDENCE_MISSING', angle_deg=0.,
        side_measurement_basis='before_common_support_intersection',
        missing_side_policy='insufficient_samples_or_weakest_independent_height',
        minimum_points_per_side=12, negative_measured_points=50, positive_measured_points=60,
        negative_measured_height_m=(.001 if sign == -1 else .004)*scale,
        positive_measured_height_m=(.004 if sign == -1 else .001)*scale,
        negative_support_height_bounds_m=([.008, .009] if sign == -1 else [0., .004]),
        positive_support_height_bounds_m=([0., .004] if sign == -1 else [.008, .009]),
        object_height_m=height*scale, whole_obb_width_m=width*scale,
        target_center_base_m=shift.tolist(), jaw_axis_base=(R @ [1., 0., 0.]).tolist(),
        initial_camera_origin_base_m=(shift+scale*(R @ [0., 0., .2])).tolist(),
        missing_side_signs=[sign], target_epoch=4, target_track_id='synthetic-generic',
        source_view_stamps_ns=[30], contact_or_motion_authorized=False)
    for side in ('negative', 'positive'):
        gap[side+'_support_height_bounds_m'] = [v*scale for v in gap[side+'_support_height_bounds_m']]
    camera = shift+scale*(R @ np.array([sign*.12, 0., .16]))
    return gap, camera


FIXTURE = Path(__file__).parent / 'fixtures/gen58_observation_shared_band_gain.json'
FIXTURE_SHA = 'e4e68afa607469334f63c7ac8859ae970ebec8a6e739edc2b6c9291331201711'


def recorded():
    raw = FIXTURE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == FIXTURE_SHA
    return json.loads(raw)


def selected_gap(data):
    angle = data['selected_original_gain']['angle_deg']
    return next(g for g in data['original_gaps'] if g['angle_deg'] == angle)


def no_authority(audit):
    assert audit['contact_or_motion_authorized'] is False
    assert audit['measured_registration_required'] is True
    assert audit.get('common_contact_band_predicted', False) is False


@pytest.mark.parametrize('width,height,sign,scale,transformed,expected', [
    (.014, .003, -1, 1., False, False),
    (.014, .004, -1, 1., False, True),
    (.032, .014, 1, 1., True, True),
    (.048, .08, -1, 1., True, True),
    (.03, .02, 1, .5, False, True),
    (.03, .02, -1, 2., True, True),
])
def test_task_target_generalizes_without_replacing_noise_or_opposite_span(
        width, height, sign, scale, transformed, expected):
    gap, camera = task_scale_gap(width=width, height=height, sign=sign,
        scale=scale, transformed=transformed)
    before = deepcopy(gap)
    legacy = missing_side_view_gain([gap], camera, .003*scale)
    explicit = missing_side_view_gain([gap], camera, .003*scale, predicted_gain_target_m=.002*scale)
    assert not legacy['ok'] and not explicit['ok']
    # This sideward camera exposes a band above the same unchanged noise.
    R = rotation() if transformed else np.eye(3)
    shift = np.asarray(gap['target_center_base_m'])
    sideward = shift+scale*(R @ np.array([sign*.16,0.,.04]))
    explicit = missing_side_view_gain([gap],sideward,.003*scale,predicted_gain_target_m=.002*scale)
    assert explicit['ok'] is expected
    if expected:
        assert explicit['predicted_new_side_gain_m'] > .003*scale
        assert explicit['observation_span_m'] == pytest.approx(.004*scale)
        assert explicit['depth_uncertainty_m'] == pytest.approx(.003*scale)
        assert explicit['predicted_gain_target_m'] == pytest.approx(.002*scale)
        assert explicit['effective_gain_target_m'] == pytest.approx(.003*scale)
        assert explicit['predicted_gain_policy'] == 'source_contact_overlap_prediction_v1'
        assert explicit['prediction_only'] is True
        assert explicit['certifies_new_view_perception'] is False
    no_authority(explicit)
    assert gap == before


@pytest.mark.parametrize('target', [.0024, .003, .004])
def test_equal_or_higher_task_target_still_requires_strict_excess_gain(target):
    gap, camera = task_scale_gap()
    assert not missing_side_view_gain([gap], camera, .003, predicted_gain_target_m=target)['ok']


@pytest.mark.parametrize('target', [0., -.001, float('nan'), float('inf'), True, False, '.002'])
def test_explicit_invalid_prediction_target_is_not_a_zero_threshold(target):
    gap, camera = task_scale_gap()
    result = missing_side_view_gain([gap], camera, .003, predicted_gain_target_m=target)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'
    no_authority(result)


@pytest.mark.parametrize('fault', ['opposite_too_short', 'same_direction', 'inside_face', 'noise_invalid'])
def test_task_target_preserves_retained_span_direction_face_and_input_gates(fault):
    gap, camera = task_scale_gap()
    noise = .003
    if fault == 'opposite_too_short':
        gap['positive_measured_height_m'] = .0019
        gap['positive_support_height_bounds_m'] = [0., .0019]
    elif fault == 'same_direction': gap['initial_camera_origin_base_m'] = (camera*.8).tolist()
    elif fault == 'inside_face': camera = np.array([-.01, 0., .001])
    else: noise = float('nan')
    result = missing_side_view_gain([gap], camera, noise, predicted_gain_target_m=.002)
    assert not result['ok']; no_authority(result)


@pytest.mark.parametrize('depth,focal', [(.15, 300.), (.22, 610.), (.4, 900.)])
def test_task_utility_labels_target_separately_and_keeps_measured_noise(depth, focal):
    from alicia_grasp_modes.observation_view_utility import predicted_information_utility
    original = predicted_information_utility(.0024, .003, depth, focal)
    selected = predicted_information_utility(.0024, .003, depth, focal, predicted_gain_target_m=.002)
    assert original['utility'] == selected['utility'] == 0.
    assert selected['effective_gain_target_m'] == .003
    assert original['policy'] == 'predicted_resolvable_information_per_motion_v1'
    assert selected['policy'] != original['policy']
    assert selected['noise_px'] == original['noise_px'] == pytest.approx(.003*focal/depth)
    assert selected['predicted_gain_target_px'] == pytest.approx(.002*focal/depth)
    assert selected['sampling_uncertainty_px'] == original['sampling_uncertainty_px'] == 1.
    assert selected['certifies_new_view_perception'] is False
    assert selected['prediction_only'] and selected['measured_registration_required']
    assert not selected['execution_authority'] and not selected['probability_of_success']
    assert predicted_information_utility(.0035, .003, depth, focal, predicted_gain_target_m=.004)['utility'] == 0.


@pytest.mark.parametrize('pose_key', ['planned_T_base_camera_link', 'actual_later_T_base_camera_link'])
def test_recorded_selected_view_has_insufficient_opposite_band_projection(pose_key):
    data = recorded(); before = deepcopy(data)
    camera = np.asarray(data[pose_key])[:3, 3]
    gap = selected_gap(data)
    span, evidence = missing_side_observation_span(gap, -1)
    assert span == gap['positive_measured_height_m']
    assert span == pytest.approx(.008696457249972032)
    assert evidence['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
    assert evidence['opposite_measured_points'] == 102
    no_authority(evidence)
    delta = camera - gap['target_center_base_m']
    projection = -np.asarray(gap['jaw_axis_base']) @ delta / np.linalg.norm(delta)
    expected = .0026086234298861163 if pose_key.startswith('planned') else .0026122994640402496
    assert span * projection == pytest.approx(expected, abs=1e-14)
    assert data['selected_original_gain']['ok'] is True
    assert data['selected_original_gain']['predicted_new_side_gain_m'] == pytest.approx(.006816122478480197)
    assert not missing_side_view_gain(data['original_gaps'], camera, .003)['ok']
    assert data == before


@pytest.mark.parametrize('index', range(6))
def test_more_lateral_record_bound_origins_pass_information_only_at_original_range(index):
    data = recorded(); alternative = data['hypothetical_origins'][index]
    camera = np.asarray(alternative['camera_origin_base_m'])
    actual_distance = np.linalg.norm(camera - data['planned_target_base_xyz'])
    assert actual_distance == pytest.approx(alternative['distance_m'], abs=1e-14)
    assert .18 - 1e-14 <= actual_distance <= .22 + 1e-14
    # Each origin keeps the retained target center for the information test;
    # it is not silently recentered to the later construction target.
    result = missing_side_view_gain([selected_gap(data)], camera, .003)
    assert result['ok'] and result['predicted_new_side_gain_m'] > .003
    assert result['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
    no_authority(result)
    assert alternative['hypothetical_only'] and not alternative['motion_feasibility_checked']


def rotation():
    axis = np.asarray([1., -2., 3.]); axis /= np.linalg.norm(axis)
    x, y, z = axis; skew = np.asarray([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    angle = .91
    return np.eye(3) + np.sin(angle)*skew + (1.-np.cos(angle))*(skew @ skew)


@pytest.mark.parametrize('scale', [.5, 1., 1.7])
@pytest.mark.parametrize('transformed_frame', [False, True])
def test_gain_is_covariant_across_size_frame_and_translation(scale, transformed_frame):
    data = recorded(); gap = deepcopy(selected_gap(data))
    camera = np.asarray(data['hypothetical_origins'][3]['camera_origin_base_m'])
    expected = missing_side_view_gain([gap], camera, .003)
    R = rotation() if transformed_frame else np.eye(3)
    shift = np.asarray([.37, -.22, .09])
    for key in ('target_center_base_m', 'initial_camera_origin_base_m'):
        gap[key] = (scale*R @ np.asarray(gap[key]) + shift).tolist()
    gap['jaw_axis_base'] = (R @ gap['jaw_axis_base']).tolist()
    for key in ('object_height_m', 'whole_obb_width_m', 'negative_measured_height_m', 'positive_measured_height_m'):
        gap[key] *= scale
    transformed = scale*R @ camera + shift
    result = missing_side_view_gain([gap], transformed, .003*scale)
    assert result['ok'] == expected['ok']
    assert result['predicted_new_side_gain_m'] == pytest.approx(expected['predicted_new_side_gain_m']*scale, abs=1e-13)
    assert result['observation_span_m'] == pytest.approx(expected['observation_span_m']*scale)
    no_authority(result)


def test_noise_increase_and_no_direction_change_do_not_create_gain():
    data = recorded(); gaps = data['original_gaps']; gap = selected_gap(data)
    camera = np.asarray(data['hypothetical_origins'][0]['camera_origin_base_m'])
    assert missing_side_view_gain([gap], camera, .003)['ok']
    assert not missing_side_view_gain([gap], camera, .005)['ok']
    center = np.asarray(gap['target_center_base_m']); initial = np.asarray(gap['initial_camera_origin_base_m'])
    for factor in (.6, 1., 1.4):
        assert not missing_side_view_gain(gaps, center+factor*(initial-center), .003)['ok']


@pytest.mark.parametrize('field,value', [
    ('positive_measured_points', True), ('positive_measured_points', -1),
    ('positive_measured_points', 12.5), ('positive_measured_points', '102'),
    ('minimum_points_per_side', 11), ('minimum_points_per_side', 12.),
    ('positive_measured_height_m', True), ('positive_measured_height_m', '0.009'),
    ('positive_measured_height_m', -.001), ('positive_measured_height_m', float('nan')),
    ('positive_measured_height_m', float('inf')),
])
def test_malformed_known_opposite_measurements_cannot_fall_back_to_full_height(field, value):
    data = recorded(); gap = deepcopy(selected_gap(data)); gap[field] = value
    with pytest.raises((ValueError, KeyError)):
        missing_side_observation_span(gap, -1)
    camera = data['hypothetical_origins'][3]['camera_origin_base_m']
    result = missing_side_view_gain([gap], camera, .003)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


def test_missing_known_measurement_is_not_invented():
    data = recorded(); gap = deepcopy(selected_gap(data)); del gap['positive_measured_height_m']
    result = missing_side_view_gain([gap], data['hypothetical_origins'][3]['camera_origin_base_m'], .003)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


@pytest.mark.parametrize('camera,noise', [([float('nan'), 0., 1.], .003), ([0., 1.], .003),
                                       ([0., 0., 1.], -.003), ([0., 0., 1.], float('nan'))])
def test_invalid_camera_or_noise_remains_invalid(camera, noise):
    assert not missing_side_view_gain(recorded()['original_gaps'], camera, noise)['ok']


@pytest.mark.parametrize('field,value', [('positive_measured_points', 0), ('positive_measured_points', 11),
                                       ('positive_measured_height_m', 0.)])
def test_unresolved_opposite_side_preserves_exploration_without_common_contact_claim(field, value):
    gap = deepcopy(selected_gap(recorded())); gap[field] = value
    span, audit = missing_side_observation_span(gap, -1)
    assert span == gap['object_height_m']
    assert audit['observation_span_policy'] == 'first_side_exploration_opposite_unresolved'
    no_authority(audit)


def test_top_only_two_sides_are_exploration_not_fabricated_common_support():
    gap = deepcopy(selected_gap(recorded())); gap['missing_side_signs'] = [-1, 1]
    for side in ('negative', 'positive'):
        gap[side+'_measured_points'] = 0; gap[side+'_measured_height_m'] = 0.
    for sign in (-1, 1):
        span, audit = missing_side_observation_span(gap, sign)
        assert span == gap['object_height_m']
        assert audit['observation_span_policy'] == 'first_side_exploration_opposite_unresolved'
        no_authority(audit)


@pytest.mark.parametrize('kind', ['unqualified_box_depth_and_edge_precision', 'chosen_box_face_precision', 'silhouette_only'])
def test_non_measured_precision_and_silhouette_contracts_remain_unpaired(kind):
    gap = deepcopy(selected_gap(recorded())); del gap['side_measurement_basis']
    gap['view_deficit_kind'] = kind
    del gap['minimum_points_per_side']
    del gap['missing_side_policy']
    for side in ('negative', 'positive'):
        del gap[side+'_measured_points']; del gap[side+'_measured_height_m']
    span, audit = missing_side_observation_span(gap, -1)
    assert span == gap['object_height_m']
    assert audit['observation_span_policy'] == 'whole_height_unpaired_observation'
    no_authority(audit)


def test_opposite_span_never_expands_beyond_object_height():
    gap = deepcopy(selected_gap(recorded())); gap['positive_measured_height_m'] = 2.*gap['object_height_m']
    span, audit = missing_side_observation_span(gap, -1)
    assert span == gap['object_height_m']
    assert audit['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
    no_authority(audit)


@pytest.mark.parametrize('distance', [.18, .20, .22])
def test_directed_generation_and_distance_binding_use_the_same_capped_gain(distance):
    data = recorded(); gap = deepcopy(selected_gap(data)); before = deepcopy(gap)
    center = np.asarray(gap['target_center_base_m']); origin = np.asarray(gap['initial_camera_origin_base_m'])
    normal = np.asarray(data['derived_gap_jaw_plane_normal_base'])
    optical = center-origin; optical /= np.linalg.norm(optical)
    second = np.cross(normal, optical); second /= np.linalg.norm(second)
    camera_rotation = np.column_stack((optical, second, np.cross(optical, second)))
    views = directed_missing_side_rotations([gap], camera_rotation, normal, .003, distance,
                                          allow_geometry_directed_expansion=True)
    assert views
    for proposed, audit in views:
        proposed_origin = center-distance*proposed[:, 0]
        checked = missing_side_view_gain([gap], proposed_origin, .003)
        assert checked['ok']
        assert audit['predicted_new_side_gain_m'] == pytest.approx(checked['predicted_new_side_gain_m'], abs=1e-12)
        assert checked['observation_span_m'] == gap['positive_measured_height_m']
        no_authority(checked)
        unchanged, retained = distance_bound_missing_side_rotation([gap], proposed, audit,
            center, distance, .003, normal)
        assert np.array_equal(unchanged, proposed) and retained is audit
    assert gap == before


@pytest.mark.parametrize('mutation', [
    'missing_basis', 'unknown_basis', 'null_basis',
    'negative_measured_points', 'negative_measured_height_m',
    'positive_measured_points', 'positive_measured_height_m',
    'minimum_points_per_side', 'minimum_only_without_basis',
])
def test_partial_or_unknown_measurement_contract_never_falls_back(mutation):
    data = recorded(); gap = deepcopy(selected_gap(data))
    if mutation == 'missing_basis':
        del gap['side_measurement_basis']
    elif mutation in ('unknown_basis', 'null_basis'):
        gap['side_measurement_basis'] = 'unrecognized_measurement_frame' if mutation == 'unknown_basis' else None
    elif mutation == 'minimum_only_without_basis':
        del gap['side_measurement_basis']
        for side in ('negative', 'positive'):
            del gap[side+'_measured_points']; del gap[side+'_measured_height_m']
    else:
        del gap[mutation]
    with pytest.raises(ValueError):
        missing_side_observation_span(gap, -1)
    result = missing_side_view_gain([gap], data['hypothetical_origins'][3]['camera_origin_base_m'], .003)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


@pytest.mark.parametrize('lower,use_tuple', [(-.02, False), (0., False), (.013, True)])
def test_matching_optional_opposite_support_bounds_preserve_gain(lower, use_tuple):
    data = recorded(); gap = deepcopy(selected_gap(data))
    camera = data['hypothetical_origins'][3]['camera_origin_base_m']
    expected = missing_side_view_gain([gap], camera, .003)
    bounds = [lower, lower+gap['positive_measured_height_m']]
    gap['positive_support_height_bounds_m'] = tuple(bounds) if use_tuple else bounds
    span, audit = missing_side_observation_span(gap, -1)
    assert span == gap['positive_measured_height_m']
    assert audit['opposite_support_height_bounds_m'] == bounds
    actual = missing_side_view_gain([gap], camera, .003)
    assert actual['ok'] == expected['ok']
    assert actual['predicted_new_side_gain_m'] == pytest.approx(expected['predicted_new_side_gain_m'])
    no_authority(audit); no_authority(actual)


@pytest.mark.parametrize('bounds', [
    [0., .02], [.02, 0.], [float('nan'), .01], [0., float('inf')],
    [0.], [0., .01, .02], [False, .01], ['0', .01], '0,0.01',
])
def test_invalid_optional_opposite_support_bounds_cannot_authorize_gain(bounds):
    data = recorded(); gap = deepcopy(selected_gap(data))
    gap['positive_support_height_bounds_m'] = bounds
    with pytest.raises(ValueError):
        missing_side_observation_span(gap, -1)
    result = missing_side_view_gain([gap], data['hypothetical_origins'][3]['camera_origin_base_m'], .003)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


def legacy_common_gap():
    # Synthetic closed old schema: its samples describe common-height
    # statistics and must never be relabelled as independent face spans.
    gap = deepcopy(selected_gap(recorded()))
    del gap['side_measurement_basis']
    gap['missing_side_policy'] = 'insufficient_samples_or_weakest_common_height'
    return gap


@pytest.mark.parametrize('sign', [-1, 1])
def test_explicit_complete_legacy_common_schema_remains_unpaired(sign):
    gap = legacy_common_gap(); before = deepcopy(gap)
    span, audit = missing_side_observation_span(gap, sign)
    assert span == gap['object_height_m']
    assert audit['observation_span_policy'] == 'legacy_common_height_unpaired_observation'
    assert audit['legacy_measurement_basis'] == 'common_height_statistics_not_independent_face_span'
    assert 'opposite_measured_span_m' not in audit
    no_authority(audit)
    assert gap == before


@pytest.mark.parametrize('field,value', [
    ('negative_measured_points', None), ('positive_measured_points', None),
    ('negative_measured_height_m', None), ('positive_measured_height_m', None),
    ('minimum_points_per_side', None),
    ('negative_measured_points', -1), ('positive_measured_points', True),
    ('negative_measured_height_m', float('nan')), ('positive_measured_height_m', float('inf')),
    ('minimum_points_per_side', True), ('missing_side_policy', None),
    ('missing_side_policy', 'unknown_policy'),
    ('missing_side_policy', 'insufficient_samples_or_weakest_independent_height'),
    ('side_measurement_basis', 'unknown_basis'),
])
def test_legacy_schema_requires_complete_valid_both_side_statistics_and_exact_policy(field, value):
    data = recorded(); gap = legacy_common_gap()
    if value is None:
        del gap[field]
    else:
        gap[field] = value
    with pytest.raises(ValueError):
        missing_side_observation_span(gap, -1)
    result = missing_side_view_gain([gap], data['hypothetical_origins'][3]['camera_origin_base_m'], .003)
    assert not result['ok'] and result['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


@pytest.mark.parametrize('field,value', [
    ('negative_support_height_bounds_m', [0., .001]),
    ('positive_support_height_bounds_m', None),
    ('support_height_reference', 'current_independent_reference'),
    ('side_measurement_basis', None),
])
def test_partial_independent_metadata_cannot_downgrade_into_legacy(field, value):
    gap = legacy_common_gap(); gap[field] = value
    with pytest.raises(ValueError):
        missing_side_observation_span(gap, -1)
    no_gain = missing_side_view_gain([gap], recorded()['hypothetical_origins'][3]['camera_origin_base_m'], .003)
    assert not no_gain['ok'] and no_gain['code'] == 'OBSERVE_RETURN_VIEW_GAIN_INPUT_INVALID'


def test_valid_current_measurement_basis_wins_over_legacy_policy():
    data = recorded(); gap = deepcopy(selected_gap(data))
    gap['missing_side_policy'] = 'insufficient_samples_or_weakest_common_height'
    span, audit = missing_side_observation_span(gap, -1)
    assert span == gap['positive_measured_height_m'] < gap['object_height_m']
    assert audit['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
    assert 'legacy_measurement_basis' not in audit
    no_authority(audit)
    camera = np.asarray(data['planned_T_base_camera_link'])[:3, 3]
    assert not missing_side_view_gain([gap], camera, .003)['ok']


@pytest.mark.parametrize('field,value', [
    ('negative_support_height_bounds_m', None), ('positive_support_height_bounds_m', [0., .01]),
    ('support_height_reference', 'only_reference'), ('side_measurement_basis', 'unknown_basis'),
    ('missing_side_policy', 'insufficient_samples_or_weakest_common_height'),
    ('missing_side_policy', 'insufficient_samples_or_weakest_independent_height'),
])
def test_measurement_marker_without_payload_never_becomes_untyped_geometry(field, value):
    gap = deepcopy(selected_gap(recorded()))
    for key in ('side_measurement_basis', 'missing_side_policy', 'minimum_points_per_side',
                'negative_measured_points', 'positive_measured_points',
                'negative_measured_height_m', 'positive_measured_height_m'):
        del gap[key]
    gap[field] = value
    with pytest.raises(ValueError):
        missing_side_observation_span(gap, -1)


def test_original_gen14_probe_fails_retained_opposite_band_without_relaxing_gates():
    """Keep the original GEN14 data: its old feasible pose lacks new side gain."""
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    root = Path(__file__).resolve().parents[3]
    folder = Path(__file__).parent / 'fixtures'
    metadata_path = folder / 'gen14_fk_source.json'
    assert hashlib.sha256(metadata_path.read_bytes()).hexdigest() == '24793d14f0d175ef021a232d3bc44aac56d5337f02b2a9c082bfb6b47a18fb42'
    metadata = json.loads(metadata_path.read_text())
    arrays_path = folder / 'gen14_fk_source.npz'
    assert hashlib.sha256(arrays_path.read_bytes()).hexdigest() == metadata['arrays_sha256']
    with np.load(str(arrays_path)) as arrays:
        mount = arrays['tool_from_camera'].copy()
    fk = SerialUrdfFk((root / metadata['model_relative_path']).read_text(),
                      ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == metadata['model_sha256']
    original_probe = np.array([-1.9662889215842203, -.541789220533962,
        1.1962718646211747, -.012362442272550192, -1.4279141953957244, .13385894906599669])
    camera = (fk(original_probe) @ mount)[:3, 3]
    old_gains, bounded_gains = [], []
    for gap in metadata['gaps']:
        assert gap['side_measurement_basis'] == 'before_common_support_intersection'
        center, jaw = np.asarray(gap['target_center_base_m']), np.asarray(gap['jaw_axis_base'])
        initial = np.asarray(gap['initial_camera_origin_base_m']) - center
        proposed = camera - center
        for sign in gap['missing_side_signs']:
            span, audit = missing_side_observation_span(gap, sign)
            assert audit['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
            no_authority(audit)
            delta = max(0., sign * float(jaw @ proposed) / np.linalg.norm(proposed)) - max(
                0., sign * float(jaw @ initial) / np.linalg.norm(initial))
            old_gains.append(gap['object_height_m'] * delta)
            bounded_gains.append(span * delta)
    assert max(old_gains) > .003
    assert max(bounded_gains) < .003
    result = missing_side_view_gain(metadata['gaps'], camera, .003)
    assert not result['ok']
    assert result['code'] == 'OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED'
    no_authority(result)
