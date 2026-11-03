"""Independent v2 family/enclosure checks through the real public qualifier.

These finite ray-scene checks are empirical contract tests, not a camera
calibration guarantee or evidence of a successful physical grasp.
"""
from copy import deepcopy
from itertools import product
from pathlib import Path
import hashlib
import json

import numpy as np
import pytest

from test_gen31_outer_rgb_box import visual_edge_gap_scene, qualify_visual_scene
from alicia_flexible_grasp.vision.regular_box_inference import (
    RegularBoxInference, enclose_inferred_box_geometry,
    inferred_box_enclosure_vertices, regular_box_geometry_sha256)
from alicia_flexible_grasp.vision.outer_rgb_rectangle import (
    _family_budget, validate_outer_rgb_interval)


@pytest.fixture(scope='module')
def qualified_pair():
    scene = visual_edge_gap_scene()
    result = qualify_visual_scene(scene)
    assert result.ok, (result.code, result.audit)
    assert result.model.schema_version == 2
    return scene, result.model


def rehash(evidence):
    evidence['model_sha256'] = hashlib.sha256(json.dumps(
        {k: v for k, v in evidence.items() if k != 'model_sha256'},
        sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    return evidence


def test_all_face_interval_corners_and_shared_plane_shifts_fit_collision_enclosure(qualified_pair):
    scene, model = qualified_pair
    evidence = model.qualification['outer_rgb_interval']
    faces = evidence['position_intervals']
    yaw_lo, yaw_hi = evidence['common_yaw_interval_rad']
    theta = evidence['nominal_yaw_rad']
    # Cross-product of independently bounded faces deliberately includes
    # infeasible combinations: containing this superset also contains the
    # coupled strip family. Sample the entire yaw interval, including ends.
    corners = np.array(list(product(
        [faces[i][bound] for i in (1, 3) for bound in ('minimum_m', 'maximum_m')],
        [faces[i][bound] for i in (0, 2) for bound in ('minimum_m', 'maximum_m')]
    )))
    collision = (inferred_box_enclosure_vertices(model)-model.center_base) @ model.axes_base
    half_xy = np.max(np.abs(collision[:, :2]), axis=0)
    camera_xy = (scene.T[:3, 3]-model.center_base) @ model.axes_base[:, :2]
    # The exact synthetic measured top is z=.025. A common parallel-plane
    # shift scales every ray intersection about the same camera position.
    distance = .025-scene.T[2, 3]
    slab = evidence['top_plane_slab_m']
    maximum = np.zeros(2)
    for yaw in np.linspace(yaw_lo, yaw_hi, 257):
        delta = yaw-theta
        rotation = np.array([[np.cos(delta), -np.sin(delta)],
                             [np.sin(delta), np.cos(delta)]])
        points = corners @ rotation.T
        for shift in (-slab, 0., slab):
            shifted = camera_xy+(points-camera_xy)*(1.+shift/distance)
            maximum = np.maximum(maximum, np.max(np.abs(shifted), axis=0))
    assert np.all(maximum <= half_xy+1e-10), (maximum, half_xy)
    # This test reaches nonzero two-axis face and shared-plane uncertainty.
    assert evidence['position_family_error_m'] > 0.
    assert evidence['common_plane_translation_m'] > 0.
    assert evidence['fixed_axis_position_error_m'] > evidence['position_family_error_m']


def test_planning_union_contains_v2_family_and_keeps_measured_data(qualified_pair):
    scene, model = qualified_pair
    measured = scene.geometry.object_points_base.copy()
    source_digest = regular_box_geometry_sha256(scene.geometry)
    planning, audit = enclose_inferred_box_geometry(scene.geometry, model)
    vertices = np.vstack((measured, inferred_box_enclosure_vertices(model)))
    local = (vertices-planning.center_base) @ planning.axes_base
    assert np.all(np.abs(local) <= planning.size_xyz_m/2.+1e-12)
    np.testing.assert_array_equal(planning.object_points_base, measured)
    np.testing.assert_array_equal(scene.geometry.object_points_base, measured)
    assert audit['measured_points_unchanged'] is True
    assert audit['measured_side_surfaces'] is False
    assert audit['model_sha256'] == model.model_sha256
    assert audit['policy'] == 'source_bound_measured_points_and_inferred_box_uncertainty'
    assert audit['source_geometry_sha256'] == source_digest == model.source['geometry_sha256']
    assert regular_box_geometry_sha256(scene.geometry) == source_digest
    assert audit['producer_verified_all_points'] is True
    assert audit['consumer_independent_measured_point_verification'] is False


@pytest.mark.parametrize('fault', [
    'missing_face', 'empty_yaw_intersection', 'nominal_outside_yaw',
    'omit_rotated_offset_coupling', 'omit_shared_plane_shift',
    'omit_pixel_sampling', 'unsupported_complete_edge',
    'unmeasured_exterior_count', 'certified_claim', 'incomplete_mask',
    'schema_downgrade', 'wrong_contour_source',
])
def test_rehashed_v2_evidence_cannot_drop_required_family_terms(qualified_pair, fault):
    _, model = qualified_pair
    evidence = deepcopy(model.to_evidence_dict())
    q = evidence['qualification']
    outer = q['outer_rgb_interval']
    if fault == 'missing_face':
        outer['position_intervals'].pop()
    elif fault == 'empty_yaw_intersection':
        outer['common_yaw_interval_rad'].reverse()
    elif fault == 'nominal_outside_yaw':
        outer['nominal_yaw_rad'] = outer['common_yaw_interval_rad'][1]+.001
    elif fault == 'omit_rotated_offset_coupling':
        outer['fixed_axis_position_error_m'] = outer['position_family_error_m']
    elif fault == 'omit_shared_plane_shift':
        evidence['empirical_edge_uncertainty_m'] -= outer['common_plane_translation_m']
        outer['empirical_edge_uncertainty_m'] = evidence['empirical_edge_uncertainty_m']
    elif fault == 'omit_pixel_sampling':
        outer['edge_evidence'][0]['strip_radius_m'] = outer['edge_evidence'][0]['maximum_line_residual_m']
    elif fault == 'unsupported_complete_edge':
        outer['full_edge_rgb_distances_m'][0] = .05
    elif fault == 'unmeasured_exterior_count':
        outer['exterior'][0]['valid'] = outer['exterior'][0]['pixels']+1
    elif fault == 'certified_claim':
        outer['certified_error_bound'] = True
    elif fault == 'incomplete_mask':
        outer['complete_mask_used'] = False
    elif fault == 'schema_downgrade':
        evidence['schema_version'] = 1
    elif fault == 'wrong_contour_source':
        q['contour_source'] = 'rgb_edges_on_measured_top_plane'
    with pytest.raises(ValueError):
        RegularBoxInference.from_evidence_dict(rehash(evidence))


def test_model_keeps_immutable_family_evidence_and_source_binding(qualified_pair):
    scene, model = qualified_pair
    original = model.to_evidence_dict()
    borrowed = model.qualification
    borrowed['outer_rgb_interval']['position_intervals'][0]['minimum_m'] -= .1
    assert model.to_evidence_dict() == original
    assert model.source['color_sha256'] == hashlib.sha256(scene.color.tobytes()).hexdigest()
    tampered = deepcopy(original)
    tampered['source']['color_sha256'] = '0'*64
    with pytest.raises(ValueError):
        RegularBoxInference.from_evidence_dict(tampered)


@pytest.mark.parametrize('second_angle', [-.075, .125])
def test_union_budget_contains_offset_rotated_larger_second_family(second_angle):
    # Numerical inputs to the pure projection helper, not qualified models or
    # fabricated image evidence. A shifted, differently sized second family
    # must survive reference selection and all of its own yaw uncertainty.
    members = [
        dict(center_offset_reference_m=[0., 0.], size_xy_m=[.061, .034],
             nominal_yaw_offset_rad=0., interval=dict(
                 empirical_edge_uncertainty_m=.0012,
                 empirical_yaw_uncertainty_rad=.018)),
        dict(center_offset_reference_m=[-.003, .002], size_xy_m=[.065, .037],
             nominal_yaw_offset_rad=second_angle, interval=dict(
                 empirical_edge_uncertainty_m=.0025,
                 empirical_yaw_uncertainty_rad=.035)),
    ]
    edge, yaw = _family_budget(members)
    assert yaw >= abs(second_angle)+.035
    reference = np.asarray(members[0]['size_xy_m'])
    # Evaluate support of the reference rectangle independently over its
    # full combined yaw. Include each analytical stationary angle as well.
    angles = list(np.linspace(-yaw, yaw, 257))
    for axis in range(2):
        critical = np.arctan2(reference[1-axis], reference[axis])
        angles += [a for a in (-critical, critical) if -yaw <= a <= yaw]
    reference_corners = np.asarray(list(product((-1., 1.), repeat=2)))*reference/2.
    base_half = np.zeros(2)
    for angle in angles:
        R = np.array([[np.cos(angle), -np.sin(angle)],
                      [np.sin(angle), np.cos(angle)]])
        base_half = np.maximum(base_half, np.max(np.abs(reference_corners @ R.T), axis=0))
    actual_maximum = np.zeros(2)
    for member in members:
        nominal = member['nominal_yaw_offset_rad']
        R = np.array([[np.cos(nominal), -np.sin(nominal)],
                      [np.sin(nominal), np.cos(nominal)]])
        corners = np.asarray(list(product((-1., 1.), repeat=2)))*np.asarray(member['size_xy_m'])/2.
        errors = np.asarray(list(product((-1., 1.), repeat=2)))*member['interval']['empirical_edge_uncertainty_m']
        spread = member['interval']['empirical_yaw_uncertainty_rad']
        for angle in np.linspace(-spread, spread, 65):
            turn = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
            points = ((corners @ turn.T)[:, None, :]+errors[None, :, :]).reshape(-1, 2)
            points = points @ R.T+member['center_offset_reference_m']
            actual_maximum = np.maximum(actual_maximum, np.max(np.abs(points), axis=0))
    assert np.all(actual_maximum <= base_half+edge+1e-12)
    # Dropping the second member's positional budget is observably unsafe,
    # even if its larger yaw was accidentally retained.
    assert np.any(actual_maximum > base_half+members[0]['interval']['empirical_edge_uncertainty_m']+.0005)


def actual_precision_interval():
    # Historical producer evidence is a precision-only candidate with no
    # grasp model. Parsing its interval never upgrades that qualification.
    path = Path(__file__).with_name('fixtures') / 'gen31_initial_precision_outer_interval.json'
    value = json.loads(path.read_text())
    assert value['grasp_qualified'] is False and value['model'] is None
    assert value['original_result_code'] == 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    return value


def test_actual_two_hypothesis_interval_retains_both_unqualified_families():
    value = actual_precision_interval()
    outer = value['outer_rgb_interval']
    members = outer['top_side_hypotheses']['members']
    assert len(members) == 2
    assert validate_outer_rgb_interval(outer, value['size_xyz_m'][:2],
        value['edge_uncertainty_m'], value['yaw_uncertainty_rad']) == outer
    assert any(abs(x) > .0001 for x in members[1]['center_offset_reference_m'])
    assert value['edge_uncertainty_m'] > members[0]['interval']['empirical_edge_uncertainty_m']


@pytest.mark.parametrize('fault', [
    'drop_member', 'reference_budget_only', 'member_empty_yaw',
    'member_missing_face', 'wrong_reference_center', 'discard_unresolved_side',
    'measured_side_as_top', 'recursive_member',
])
def test_actual_union_parser_rejects_omitted_or_false_member_proof(fault):
    value = actual_precision_interval()
    outer = value['outer_rgb_interval']
    family = outer['top_side_hypotheses']
    member = family['members'][1]
    if fault == 'drop_member':
        family['members'].pop()
    elif fault == 'reference_budget_only':
        value['edge_uncertainty_m'] = family['members'][0]['interval']['empirical_edge_uncertainty_m']
        outer['empirical_edge_uncertainty_m'] = value['edge_uncertainty_m']
    elif fault == 'member_empty_yaw':
        member['interval']['common_yaw_interval_rad'].reverse()
    elif fault == 'member_missing_face':
        member['interval']['position_intervals'].pop()
    elif fault == 'wrong_reference_center':
        family['members'][0]['center_offset_reference_m'][0] = .001
    elif fault == 'discard_unresolved_side':
        member['silhouette_side_resolved'][0] = not member['silhouette_side_resolved'][0]
    elif fault == 'measured_side_as_top':
        member['top_side_line_evidence'][0]['inside'] = dict(
            valid_fraction=1., top_fraction=0., above_fraction=0.)
    elif fault == 'recursive_member':
        member['interval']['top_side_hypotheses'] = {}
    with pytest.raises(ValueError):
        validate_outer_rgb_interval(outer, value['size_xyz_m'][:2],
            value['edge_uncertainty_m'], value['yaw_uncertainty_rad'])
