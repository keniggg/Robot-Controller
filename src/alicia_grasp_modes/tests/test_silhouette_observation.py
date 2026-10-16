"""An ambiguity may propose a new view, never certify a contact or erase points."""
from copy import deepcopy
from functools import lru_cache
import math

import numpy as np
import pytest

from alicia_grasp_modes.silhouette_ambiguity import evaluate_silhouette_ambiguity
from alicia_grasp_modes.silhouette_observation import (
    validate_silhouette_ambiguity, silhouette_view_gain, directed_silhouette_rotations)
from test_silhouette_ambiguity import true_multicolour_object


@lru_cache(maxsize=1)
def _fixture():
    inputs, options, _, _ = true_multicolour_object()
    measurement = evaluate_silhouette_ambiguity(*inputs, **options)
    stamp = measurement['source_stamp_ns']
    rotation = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
    result = dict(selection=dict(mode='unknown', strategy='observe_return', generation=4, stamp_ns=1),
        request_id=10, generation=8, target_epoch=2, target_track_id='g8-t2',
        snapshot_stamp_ns=stamp, source_stamp_ns=stamp, baseline_mask_reproduced=True,
        measurement=measurement, view_geometry=dict(target_center_base_m=[0.,0.,.01],
            ambiguity_anchor_base_m=[.022,0.,.006], support_normal_base=[0.,0.,1.],
            support_offset_m=0., initial_camera_origin_base_m=[0.,0.,.31],
            initial_camera_rotation_base=rotation.tolist(), depth_uncertainty_m=.003,
            source_frame_id='camera_link', target_height_m=.02))
    result.update(ok=True, diagnostic_only=True, execution_authority=False,
        observation_authority=False, code='SILHOUETTE_SOURCE_DIAGNOSTIC_READY',
        source_kind='original_single_frame_of_fused_snapshot',
        geometry_used_only_as_request_context=True, snapshot_source_stamps_ns=[stamp],
        plane_provenance=dict(kind='same_source_perception_verified_model',
            verification_stamp_ns=stamp, anchor_source_stamp_ns=stamp),
        raw_source_evidence={name: dict(sha256='a'*64, dtype=dtype, shape=shape) for name,dtype,shape in (
            ('rgb','uint8',[240,320,3]), ('depth','uint16',[240,320]), ('published_mask','uint8',[240,320]))},
        crop_xyxy=[112,80,198,151], frame_id='camera_link',
        projection_evidence=dict(intrinsics=[300.,300.,160.,120.],depth_scale=.0001,transform_sha256='b'*64))
    result['view_geometry'].update(camera_rotation_convention='ros_camera_link',
        target_geometry_source='original_fused_request_collision_geometry',
        target_geometry_source_stamps_ns=[stamp], ambiguity_anchor_source_stamp_ns=stamp,
        ambiguity_anchor_source='mean_of_current_original_valid_depth_disagreement_rays')
    return result


def silhouette_evidence():
    return deepcopy(_fixture())


def validate(e):
    reference = silhouette_evidence()
    return validate_silhouette_ambiguity(e, **{k:reference[k] for k in (
        'selection','request_id','generation','target_epoch','snapshot_stamp_ns','target_track_id')})


def test_current_bound_evidence_returns_independent_copy_and_no_certification():
    evidence = silhouette_evidence(); result = validate(evidence)
    assert result == evidence
    result['view_geometry']['target_center_base_m'][0] = 1.
    assert evidence['view_geometry']['target_center_base_m'][0] == 0.
    assert not result['measurement']['execution_authority']
    assert not result['measurement']['physical_contour_certified']


@pytest.mark.parametrize('field', ['request_id','generation','target_epoch','target_track_id',
                                  'snapshot_stamp_ns','source_stamp_ns','selection'])
def test_old_or_changed_request_cannot_authorize_an_observation(field):
    evidence = silhouette_evidence()
    if field == 'selection': evidence[field]['generation'] += 1
    elif isinstance(evidence[field], str): evidence[field] += '-different'
    else: evidence[field] += 1
    with pytest.raises(ValueError): validate(evidence)


@pytest.mark.parametrize('fault', ['small','few_depths','hard_labels','preferred','certificate',
                                  'mask_mismatch','missing_hash','malformed_hash','component','wrong_frame_stamp'])
def test_incomplete_or_speculative_evidence_does_not_enter_observation(fault):
    e = silhouette_evidence(); m = e['measurement']; d = m['disagreement']
    if fault == 'small': d['fraction_of_union'] = .001
    elif fault == 'few_depths': d['valid_depth_pixels'] = 24
    elif fault == 'hard_labels': d['hard_labels_touched'] = True
    elif fault == 'preferred': m['preferred_hypothesis'] = 'counterfactual'
    elif fault == 'certificate': m['physical_contour_certified'] = True
    elif fault == 'mask_mismatch': e['baseline_mask_reproduced'] = False
    elif fault == 'missing_hash': del m['input_evidence']['depth']
    elif fault == 'malformed_hash': m['input_evidence']['depth']['sha256'] = 'abc'
    elif fault == 'component': m['counterfactual']['core_component_unambiguous'] = False
    elif fault == 'wrong_frame_stamp': m['source_stamps_ns']['depth'] += 1
    with pytest.raises(ValueError): validate(e)


def camera(e, position):
    t = np.eye(4); t[:3,:3] = e['view_geometry']['initial_camera_rotation_base']
    t[:3,3] = position
    return t


def test_radial_approach_roll_and_move_to_opposite_edge_have_no_gain():
    e = silhouette_evidence()
    for p in ([0.,0.,.27], [0.,0.,.31], [-.1,0.,.25]):
        t = camera(e,p)
        assert not silhouette_view_gain(e,t,.003)['ok']
        t[:3,:3] = t[:3,:3]@np.diag([1.,-1.,-1.])
        assert not silhouette_view_gain(e,t,.003)['ok']


def test_same_camera_ray_at_another_distance_is_not_new_information():
    e = silhouette_evidence(); e['view_geometry']['initial_camera_origin_base_m'] = [.1,0.,.31]
    center = np.asarray(e['view_geometry']['target_center_base_m'])
    origin = np.asarray(e['view_geometry']['initial_camera_origin_base_m'])
    assert not silhouette_view_gain(e,camera(e,center+.7*(origin-center)),.003)['ok']


@pytest.mark.parametrize('yaw', [0.,67.,-134.])
def test_measured_edge_proposals_are_equivariant_bounded_and_only_predictions(yaw):
    e = silhouette_evidence(); g = e['view_geometry']; angle = math.radians(yaw)
    r = np.array([[math.cos(angle),-math.sin(angle),0.],
                  [math.sin(angle), math.cos(angle),0.],[0.,0.,1.]])
    for key in ('target_center_base_m','ambiguity_anchor_base_m','initial_camera_origin_base_m','support_normal_base'):
        g[key] = (r@np.asarray(g[key])).tolist()
    rotation = r@np.asarray(g['initial_camera_rotation_base'])
    g['initial_camera_rotation_base'] = rotation.tolist()
    result = directed_silhouette_rotations(e,rotation,g['support_normal_base'],.003,.22)
    assert 1 <= len(result) <= 2
    for proposed, metadata in result:
        np.testing.assert_allclose(proposed.T@proposed,np.eye(3),atol=1e-10)
        assert 0. < metadata['camera_rotation_offset_deg'] <= 45.
        t = np.eye(4);t[:3,:3]=proposed
        t[:3,3]=np.asarray(g['target_center_base_m'])-.22*proposed[:,0]
        gain = silhouette_view_gain(e,t,.003)
        assert gain['ok'] and gain['predicted_gain_m'] > .003
        assert gain['prediction_only'] and gain['diagnostic_observation_only']
        assert not gain['contact_or_motion_authorized']
        assert not gain['physical_contour_certified']
        assert not metadata['global_path_optimality_claimed']


def test_too_small_information_gain_yields_no_proposal_instead_of_relaxation():
    e = silhouette_evidence(); g = e['view_geometry']; g['target_height_m'] = .002
    assert not directed_silhouette_rotations(e,g['initial_camera_rotation_base'],g['support_normal_base'],.003,.22)


@pytest.mark.parametrize('fault', ['nan','nonrigid','zero_radius','bad_normal','zero_height'])
def test_invalid_scene_cannot_create_view(fault):
    e = silhouette_evidence();g=e['view_geometry']
    if fault=='nan':g['depth_uncertainty_m']=float('nan')
    elif fault=='nonrigid':g['initial_camera_rotation_base'][0][0]=3.
    elif fault=='zero_radius':g['ambiguity_anchor_base_m']=g['target_center_base_m']
    elif fault=='bad_normal':g['support_normal_base']=[0.,0.,2.]
    elif fault=='zero_height':g['target_height_m']=0.
    with pytest.raises(ValueError):validate(e)
    assert not directed_silhouette_rotations(e,g['initial_camera_rotation_base'],g['support_normal_base'],.003,.22)


@pytest.mark.parametrize('bad', [float('nan'),float('inf'),1.01,-1.])
def test_invalid_hard_core_fraction_is_rejected(bad):
    e=silhouette_evidence();e['measurement']['original']['hard_foreground_retained_fraction']=bad
    with pytest.raises(ValueError):validate(e)


@pytest.mark.parametrize('maximum', [1,2])
def test_explicit_proposal_budget_is_observed(maximum):
    e=silhouette_evidence();g=e['view_geometry']
    result=directed_silhouette_rotations(e,g['initial_camera_rotation_base'],g['support_normal_base'],.003,.22,maximum_directions=maximum)
    assert 1 <= len(result) <= maximum
    assert all(row[1]['maximum_directions']==maximum for row in result)


@pytest.mark.parametrize('noise', [-1.,float('nan'),float('inf')])
def test_invalid_uncertainty_cannot_be_hidden_by_the_frozen_bound(noise):
    e=silhouette_evidence();g=e['view_geometry']
    assert not directed_silhouette_rotations(e,g['initial_camera_rotation_base'],g['support_normal_base'],noise,.22)
    assert not silhouette_view_gain(e,camera(e,[.1,0.,.25]),noise)['ok']


@pytest.mark.parametrize('field', ['raw_source_evidence','projection_evidence','plane_provenance'])
def test_missing_source_adapter_proof_cannot_enter_observation(field):
    e=silhouette_evidence();del e[field]
    with pytest.raises((ValueError,KeyError)):validate(e)
