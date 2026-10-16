"""Retained raw pixels and fused geometry retain distinct evidence roles."""
from dataclasses import replace
import hashlib

import numpy as np
import pytest

from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_snapshot, qualify_regular_box_top)
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from test_gen26_regular_box_inference_independent import ray_scene


def array_hash(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def admitted_scene(*, last_depth_missing=False):
    scene = ray_scene()
    ys, xs = np.nonzero(scene.mask)
    bbox = (int(xs.min()), int(ys.min()), int(np.ptp(xs)) + 1, int(np.ptp(ys)) + 1)
    color = np.zeros(scene.depth.shape + (3,), dtype=np.uint8)
    samples = []
    for index in range(3):
        stamp = scene.stamp - (2 - index) * 100_000_000
        depth = (scene.depth + (10 if index < 2 and not last_depth_missing else 0)).copy()
        if last_depth_missing and index == 2:
            depth[:] = 0
        samples.append(RgbdSample(color.copy(), depth, scene.mask.copy(), bbox,
            None, stamp * 1e-9, 'camera_optical', np.zeros(6), stamp,
            scene.identity.epoch, scene.identity))
    snapshot = fuse_stable_samples(samples, require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01,
        erosion_px=0 if last_depth_missing else 2, depth_scale=.0001,
        depth_min_m=.03, depth_max_m=2., mad_scale=3., mad_absolute_floor_m=.002,
        internal_hole_max_area_px=0)
    assert snapshot.ok, snapshot.failure_reason
    return scene, snapshot


def qualify_snapshot(scene, snapshot, **changes):
    kwargs = dict(snapshot=snapshot, camera_intrinsics=scene.K, depth_scale_m=.0001,
        T_base_camera=scene.T, geometry=scene.geometry,
        source_identity=scene.identity, source_stamp_ns=scene.stamp)
    kwargs.update(changes)
    return qualify_regular_box_snapshot(**kwargs)


def qualify_pixels(scene, depth, mask, color, geometry=None):
    return qualify_regular_box_top(depth, mask, scene.K, .0001, scene.T,
        scene.geometry if geometry is None else geometry, scene.identity, scene.stamp,
        'instance_mask', color_bgr=color)


def test_real_fusion_selects_raw_pixels_without_changing_qualifier():
    scene, snapshot = admitted_scene()
    raw = snapshot.original_sample
    assert not np.array_equal(raw.depth_raw, snapshot.depth_raw)
    assert np.count_nonzero(raw.object_mask) > np.count_nonzero(snapshot.object_mask)
    fused_result = qualify_pixels(scene, snapshot.depth_raw, snapshot.object_mask, snapshot.color_bgr)
    raw_result = qualify_pixels(scene, raw.depth_raw, raw.object_mask, raw.color_bgr)
    result = qualify_snapshot(scene, snapshot)
    assert raw_result.ok and result.ok, (raw_result.audit, result.audit)
    assert result.model.to_evidence_dict() == raw_result.model.to_evidence_dict()
    # A small erosion need not invalidate an ideal rectangle. Its distinct
    # source must nevertheless never replace the retained original pixels.
    if fused_result.ok:
        assert result.model.to_evidence_dict() != fused_result.model.to_evidence_dict()
    assert result.audit['pixel_source'] == 'retained_exact_original_sample'
    assert result.audit['geometry_source'] == 'unchanged_request_fused_measured_geometry'
    assert result.audit['source_binding_valid'] is True
    assert result.audit['source_frame_id'] == 'camera_optical'
    assert result.audit['final_sample_binary_mask_sha256'] == snapshot.sample_mask_sha256[-1]


def test_raw_hashes_are_separate_from_unchanged_geometry_and_fused_input_hashes():
    scene, snapshot = admitted_scene()
    arrays = [snapshot.depth_raw, snapshot.object_mask, snapshot.original_sample.depth_raw,
              snapshot.original_sample.object_mask, scene.geometry.object_points_base]
    before = [array.copy() for array in arrays]
    result = qualify_snapshot(scene, snapshot)
    assert result.ok, result.audit
    source = result.model.source
    for field, array in [('depth_sha256', snapshot.original_sample.depth_raw),
                         ('mask_sha256', snapshot.original_sample.object_mask),
                         ('color_sha256', snapshot.original_sample.color_bgr)]:
        assert source[field] == array_hash(array)
        assert result.audit['retained_pixel_sha256'][field] == source[field]
    assert source['depth_sha256'] != array_hash(snapshot.depth_raw)
    assert source['mask_sha256'] != array_hash(snapshot.object_mask)
    # Changing only upstream measured geometry changes its separate digest,
    # while raw pixels, calibration and inferred rectangle stay identical.
    geometry = replace(scene.geometry, size_xyz_m=scene.geometry.size_xyz_m + [.001, 0., 0.])
    changed = qualify_snapshot(scene, snapshot, geometry=geometry)
    direct = qualify_pixels(scene, snapshot.original_sample.depth_raw,
        snapshot.original_sample.object_mask, snapshot.original_sample.color_bgr, geometry)
    assert changed.ok and direct.ok, (changed.audit, direct.audit)
    assert changed.model.to_evidence_dict() == direct.model.to_evidence_dict()
    assert changed.model.source['geometry_sha256'] != source['geometry_sha256']
    for key in source:
        if key != 'geometry_sha256':
            assert changed.model.source[key] == source[key]
    for array, old in zip(arrays, before):
        np.testing.assert_array_equal(array, old)


def test_unusable_original_depth_never_falls_back_to_qualifiable_temporal_depth():
    scene, snapshot = admitted_scene(last_depth_missing=True)
    assert not np.any(snapshot.original_sample.depth_raw)
    assert qualify_pixels(scene, snapshot.depth_raw, snapshot.object_mask, snapshot.color_bgr).ok
    result = qualify_snapshot(scene, snapshot)
    assert not result.ok and result.model is None
    assert result.audit['source_binding_valid'] is True
    assert result.audit['retained_pixel_sha256']['depth_sha256'] == array_hash(snapshot.original_sample.depth_raw)
    assert result.code == 'BOX_TOP_CONTOUR_INCOMPLETE'


@pytest.mark.parametrize('fault', ['missing', 'mask_hash_missing', 'mask_hash_count',
    'mask_hash_wrong', 'raw_mask_changed', 'raw_mask_nonbinary', 'raw_rgb_shape',
    'raw_depth_shape', 'unadmitted', 'source_mode'])
def test_missing_or_malformed_source_cannot_authorize_inferred_model(fault):
    scene, snapshot = admitted_scene()
    original = snapshot.original_sample
    if fault == 'missing':
        snapshot = replace(snapshot, original_sample=None)
    elif fault == 'mask_hash_missing':
        snapshot = replace(snapshot, sample_mask_sha256=())
    elif fault == 'mask_hash_count':
        snapshot = replace(snapshot, sample_mask_sha256=snapshot.sample_mask_sha256[-1:])
    elif fault == 'mask_hash_wrong':
        snapshot = replace(snapshot, sample_mask_sha256=snapshot.sample_mask_sha256[:-1] + ('0' * 64,))
    elif fault in ('raw_mask_changed', 'raw_mask_nonbinary'):
        mask = original.object_mask.copy()
        y, x = np.argwhere(mask > 0)[0]
        mask[y, x] = 0 if fault == 'raw_mask_changed' else 1
        snapshot = replace(snapshot, original_sample=replace(original, object_mask=mask))
    elif fault == 'raw_rgb_shape':
        snapshot = replace(snapshot, original_sample=replace(original, color_bgr=original.color_bgr[:, :-1]))
    elif fault == 'raw_depth_shape':
        snapshot = replace(snapshot, original_sample=replace(original, depth_raw=original.depth_raw[:, :-1]))
    elif fault == 'unadmitted':
        snapshot = replace(snapshot, ok=False)
    else:
        snapshot = replace(snapshot, source_mode='bbox_depth')
    result = qualify_snapshot(scene, snapshot)
    assert not result.ok and result.model is None
    assert result.code == ('BOX_INFERENCE_SOURCE_UNAVAILABLE' if fault == 'missing' else 'BOX_INFERENCE_SOURCE_MISMATCH')
    assert result.audit['source_binding_valid'] is False
    assert result.audit['reason']


@pytest.mark.parametrize('fault', ['observation_identity', 'observation_stamp'])
def test_retained_source_also_matches_current_observation(fault):
    scene, snapshot = admitted_scene()
    changes = (dict(source_identity=TargetTrackIdentity(scene.identity.epoch, 'other-target'))
        if fault == 'observation_identity' else dict(source_stamp_ns=scene.stamp + 1))
    result = qualify_snapshot(scene, snapshot, **changes)
    assert not result.ok and result.model is None
    assert result.code == 'BOX_INFERENCE_SOURCE_MISMATCH'
    assert 'current target observation' in result.audit['reason']
