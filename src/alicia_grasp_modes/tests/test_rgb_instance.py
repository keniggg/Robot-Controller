"""Recorded current-frame masks exercise the actual downstream snapshot gate."""
from pathlib import Path
import numpy as np
import pytest

from alicia_grasp_modes.rgb_instance import instance_mask_from_seed, refine_candidate
from alicia_grasp_modes.acquisition import InitialAcquisition
from alicia_grasp_modes.tabletop import Candidate, Config, _project, _measured_support_footprint
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


@pytest.fixture(scope='module')
def recorded():
    with np.load(Path(__file__).parent/'fixtures'/'unknown_rgbd_silhouette.npz') as data:
        yield dict(data)


def make_candidate(data, index, depth=None):
    raw = data['depth'][index]
    depth = raw.astype(np.float32)*float(data['depth_scale']) if depth is None else depth
    mask = data['mask'][index]
    transform, plane = data['T'][index], data['plane'][index]
    xyz = _project(depth, data['intrinsics'])[mask > 0]
    xyz = xyz[np.all(np.isfinite(xyz), axis=1) & (xyz[:, 0] > .03)]
    center = np.median(xyz, axis=0)
    base = (transform @ np.r_[center, 1.])[:3]
    centered = xyz-xyz.mean(0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    extent = np.sort(np.percentile(centered@vh.T, 98, axis=0)
                     - np.percentile(centered@vh.T, 2, axis=0))[::-1]
    ys, xs = np.where(mask > 0)
    return Candidate(mask, np.array([xs.mean(), ys.mean()]),
        (int(xs.min()), int(ys.min()), int(xs.max()-xs.min()+1), int(ys.max()-ys.min()+1)),
        center, base, extent, center[0], .85, dict(support_plane_base=plane.tolist()),
        _measured_support_footprint(xyz@transform[:3,:3].T+transform[:3,3], plane))


def test_recorded_noise_passes_original_fusion_without_relaxing_iou_or_synthesizing_depth(recorded):
    data = recorded
    original_depth = data['depth'].copy()
    original_rgb = data['bgr'].copy()
    old, refined = [], []
    identity = TargetTrackIdentity.from_stream(15, 15)
    for i, ns in enumerate(data['stamps']):
        candidate = make_candidate(data, i)
        current = refine_candidate(data['bgr'][i], data['depth'][i].astype(np.float32)*float(data['depth_scale']),
                                   data['intrinsics'], data['T'][i], candidate, Config())
        for output, selected in ((old, candidate), (refined, current)):
            output.append(RgbdSample(data['bgr'][i], data['depth'][i], selected.mask,
                selected.bbox, None, int(ns)/1e9, 'camera_link', data['joints'][i],
                stamp_ns=int(ns), target_epoch=15, target_identity=identity))
        assert current is not candidate
        assert not np.shares_memory(current.mask, candidate.mask)
    kwargs = dict(require_mask=True, min_mask_iou=.85, max_centroid_shift_px=5.,
                  max_joint_delta_rad=.01, erosion_px=2, depth_scale=float(data['depth_scale']),
                  depth_min_m=.03, depth_max_m=2., mad_scale=3.5,
                  mad_absolute_floor_m=.002, internal_hole_max_area_px=0)
    old_results = [fuse_stable_samples(old[i:i+3], **kwargs) for i in range(len(old)-2)]
    new_results = [fuse_stable_samples(refined[i:i+3], **kwargs) for i in range(len(refined)-2)]
    assert not any(result.ok for result in old_results)
    assert sum(result.ok for result in new_results) >= 4
    for result in new_results:
        if result.ok:
            assert result.quality.valid_depth_points >= 120
            assert not np.any((result.depth_raw == 0) & (result.target_depth_raw > 0))
    np.testing.assert_array_equal(data['depth'], original_depth)
    np.testing.assert_array_equal(data['bgr'], original_rgb)


def test_current_silhouette_does_not_invent_missing_measurements(recorded):
    data = recorded
    candidate = make_candidate(data, 0)
    depth = data['depth'][0].astype(np.float32)*float(data['depth_scale'])
    # A hole inside the visible object remains zero although RGB can still
    # establish ownership of that pixel.
    depth[273:276, 313:316] = 0
    before = depth.copy()
    current = refine_candidate(data['bgr'][0], depth, data['intrinsics'], data['T'][0], candidate, Config())
    assert np.any(current.mask[273:276, 313:316])
    assert current.metrics['rgb_mask_pixels_without_valid_depth'] >= 9
    np.testing.assert_array_equal(depth, before)


def test_no_measured_seed_cannot_create_target_from_colour(recorded):
    with pytest.raises(ValueError, match='insufficient_measured_foreground'):
        instance_mask_from_seed(recorded['bgr'][0], np.zeros_like(recorded['mask'][0]))


def test_unrelated_rgb_component_is_not_added_to_selected_instance():
    color = np.full((480,640,3), (30,30,190), np.uint8)
    color[230:250,300:322] = (15,15,15)
    color[230:250,330:338] = (15,15,15)
    seed = np.zeros((480,640), np.uint8)
    seed[232:248,302:320] = 255
    mask, _ = instance_mask_from_seed(color, seed)
    assert np.count_nonzero(mask[230:250,300:322]) > 350
    assert not np.any(mask[:,330:338])


def test_initial_lock_rejects_small_depth_fragment(recorded):
    candidate = make_candidate(recorded, 0)
    candidate.metrics['valid_points'] = 66
    acquisition = InitialAcquisition(Config())
    for ns in (1, 2, 3, 4):
        assert not acquisition.observe(candidate, candidate, ns)[0]
    assert acquisition.samples == []


def test_initial_lock_requires_three_distinct_consistent_frames(recorded):
    candidate = make_candidate(recorded, 0)
    candidate.metrics['valid_points'] = 400
    acquisition = InitialAcquisition(Config())
    assert not acquisition.observe(candidate, candidate, 1)[0]
    assert not acquisition.observe(candidate, candidate, 2)[0]
    assert acquisition.observe(candidate, candidate, 3)[0]
    assert not acquisition.observe(candidate, candidate, 3)[0]
    assert not acquisition.samples


def test_initial_lock_restarts_confirmation_when_outline_changes(recorded):
    from dataclasses import replace
    candidate = make_candidate(recorded, 0)
    candidate.metrics['valid_points'] = 400
    changed = replace(candidate, mask=np.roll(candidate.mask, 15, axis=1))
    acquisition = InitialAcquisition(Config())
    acquisition.observe(candidate, candidate, 1)
    acquisition.observe(candidate, candidate, 2)
    assert not acquisition.observe(candidate, changed, 3)[0]
    assert len(acquisition.samples) == 1


@pytest.fixture(scope='module')
def shadow_jump():
    with np.load(Path(__file__).parent/'fixtures'/'shadow_jump_20260923.npz') as data:
        yield dict(data)


@pytest.mark.parametrize('index', [0, 1, 2])
def test_measured_local_jump_does_not_force_shadow_into_foreground(shadow_jump, index):
    data = shadow_jump
    candidate = make_candidate(data, index)
    depth = data['depth'][index].astype(np.float32)*float(data['depth_scale'])
    before = depth.copy()
    rgb_before = data['bgr'][index].copy()
    current = refine_candidate(data['bgr'][index], depth, data['intrinsics'], data['T'][index], candidate, Config())
    # These independently inspected image regions are evaluation references,
    # not algorithm parameters. Retain the dark object face, reject the table.
    assert np.count_nonzero(current.mask[299:309,321:362]) <= 10
    assert np.count_nonzero(current.mask[235:280,374:383]) <= 5
    assert np.count_nonzero(current.mask[285:294,313:366]) >= 475
    assert current.metrics['trusted_core_retained_fraction'] >= .85
    assert current.metrics['trusted_side_pixels'] > 0
    assert current.metrics['support_background_height_separated']
    np.testing.assert_array_equal(depth, before)
    np.testing.assert_array_equal(data['bgr'][index], rgb_before)
    if index == 1:
        legacy, _ = instance_mask_from_seed(data['bgr'][index], candidate.mask)
        assert np.count_nonzero(legacy[299:309,321:362]) >= 400


def test_mask_evidence_is_invariant_to_base_coordinates(shadow_jump):
    from dataclasses import replace
    data = shadow_jump
    candidate = make_candidate(data, 1)
    depth = data['depth'][1].astype(np.float32)*float(data['depth_scale'])
    expected = refine_candidate(data['bgr'][1], depth, data['intrinsics'], np.eye(4), candidate, Config())
    angle = .43
    transform = np.array([[np.cos(angle),0,np.sin(angle),.17], [0,1,0,-.08],
                          [-np.sin(angle),0,np.cos(angle),.21], [0,0,0,1.]])
    plane = np.linalg.inv(transform).T @ data['plane'][1]
    metrics = dict(candidate.metrics, support_plane_base=plane.tolist())
    changed = replace(candidate, position_base=(transform @ np.r_[candidate.position_camera,1])[:3], metrics=metrics)
    actual = refine_candidate(data['bgr'][1], depth, data['intrinsics'], transform, changed, Config())
    np.testing.assert_array_equal(actual.mask, expected.mask)
    np.testing.assert_allclose(actual.position_base, (transform @ np.r_[expected.position_camera,1])[:3])


def test_low_profile_surface_is_not_declared_certain_background(recorded):
    candidate = make_candidate(recorded, 0)
    current = refine_candidate(recorded['bgr'][0], recorded['depth'][0].astype(np.float32)*float(recorded['depth_scale']),
                               recorded['intrinsics'], recorded['T'][0], candidate, Config())
    assert not current.metrics['support_background_height_separated']
    assert current.metrics['measured_support_background_pixels'] == 0
    assert current.metrics['trusted_core_retained_fraction'] >= .85
