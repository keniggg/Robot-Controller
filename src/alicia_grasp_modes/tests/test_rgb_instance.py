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


def test_all_observed_rgb_keeps_refined_mask_and_geometry_identical(recorded):
    data = recorded
    depth = data['depth'][0].astype(np.float32)*float(data['depth_scale'])
    candidate = make_candidate(data, 0)
    expected = refine_candidate(data['bgr'][0], depth, data['intrinsics'], data['T'][0], candidate, Config())
    actual = refine_candidate(data['bgr'][0], depth, data['intrinsics'], data['T'][0], candidate, Config(),
                              rgb_valid=np.ones(depth.shape, bool))
    for name in ('mask', 'position_camera', 'position_base', 'extent_m', 'support_footprint_base'):
        np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name))
    assert actual.metrics == expected.metrics


@pytest.mark.parametrize('domain', ['seed', 'search_background'])
def test_unobserved_rgb_rejects_refinement_without_erasing_real_depth(recorded, monkeypatch, domain):
    import cv2
    data = recorded
    depth = data['depth'][0].astype(np.float32)*float(data['depth_scale'])
    candidate = make_candidate(data, 0)
    ys, xs = np.nonzero(candidate.mask)
    valid = np.ones(depth.shape, bool)
    if domain == 'seed':
        valid[ys[0], xs[0]] = False
    else:
        valid[max(0, ys.min()-2), max(0, xs.min()-2)] = False
    before_depth, before_mask, before_rgb = depth.copy(), candidate.mask.copy(), data['bgr'][0].copy()
    def forbidden(*args):
        pytest.fail('unobserved RGB must not train GrabCut colour models')
    monkeypatch.setattr(cv2, 'grabCut', forbidden)
    with pytest.raises(ValueError, match='contains_unobserved_rgb'):
        refine_candidate(data['bgr'][0], depth, data['intrinsics'], data['T'][0], candidate, Config(), rgb_valid=valid)
    np.testing.assert_array_equal(depth, before_depth)
    np.testing.assert_array_equal(candidate.mask, before_mask)
    np.testing.assert_array_equal(data['bgr'][0], before_rgb)


def test_larger_boundary_retry_cannot_train_on_unobserved_rgb(monkeypatch):
    import cv2
    color = np.full((480, 640, 3), 100, np.uint8)
    seed = np.zeros((480, 640), np.uint8); seed[230:250, 310:330] = 255
    valid = np.ones(seed.shape, bool); valid[200, 300] = False
    calls = []
    def boundary(patch, labels, *args):
        calls.append(labels.shape)
        labels[:] = cv2.GC_BGD
        labels[3:-3, 3:-3] = cv2.GC_PR_FGD
    monkeypatch.setattr(cv2, 'grabCut', boundary)
    with pytest.raises(ValueError, match='rgb_search_contains_unobserved_rgb'):
        instance_mask_from_seed(color, seed, rgb_valid=valid)
    assert calls == [(52, 52)]


@pytest.mark.parametrize('valid', [np.ones((480, 640), np.uint8), np.ones((1, 1), bool)])
def test_refinement_rejects_malformed_rgb_domain(recorded, valid):
    with pytest.raises(ValueError, match='invalid_rgb_validity_mask'):
        instance_mask_from_seed(recorded['bgr'][0], recorded['mask'][0], rgb_valid=valid)


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


@pytest.mark.parametrize('index', [0, 1, 2])
def test_recorded_short_side_is_not_erased_by_support_coloured_depth_ramps(index):
    from alicia_flexible_grasp.vision.object_geometry import _fit_obb
    with np.load(Path(__file__).parent/'fixtures'/'short_side_rgbd_20260925.npz') as data:
        candidate = make_candidate(data, index)
        depth = data['depth'][index].astype(np.float32)*float(data['depth_scale'])
        before = depth.copy()
        colour = data['bgr'][index].copy()
        transform, plane = data['T'][index], data['plane'][index]
        current = refine_candidate(colour, depth, data['intrinsics'], transform, candidate, Config())
        # Independently inspected motor body and adjacent table in the crop.
        # These image coordinates never enter the segmentation algorithm.
        assert np.count_nonzero(current.mask[36:59, 40:61]) >= 520 - 40
        assert np.count_nonzero(current.mask[33:65, 20:34]) <= 5
        assert np.count_nonzero(current.mask[33:65, 69:81]) <= 5
        assert current.bbox[2] <= 33 and 36 <= current.bbox[3] <= 44
        assert current.metrics['support_colour_demoted_side_pixels'] > 100
        assert current.metrics['trusted_core_retained_fraction'] >= .85
        points = _project(depth, data['intrinsics']) @ transform[:3, :3].T + transform[:3, 3]
        height = points @ plane[:3] + plane[3]
        measured = ((current.mask > 0) & (depth > .03) & (height >= .004) & (height <= .12))
        _, _, size = _fit_obb(points[measured], plane[:3], plane[3], .001, .15, .12, None)
        # The actual measured silhouette recovers a distinct tabletop short
        # side. A roughly square depth halo must not define grasp orientation.
        assert size[0]/size[1] > 1.30
        assert .017 < size[1] < .024
        np.testing.assert_array_equal(depth, before)
        np.testing.assert_array_equal(colour, data['bgr'][index])


def test_support_colour_cannot_override_top_or_ambiguous_same_colour_surface():
    from alicia_grasp_modes.measured_mask_seeds import _side_seed_colour_evidence
    colour = np.full((30, 30, 3), (10, 20, 200), np.uint8)
    core = np.zeros((30, 30), bool)
    core[10:20, 10:20] = True
    support = ~core
    height = np.zeros((30, 30))
    height[10:15, 10:20] = .02
    result, evidence = _side_seed_colour_evidence(colour, core, support, height, .02, np.ones_like(height))
    np.testing.assert_array_equal(result, core)
    assert evidence['support_colour_demoted_side_pixels'] == 0
    result, _ = _side_seed_colour_evidence(colour, core, np.zeros_like(core), height, .02, np.ones_like(height))
    np.testing.assert_array_equal(result, core)


def test_boundary_retry_recovers_complete_same_frame_instance_without_filling_depth(monkeypatch):
    import cv2
    color = np.full((480, 640, 3), (20, 20, 180), np.uint8)
    color[190:270, 290:340] = (230, 230, 230)
    seed = np.zeros((480, 640), np.uint8)
    seed[210:265, 295:335] = 255
    depth = np.full(seed.shape, .3, np.float32)
    depth[190:270, 290:340] = .28
    depth[190:210, 290:340] = 0.
    points = _project(depth, [400., 400., 320., 240.])
    heights = .3 - points[..., 0]
    before = depth.copy(); before_color = color.copy(); before_seed = seed.copy()
    calls = []
    original = cv2.grabCut

    def observed(patch, labels, *args):
        ys, xs = np.nonzero(labels == cv2.GC_FGD)
        # The first ROI starts (279,194), the retry (263,178).
        origin = (279, 194) if not calls else (263, 178)
        calls.append(set(zip((xs+origin[0]).tolist(), (ys+origin[1]).tolist())))
        return original(patch, labels, *args)

    monkeypatch.setattr(cv2, 'grabCut', observed)
    mask, evidence = instance_mask_from_seed(color, seed,
        geometry=(points, depth, heights, np.array([-1., 0, 0]), Config()))
    assert len(calls) == 2 and calls[0] == calls[1]
    assert evidence['rgb_search_attempts'] == 2
    assert evidence['rgb_search_initial_margin_px'] == 16
    assert evidence['rgb_search_margin_px'] == 32
    assert evidence['rgb_search_retry_reason'] == 'rgb_instance_reaches_search_boundary'
    assert np.count_nonzero(mask[190:270, 290:340]) == 4000
    assert np.count_nonzero(mask) == 4000
    assert np.count_nonzero(mask[190:210, 290:340]) == 1000
    assert evidence['trusted_core_retained_fraction'] >= .85
    assert evidence['rgb_to_depth_seed_area_ratio'] <= 4.
    np.testing.assert_array_equal(depth, before)
    np.testing.assert_array_equal(color, before_color)
    np.testing.assert_array_equal(seed, before_seed)


def test_first_search_success_does_not_try_larger_mask(monkeypatch):
    import cv2
    color = np.full((480, 640, 3), (20, 20, 180), np.uint8)
    color[230:250, 300:322] = (15, 15, 15)
    seed = np.zeros((480, 640), np.uint8); seed[232:248, 302:320] = 255
    original = cv2.grabCut; calls = []

    def counted(*args):
        calls.append(1)
        return original(*args)

    monkeypatch.setattr(cv2, 'grabCut', counted)
    _, evidence = instance_mask_from_seed(color, seed)
    assert len(calls) == 1
    assert evidence['rgb_search_attempts'] == 1
    assert evidence['rgb_search_margin_px'] == 16
    assert evidence['rgb_search_retry_reason'] == ''


@pytest.mark.parametrize('second_failure', ['boundary', 'expansion', 'fragmented'])
def test_retry_retains_original_rejection_and_never_attempts_third_search(monkeypatch, second_failure):
    import cv2
    color = np.full((480, 640, 3), 100, np.uint8)
    seed = np.zeros((480, 640), np.uint8); seed[230:250, 310:330] = 255
    calls = []

    def forced(patch, labels, *args):
        calls.append(labels.shape)
        labels[:] = cv2.GC_BGD
        if len(calls) == 1 or second_failure == 'boundary':
            labels[3:-3, 3:-3] = cv2.GC_PR_FGD
        elif second_failure == 'expansion':
            labels[5:-5, 5:-5] = cv2.GC_PR_FGD

    monkeypatch.setattr(cv2, 'grabCut', forced)
    reason = dict(boundary='reaches_search_boundary', expansion='not_supported_by_depth_seed',
                  fragmented='ambiguous_or_fragmented')[second_failure]
    with pytest.raises(ValueError, match=reason):
        instance_mask_from_seed(color, seed)
    assert len(calls) == 2
    assert calls == [(52, 52), (84, 84)]


def test_non_boundary_failure_does_not_retry(monkeypatch):
    import cv2
    color = np.full((480, 640, 3), 100, np.uint8)
    seed = np.zeros((480, 640), np.uint8); seed[230:250, 310:330] = 255
    calls = []

    def fragmented(patch, labels, *args):
        calls.append(1); labels[:] = cv2.GC_BGD

    monkeypatch.setattr(cv2, 'grabCut', fragmented)
    with pytest.raises(ValueError, match='ambiguous_or_fragmented'):
        instance_mask_from_seed(color, seed)
    assert len(calls) == 1


def test_real_image_edge_truncation_still_rejected():
    color = np.full((480, 640, 3), (20, 20, 180), np.uint8)
    color[:60, 290:340] = (230, 230, 230)
    seed = np.zeros((480, 640), np.uint8); seed[10:50, 295:335] = 255
    with pytest.raises(ValueError, match='reaches_search_boundary'):
        instance_mask_from_seed(color, seed)
