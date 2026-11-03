"""RGB attention regions are image evidence and never metric target geometry."""
from dataclasses import replace
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_region import (
    CenterSearchPolicy, RgbObservationTracker, propose_rgb_observation_regions,
)


POLICY = CenterSearchPolicy((.5, .5))


def scene(shape=(240, 320), foreground=(20, 20, 20), background=(60, 90, 190),
          rect=None):
    image = np.full((*shape, 3), background, np.uint8)
    h, w = shape
    rect = rect or (int(.46 * w), int(.44 * h), int(.08 * w), int(.12 * h))
    x, y, width, height = rect
    image[y:y+height, x:x+width] = foreground
    return image


def detect(image, **kwargs):
    return propose_rgb_observation_regions(image, policy=POLICY, **kwargs)


@pytest.mark.parametrize('foreground,background', [
    ((15, 15, 15), (220, 220, 220)),
    ((240, 240, 240), (15, 15, 15)),
    ((20, 30, 230), (230, 130, 20)),
    ((200, 50, 30), (30, 170, 180)),
    ((40, 180, 40), (160, 40, 180)),
])
def test_colour_independent_region_without_depth(foreground, background):
    image = scene(foreground=foreground, background=background)
    before = image.copy()
    result = detect(image)
    assert result.reason == 'observation_region_found'
    assert result.selected is not None
    region = result.selected
    assert region.bbox_xywh == (147, 105, 25, 28)
    assert region.area_px == 25 * 28
    assert region.valid_depth_fraction is None
    assert region.observation_only
    assert not region.metric_geometry_authority
    assert not region.object_identity_confirmed
    assert not region.mask.flags.writeable
    assert not np.shares_memory(image, region.mask)
    np.testing.assert_array_equal(image, before)


def test_same_luminance_chromatic_region_is_observable():
    # Both BGR colours round to the same grayscale value; chromatic channels
    # must still identify this place without assuming a dark object.
    foreground, background = (220, 20, 20), (20, 20, 96)
    image = scene(foreground=foreground, background=background)
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    assert abs(int(grey[120, 160]) - int(grey[0, 0])) <= 1
    result = detect(image)
    assert result.selected is not None
    assert any(channel in ('lab_a', 'lab_b') for channel in result.selected.channels)


@pytest.mark.parametrize('shape', [(120, 160), (240, 320), (480, 640), (960, 1280)])
def test_selection_scales_with_image_resolution(shape):
    result = detect(scene(shape=shape))
    assert result.selected is not None
    x, y = result.selected.centroid_xy
    assert abs(x / shape[1] - .5) < .02
    assert abs(y / shape[0] - .5) < .02
    assert result.evaluated_regions <= POLICY.max_evaluated_regions


def test_valid_depth_does_not_change_rgb_evidence_or_score():
    image = scene()
    outputs = [detect(image, depth_valid=mask) for mask in
               (None, np.zeros(image.shape[:2], bool), np.ones(image.shape[:2], bool))]
    for actual in outputs[1:]:
        assert actual.reason == outputs[0].reason
        assert actual.total_candidate_count == outputs[0].total_candidate_count
        assert actual.selected.score == outputs[0].selected.score
        np.testing.assert_array_equal(actual.selected.mask, outputs[0].selected.mask)
    assert outputs[1].selected.valid_depth_fraction == 0.
    assert outputs[2].selected.valid_depth_fraction == 1.


def test_all_observed_rgb_domain_preserves_exact_legacy_regions():
    image = scene()
    original = detect(image)
    actual = detect(image, rgb_valid=np.ones(image.shape[:2], bool))
    assert actual.reason == original.reason
    assert actual.total_candidate_count == original.total_candidate_count
    assert actual.evaluated_regions == original.evaluated_regions
    for expected, region in zip(original.candidates, actual.candidates):
        assert region.bbox_xywh == expected.bbox_xywh
        assert region.score == expected.score and region.channels == expected.channels
        np.testing.assert_array_equal(region.mask, expected.mask)


@pytest.mark.parametrize('domain', ['whole_candidate', 'one_candidate_pixel', 'contrast_ring'])
def test_padding_cannot_supply_candidate_or_contrast_evidence(domain):
    image = scene(foreground=(0, 0, 0))
    candidate = detect(image).selected
    assert candidate is not None
    valid = np.ones(image.shape[:2], bool)
    x, y, _, _ = candidate.bbox_xywh
    if domain == 'whole_candidate':
        valid[candidate.mask] = False
    elif domain == 'one_candidate_pixel':
        valid[y+1, x+1] = False
    else:
        valid[y-1, x] = False
    before, before_valid = image.copy(), valid.copy()
    depth_valid = np.ones(valid.shape, bool)
    result = detect(image, rgb_valid=valid, depth_valid=depth_valid)
    assert not result.candidates and result.selected is None
    np.testing.assert_array_equal(image, before)
    np.testing.assert_array_equal(valid, before_valid)
    assert depth_valid.all()


def test_unobserved_rgb_far_from_candidate_does_not_change_its_evidence():
    image = scene()
    expected = detect(image).selected
    valid = np.ones(image.shape[:2], bool); valid[:10] = False
    actual = detect(image, rgb_valid=valid).selected
    np.testing.assert_array_equal(actual.mask, expected.mask)
    assert actual.score == expected.score


@pytest.mark.parametrize('valid', [np.ones((240, 320), np.uint8), np.ones((239, 320), bool)])
def test_invalid_rgb_domain_rejected(valid):
    with pytest.raises(ValueError, match='invalid_rgb_validity_mask'):
        detect(scene(), rgb_valid=valid)


def test_depth_hole_alone_cannot_create_any_region():
    image = np.full((240, 320, 3), 125, np.uint8)
    depth_valid = np.ones(image.shape[:2], bool)
    depth_valid[100:140, 140:180] = False
    result = detect(image, depth_valid=depth_valid)
    assert result.selected is None
    assert not result.candidates


def test_symmetric_distinct_regions_are_ambiguous():
    image = scene(rect=(127, 108, 18, 24))
    image[108:132, 175:193] = (20, 20, 20)
    result = detect(image)
    assert len(result.candidates) == 2
    assert result.reason == 'ambiguous_rgb_regions'
    assert result.selected is None


def test_explicit_center_policy_changes_attention_not_scene_geometry():
    image = scene(rect=(127, 108, 18, 24))
    image[108:132, 175:193] = (20, 20, 20)
    result = propose_rgb_observation_regions(image, policy=CenterSearchPolicy((.425, .5)))
    assert result.selected is not None
    assert result.selected.bbox_xywh == (127, 108, 18, 24)
    assert not result.metric_geometry_authority


def test_search_radius_filters_without_rescaling_candidate_evidence():
    image = scene(rect=(135, 103, 20, 26))
    image[122:134, 163:177] = (180, 130, 90)
    wide = propose_rgb_observation_regions(image, policy=POLICY)
    narrow = propose_rgb_observation_regions(image, policy=replace(POLICY, radius_fraction=.125))
    assert len(wide.candidates) == len(narrow.candidates)
    assert wide.reason == narrow.reason
    assert [(r.bbox_xywh, r.score) for r in wide.candidates] == [
        (r.bbox_xywh, r.score) for r in narrow.candidates]


def test_nested_concentric_colour_parts_are_one_observation_location():
    image = scene(rect=(138, 98, 42, 42))
    image[108:129, 148:169] = (220, 210, 200)
    result = detect(image)
    assert result.selected is not None
    assert result.total_candidate_count == 1


def test_offcentre_nested_region_is_not_silently_merged():
    image = scene(rect=(132, 94, 54, 54))
    image[111:133, 158:180] = (220, 210, 200)
    result = detect(image)
    assert result.total_candidate_count >= 2


def test_image_edge_cannot_supply_closed_object_boundary():
    image = scene(rect=(0, 90, 40, 40))
    result = propose_rgb_observation_regions(image, policy=CenterSearchPolicy((0., .5)))
    assert result.selected is None
    assert not result.candidates


def test_attention_crop_edge_is_rejected_even_if_centroid_inside_search():
    image = scene(rect=(20, 104, 150, 26))
    result = detect(image)
    assert result.selected is None


@pytest.mark.parametrize('kind', ['background', 'fragment', 'line', 'flat'])
def test_fragments_broad_background_and_thin_lines_are_not_observation_seeds(kind):
    image = np.full((240, 320, 3), (80, 100, 180), np.uint8)
    if kind == 'background':
        image[40:200, 50:270] = 20
    elif kind == 'fragment':
        image[119:121, 159:162] = 20
    elif kind == 'line':
        image[70:170, 159:162] = 20
    assert detect(image).selected is None


def test_printed_patch_is_only_attention_evidence_not_object_confirmation():
    # RGB alone cannot distinguish a printed square from a thin solid item.
    result = detect(scene())
    assert result.selected is not None
    assert not result.selected.object_identity_confirmed
    assert result.observation_only and not result.metric_geometry_authority
    assert not hasattr(result.selected, 'position_base')
    assert not hasattr(result.selected, 'height_m')


def test_exceeded_analysis_budget_abstains():
    image = scene()
    policy = replace(POLICY, max_analysis_pixels=144)
    result = propose_rgb_observation_regions(image, policy=policy)
    assert result.reason == 'rgb_analysis_resolution_budget_exceeded'
    assert result.selected is None
    policy = replace(POLICY, max_evaluated_regions=2)
    image[93:105, 168:180] = 200
    image[121:135, 185:200] = 30
    result = propose_rgb_observation_regions(image, policy=policy)
    assert result.reason == 'rgb_analysis_region_budget_exhausted'
    assert result.selected is None
    assert result.evaluated_regions == 2
    result = propose_rgb_observation_regions(scene(), policy=replace(
        POLICY, max_candidate_mask_pixels=144))
    assert result.reason == 'rgb_candidate_mask_budget_exhausted'
    assert result.selected is None


def test_many_nested_threshold_variants_do_not_allocate_full_masks(monkeypatch):
    import alicia_grasp_modes.rgb_observation_region as module

    height, width = 480, 640
    yy, xx = np.indices((height, width))
    radius = np.maximum(np.abs(xx - 320), np.abs(yy - 240))
    levels = np.minimum(240, radius * 4).astype(np.uint8)
    image = np.repeat(levels[:, :, None], 3, axis=2)
    regions = []
    for radius in range(5, 50):
        ys, xs = np.indices((radius * 2 + 1,) * 2)
        regions.append(np.column_stack((xs.ravel() + 320 - radius,
                                        ys.ravel() + 240 - radius)))

    class ThresholdVariants:
        def detectRegions(self, channel):
            # MSER reports coordinates relative to its analysis image.
            return [region - (127, 47) for region in regions], None

    monkeypatch.setattr(module.cv2, 'MSER_create', lambda *args: ThresholdVariants())
    assert len(regions) * height * width > POLICY.max_candidate_mask_pixels
    actual = detect(image)
    reference = propose_rgb_observation_regions(image, policy=replace(
        POLICY, max_candidate_mask_pixels=POLICY.max_candidate_mask_pixels * 8))
    assert actual.reason == 'observation_region_found'
    assert actual.total_candidate_count == reference.total_candidate_count == 1
    assert actual.evaluated_regions == len(regions)
    assert actual.selected.bbox_xywh == reference.selected.bbox_xywh
    assert actual.selected.score == reference.selected.score
    np.testing.assert_array_equal(actual.selected.mask, reference.selected.mask)
    assert actual.returned_mask_pixels == height * width
    assert actual.compact_mask_pixels < height * width
    assert actual.mask_storage_peak_pixels <= POLICY.max_candidate_mask_pixels
    tight = propose_rgb_observation_regions(image, policy=replace(
        POLICY, max_candidate_mask_pixels=actual.mask_storage_peak_pixels - 1))
    assert tight.reason == 'rgb_candidate_mask_budget_exhausted'
    assert tight.selected is None
    assert tight.mask_storage_peak_pixels < actual.mask_storage_peak_pixels


def test_compact_intersection_matches_full_frame_pixel_oracle():
    import alicia_grasp_modes.rgb_observation_region as module
    from dataclasses import fields

    first = detect(scene()).selected
    second = detect(scene(rect=(158, 115, 25, 28))).selected
    assert first is not None and second is not None

    def compact(region):
        x, y, w, h = region.bbox_xywh
        args = {entry.name: getattr(region, entry.name)
                for entry in fields(module._CompactRegion)}
        args['mask'] = region.mask[y:y+h, x:x+w].copy()
        return module._CompactRegion(**args)

    expected = np.count_nonzero(first.mask & second.mask)
    assert expected > 0
    for left in (first, compact(first)):
        for right in (second, compact(second)):
            assert module._intersection(left, right) == expected
    assert module._same_region(first, compact(first))


# These read-only local recordings are optional outside the robot workspace.
# Expected ordering/pixels are from the previous full-frame representation with
# enough temporary diagnostic memory to compute its complete result. They are
# test references only: no recorded ROI or colour enters production selection.
_RECORDED_RGB_ORDER = {
    'color_0': [((304, 259, 24, 21), 'd5c42e1707009ab3'),
                ((329, 246, 28, 16), 'c94e315b20f8c672'),
                ((277, 251, 13, 11), 'cd035b1593e8011b'),
                ((277, 189, 14, 8), 'ecc3f5924c734fb4')],
    'color_1': [((302, 259, 26, 23), '7bb45a498db5291a'),
                ((328, 245, 28, 17), 'f735b52585b8b542'),
                ((277, 252, 13, 10), 'a162cf8cdfc38fa8'),
                ((346, 206, 19, 9), 'ca72cfcb00f30e7f'),
                ((278, 189, 14, 9), 'd672bd3abecab1be')],
    'color_2': [((302, 259, 26, 22), 'b32aa3e478c566a9'),
                ((329, 245, 27, 16), 'dfd3c5a8b0c7828f'),
                ((277, 252, 14, 10), 'c6f1117deea42a97'),
                ((352, 206, 13, 9), '0859fd3d76856ee9'),
                ((278, 189, 13, 8), 'b3e5c507456f5b10')],
    '1791076200971747875': [((302, 258, 26, 23), '3531de7afdf82cd9'),
        ((328, 245, 28, 16), 'e3ab24b2a5682ac3'), ((303, 275, 23, 5), 'a685162373796b3d'),
        ((272, 251, 19, 11), '26f03673a869e12f'), ((277, 188, 14, 10), '22ce5eed21cfadf7'),
        ((356, 271, 17, 13), '6333c0ae5f904ec1')],
    '1791076201504771232': [((302, 258, 26, 25), '08cfbd824cb499f2'),
        ((328, 244, 28, 17), 'b6c52173ca065f6a'), ((277, 250, 14, 12), 'ad2be1a8fe22addd'),
        ((350, 205, 15, 10), 'ba24854b7c967e4d'), ((279, 189, 12, 7), '761bfbc1cd249940')],
    '1791076202071988821': [((302, 258, 26, 22), '394cf387016b1a68'),
        ((276, 251, 15, 11), '1fb9f487270d6fca'), ((350, 205, 15, 10), '8d41153e37207c21'),
        ((276, 188, 15, 9), '1dce23b200a46034')],
    '1791076202605244159': [((302, 258, 26, 23), 'a818a98f36c80a29'),
        ((328, 245, 28, 16), '31bdd3cc1b23ea30'), ((277, 250, 14, 12), 'd244a9727fe7e100'),
        ((348, 204, 17, 11), 'f0e2207ce581bd12'), ((277, 189, 15, 9), '344d4e3703078695')],
    '1791076203172020673': [((302, 258, 26, 24), 'f6207c80a526cd22'),
        ((276, 251, 15, 11), 'a28bae1f614d263c'), ((351, 205, 14, 9), '99dee56c9b1e8b5d')],
    '1791076204272146463': [((302, 258, 26, 24), '1baede077ca3e1a6'),
        ((328, 245, 28, 16), 'debd0ad52d4291ee'), ((277, 251, 14, 11), 'b33061453ec222fb'),
        ((351, 206, 14, 8), 'ba284e2e556a0a5d'), ((277, 188, 15, 9), 'f38fed680c637bd0')],
    '1791076205372523307': [((302, 258, 26, 23), 'ea7980b91952c6ee'),
        ((275, 251, 16, 12), '54340095de892bd9'), ((351, 206, 13, 9), '90b0ce6307384f17'),
        ((277, 188, 15, 9), '98a040e2b9402e86')],
    '1791076206505722761': [((302, 258, 26, 23), 'b6d15bea26f201f0'),
        ((328, 245, 28, 16), '2f620754cd883336'), ((277, 188, 14, 9), '483a883d75caa97c')],
}


@pytest.mark.parametrize('name', list(_RECORDED_RGB_ORDER))
def test_saved_frames_preserve_full_mask_reference_order_and_pixels(name):
    root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
    if name.startswith('color_'):
        path = root / 'full_ros_resume_20261004T002930Z/aligned_capture_20261004T003847Z' / (name + '.png')
    else:
        path = root / 'rgb_observation_bootstrap_20261004T011000Z' / (name + '_rgb.png')
    if not path.is_file():
        pytest.skip('local_only external RGB recording is not mounted in this environment')
    image = cv2.imread(str(path))
    assert image is not None
    if not name.startswith('color_'):
        with np.load(path.with_name(name + '_source.npz'), allow_pickle=False) as data:
            np.testing.assert_array_equal(image, data['color_bgr'])
    policy = replace(POLICY, radius_fraction=.125)
    result = propose_rgb_observation_regions(image, policy=policy)
    assert result.reason == 'observation_region_found'
    assert result.selected_index == 0
    actual = [(region.bbox_xywh, hashlib.sha256(region.mask.tobytes()).hexdigest()[:16])
              for region in result.candidates]
    assert actual == _RECORDED_RGB_ORDER[name]
    assert result.compact_mask_pixels < image.shape[0] * image.shape[1]
    assert result.mask_storage_peak_pixels <= policy.max_candidate_mask_pixels
    assert result.returned_mask_pixels == sum(region.mask.size for region in result.candidates)


def track(tracker, result, stamp=1, source='camera/calibration/epoch', generation=1,
          joints=(0., 0., 0., 0., 0., 0.)):
    return tracker.observe(result, source_id=source, generation=generation,
                           stamp_ns=stamp, joints_rad=joints)


def test_requires_three_distinct_frames_and_remains_observation_only():
    tracker, result = RgbObservationTracker(), detect(scene())
    assert not track(tracker, result, 1).confirmed
    assert not track(tracker, result, 2).confirmed
    state = track(tracker, result, 3)
    assert state.confirmed and state.consistent_frames == 3
    assert state.source_stamps_ns == (1, 2, 3)
    assert state.observation_only and not state.metric_geometry_authority


@pytest.mark.parametrize('bad_stamp', [2, 1])
def test_repeated_or_out_of_order_frame_resets_confirmation(bad_stamp):
    tracker, result = RgbObservationTracker(), detect(scene())
    for stamp in (1, 2, 3):
        track(tracker, result, stamp)
    state = track(tracker, result, bad_stamp)
    assert not state.confirmed and not state.source_stamps_ns
    assert state.reason == 'nonincreasing_source_stamp'
    assert not track(tracker, result, 4).confirmed
    assert not track(tracker, result, 5).confirmed
    assert track(tracker, result, 6).confirmed


@pytest.mark.parametrize('change', ['source', 'generation', 'joints', 'gap', 'shape'])
def test_context_changes_restart_evidence(change):
    tracker, result = RgbObservationTracker(), detect(scene())
    track(tracker, result, 1)
    track(tracker, result, 2)
    kwargs = dict(stamp=3)
    if change == 'source':
        kwargs['source'] = 'new/calibration/epoch'
    elif change == 'generation':
        kwargs['generation'] = 2
    elif change == 'joints':
        kwargs['joints'] = (0., 0., 0., .01, 0., 0.)
    elif change == 'gap':
        kwargs['stamp'] = 2_000_000_000
    else:
        result = detect(scene(shape=(480, 640)))
    state = track(tracker, result, **kwargs)
    assert not state.confirmed and state.consistent_frames == 1


def test_joint_comparison_uses_first_pose_not_incremental_drift():
    tracker, result = RgbObservationTracker(), detect(scene())
    track(tracker, result, 1)
    track(tracker, result, 2, joints=(0., 0., 0., .003, 0., 0.))
    state = track(tracker, result, 3, joints=(0., 0., 0., .006, 0., 0.))
    assert not state.confirmed and state.consistent_frames == 1


def test_mask_change_or_ambiguity_discards_old_confirmation():
    tracker, result = RgbObservationTracker(), detect(scene())
    for stamp in (1, 2, 3):
        track(tracker, result, stamp)
    moved = detect(scene(rect=(158, 105, 25, 28)))
    assert track(tracker, moved, 4).consistent_frames == 1
    empty = detect(np.full((240, 320, 3), 100, np.uint8))
    assert track(tracker, empty, 5).consistent_frames == 0
    assert not track(tracker, result, 6).confirmed


def test_confirmed_tracker_evidence_storage_is_bounded():
    tracker, result = RgbObservationTracker(), detect(scene())
    for stamp in range(1, 2001):
        state = track(tracker, result, stamp)
    assert state.confirmed
    assert state.consistent_frames == 3
    assert state.source_stamps_ns == (1998, 1999, 2000)
    # The anchor is still the initial pose, not the oldest retained timestamp.
    state = track(tracker, result, 2001, joints=(0., .01, 0., 0., 0., 0.))
    assert state.consistent_frames == 1


@pytest.mark.parametrize('kwargs', [
    {'center_xy': (float('nan'), .5)}, {'center_xy': (1.1, .5)},
    {'center_xy': (.5, .5), 'min_area_fraction': .2},
    {'center_xy': (.5, .5), 'max_returned_candidates': True},
    {'center_xy': (.5, .5), 'max_evaluated_regions': True},
    {'center_xy': (.5, .5), 'radius_fraction': 0.},
])
def test_invalid_search_policy_is_rejected(kwargs):
    with pytest.raises(ValueError):
        CenterSearchPolicy(**kwargs)


def test_invalid_input_does_not_silently_construct_region():
    with pytest.raises(ValueError, match='invalid_bgr'):
        detect(scene().astype(float))
    with pytest.raises(ValueError, match='depth_validity'):
        detect(scene(), depth_valid=np.zeros((240, 320), np.uint16))
    with pytest.raises(ValueError, match='explicit_center'):
        propose_rgb_observation_regions(scene(), policy=None)


@pytest.mark.parametrize('kwargs', [{'required_frames': 2}, {'required_frames': True},
                                  {'max_joint_delta_rad': float('nan')}])
def test_invalid_tracker_policy_is_rejected(kwargs):
    with pytest.raises(ValueError):
        RgbObservationTracker(**kwargs)


@pytest.mark.parametrize('kwargs', [{'stamp': 0}, {'stamp': True}, {'generation': True},
                                  {'source': ''}, {'joints': (float('nan'),)}])
def test_invalid_source_context_discards_prior_confirmation(kwargs):
    tracker, result = RgbObservationTracker(), detect(scene())
    for stamp in (1, 2, 3):
        track(tracker, result, stamp)
    with pytest.raises(ValueError):
        track(tracker, result, **kwargs)
    assert not track(tracker, result, 4).confirmed
