"""Independent counterexamples for bound arrival raw-zero fragment recovery."""
from copy import deepcopy
from dataclasses import replace

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_fragment_group as group
from alicia_grasp_modes.rgb_instance import freeze_observation_spatial_reference
from alicia_grasp_modes.rgb_observation_handoff import _appearance, _mask
from alicia_grasp_modes.tabletop import (
    Candidate, Config, Segmentation, _matching_candidates,
    _measured_support_footprint, _project)


K = [400., 400., 320., 240.]
PLANE = np.array([-1., 0., 0., .3])


def measured_candidate(mask, depth):
    xyz = _project(depth, K)[mask > 0].astype(float)
    position = np.median(xyz, axis=0)
    centered = xyz - xyz.mean(0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    projected = centered @ axes.T
    extent = np.sort(np.percentile(projected, 98, axis=0)
                     - np.percentile(projected, 2, axis=0))[::-1]
    yy, xx = np.nonzero(mask)
    return Candidate(mask.copy(), np.array([xx.mean(), yy.mean()]),
        (int(xx.min()), int(yy.min()), int(np.ptp(xx)+1), int(np.ptp(yy)+1)),
        position.copy(), position.copy(), extent, float(position[0]), .9,
        dict(valid_points=len(xyz), support_plane_base=PLANE.tolist(),
             support_residual_m=.0005, median_height_m=float(np.median(.3-xyz[:, 0]))),
        _measured_support_footprint(xyz, PLANE))


def fragment_scene():
    rgb = np.full((480, 640, 3), (60, 90, 190), np.uint8)
    full = np.zeros(rgb.shape[:2], np.uint8)
    full[210:250, 285:345] = 255
    rgb[full > 0] = 20
    reference_depth = np.full(full.shape, .3, np.float32)
    reference_depth[full > 0] = .28
    anchor = measured_candidate(full, reference_depth)
    depth = reference_depth.copy()
    depth[full > 0] = 0.
    members = []
    for sl in (np.s_[212:220, 287:343], np.s_[240:248, 287:343]):
        member = np.zeros(full.shape, np.uint8)
        member[sl] = 255
        depth[sl] = .28
        members.append(member)
    config = Config()
    candidates = [measured_candidate(mask, depth) for mask in members]
    result = Segmentation(candidates, PLANE.copy(),
        dict(support_current_audit_available=True))
    assert not _matching_candidates(candidates, anchor, anchor, config)
    histogram, contour = _appearance(rgb, _mask(full, full.shape))
    reference = freeze_observation_spatial_reference(anchor, PLANE)
    return dict(result=result, rgb=rgb, depth=depth, intrinsics=K,
        transform=np.eye(4), config=config, anchor=anchor, last=anchor,
        spatial_reference=reference, histogram=histogram, contour=contour,
        minimum_quality=.5)


def test_fragment_positive_control_uses_only_current_depth_and_keeps_reference():
    inputs = fragment_scene()
    before_depth = inputs['depth'].copy()
    before_footprint = inputs['anchor'].support_footprint_base.copy()
    result, pair = group.recover_arrival_fragment_group(**inputs)
    assert pair is not None and len(result.candidates) == 1
    raw, refined = pair
    assert _matching_candidates([refined], inputs['anchor'], inputs['last'], inputs['config'])
    assert not np.any((raw.mask > 0) & (inputs['depth'] == 0.))
    assert raw.metrics['valid_points'] == np.count_nonzero(raw.mask)
    assert raw.metrics['observation_fragment_group']['historical_depth_points'] == 0
    np.testing.assert_array_equal(inputs['depth'], before_depth)
    np.testing.assert_array_equal(inputs['anchor'].support_footprint_base, before_footprint)


@pytest.mark.parametrize('case', ['outside_neighbour', 'missing_current_depth'])
def test_history_or_outside_neighbour_cannot_supply_missing_current_member(monkeypatch, case):
    inputs = fragment_scene()
    if case == 'outside_neighbour':
        outside = np.zeros(inputs['depth'].shape, np.uint8)
        outside[240:248, 360:416] = 255
        inputs['depth'][outside > 0] = .28
        inputs['result'].candidates[1] = measured_candidate(outside, inputs['depth'])
    else:
        for candidate in inputs['result'].candidates:
            inputs['depth'][candidate.mask > 0] = 0.
    before = inputs['depth'].copy()
    def forbidden(*args, **kwargs):
        pytest.fail('historical or foreign points must not reach RGB group refinement')
    monkeypatch.setattr(group._FrameRefinement, 'refine', forbidden)
    with pytest.raises(ValueError, match='^arrival_fragment_group_insufficient_members$'):
        group.recover_arrival_fragment_group(**inputs)
    np.testing.assert_array_equal(inputs['depth'], before)


def disconnected_scene(zero_core_large=False):
    inputs = fragment_scene()
    depth = np.full(inputs['depth'].shape, .3, np.float32)
    big = np.zeros(depth.shape, np.uint8); big[212:222, 287:343] = 255
    small = np.zeros(depth.shape, np.uint8); small[240:248, 318:328] = 255
    depth[big > 0] = .285 if zero_core_large else .28
    depth[small > 0] = .275 if zero_core_large else .28
    rgb = np.full_like(inputs['rgb'], (60, 90, 190))
    rgb[big > 0] = 20; rgb[small > 0] = (20, 180, 20)
    inputs.update(depth=depth, rgb=rgb)
    inputs['result'].candidates = [measured_candidate(mask, depth) for mask in (big, small)]
    assert not _matching_candidates(inputs['result'].candidates, inputs['anchor'], inputs['last'], inputs['config'])
    return inputs, big, small


def test_small_separate_rgb_instance_cannot_hide_below_aggregate_fifteen_percent(monkeypatch):
    inputs, big, small = disconnected_scene()
    # Real planar depths yield 432 big-core and 48 small-core pixels: the
    # single aggregate .85 test would accept the big component alone.
    points = _project(inputs['depth'], K)
    core, _, _ = group.confidence_labels((big | small) > 0, points, inputs['depth'],
        .3-points[..., 0], PLANE[:3], inputs['config'], bgr=inputs['rgb'])
    assert np.count_nonzero(core & (big > 0))/np.count_nonzero(core) > .85
    assert np.count_nonzero(core & (small > 0)) > 0
    selected = measured_candidate(big, inputs['depth'])
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *args, **kwargs: selected)
    with pytest.raises(ValueError, match='^arrival_fragment_group_member_not_owned$'):
        group.recover_arrival_fragment_group(**inputs)


def test_unowned_coreless_member_cannot_lend_geometry_to_final_instance(monkeypatch):
    inputs, big, small = disconnected_scene(zero_core_large=True)
    points = _project(inputs['depth'], K)
    core, _, _ = group.confidence_labels((big | small) > 0, points, inputs['depth'],
        .3-points[..., 0], PLANE[:3], inputs['config'], bgr=inputs['rgb'])
    assert np.count_nonzero(core & (big > 0)) == 0
    assert np.count_nonzero(core & (small > 0)) >= 25
    selected = measured_candidate(small, inputs['depth'])
    assert not _matching_candidates([selected], inputs['anchor'], inputs['last'], inputs['config'])
    monkeypatch.setattr(group._FrameRefinement, 'refine', lambda *args, **kwargs: selected)
    with pytest.raises(ValueError, match='^arrival_fragment_group_final_geometry_required$'):
        group.recover_arrival_fragment_group(**inputs)


@pytest.mark.parametrize('case', ['one_raw_match', 'two_raw_matches', 'no_bound_reference'])
def test_existing_raw_match_or_absent_observation_scope_keeps_original_path(monkeypatch, case):
    inputs = fragment_scene()
    if case == 'no_bound_reference':
        inputs['spatial_reference'] = None
    else:
        inputs['result'].candidates = [deepcopy(inputs['anchor'])] * (1 if case == 'one_raw_match' else 2)
    before = inputs['result']
    def forbidden(*args, **kwargs):
        pytest.fail('existing raw identity or unbound scope must not use fragment recovery')
    monkeypatch.setattr(group._FrameRefinement, 'refine', forbidden)
    result, pair = group.recover_arrival_fragment_group(**inputs)
    assert result is before and pair is None


def fragment_frame(node, clock, joint):
    from test_locked_rgb_hold import custom_frame
    depth = np.zeros((28, 25), np.float32)
    depth[2:10, 3:21] = .35
    depth[18:26, 3:21] = .35
    clock[0] += .2
    joint(clock[0]-.2); joint(clock[0]-.1)
    return custom_frame(node, clock, joint, target_depth=depth)


@pytest.mark.parametrize('scope', ['ordinary', 'expired_session', 'fixed_q_excursion'])
def test_raw_zero_recovery_cannot_outlive_qualified_observation_station(monkeypatch, scope):
    from test_observation_arrival_station import completed
    from test_locked_rgb_hold import locked, module

    if scope == 'ordinary':
        node, clock, feed, joint, params = locked(monkeypatch)
    else:
        node, clock, feed, joint, params, marker = completed(monkeypatch)
        if scope == 'expired_session':
            marker['session_deadline_ns'] = module.rospy.Time.now().to_nsec()
        else:
            joint(clock[0], [.005, 0., 0., 0., 0., 0.])
            joint(clock[0]+.01, [0.]*6)
    anchor = node.tracker.anchor
    before_source = node.last_good_source_stamp_ns
    def forbidden(*args, **kwargs):
        pytest.fail('unqualified station must not enter raw-zero fragment recovery')
    monkeypatch.setattr(module, 'recover_arrival_fragment_group', forbidden)
    fragment_frame(node, clock, joint)
    assert node.tracker.anchor is anchor
    assert node.last_good_source_stamp_ns == before_source
    assert not node.pubs['object'].publish.call_args[0][0].detected


def test_normal_fragment_recovery_preserves_fixed_identity_and_never_fills_next_depth_gap(monkeypatch):
    from test_observation_arrival_station import completed
    from test_locked_rgb_hold import custom_frame, latest_status

    node, clock, feed, joint, params, marker = completed(monkeypatch)
    anchor = node.tracker.anchor
    plane = node.tracker.plane_base.copy()
    reference = marker['spatial_reference']
    footprint = reference['footprint_base'].copy()
    histogram = marker['fragment_histogram'].copy()
    contour = marker['fragment_contour'].copy()
    stamps = []
    for _ in range(3):
        pair = fragment_frame(node, clock, joint)
        stamps.append(pair[0].header.stamp.to_nsec())
        assert latest_status(node)['state'] == 'ready', latest_status(node)
        assert node.pubs['object'].publish.call_args[0][0].detected
        assert node.last_good_source_stamp_ns == stamps[-1]
        assert node._locked_rgb_hold_reference['observation_arrival_retry'] is marker
        assert marker['spatial_reference'] is reference
        assert node.tracker.anchor is anchor
        np.testing.assert_array_equal(node.tracker.plane_base, plane)
        np.testing.assert_array_equal(reference['footprint_base'], footprint)
        np.testing.assert_array_equal(marker['fragment_histogram'], histogram)
        np.testing.assert_array_equal(marker['fragment_contour'], contour)
    assert stamps == sorted(set(stamps))
    clock[0] += .2
    custom_frame(node, clock, joint, target_depth=0.)
    assert node.last_good_source_stamp_ns == stamps[-1]
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.tracker.anchor is anchor
