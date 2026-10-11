"""Arrival fragment proposals never replace current depth or unique identity."""
from copy import deepcopy
from dataclasses import replace

import cv2
import numpy as np
import pytest

from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.tabletop import Candidate, Config, Segmentation


@pytest.mark.parametrize('case', ['two_original_matches', 'only_incomplete_fragments'])
def test_arrival_spatial_proposal_cannot_relax_original_raw_uniqueness(monkeypatch, case):
    import alicia_grasp_modes.observation_arrival_confirmation as arrival

    config = Config()
    plane = np.array([-1., 0., 0., .3])
    seed = np.zeros((80, 100), np.uint8)
    seed[30:50, 40:60] = 255
    anchor = Candidate(seed, np.array([49.5, 39.5]), (40, 30, 20, 20),
        np.array([.28, 0., 0.]), np.array([.28, 0., 0.]),
        np.array([.04, .02, .01]), .28, .9,
        dict(valid_points=400, median_height_m=.02, support_plane_base=plane.tolist()),
        np.array([[.3, -.02, -.01], [.3, .02, -.01],
                  [.3, .02, .01], [.3, -.02, .01]]))
    if case == 'two_original_matches':
        candidates = [deepcopy(anchor), deepcopy(anchor)]
        candidates[1].position_base[1] += .005
    else:
        candidates = [replace(anchor, extent_m=np.array([.04, .009, .005]))]
    result = Segmentation(candidates, plane.copy(), dict(
        support_current_audit_available=True,
        support_current_audit_residual_m=.001,
        support_reference_angle_deg=0., support_reference_max_distance_m=.001,
        support_current_audit_inlier_ratio=.9, support_current_audit_quadrants=4))
    before = (anchor.position_base.copy(), anchor.support_footprint_base.copy(), plane.copy())
    helper = ObservationArrivalConfirmation(config, anchor, anchor, plane,
        np.ones(16)/16., np.array([[[0, 0]], [[1, 0]], [[1, 1]], [[0, 1]]], np.int32), .5)

    def forbidden(*args, **kwargs):
        pytest.fail('RGB or historical geometry must not resolve non-unique raw identity')

    monkeypatch.setattr(arrival, 'refine_candidate', forbidden)
    with pytest.raises(ValueError, match='^arrival_unique_original_identity_required$'):
        helper.observe(result, np.full((80, 100, 3), 100, np.uint8),
            np.full((80, 100), .28, np.float32), [400., 400., 50., 40.],
            np.eye(4), 123)
    assert helper.source_stamps_ns == []
    assert helper.last_evidence is None
    np.testing.assert_array_equal(anchor.position_base, before[0])
    np.testing.assert_array_equal(anchor.support_footprint_base, before[1])
    np.testing.assert_array_equal(helper.plane, before[2])


def spatial_scene(missing=False):
    """A small measured target, surrounded by unrelated same-height depth."""
    from alicia_grasp_modes import rgb_instance
    from alicia_grasp_modes.tabletop import _project

    config = Config()
    color = np.full((480, 640, 3), 150, np.uint8)
    seed = np.zeros(color.shape[:2], np.uint8)
    seed[230:250, 310:330] = 255
    depth = np.full(seed.shape, 0. if missing else .28, np.float32)
    depth[seed > 0] = .28
    points = _project(depth, [400., 400., 320., 240.])
    heights = .3 - points[..., 0]
    plane = np.array([-1., 0., 0., .3])
    footprint = np.array([[.3, -.0075, -.0075], [.3, .0075, -.0075],
                          [.3, .0075, .0075], [.3, -.0075, .0075]])
    anchor = Candidate(seed.copy(), np.array([319.5, 239.5]), (310, 230, 20, 20),
        np.array([.28, 0., 0.]), np.array([.28, 0., 0.]),
        np.array([.02, .02, .02]), .28, .9,
        dict(valid_points=400, median_height_m=.02, support_plane_base=plane.tolist()),
        footprint)
    reference = rgb_instance.freeze_observation_spatial_reference(anchor, plane)
    support = rgb_instance._observation_spatial_support(
        points, np.eye(4), [400., 400., 320., 240.], reference, config)
    return color, seed, (points, depth, heights, plane[:3], config), reference, support


def oversized_connected_output(monkeypatch):
    """Control the segmentation outcome to isolate the original area guard."""
    calls = []

    def grabcut(patch, labels, *args):
        core = labels == cv2.GC_FGD
        labels[:] = cv2.GC_BGD
        labels[4:-4, 4:-4] = cv2.GC_PR_FGD
        assert np.all(labels[core] == cv2.GC_PR_FGD)
        calls.append(labels.shape)

    monkeypatch.setattr(cv2, 'grabCut', grabcut)
    return calls


@pytest.mark.parametrize('missing', [False, True], ids=['neighbour_depth', 'missing_current_depth'])
def test_neighbour_or_missing_depth_cannot_be_replaced_by_historical_extent(monkeypatch, missing):
    from alicia_grasp_modes.rgb_instance import instance_mask_from_seed

    color, seed, geometry, reference, support = spatial_scene(missing)
    before = [value.copy() for value in (color, seed, geometry[0], geometry[1])]
    assert support['domain_mask'][240, 320]
    if not missing:
        assert not support['domain_mask'][240, 345]
    else:
        # A zero-depth point can project inside the lateral domain. The
        # separate CURRENT depth/height check still must not count it.
        assert geometry[1][240, 345] == 0.
    calls = oversized_connected_output(monkeypatch)
    with pytest.raises(ValueError, match='^rgb_instance_not_supported_by_current_target_depth$'):
        instance_mask_from_seed(color, seed, geometry=geometry,
            observation_arrival_retry=True, observation_spatial_support=support)
    assert len(calls) == 1  # A failed support area is not a boundary retry.
    for actual, expected in zip((color, seed, geometry[0], geometry[1]), before):
        np.testing.assert_array_equal(actual, expected)


def test_ordinary_rgb_keeps_original_area_and_search_without_arrival_proposal(monkeypatch):
    from alicia_grasp_modes.rgb_instance import instance_mask_from_seed
    from test_observation_arrival_rgb_retry import scene
    from test_observation_arrival_probable_foreground import geometry

    color, seed = scene()
    calls = oversized_connected_output(monkeypatch)
    with pytest.raises(ValueError, match='^rgb_instance_not_supported_by_depth_seed$'):
        instance_mask_from_seed(color, seed, geometry=geometry(seed))
    assert calls == [(52, 52)]
