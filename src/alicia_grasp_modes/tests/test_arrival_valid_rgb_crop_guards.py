"""Only invalid optional background can be trimmed from a spatial RGB search."""
from types import SimpleNamespace

import numpy as np
import pytest

from alicia_grasp_modes.rgb_instance import (
    instance_mask_from_seed, freeze_observation_spatial_reference,
    _observation_spatial_support)
from alicia_grasp_modes.tabletop import Config, _project
from test_observation_arrival_rgb_retry import scene, grabcut_stub


def inputs(*, small_proposal=False):
    rgb, seed = scene()
    config = Config()
    depth = np.full(seed.shape, .3, np.float32)
    depth[seed > 0] = .28
    points = _project(depth, [400., 400., 320., 240.])
    plane = np.array([-1., 0., 0., .3])
    half_y, half_z = (.0075, .0075) if small_proposal else (.03, .02)
    footprint = np.array([[.3, -half_y, -half_z], [.3, half_y, -half_z],
                          [.3, half_y, half_z], [.3, -half_y, half_z]])
    anchor = SimpleNamespace(support_footprint_base=footprint,
        metrics=dict(median_height_m=.02))
    reference = freeze_observation_spatial_reference(anchor, plane)
    spatial = _observation_spatial_support(points, np.eye(4),
        [400., 400., 320., 240.], reference, config)
    return rgb, seed, (points, depth, .3-points[..., 0], plane[:3], config), spatial


@pytest.mark.parametrize('region', ['seed', 'first16', 'proposal', 'proposal_guard'])
def test_invalid_required_rgb_cannot_be_trimmed_or_relabelled(monkeypatch, region):
    rgb, seed, geometry, spatial = inputs(small_proposal=region == 'first16')
    valid = np.ones(seed.shape, bool)
    point = dict(seed=(240, 320), first16=(215, 300),
                 proposal=(240, 280), proposal_guard=(240, 274))[region]
    valid[point] = False
    original = [value.copy() for value in (rgb, seed, geometry[1], valid)]
    calls = grabcut_stub(monkeypatch, [])
    reason = dict(seed='rgb_seed_contains_unobserved_rgb',
                  first16='rgb_search_contains_unobserved_rgb',
                  proposal='rgb_required_spatial_region_unobserved',
                  proposal_guard='rgb_required_spatial_region_unobserved')[region]
    with pytest.raises(ValueError, match='^'+reason+'$'):
        instance_mask_from_seed(rgb, seed, geometry=geometry, rgb_valid=valid,
            observation_arrival_retry=True, observation_spatial_support=spatial)
    assert calls == []
    for actual, expected in zip((rgb, seed, geometry[1], valid), original):
        np.testing.assert_array_equal(actual, expected)


def test_padding_only_crop_keeps_whole_required_region_and_original_frozen_core(monkeypatch):
    rgb, seed, geometry, spatial = inputs()
    valid = np.ones(seed.shape, bool)
    valid[:, :270] = False  # Desired left starts at 261; proposal guard starts at 273.
    original = [value.copy() for value in (rgb, seed, geometry[1], valid)]
    calls = grabcut_stub(monkeypatch, ['valid', 'valid'])
    baseline, baseline_evidence = instance_mask_from_seed(rgb, seed, geometry=geometry,
        rgb_valid=np.ones(seed.shape, bool), observation_arrival_retry=True,
        observation_spatial_support=spatial)
    actual, evidence = instance_mask_from_seed(rgb, seed, geometry=geometry,
        rgb_valid=valid, observation_arrival_retry=True, observation_spatial_support=spatial)
    x0, y0, x1, y1 = evidence['rgb_search_box_xyxy']
    bx0, by0, bx1, by1 = baseline_evidence['rgb_search_box_xyxy']
    assert bx0 < 270 <= x0 and bx0 <= x0 < x1 <= bx1 and by0 <= y0 < y1 <= by1
    assert np.all(valid[y0:y1, x0:x1])
    px0, py0, px1, py1 = spatial['search_box_xyxy']
    assert x0 <= min(294, px0-4) and y0 <= min(214, py0-4)
    assert x1 >= max(346, px1+4) and y1 >= max(266, py1+4)
    assert evidence['rgb_search_attempts'] == 1
    np.testing.assert_array_equal(actual, baseline)
    np.testing.assert_array_equal(actual, seed)
    assert len(calls) == 2
    np.testing.assert_array_equal(calls[0][1], calls[1][1])
    for observed, expected in zip((rgb, seed, geometry[1], valid), original):
        np.testing.assert_array_equal(observed, expected)


def test_target_touching_trimmed_search_edge_still_fails_without_extra_attempt(monkeypatch):
    rgb, seed, geometry, spatial = inputs()
    valid = np.ones(seed.shape, bool); valid[:, :270] = False
    calls = grabcut_stub(monkeypatch, [('left',), ('left',)])
    with pytest.raises(ValueError, match='^rgb_instance_reaches_search_boundary$'):
        instance_mask_from_seed(rgb, seed, geometry=geometry, rgb_valid=valid,
            observation_arrival_retry=True, observation_spatial_support=spatial)
    assert len(calls) == 2
    np.testing.assert_array_equal(calls[0][1], calls[1][1])


def test_ordinary_initial_search_keeps_its_original_invalid_background_rejection(monkeypatch):
    rgb, seed, geometry, _ = inputs()
    valid = np.ones(seed.shape, bool); valid[215, 300] = False
    calls = grabcut_stub(monkeypatch, [])
    with pytest.raises(ValueError, match='^rgb_search_contains_unobserved_rgb$'):
        instance_mask_from_seed(rgb, seed, geometry=geometry, rgb_valid=valid)
    assert calls == []
