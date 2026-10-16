"""Locked identity survives distant clutter without clipping measured components."""
import numpy as np
import pytest
from alicia_grasp_modes.tabletop import _depth_components, _project


def components(mask, depth, target=None):
    return _depth_components(mask, depth, 20, .008,
        points_camera=_project(depth, (100., 100., 100., 100.)),
        target_camera=target, association_distance=.025)


def test_distant_clutter_does_not_exhaust_locked_budget_but_initial_cap_remains():
    depth = np.full((200, 200), .3, np.float32)
    mask = np.zeros(depth.shape, np.uint8)
    for y in range(5, 70, 10):
        for x in range(5, 190, 10): mask[y:y+5, x:x+5] = 1
    mask[97:104, 97:104] = 1
    with pytest.raises(ValueError, match='too_many_foreground_instances'):
        components(mask, depth)
    found = components(mask, depth, np.array([.3, 0, 0]))
    assert len(found) == 1
    assert np.count_nonzero(found[0]) == 49
    assert np.all(found[0][97:104, 97:104])


def test_both_nearby_instances_are_retained_for_ambiguity_gate():
    depth = np.full((200, 200), .3, np.float32)
    mask = np.zeros(depth.shape, np.uint8)
    mask[95:115, 93:97] = mask[95:115, 99:103] = 1
    found = components(mask, depth, np.array([.3, 0, 0]))
    assert len(found) == 2
    assert np.array_equal(np.any(found, axis=0), mask.astype(bool))


def test_intersecting_parent_kept_whole_even_when_its_median_is_far():
    depth = np.full((200, 200), .4, np.float32)
    mask = np.zeros(depth.shape, np.uint8)
    mask[100:110, 100:180] = 1
    depth[100:110, 100:110] = .3
    target = np.array([.3, -.015, -.015])
    found = components(mask, depth, target)
    assert len(found) == 2  # Both measured children survive; association runs later.
    assert np.array_equal(np.any(found, axis=0), mask.astype(bool))


def test_more_than_32_nearby_components_still_fail_without_truncating_candidates():
    depth = np.full((200, 200), .3, np.float32)
    mask = np.zeros(depth.shape, np.uint8)
    for y in range(5, 70, 10):
        for x in range(5, 70, 10): mask[y:y+5, x:x+5] = 1
    points = np.zeros(depth.shape + (3,)); points[..., 0] = .3
    with pytest.raises(ValueError, match='too_many_foreground_instances'):
        _depth_components(mask, depth, 20, .008, points_camera=points,
                          target_camera=np.array([.3, 0, 0]), association_distance=.025)


@pytest.mark.parametrize('target,radius', [([float('nan'),0,0],.025),
                                         ([.3,0],.025),([.3,0,0],-1)])
def test_invalid_region_fails_closed(target, radius):
    depth = np.full((10,10), .3, np.float32)
    with pytest.raises(ValueError, match='invalid_locked_component_region'):
        _depth_components(np.ones((10,10), np.uint8),depth,20,.008,
            points_camera=np.zeros((10,10,3)),target_camera=target,association_distance=radius)
