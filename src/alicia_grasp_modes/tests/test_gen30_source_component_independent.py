"""Independent scene-wide measurement vetoes for source-component recovery.

The ray-cast fixture supplies calibrated current geometry. An extra measured
object is deliberately absent from the accelerated candidate list. Only the
RGB output is interposed in the veto tests, so these prove the consumer check;
the separate recorded-frame replay establishes real RGB reachability.
"""
from dataclasses import replace

import numpy as np
import pytest

from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.tabletop import _project
from test_gen30_source_connected_owner import source_scene


def _add_foreign_raw_object(scene, arguments):
    mask = np.zeros(scene['depth'].shape, bool)
    mask[20:32, 20:32] = True
    assert not np.any(mask & scene['truth'])
    assert not any(np.any(mask & (c.mask > 0)) for c in arguments['result'].candidates)
    rays = _project(np.ones(mask.shape, np.float32), scene['intrinsics'])
    plane_camera = scene['transform'].T @ scene['plane']
    denominator = np.einsum('ijk,k->ij', rays, plane_camera[:3])
    depth = ((.028-plane_camera[3])/denominator).astype(np.float32)
    scene['depth'][mask] = depth[mask]
    scene['rgb'][mask] = (220, 220, 220)
    assert mask.sum() >= arguments['config'].min_object_points
    return mask


@pytest.mark.parametrize('overlap_count', [1, 144])
def test_foreign_raw_component_cannot_hide_outside_candidate_filter(monkeypatch, overlap_count):
    scene, arguments = source_scene()
    original = owner.recover_source_connected_fragment_owner(**arguments).owner
    neighbor = _add_foreign_raw_object(scene, arguments)
    expanded = original.mask.copy()
    flat = np.flatnonzero(neighbor)[:overlap_count]
    expanded.ravel()[flat] = 255
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *a, **k: replace(original, mask=expanded))
    with pytest.raises(ValueError, match='foreign_raw_source_component_overlap'):
        owner.recover_source_connected_fragment_owner(**arguments)


def test_unrelated_measured_object_does_not_change_recovery_seed_or_output():
    scene, arguments = source_scene()
    neighbor = _add_foreign_raw_object(scene, arguments)
    before = scene['depth'].copy()
    result = owner.recover_source_connected_fragment_owner(**arguments)
    assert result is not None
    assert not np.any((result.owner.mask > 0) & neighbor)
    assert result.audit['foreign_raw_foreground_components_checked'] >= 1
    assert result.audit['historical_pixels_used'] == 0
    np.testing.assert_array_equal(before, scene['depth'])


def test_raw_only_measured_bridge_is_checked_before_rgb_growth(monkeypatch):
    scene, arguments = source_scene()
    y = int(np.median(np.nonzero(scene['top'])[0]))
    edge = int(np.flatnonzero(scene['top'][y])[0])
    extra = np.zeros(scene['depth'].shape, bool)
    extra[y-6:y+6, edge-19:edge-7] = True
    extra[y, edge-7:edge] = True
    assert not np.any(extra & scene['truth'])
    assert not any(np.any(extra & (c.mask > 0)) for c in arguments['result'].candidates)
    rays = _project(np.ones(extra.shape, np.float32), scene['intrinsics'])
    pc = scene['transform'].T @ scene['plane']
    depth = ((.028-pc[3])/np.einsum('ijk,k->ij', rays, pc[:3])).astype(np.float32)
    scene['depth'][extra] = depth[extra]
    scene['rgb'][extra] = (220, 220, 220)
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *a, **k: pytest.fail('unproven raw bridge reached RGB growth'))
    with pytest.raises(ValueError, match='single_pixel_measured_bridge'):
        owner.recover_source_connected_fragment_owner(**arguments)


def test_identity_source_diagonal_contact_is_not_a_four_neighbor_connection():
    # An identity map supplies no evidence for restoring diagonal adjacency.
    # Two real sampled squares share only a corner, even at identical depth.
    group = np.zeros((30, 30), bool)
    first, second = np.zeros_like(group), np.zeros_like(group)
    first[3:13, 3:13] = True
    second[13:23, 13:23] = True
    group |= first | second
    # The helper normally receives a precomputed connected component. This
    # deliberately invalid whole group may fail the cut or connectivity check;
    # neither outcome may turn diagonal source cells into an admitted object.
    with pytest.raises(ValueError, match='(measured_group_disconnected|single_pixel_measured_bridge)'):
        owner._reject_single_pixel_bridge(group, np.full(group.shape, .3),
            [first, second], .008, lambda: None)
