"""Source-topology ownership with independently ray-cast current geometry."""
from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest
from alicia_grasp_modes import source_connected_owner as owner
from alicia_grasp_modes.tabletop import Candidate, Config, _matching_candidates, _project, _measured_support_footprint
from alicia_grasp_modes.rgb_instance import refine_candidate, freeze_observation_spatial_reference


def _candidate(mask, depth, intrinsics, transform, plane):
    points = _project(depth, intrinsics)[mask > 0].astype(float)
    assert np.all(np.isfinite(points)) and np.all(points[:, 0] >= .03)
    camera_center = np.median(points, axis=0)
    base_points = points @ transform[:3, :3].T + transform[:3, 3]
    heights = base_points @ plane[:3] + plane[3]
    centered = points - points.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    coordinates = centered @ axes.T
    extent = np.sort(np.percentile(coordinates, 98, axis=0)
                     - np.percentile(coordinates, 2, axis=0))[::-1]
    yy, xx = np.nonzero(mask)
    return Candidate(mask.copy(), np.array([xx.mean(), yy.mean()]),
        (int(xx.min()), int(yy.min()), int(np.ptp(xx) + 1), int(np.ptp(yy) + 1)),
        camera_center, transform[:3, :3] @ camera_center + transform[:3, 3],
        extent, float(camera_center[0]), .9,
        dict(valid_points=len(points), median_height_m=float(np.median(heights)),
             support_plane_base=plane.tolist(), support_residual_m=.0001),
        _measured_support_footprint(base_points, plane))


def current_box_scene(*, side_colour=(35, 35, 35), missing_side=False):
    """A dark measured side belongs to the same prism as its bright top."""
    shape = (240, 320)
    intrinsics = np.array([420., 420., 160., 120.])
    origin = np.array([0., -.30, .25])
    forward = np.array([0., .30, -.235])
    forward /= np.linalg.norm(forward)
    left = np.array([-1., 0., 0.])
    up = np.cross(forward, left)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((forward, left, up))
    transform[:3, 3] = origin
    yy, xx = np.indices(shape)
    rays = np.stack((np.ones(shape), -(xx - intrinsics[2]) / intrinsics[0],
                     -(yy - intrinsics[3]) / intrinsics[1]), axis=-1)
    world_rays = rays @ transform[:3, :3].T
    table_depth = -origin[2] / world_rays[..., 2]
    low = np.array([-.035, -.025, 0.])
    high = np.array([.035, .025, .028])
    with np.errstate(divide='ignore', invalid='ignore'):
        times_a = (low - origin) / world_rays
        times_b = (high - origin) / world_rays
    entries = np.minimum(times_a, times_b)
    exits = np.maximum(times_a, times_b)
    first, last = np.max(entries, axis=-1), np.min(exits, axis=-1)
    object_pixels = (first > 0.) & (first <= last) & (first < table_depth)
    entry_axis = np.argmax(entries, axis=-1)
    top = object_pixels & (entry_axis == 2)
    side = object_pixels & (entry_axis == 1)
    assert np.all(object_pixels == (top | side))
    depth = np.where(object_pixels, first, table_depth).astype(np.float32)
    bgr = np.full(shape + (3,), (55, 100, 185), np.uint8)
    bgr[top] = (220, 220, 220)
    bgr[side] = side_colour
    plane = np.array([0., 0., 1., 0.])
    full = (object_pixels.astype(np.uint8) * 255)
    # The reference supplies only a measured spatial proposal; it has no RGB
    # mask parameter and cannot write any current foreground pixels.
    anchor = _candidate(full, depth, intrinsics, transform, plane)
    spatial = freeze_observation_spatial_reference(anchor, plane)
    selected = _candidate(top.astype(np.uint8) * 255, depth,
                          intrinsics, transform, plane)
    if missing_side:
        depth[side] = 0.
    return dict(rgb=bgr, depth=depth, intrinsics=intrinsics, transform=transform,
        config=Config(), candidate=selected, spatial=spatial,
        truth=object_pixels, top=top, side=side, plane=plane)


def run_current_refiner(scene, *, spatial=True):
    return refine_candidate(scene['rgb'], scene['depth'], scene['intrinsics'],
        scene['transform'], scene['candidate'], scene['config'],
        observation_arrival_retry=spatial,
        observation_spatial_reference=scene['spatial'] if spatial else None)


def scene():
    s = current_box_scene()
    anchor = _candidate(s['truth'].astype(np.uint8)*255, s['depth'], s['intrinsics'], s['transform'], s['plane'])
    points = _project(s['depth'], s['intrinsics'])
    plane_camera = s['transform'].T @ s['plane']
    heights = np.einsum('ijk,k->ij', points, plane_camera[:3]) + plane_camera[3]
    measured_side = s['side'] & (heights >= s['config'].foreground_min_m)
    side = _candidate(measured_side.astype(np.uint8)*255, s['depth'], s['intrinsics'], s['transform'], s['plane'])
    result = NS(candidates=[s['candidate'], side], plane_base=s['plane'],
        metrics={'support_current_audit_available': True})
    assert len(_matching_candidates(result.candidates, anchor, s['candidate'], s['config'])) == 1
    arguments = dict(result=result, rgb=s['rgb'], depth=s['depth'], intrinsics=s['intrinsics'],
        transform=s['transform'], config=s['config'], anchor=anchor, last=s['candidate'],
        spatial_reference=s['spatial'], primary_refined=run_current_refiner(s), minimum_quality=.5,
        rgb_valid=np.ones(s['depth'].shape, bool), check_current=lambda: None)
    return s, arguments


@pytest.mark.parametrize('kind', ['one_pixel_bridge', 'missing_depth_bridge'])
def test_neighbor_objects_cannot_be_joined_by_a_one_pixel_or_missing_depth_bridge(kind):
    mask = np.zeros((30, 40), bool)
    mask[8:20, 3:15] = True
    mask[8:20, 25:37] = True
    mask[13, 15:25] = True
    depth = np.full(mask.shape, .3)
    left, right = np.zeros_like(mask), np.zeros_like(mask)
    left[10:18, 5:13] = True
    right[10:18, 27:35] = True
    if kind == 'missing_depth_bridge':
        depth[13, 20] = 0.
    with pytest.raises(ValueError, match='current_fragment_(single_pixel_measured_bridge|measured_group_disconnected)'):
        owner._reject_single_pixel_bridge(mask, depth, [left, right], .008, lambda: None)


def test_broad_measured_surface_is_not_a_single_pixel_bridge():
    mask = np.zeros((30, 40), bool)
    mask[8:20, 3:37] = True
    left, right = np.zeros_like(mask), np.zeros_like(mask)
    left[10:18, 5:13] = True
    right[10:18, 27:35] = True
    owner._reject_single_pixel_bridge(mask, np.full(mask.shape, .3), [left, right], .008, lambda: None)


def source_scene():
    from alicia_grasp_modes.source_sampling_topology import SourceSamplingTopology
    from test_gen30_source_sampling_topology import identity_camera
    s, arguments = scene()
    camera = identity_camera(s['depth'].shape, s['intrinsics'])
    arguments.update(camera=camera, source_stamp_ns=100,
        source_topology=SourceSamplingTopology(camera))
    return s, arguments


def test_complete_source_measurement_uses_original_rgb_and_all_native_cores():
    s, arguments = source_scene()
    before = s['depth'].copy()
    result = owner.recover_source_connected_fragment_owner(**arguments)
    assert result is not None
    assert np.mean(result.owner.mask[s['side']] > 0) >= .85
    assert np.mean(result.owner.mask[s['top']] > 0) >= .85
    assert not np.any((result.owner.mask > 0) & ~s['truth'])
    assert all(row['retained_fraction'] >= .85 for row in result.audit['native_members'])
    assert result.audit['connectivity_raster'] == 'pre_rectification_aligned_color'
    assert result.audit['measured_connectivity'] == 4
    assert result.audit['original_source_component_restored']
    assert result.audit['partial_outline_tolerance_used'] is False
    assert result.audit['depth_filled'] is False
    np.testing.assert_array_equal(before, s['depth'])


def test_source_component_cannot_lose_a_failed_native_core_or_original_rgb_gate(monkeypatch):
    _, arguments = source_scene()
    monkeypatch.setattr(owner._FrameRefinement, 'refine',
        lambda *a, **k: arguments['primary_refined'])
    with pytest.raises(ValueError, match='native_core_not_owned'):
        owner.recover_source_connected_fragment_owner(**arguments)
    def failed(*a, **k): raise ValueError('original_rgb_gate_failed')
    monkeypatch.setattr(owner._FrameRefinement, 'refine', failed)
    with pytest.raises(ValueError, match='original_rgb_gate_failed'):
        owner.recover_source_connected_fragment_owner(**arguments)


@pytest.mark.parametrize('kind', ['missing', 'wrong_intrinsics', 'old_activation'])
def test_source_component_requires_exact_current_geometry(kind):
    _, arguments = source_scene()
    if kind == 'missing': arguments['source_topology'] = None
    elif kind == 'wrong_intrinsics': arguments['intrinsics'] = arguments['intrinsics'] + [1., 0., 0., 0.]
    else: arguments['camera']['runtime_profile']['geometry_activation_stamp_ns'] = '100'
    with pytest.raises(ValueError):
        owner.recover_source_connected_fragment_owner(**arguments)


def test_source_component_deadline_exception_propagates():
    _, arguments = source_scene()
    class Deadline(ValueError): pass
    def expired(): raise Deadline('original_source_age')
    arguments['check_current'] = expired
    with pytest.raises(Deadline, match='original_source_age'):
        owner.recover_source_connected_fragment_owner(**arguments)
