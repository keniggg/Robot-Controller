"""Independent exact ray/box scenes with visible sides and a measured table.

RGB, raw optical depth and whole-object masks come from the same intersections.
The ordinary geometry estimator is real; no inferred-box evidence is fabricated.
"""
from types import SimpleNamespace as NS

import numpy as np

from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_top, inferred_box_enclosure_vertices)
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def update_geometry(scene, *, require_ok=True):
    yy, xx = np.where(scene.mask > 0)
    scene.bbox = (int(xx.min()), int(yy.min()), int(xx.max()-xx.min()+1),
                  int(yy.max()-yy.min()+1))
    target = np.where(scene.mask > 0, scene.depth, 0)
    scene.geometry = estimate_object_geometry(
        scene.depth, target, scene.mask, scene.bbox, scene.K, scene.scale,
        scene.T, 'instance_mask', support_bbox_expand_ratio=.30,
        support_distance_threshold_m=.0035, voxel_size_m=.0015,
        min_support_points=200, min_object_points=80, min_size_m=.005,
        max_size_m=.6, max_height_m=.5, previous_axes_base=None,
        outlier_neighbors=16, outlier_std_ratio=2.,
        support_minimum_span_m=.1132)
    if require_ok:
        assert scene.geometry.ok, (scene.geometry.failure_code, scene.geometry.failure_reason)
    return scene


def oblique_box_scene(*, missing_top_edge=True):
    height, width = 240, 320
    K = NS(width=width, height=height, fx=500., fy=500.,
           cx=(width-1)/2., cy=(height-1)/2.)
    yaw = np.radians(21.)
    axes = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                     [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    size = np.array([.065, .035, .025])
    camera = axes @ np.array([.19, -.16, .32])
    forward = np.array([0., 0., size[2]/2.])-camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    T = np.eye(4)
    T[:3, :3] = np.column_stack((right, down, forward))
    T[:3, 3] = camera
    yy, xx = np.mgrid[:height, :width]
    rays = np.stack(((xx-K.cx)/K.fx, (yy-K.cy)/K.fy,
                     np.ones((height, width))), axis=-1) @ T[:3, :3].T
    local_rays, local_camera = rays @ axes, camera @ axes
    lower = np.array([-size[0]/2., -size[1]/2., 0.])
    upper = np.array([size[0]/2., size[1]/2., size[2]])
    with np.errstate(divide='ignore', invalid='ignore'):
        near_planes = np.minimum((lower-local_camera)/local_rays,
                                 (upper-local_camera)/local_rays)
        far_planes = np.maximum((lower-local_camera)/local_rays,
                                (upper-local_camera)/local_rays)
    entry, exit_ = near_planes.max(axis=2), far_planes.min(axis=2)
    table_t = -camera[2]/rays[:, :, 2]
    foreground = (entry > 0.) & (entry <= exit_) & (entry < table_t)
    optical_z = np.where(foreground, entry, table_t)
    exact_points = camera+rays*optical_z[:, :, None]
    local = exact_points @ axes
    top = foreground & (np.argmax(near_planes, axis=2) == 2)
    side = foreground & ~top
    color = np.full((height, width, 3), 24, np.uint8)
    color[top] = [70, 140, 220]
    color[side] = [35, 75, 120]
    scale = .0001
    depth = np.rint(optical_z/scale).astype(np.uint16)
    if missing_top_edge:
        # A narrow true top edge is unmeasured, while its current RGB and
        # both actually visible side faces remain intact.
        depth[top & (local[:, :, 1] > size[1]/2.-.0025)] = 0
    scene = NS(depth=depth, color=color, mask=foreground.astype(np.uint8)*255,
        top=top, side=side, exact_points=exact_points, local=local, rays=rays,
        K=K, T=T, scale=scale, true_axes=axes, true_size=size,
        identity=TargetTrackIdentity(1, 'g4-t1'), stamp=123_456_789_012)
    return update_geometry(scene)


def qualify_oblique(scene):
    return qualify_regular_box_top(depth_raw=scene.depth, object_mask=scene.mask,
        camera_intrinsics=scene.K, depth_scale_m=scene.scale,
        T_base_camera=scene.T, geometry=scene.geometry,
        source_identity=scene.identity, source_stamp_ns=scene.stamp,
        source_mode='instance_mask', color_bgr=scene.color)


def test_oblique_renderer_contains_measured_top_side_and_support():
    scene = oblique_box_scene()
    assert np.count_nonzero(scene.top) > 500
    assert np.count_nonzero(scene.side) > 500
    assert np.any(scene.top & (scene.depth == 0))
    np.testing.assert_allclose(scene.exact_points[~(scene.mask > 0), 2], 0., atol=1e-14)
    np.testing.assert_allclose(scene.local[scene.top, 2], scene.true_size[2], atol=1e-14)
    p = scene.local[scene.side]
    boundary_distance = np.minimum(np.abs(np.abs(p[:, 0])-scene.true_size[0]/2.),
                                   np.abs(np.abs(p[:, 1])-scene.true_size[1]/2.))
    assert np.max(boundary_distance) < 1e-14
    assert np.ptp(scene.geometry.object_points_base[:, 2]) > .01
    assert scene.geometry.support_inlier_ratio > .95


def assert_true_top_enclosed(scene, result):
    assert result.ok, (result.code, result.audit)
    model = result.model
    np.testing.assert_allclose(sorted(model.size_xyz_m[:2]),
                               sorted(scene.true_size[:2]), atol=.002, rtol=0.)
    corners = np.array([[x, y, scene.true_size[2]]
        for x in (-scene.true_size[0]/2., scene.true_size[0]/2.)
        for y in (-scene.true_size[1]/2., scene.true_size[1]/2.)]) @ scene.true_axes.T
    enclosure = inferred_box_enclosure_vertices(model) @ model.axes_base
    local = corners @ model.axes_base
    assert np.all(local >= enclosure.min(axis=0)-1e-10)
    assert np.all(local <= enclosure.max(axis=0)+1e-10)


def test_visible_side_silhouette_does_not_become_a_larger_top_rectangle():
    scene = oblique_box_scene()
    top_t = (scene.true_size[2]-scene.T[2, 3])/scene.rays[:, :, 2]
    projected = (scene.T[:3, 3]+scene.rays*top_t[:, :, None]) @ scene.true_axes
    # A producer projecting the whole silhouette onto the top would extend
    # at least one edge by >4 mm. Correct top-edge localization must not.
    assert np.max(np.abs(projected[scene.side, :2])-scene.true_size[:2]/2.) > .004
    measured = scene.geometry.object_points_base.copy()
    assert_true_top_enclosed(scene, qualify_oblique(scene))
    np.testing.assert_array_equal(scene.geometry.object_points_base, measured)


def test_table_shadow_and_small_mask_spill_do_not_expand_the_box():
    scene = oblique_box_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    shadow = (scene.mask == 0) & (np.abs(x) < .035) & (y < -.0175) & (y > -.034)
    spill = shadow & (np.abs(x) < .005) & (y > -.0195)
    assert np.count_nonzero(shadow) > 20 and np.count_nonzero(spill) > 0
    depth = scene.depth.copy()
    scene.color[shadow] = 8
    scene.mask[spill] = 255
    update_geometry(scene)
    np.testing.assert_array_equal(scene.depth, depth)
    np.testing.assert_allclose(scene.exact_points[shadow, 2], 0., atol=1e-14)
    assert_true_top_enclosed(scene, qualify_oblique(scene))


def test_internal_printing_cannot_replace_true_top_edges():
    scene = oblique_box_scene()
    x, y = scene.local[:, :, 0], scene.local[:, :, 1]
    inside = (np.abs(x) < .024) & (np.abs(y) < .010)
    border = scene.top & inside & ((np.abs(x) > .023) | (np.abs(y) > .009))
    scene.color[border] = 5
    result = qualify_oblique(scene)
    # A conservative rejection is allowed; an accepted inner print rectangle
    # must not shrink the physical box to fit the gripper.
    if result.ok:
        assert_true_top_enclosed(scene, result)


def test_cropped_top_edge_cannot_be_replaced_with_an_infinite_rgb_line():
    scene = oblique_box_scene()
    _, xx = np.where(scene.mask > 0)
    cut = int(xx.max()-4)
    for name in ('depth', 'color', 'mask', 'top', 'side', 'exact_points', 'local', 'rays'):
        setattr(scene, name, getattr(scene, name)[:, :cut].copy())
    scene.K.width = cut
    update_geometry(scene, require_ok=False)
    if scene.geometry.ok:
        result = qualify_oblique(scene)
        assert not result.ok, result.audit
        assert result.model is None


def test_large_unknown_top_band_keeps_rgb_box_unqualified():
    scene = oblique_box_scene()
    band = scene.top & (np.abs(scene.local[:, :, 0]) < .018)
    assert np.count_nonzero(band) > .4*np.count_nonzero(scene.top)
    scene.depth[band] = 0
    update_geometry(scene, require_ok=False)
    if scene.geometry.ok:
        result = qualify_oblique(scene)
        assert not result.ok, result.audit
        assert result.model is None
