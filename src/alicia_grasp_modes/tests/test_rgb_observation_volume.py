"""Observation-only RGB/slab geometry cannot substitute for grasp evidence."""
from dataclasses import FrozenInstanceError, asdict, replace
import json

import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_volume import (
    RGBObservationVolume, SupportPlaneProvenance, build_rgb_observation_volume,
    project_rgb_observation_volume)


STAMP = 1791073779552586078
K = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])
T = np.array([[0., -1., 0., .1], [0., 0., 1., -.2],
              [-1., 0., 0., .5], [0., 0., 0., 1.]])
PLANE = (0., 0., 1., 0.)


def evidence(**overrides):
    values = dict(source_stamp_ns=STAMP, support_plane_base=PLANE,
        source_record_id='exact-frame/independent-support-fit', inlier_count=830,
        inlier_ratio=.94, fit_rmse_m=.0006, spatial_quadrants=4)
    values.update(overrides)
    return SupportPlaneProvenance(**values)


def inputs(**overrides):
    values = dict(bbox_xywh=(300, 220, 40, 40), image_shape=(480, 640),
        camera_matrix=K.copy(), T_base_camera=T.copy(), support_plane_base=PLANE,
        support_provenance=evidence(), foreground_max_m=.12,
        source_stamp_ns=STAMP, working_depth_range_m=(.07, .7))
    values.update(overrides)
    return values


def build(**overrides):
    return build_rgb_observation_volume(**inputs(**overrides))


def project(volume, **overrides):
    values = dict(T_base_camera=T, camera_matrix=K, image_shape=(480, 640),
                  working_depth_range_m=(.07, .7))
    values.update(overrides)
    return project_rgb_observation_volume(volume, **values)


def test_encloses_whole_pixel_areas_and_intersects_both_slab_boundaries():
    volume = build()
    boundary = np.array([[299.5, 219.5], [339.5, 219.5],
                         [339.5, 259.5], [299.5, 259.5]])
    np.testing.assert_array_equal(volume.pixel_boundary_xy, boundary)
    vertices = np.asarray(volume.vertices_base_m)
    np.testing.assert_allclose(vertices[:4, 2], 0., atol=1e-15)
    np.testing.assert_allclose(vertices[4:, 2], .12, atol=1e-15)
    np.testing.assert_allclose(vertices[:, 0], .1+np.tile((boundary[:, 0]-320)/400, 2)
                               *np.repeat([.5, .38], 4))
    np.testing.assert_allclose(vertices[:, 1], -.2-np.tile((boundary[:, 1]-240)/400, 2)
                               *np.repeat([.5, .38], 4))
    np.testing.assert_allclose(volume.vertex_optical_depths_m, [.5]*4+[.38]*4)
    np.testing.assert_array_equal(volume.bounds_base_m, [vertices.min(0), vertices.max(0)])
    np.testing.assert_allclose(project(volume), np.vstack((boundary, boundary)), atol=1e-12)


def test_explicit_general_scene_height_is_required_not_five_mm_object_prior():
    args = inputs()
    del args['foreground_max_m']
    with pytest.raises(TypeError):
        build_rgb_observation_volume(**args)
    assert build().foreground_max_m == .12
    assert np.max(np.asarray(build().vertices_base_m)[:, 2]) == pytest.approx(.12)


def test_immutable_model_has_no_measured_object_points_or_motion_rights():
    args = inputs()
    volume = build_rgb_observation_volume(**args)
    args['T_base_camera'][0, 3] = 100.
    args['camera_matrix'][0, 0] = 1.
    assert volume.T_base_camera[0][3] == .1
    assert volume.camera_matrix[0][0] == 400.
    assert volume.observation_only is True
    assert volume.synthetic_measured_points_added == 0
    for key in ('motion_authorized', 'complete_object_volume',
                'usable_as_collision_geometry', 'executable_contact_geometry'):
        assert getattr(volume, key) is False
    assert not hasattr(volume, 'object_points_base')
    assert not hasattr(volume, 'center_base')
    assert not hasattr(volume, 'size_xyz_m')
    assert 'not_measurements' in volume.vertex_semantics
    with pytest.raises(FrozenInstanceError):
        volume.foreground_max_m = .005
    with pytest.raises(TypeError):
        volume.vertices_base_m[0][0] = 0.
    assert json.loads(json.dumps(asdict(volume), allow_nan=False))['source_stamp_ns'] == STAMP


@pytest.mark.parametrize('yaw,roll,shift', [
    (0., 0., (8., -13., 4.)), (31., -17., (.3, -.5, 1.)),
    (-150., 78., (-3., 6., -2.)), (89., -89., (1000., -1000., 1000.))])
def test_coordinate_covariance_preserves_full_frustum(yaw, roll, shift):
    a, b = np.deg2rad([yaw, roll])
    rz = np.array([[np.cos(a), -np.sin(a), 0.], [np.sin(a), np.cos(a), 0.], [0., 0., 1.]])
    rx = np.array([[1., 0., 0.], [0., np.cos(b), -np.sin(b)], [0., np.sin(b), np.cos(b)]])
    frame = np.eye(4); frame[:3, :3] = rz@rx; frame[:3, 3] = shift
    normal = frame[:3, :3]@np.array(PLANE[:3])
    plane = np.r_[normal, PLANE[3]-normal@np.array(shift)]
    transformed = build(T_base_camera=frame@T, support_plane_base=plane,
                        support_provenance=evidence(support_plane_base=plane))
    expected = np.asarray(build().vertices_base_m)@frame[:3, :3].T+shift
    np.testing.assert_allclose(transformed.vertices_base_m, expected, atol=1e-10, rtol=0.)
    np.testing.assert_allclose(transformed.vertex_optical_depths_m,
                               build().vertex_optical_depths_m, atol=1e-10, rtol=0.)
    np.testing.assert_allclose(project(transformed, T_base_camera=frame@T),
                               project(build()), atol=1e-8, rtol=0.)


def test_nonzero_intrinsic_skew_is_handled_without_optical_frame_confusion():
    skewed = K.copy(); skewed[0, 1] = 13.
    volume = build(camera_matrix=skewed)
    expected = np.tile(volume.pixel_boundary_xy, (2, 1))
    np.testing.assert_allclose(project(volume, camera_matrix=skewed), expected, atol=1e-12)


def test_slanted_camera_has_unequal_depths_but_coplanar_boundaries():
    a = np.deg2rad(25.)
    rotation = np.array([[np.cos(a), 0., np.sin(a)], [0., 1., 0.],
                         [-np.sin(a), 0., np.cos(a)]])
    tilted = T.copy(); tilted[:3, :3] = rotation@T[:3, :3]
    volume = build(T_base_camera=tilted)
    vertices = np.asarray(volume.vertices_base_m)
    assert np.ptp(volume.vertex_optical_depths_m[:4]) > .02
    np.testing.assert_allclose(vertices[:4, 2], 0., atol=1e-15)
    np.testing.assert_allclose(vertices[4:, 2], .12, atol=1e-15)
    np.testing.assert_allclose(project(volume, T_base_camera=tilted),
                               np.tile(volume.pixel_boundary_xy, (2, 1)), atol=1e-12)


@pytest.mark.parametrize('box', [(0, 0, 640, 480), (0, 0, 1, 1), (639, 479, 1, 1),
                                (.25, .25, .5, .5), (300, 220, 40, 40)])
def test_image_boundary_boxes_are_enclosed_without_trimming(box):
    volume = build(bbox_xywh=box)
    np.testing.assert_allclose(project(volume), np.tile(volume.pixel_boundary_xy, (2, 1)), atol=1e-12)


@pytest.mark.parametrize('box', [(-.001, 0, 1, 1), (0, -.001, 1, 1), (639, 0, 1.001, 1),
    (0, 479, 1, 1.001), (0, 0, 0, 1), (0, 0, 1, -1), (0, 0, True, 1),
    (0, 0, '1', 1), (0, 0, np.nan, 1), (0, 0, np.inf, 1), (0, 0, 1), None])
def test_invalid_boxes_are_rejected(box):
    with pytest.raises(ValueError, match='RGB_OBSERVATION_VOLUME_BBOX'):
        build(bbox_xywh=box)


@pytest.mark.parametrize('shape', [(480., 640), (480, True), (0, 640), (480, 640, 3), None])
def test_image_shape_requires_explicit_positive_integer_height_width(shape):
    with pytest.raises(ValueError, match='IMAGE_SHAPE_INVALID'):
        build(image_shape=shape)


@pytest.mark.parametrize('stamp', [True, 0, -1, float(STAMP), str(STAMP), np.int64(STAMP)])
def test_exact_source_timestamp_type_is_mandatory(stamp):
    with pytest.raises(ValueError, match='SOURCE_STAMP_INVALID'):
        build(source_stamp_ns=stamp)


@pytest.mark.parametrize('overrides', [
    dict(support_provenance=None), dict(support_provenance={}),
    dict(support_provenance={'quality_accepted': True}),
    dict(support_provenance=evidence(source_stamp_ns=STAMP+1)),
    dict(support_provenance=evidence(support_plane_base=(0., 0., 1., .001))),
])
def test_missing_or_different_source_provenance_is_rejected(overrides):
    with pytest.raises(ValueError, match='SUPPORT_(PROVENANCE|SOURCE_STAMP)'):
        build(**overrides)


@pytest.mark.parametrize('overrides', [
    dict(estimator='rgb_bbox'), dict(estimator='synthetic_slab'),
    dict(quality_accepted=False), dict(quality_accepted=1),
    dict(source_record_id=''), dict(source_record_id='  '), dict(source_record_id=1),
    dict(inlier_count=2), dict(inlier_count=3), dict(inlier_count=119),
    dict(inlier_count=True), dict(inlier_count=120.),
    dict(min_inlier_count=119), dict(min_inlier_count=True),
    dict(min_inlier_count=120.), dict(min_inlier_count=831),
    dict(inlier_ratio=.699), dict(inlier_ratio=1.01), dict(inlier_ratio=True),
    dict(fit_rmse_m=.002001), dict(fit_rmse_m=-1.), dict(fit_rmse_m=np.nan),
    dict(spatial_quadrants=2), dict(spatial_quadrants=5), dict(spatial_quadrants=True),
    dict(min_inlier_ratio=.69), dict(max_fit_rmse_m=.002001),
    dict(min_spatial_quadrants=2), dict(min_spatial_quadrants=True),
    dict(min_inlier_ratio=.95), dict(max_fit_rmse_m=.0005),
    dict(spatial_quadrants=3, min_spatial_quadrants=4),
])
def test_quality_provenance_requires_actual_original_standards(overrides):
    with pytest.raises(ValueError, match='SUPPORT_PROVENANCE_INVALID'):
        evidence(**overrides)


def test_quality_boundary_and_stricter_criteria_are_preserved():
    boundary = evidence(inlier_ratio=.70, fit_rmse_m=.002, spatial_quadrants=3, inlier_count=120)
    assert build(support_provenance=boundary).support_provenance is boundary
    strict = evidence(min_inlier_ratio=.9, max_fit_rmse_m=.001, min_spatial_quadrants=4,
                      min_inlier_count=830)
    assert build(support_provenance=strict).support_provenance.min_spatial_quadrants == 4
    assert build(support_provenance=strict).support_provenance.min_inlier_count == 830


@pytest.mark.parametrize('plane', [(0, 0, 2, 0), (0, 0, 0, 0), (0, 0, 1, np.nan),
                                   (0, 0, True, 0), (0, 0, 1), None])
def test_plane_must_be_finite_and_unit_without_silent_normalization(plane):
    with pytest.raises(ValueError, match='SUPPORT_PLANE'):
        build(support_plane_base=plane)


def test_plane_orientation_must_face_camera_not_silently_flip():
    plane = (0., 0., -1., 0.)
    with pytest.raises(ValueError, match='SUPPORT_NOT_FACING_CAMERA'):
        build(support_plane_base=plane, support_provenance=evidence(support_plane_base=plane))


@pytest.mark.parametrize('height', [.12, .119, .001])
def test_camera_inside_or_touching_slab_is_rejected(height):
    transform = T.copy(); transform[2, 3] = height
    with pytest.raises(ValueError, match='CAMERA_INSIDE_OR_TOUCHING_SLAB'):
        build(T_base_camera=transform)


def test_any_horizon_or_away_corner_rejects_instead_of_clipping_volume():
    transform = T.copy(); transform[:3, :3] = np.eye(3)
    with pytest.raises(ValueError, match='RAY_DOES_NOT_INTERSECT_FORWARD_SLAB'):
        build(T_base_camera=transform)
    with pytest.raises(ValueError, match='RAY_PARALLEL_TO_SUPPORT'):
        build(T_base_camera=transform, bbox_xywh=(300., 240.5, 40., 20.))


@pytest.mark.parametrize('limits', [(0., .7), (.7, .7), (.8, .7), (True, .7),
                                   (.07, np.inf), (.07,), None])
def test_invalid_working_depth_ranges_are_rejected(limits):
    with pytest.raises(ValueError, match='WORKING_DEPTH_RANGE_INVALID'):
        build(working_depth_range_m=limits)


@pytest.mark.parametrize('limits', [(.381, .7), (.07, .499)])
def test_all_vertices_must_be_in_working_depth_no_partial_acceptance(limits):
    with pytest.raises(ValueError, match='VERTEX_OUTSIDE_WORKING_DEPTH'):
        build(working_depth_range_m=limits)


def test_working_depth_boundary_is_inclusive():
    volume = build(working_depth_range_m=(.38, .5))
    project(volume, working_depth_range_m=(.38, .5))


@pytest.mark.parametrize('height', [0., -1., True, '0.12', np.nan, np.inf, None])
def test_invalid_scene_height_is_rejected(height):
    with pytest.raises(ValueError, match='FOREGROUND_HEIGHT_INVALID'):
        build(foreground_max_m=height)


@pytest.mark.parametrize('change', ['reflection', 'scale', 'last_row', 'nan', 'shape'])
def test_transform_requires_proper_rigid_transform(change):
    transform = T.copy()
    if change == 'reflection': transform[:3, 0] *= -1
    if change == 'scale': transform[:3, 0] *= 2
    if change == 'last_row': transform[3, 0] = .01
    if change == 'nan': transform[1, 1] = np.nan
    if change == 'shape': transform = transform[:3]
    with pytest.raises(ValueError, match='TRANSFORM_INVALID'):
        build(T_base_camera=transform)


@pytest.mark.parametrize('change', ['zero_focal', 'negative_focal', 'last_row', 'lower_left', 'nan', 'shape', 'bool'])
def test_invalid_intrinsics_are_rejected(change):
    matrix = K.copy()
    if change == 'zero_focal': matrix[0, 0] = 0.
    if change == 'negative_focal': matrix[1, 1] = -1.
    if change == 'last_row': matrix[2, 0] = .01
    if change == 'lower_left': matrix[1, 0] = .01
    if change == 'nan': matrix[1, 1] = np.nan
    if change == 'shape': matrix = matrix[0]
    if change == 'bool': matrix = [[True, 0, 320], [0, 400, 240], [0, 0, 1]]
    with pytest.raises(ValueError, match='INTRINSICS_INVALID'):
        build(camera_matrix=matrix)


def test_other_camera_projection_keeps_all_convex_vertices_in_view():
    transform = T.copy(); transform[0, 3] += .015; transform[2, 3] -= .015
    pixels = np.asarray(project(build(), T_base_camera=transform))
    assert pixels.shape == (8, 2)
    assert np.min(pixels) > -.5 and np.max(pixels[:, 0]) < 639.5
    assert not np.allclose(pixels[:4], pixels[4:])


def test_projection_rejects_partial_visibility_even_when_volume_centre_is_visible():
    volume = build(bbox_xywh=(0, 220, 640, 40))
    transform = T.copy(); transform[0, 3] += .001
    with pytest.raises(ValueError, match='PROJECTED_VOLUME_OUTSIDE_IMAGE'):
        project(volume, T_base_camera=transform)


def test_projection_rejects_behind_camera_or_outside_working_depth():
    transform = T.copy(); transform[2, 3] = -.1
    with pytest.raises(ValueError, match='PROJECTED_VERTEX_BEHIND_CAMERA'):
        project(build(), T_base_camera=transform)
    with pytest.raises(ValueError, match='PROJECTED_VERTEX_OUTSIDE_WORKING_DEPTH'):
        project(build(), working_depth_range_m=(.4, .7))


@pytest.mark.parametrize('value', [None, {}, np.zeros((8, 3)), asdict(build())])
def test_projection_rejects_untyped_geometry(value):
    with pytest.raises(ValueError, match='VOLUME_TYPE_INVALID'):
        project(value)


def test_dataclass_replace_recomputes_derived_model_vertices_and_revalidates():
    volume = build()
    shorter = replace(volume, foreground_max_m=.08)
    assert shorter.vertices_base_m != volume.vertices_base_m
    np.testing.assert_allclose(np.asarray(shorter.vertices_base_m)[4:, 2], .08, atol=1e-15)
    with pytest.raises(ValueError, match='SUPPORT_SOURCE_STAMP_MISMATCH'):
        replace(volume, source_stamp_ns=STAMP+1)
