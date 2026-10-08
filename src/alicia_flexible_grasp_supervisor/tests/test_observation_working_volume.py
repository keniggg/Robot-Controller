"""Offline image-bound endpoint checks; no ROS graph or motion commands."""
import itertools
import json

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.observation_working_volume import (
    ObservationWorkingVolumeError, observation_obb_corners,
    validate_observation_working_volume,
)


def corners(center=(.2118, 0., 0.), size=(.02, .02, .02)):
    return observation_obb_corners(center, np.eye(3), size)


def check(target=None, **updates):
    data = dict(target_corners_base=corners() if target is None else target,
                base_from_camera=np.eye(4), intrinsics=(400., 400., 320., 240.),
                image_width=640, image_height=480,
                source_stamp_ns=123456789, camera_transform_stamp_ns=123456789)
    data.update(updates)
    return validate_observation_working_volume(**data)


def test_2118_mm_radial_distance_is_accepted_without_range_correction():
    report = check()
    assert report['ok']
    assert report['target_center_radial_distance_m'] == pytest.approx(.2118)
    assert report['minimum_depth_m'] == pytest.approx(.2018)
    assert report['maximum_depth_m'] == pytest.approx(.2218)
    assert report['minimum_image_edge_distance_px'] > 36.
    assert report['radial_distance_is_acceptance_gate'] is False
    assert report['certifies_information_sufficiency'] is False
    assert report['certifies_grasp_success'] is False
    assert report['certifies_collision_tracking_calibration_or_occlusion'] is False
    assert report['requires_independent_information_quality_check'] is True
    assert report['visible_surface_points_available'] is False
    json.dumps(report, allow_nan=False)


def test_forward_depth_not_euclidean_radius_and_rotated_camera_link():
    # The full target remains inside the image even though its centre's radial
    # distance exceeds the depth maximum; x depth remains inside that maximum.
    target = corners((.49, .12, 0.), (.002, .002, .002))
    transform = np.eye(4)
    transform[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    transform[:3, 3] = [.1, -.2, .3]
    target_base = target @ transform[:3, :3].T + transform[:3, 3]
    report = check(target_base, base_from_camera=transform)
    assert report['ok']
    assert report['target_center_radial_distance_m'] > .5
    assert report['minimum_depth_m'] == pytest.approx(.489)
    assert report['maximum_depth_m'] == pytest.approx(.491)


@pytest.mark.parametrize('center,size', [((.08, 0., 0.), (.04, .002, .002)),
                                         ((.49, 0., 0.), (.04, .002, .002))])
def test_center_inside_depth_range_does_not_hide_corner_depth_violation(center, size):
    with pytest.raises(ObservationWorkingVolumeError, match='depth range') as captured:
        check(corners(center, size))
    assert captured.value.audit['ok'] is False
    json.dumps(captured.value.audit, allow_nan=False)


def test_center_inside_image_does_not_hide_corner_clipping():
    with pytest.raises(ObservationWorkingVolumeError, match='image margin') as captured:
        check(corners((.2, .13, 0.), (.02, .04, .02)))
    assert captured.value.audit['minimum_image_edge_distance_px'] < 36.


@pytest.mark.parametrize('near,far', [(.070, .072), (.498, .500)])
def test_exact_camera_depth_boundaries_are_inclusive(near, far):
    target = np.array(list(itertools.product((near, far), (-.001, .001), (-.001, .001))))
    report = check(target)
    assert report['ok'] and report['minimum_depth_m'] == near and report['maximum_depth_m'] == far


@pytest.mark.parametrize('depth', [np.nextafter(.070, 0.), np.nextafter(.500, 1.)])
def test_depth_immediately_outside_boundary_is_rejected(depth):
    target = np.array(list(itertools.product((depth, .3), (-.001, .001), (-.001, .001))))
    with pytest.raises(ObservationWorkingVolumeError, match='depth range'):
        check(target)


def test_tighter_configured_depth_and_margin_are_enforced():
    with pytest.raises(ObservationWorkingVolumeError, match='depth range'):
        check(minimum_depth_m=.21)
    with pytest.raises(ObservationWorkingVolumeError, match='image margin'):
        check(roi_margin_px=230.)


def test_surface_points_are_optional_same_source_diagnostics_only():
    points = np.array([[.21, 0., 0.], [.9, 0., 0.]])
    report = check(visible_surface_points_base=points, visible_surface_stamp_ns=123456789)
    assert report['ok']
    assert report['visible_surface_point_count'] == 2
    assert report['visible_surface_points_in_working_volume'] == 1
    assert report['visible_surface_points_are_complete_geometry_evidence'] is False
    assert report['certifies_information_sufficiency'] is False


@pytest.mark.parametrize('updates', [
    dict(source_stamp_ns=0), dict(source_stamp_ns=True), dict(source_stamp_ns=1.),
    dict(camera_transform_stamp_ns=123456788), dict(camera_transform_stamp_ns=None),
    dict(visible_surface_stamp_ns=123456789),
    dict(visible_surface_points_base=[[.2, 0., 0.]]),
    dict(visible_surface_points_base=[[.2, 0., 0.]], visible_surface_stamp_ns=123456788),
    dict(visible_surface_points_base=[], visible_surface_stamp_ns=123456789),
    dict(visible_surface_points_base=[[.2, 0.]], visible_surface_stamp_ns=123456789),
    dict(visible_surface_points_base=[[float('nan'), 0., 0.]], visible_surface_stamp_ns=123456789),
])
def test_missing_or_mismatched_source_binding_cannot_be_used(updates):
    with pytest.raises(ObservationWorkingVolumeError) as captured:
        check(**updates)
    assert captured.value.audit['ok'] is False
    json.dumps(captured.value.audit, allow_nan=False)


@pytest.mark.parametrize('updates', [
    dict(image_width=True), dict(image_width=640.), dict(image_height=0), dict(image_height=20000),
    dict(intrinsics=[0., 400., 320., 240.]), dict(intrinsics=['400', 400., 320., 240.]),
    dict(intrinsics=[True, 400., 320., 240.]), dict(intrinsics=[400., 400., 640., 240.]),
    dict(intrinsics=[400., float('nan'), 320., 240.]), dict(intrinsics=np.eye(4)),
    dict(intrinsics=[[400., 1., 320.], [0., 400., 240.], [0., 0., 1.]]),
    dict(roi_margin_px=35.99), dict(roi_margin_px=float('inf')), dict(roi_margin_px=240.),
    dict(minimum_depth_m=.06999), dict(maximum_depth_m=.50001),
    dict(minimum_depth_m=.5), dict(maximum_depth_m=False),
    dict(base_from_camera=np.eye(3)), dict(base_from_camera=np.zeros((4, 4))),
    dict(base_from_camera=np.diag([-1., 1., 1., 1.])),
    dict(base_from_camera=np.diag([2., 1., 1., 1.])),
    dict(base_from_camera=np.diag([1., 1., 1., 2.])),
    dict(base_from_camera=np.full((4, 4), float('nan'))),
    dict(target_corners_base=np.zeros((8, 3))), dict(target_corners_base=np.zeros((7, 3))),
    dict(target_corners_base=np.full((8, 3), float('inf'))),
    dict(target_corners_base=[[1., 2.]]),
])
def test_malformed_or_relaxed_geometry_and_camera_inputs_are_rejected(updates):
    with pytest.raises(ObservationWorkingVolumeError) as captured:
        check(**updates)
    assert captured.value.audit['ok'] is False
    json.dumps(captured.value.audit, allow_nan=False)


def test_duplicate_corner_and_planar_box_rejected():
    duplicate = corners()
    duplicate[-1] = duplicate[0]
    planar = corners()
    planar[:, 0] = .2
    for target in (duplicate, planar):
        with pytest.raises(ObservationWorkingVolumeError, match='nondegenerate'):
            check(target)


@pytest.mark.parametrize('center,axes,size', [
    ([.2, 0.], np.eye(3), [.02] * 3),
    ([.2, 0., 0.], np.diag([-1., 1., 1.]), [.02] * 3),
    ([.2, 0., 0.], np.eye(3), [0., .02, .02]),
    ([.2, 0., 0.], np.eye(3), [-.01, .02, .02]),
    ([.2, 0., 0.], np.eye(3), [True, .02, .02]),
])
def test_invalid_frozen_obb_fields_rejected(center, axes, size):
    with pytest.raises(ObservationWorkingVolumeError):
        observation_obb_corners(center, axes, size)


def test_matrix_intrinsics_and_all_inputs_remain_unchanged():
    target, transform = corners(), np.eye(4)
    k = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])
    before = [value.copy() for value in (target, transform, k)]
    assert check(target, base_from_camera=transform, intrinsics=k)['ok']
    for original, saved in zip((target, transform, k), before):
        np.testing.assert_array_equal(original, saved)


def test_tf_quantization_tolerance_cannot_be_widened_into_age_allowance():
    stamp = 1785344323552103281
    report = check(source_stamp_ns=stamp, camera_transform_stamp_ns=stamp+100,
                   camera_transform_stamp_tolerance_ns=121)
    assert report['camera_transform_stamp_ns'] == stamp+100
    assert report['camera_transform_stamp_delta_ns'] == 100
    for tolerance in (-1, True, 1.5, 1000):
        with pytest.raises(ObservationWorkingVolumeError, match='quantization tolerance'):
            check(source_stamp_ns=stamp, camera_transform_stamp_ns=stamp,
                  camera_transform_stamp_tolerance_ns=tolerance)
    with pytest.raises(ObservationWorkingVolumeError, match='bound to source') as caught:
        check(source_stamp_ns=stamp, camera_transform_stamp_ns=stamp+1000,
              camera_transform_stamp_tolerance_ns=121)
    assert caught.value.audit['camera_transform_stamp_ns'] == stamp+1000


def test_planned_and_measured_audits_share_geometry_without_sharing_time_authority():
    from alicia_flexible_grasp.grasp.observation_working_volume import (
        validate_planned_observation_working_volume,
    )
    predicted = validate_planned_observation_working_volume(corners(), np.eye(4),
        (400., 400., 320., 240.), 640, 480, snapshot_stamp_ns=123456789)
    measured = check()
    for key in ('minimum_depth_m', 'maximum_depth_m', 'target_corner_depths_m',
                'minimum_image_edge_distance_px', 'target_center_radial_distance_m'):
        assert predicted[key] == measured[key]
    assert predicted['kind'] == 'planned_observation_working_volume'
    assert predicted['snapshot_stamp_ns'] == 123456789
    assert predicted['camera_transform_source'] == 'planned_camera_pose'
    assert predicted['actual_arrival_measurement_required'] is True
    assert not predicted['certifies_information_sufficiency']
    assert not predicted['visible_surface_points_available']
    for key in ('source_stamp_ns', 'image_source_stamp_ns', 'camera_transform_stamp_ns',
                'camera_transform_stamp_delta_ns', 'camera_transform_stamp_tolerance_ns',
                'camera_transform_query_stamp_ns', 'visible_surface_stamp_ns'):
        assert key not in predicted
    assert measured['kind'] == 'image_bound_observation_working_volume'
    assert measured['source_stamp_ns'] == measured['camera_transform_stamp_ns'] == 123456789
    assert 'snapshot_stamp_ns' not in measured
    json.dumps(predicted, allow_nan=False)


@pytest.mark.parametrize('stamp,target', [
    (0, None), (True, None), (1.0, None), (1, np.zeros((8, 3))),
    (1, corners((.06, 0., 0.))),
])
def test_invalid_planned_view_keeps_prediction_audit_without_measured_timestamps(stamp, target):
    from alicia_flexible_grasp.grasp.observation_working_volume import (
        validate_planned_observation_working_volume,
    )
    with pytest.raises(ObservationWorkingVolumeError) as captured:
        validate_planned_observation_working_volume(corners() if target is None else target,
            np.eye(4), (400., 400., 320., 240.), 640, 480, snapshot_stamp_ns=stamp)
    audit = captured.value.audit
    assert audit['kind'] == 'planned_observation_working_volume'
    assert audit['ok'] is False
    assert audit['actual_arrival_measurement_required'] is True
    assert 'source_stamp_ns' not in audit and 'camera_transform_stamp_ns' not in audit
    json.dumps(audit, allow_nan=False)


def test_planned_api_cannot_accept_measured_tf_or_surface_arguments():
    from alicia_flexible_grasp.grasp.observation_working_volume import (
        validate_planned_observation_working_volume,
    )
    for extra in ({'camera_transform_stamp_ns': 1}, {'source_stamp_ns': 1},
                  {'visible_surface_points_base': [[.2, 0., 0.]], 'visible_surface_stamp_ns': 1}):
        with pytest.raises(TypeError):
            validate_planned_observation_working_volume(corners(), np.eye(4),
                (400., 400., 320., 240.), 640, 480, snapshot_stamp_ns=1, **extra)
