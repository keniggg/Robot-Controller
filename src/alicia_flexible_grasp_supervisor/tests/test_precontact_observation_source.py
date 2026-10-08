"""Actual fused frames and estimator output bind evidence, never motion rights."""
from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.vision.latest_only_inference import LatestOnlyInferenceCoordinator
from alicia_flexible_grasp.vision.object_geometry import estimate_object_geometry
from alicia_flexible_grasp.vision.precontact_observation_source import (
    ObservationGeometryConfig, bind_precontact_observation_source)
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity


def ros_stamp(ns):
    return NS(secs=ns // 10**9, nsecs=ns % 10**9)


def header(ns, frame):
    return NS(stamp=ros_stamp(ns), frame_id=frame)


@pytest.fixture
def scene():
    # A real depth render: 30 mm support-resting top whose measured short
    # direction is below the unchanged 5 mm contact-geometry acceptance gate.
    depth = np.full((120, 160), .400)
    mask = np.zeros(depth.shape, np.uint8)
    mask[44:78, 30:130] = 255
    depth[mask > 0] = .370
    bbox = (30, 44, 100, 34)
    identity = TargetTrackIdentity.from_stream(4, 3)
    samples = []
    stamp = 1791163928959155797
    for ns in (stamp-100_000_000, stamp):
        obj = NS(detected=True, label='unknown', header=header(ns, 'base_link'),
            bbox_x=bbox[0], bbox_y=bbox[1], bbox_width=bbox[2], bbox_height=bbox[3],
            pose_camera=NS(header=header(ns, 'camera_optical')))
        samples.append(RgbdSample(np.zeros(depth.shape+(3,), np.uint8), depth, mask,
            bbox, obj, ns*1e-9, 'camera_optical', np.zeros(6), ns, 3, identity))
    snapshot = fuse_stable_samples(samples, require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01, erosion_px=0,
        depth_scale=1., depth_min_m=.07, depth_max_m=.7, mad_scale=3.5,
        mad_absolute_floor_m=.002, internal_hole_max_area_px=0)
    assert snapshot.ok
    intrinsics = NS(width=160, height=120, fx=3000., fy=3000., cx=80., cy=60.)
    config = ObservationGeometryConfig(.30, .004, .0003, 200, 120, .005, .600, .500)
    geometry = estimate_object_geometry(depth_raw=snapshot.depth_raw,
        target_depth_raw=snapshot.target_depth_raw, object_mask=snapshot.object_mask,
        bbox=bbox, intrinsics=intrinsics, depth_scale=1., T_base_camera=np.eye(4),
        source_mode='instance_mask', previous_axes_base=None, **vars(config))
    assert not geometry.ok and geometry.observation_deficit is not None
    coordinator = LatestOnlyInferenceCoordinator(clock=lambda: 100.)
    coordinator.start()
    ticket = coordinator.submit((snapshot, None), snapshot.stamp_sec, 3).ticket_to_start
    tf = NS(header=header(stamp, 'base_link'), child_frame_id='camera_optical',
        transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    return dict(ticket=ticket, current_ticket=ticket, current_target_identity=identity,
        geometry=geometry, geometry_config=config, intrinsics=intrinsics, depth_scale=1.,
        exact_tf=tf, source_from_optical=np.eye(4), now_stamp_ns=stamp+200_000_000,
        max_source_age_ns=800_000_000)


def new_snapshot(scene, snapshot):
    ticket = replace(scene['ticket'], payload=(snapshot, None),
                     snapshot_stamp_sec=snapshot.stamp_sec)
    scene.update(ticket=ticket, current_ticket=ticket)


def test_source_bound_measurement_preserves_failed_contact_and_missing_requirements(scene):
    result = bind_precontact_observation_source(**scene)
    snapshot = scene['ticket'].payload[0]
    assert result.observation.identity == snapshot.target_identity
    assert result.sample_stamp_ns == snapshot.sample_stamp_ns
    assert result.transform_stamp_ns == snapshot.stamp_ns
    assert result.admitted_at_ns == scene['now_stamp_ns']
    assert result.source_expires_at_ns == snapshot.stamp_ns+scene['max_source_age_ns']
    assert result.request_id == scene['ticket'].request_id
    assert not scene['geometry'].ok
    assert not result.contact_geometry_authorized
    assert not result.observation_motion_authorized
    assert not result.planning_authorized
    assert 'complete_rgb_identity' in result.missing_requirements
    assert 'independent_support_quality' in result.missing_requirements
    assert 'conservative_occupied_scene' in result.missing_requirements
    assert 'bounded_information_gain' in result.missing_requirements
    np.testing.assert_array_equal(result.observation.points_base,
                                  scene['geometry'].observation_deficit.object_points_base)
    assert bind_precontact_observation_source(**scene).source_sha256 == result.source_sha256


def test_evidence_copies_source_and_cannot_acquire_motion_rights(scene):
    result = bind_precontact_observation_source(**scene)
    for value in (result.transform_base_optical, result.observation.points_base,
                  result.observation.support_normal_base, result.deficit.target_points_base):
        with pytest.raises(ValueError):
            value.setflags(write=True)
        with pytest.raises(ValueError):
            value.flat[0] = 8.
    with pytest.raises(FrozenInstanceError):
        result.planning_authorized = True
    with pytest.raises(ValueError):
        replace(result, observation_motion_authorized=True)
    with pytest.raises(ValueError, match='disagrees'):
        replace(result, observation=replace(result.observation,
            points_base=result.observation.points_base+[.001, 0., 0.]))
    assert deepcopy(result) is result
    scene['source_from_optical'][:] = 9.
    np.testing.assert_array_equal(result.transform_base_optical, np.eye(4))


def test_camera_link_optical_conversion_is_explicit_and_bound(scene):
    snapshot = scene['ticket'].payload[0]
    obj = deepcopy(snapshot.object_msg)
    obj.pose_camera.header.frame_id = 'camera_link'
    original = replace(snapshot.original_sample, frame_id='camera_link', object_msg=obj)
    snapshot = replace(snapshot, frame_id='camera_link', object_msg=obj, original_sample=original)
    new_snapshot(scene, snapshot)
    optical = np.eye(4)
    optical[:3, :3] = [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]
    scene['exact_tf'].child_frame_id = 'camera_link'
    scene['source_from_optical'] = optical
    scene['geometry'] = estimate_object_geometry(depth_raw=snapshot.depth_raw,
        target_depth_raw=snapshot.target_depth_raw, object_mask=snapshot.object_mask,
        bbox=snapshot.bbox, intrinsics=scene['intrinsics'], depth_scale=1.,
        T_base_camera=optical, source_mode='instance_mask', previous_axes_base=None,
        **vars(scene['geometry_config']))
    assert not scene['geometry'].ok
    result = bind_precontact_observation_source(**scene)
    np.testing.assert_array_equal(result.transform_base_optical, optical)
    assert result.source_frame_id == 'camera_link'


@pytest.mark.parametrize('fault', [
    'new_request', 'new_generation', 'new_epoch', 'other_identity', 'payload_clone', 'payload_config',
    'ticket_stamp', 'future', 'expired', 'tf_1ns', 'tf_latest', 'tf_parent', 'tf_child',
    'tf_translation', 'tf_rotation', 'optical_axes', 'config_minimum', 'config_support',
    'intrinsics', 'depth_scale', 'changed_fit', 'changed_target_points', 'bad_contact',
    'no_deficit', 'no_original', 'source_bbox', 'source_joints', 'no_detected',
    'detection_stamp', 'detection_bbox', 'detection_camera', 'sample_sequence',
    'sample_final_stamp', 'sample_count', 'quality_count', 'quality_nonfinite',
    'unsupported_bbox_depth', 'mask_empty', 'mask_shape', 'unmasked_depth',
    'invented_depth', 'nonfinite_depth', 'snapshot_failed'])
def test_invalid_sources_never_become_observation_evidence(scene, fault):
    snapshot = scene['ticket'].payload[0]
    if fault == 'new_request':
        scene['current_ticket'] = replace(scene['ticket'], request_id=2)
    elif fault == 'new_generation':
        scene['current_ticket'] = replace(scene['ticket'], generation=2)
    elif fault == 'new_epoch':
        scene['current_ticket'] = replace(scene['ticket'], target_epoch=4)
    elif fault == 'other_identity':
        scene['current_target_identity'] = TargetTrackIdentity.from_stream(5, 3)
    elif fault == 'payload_clone':
        scene['current_ticket'] = replace(scene['ticket'], payload=(replace(snapshot), None))
    elif fault == 'payload_config':
        scene['current_ticket'] = replace(scene['ticket'], payload=(snapshot, object()))
    elif fault == 'ticket_stamp':
        scene['current_ticket'] = replace(scene['ticket'], snapshot_stamp_sec=snapshot.stamp_sec+1.)
    elif fault == 'future':
        scene['now_stamp_ns'] = snapshot.stamp_ns-1
    elif fault == 'expired':
        scene['now_stamp_ns'] = snapshot.stamp_ns+800_000_001
    elif fault == 'tf_1ns':
        scene['exact_tf'].header.stamp = ros_stamp(snapshot.stamp_ns+1)
    elif fault == 'tf_latest':
        scene['exact_tf'].header.stamp = ros_stamp(0)
    elif fault == 'tf_parent':
        scene['exact_tf'].header.frame_id = 'map'
    elif fault == 'tf_child':
        scene['exact_tf'].child_frame_id = 'other_camera'
    elif fault == 'tf_translation':
        scene['exact_tf'].transform.translation.z = .01
    elif fault == 'tf_rotation':
        scene['exact_tf'].transform.rotation.w = 2.
    elif fault == 'optical_axes':
        scene['source_from_optical'][0, 3] = .01
    elif fault == 'config_minimum':
        scene['geometry_config'] = replace(scene['geometry_config'], min_size_m=.003)
    elif fault == 'config_support':
        scene['geometry_config'] = replace(scene['geometry_config'], support_bbox_expand_ratio=.4)
    elif fault == 'intrinsics':
        scene['intrinsics'].cx += 1.
    elif fault == 'depth_scale':
        scene['depth_scale'] = 1.01
    elif fault in ('changed_fit', 'changed_target_points'):
        deficit = scene['geometry'].observation_deficit
        if fault == 'changed_fit':
            deficit = replace(deficit, center_base=deficit.center_base+[.001, 0., 0.])
        else:
            deficit = replace(deficit, target_points_base=deficit.target_points_base+[.001, 0., 0.])
        scene['geometry'] = replace(scene['geometry'], observation_deficit=deficit)
    elif fault == 'bad_contact':
        scene['geometry'] = replace(scene['geometry'], size_xyz_m=np.ones(3))
    elif fault == 'no_deficit':
        scene['geometry'] = replace(scene['geometry'], observation_deficit=None)
    elif fault == 'no_original':
        new_snapshot(scene, replace(snapshot, original_sample=None))
    elif fault == 'source_bbox':
        new_snapshot(scene, replace(snapshot,
            original_sample=replace(snapshot.original_sample, bbox=(0, 0, 10, 10))))
    elif fault == 'source_joints':
        new_snapshot(scene, replace(snapshot,
            original_sample=replace(snapshot.original_sample, joint_positions=np.full(6, np.nan))))
    elif fault.startswith('detection_') or fault == 'no_detected':
        obj = deepcopy(snapshot.object_msg)
        if fault == 'no_detected': obj.detected = False
        elif fault == 'detection_stamp': obj.header.stamp = ros_stamp(snapshot.stamp_ns-1)
        elif fault == 'detection_bbox': obj.bbox_width += 1
        elif fault == 'detection_camera': obj.pose_camera.header.frame_id = 'different_camera'
        new_snapshot(scene, replace(snapshot, object_msg=obj))
    elif fault == 'sample_sequence':
        new_snapshot(scene, replace(snapshot, sample_stamp_ns=(snapshot.stamp_ns, snapshot.stamp_ns)))
    elif fault == 'sample_final_stamp':
        new_snapshot(scene, replace(snapshot, sample_stamp_ns=(snapshot.stamp_ns-1,)))
    elif fault == 'sample_count':
        new_snapshot(scene, replace(snapshot, quality=replace(snapshot.quality, fused_frames=3)))
    elif fault == 'quality_count':
        new_snapshot(scene, replace(snapshot, quality=replace(snapshot.quality, valid_depth_points=2)))
    elif fault == 'quality_nonfinite':
        new_snapshot(scene, replace(snapshot, quality=replace(snapshot.quality, depth_mad_m=np.nan)))
    elif fault == 'unsupported_bbox_depth':
        new_snapshot(scene, replace(snapshot, source_mode='bbox_depth'))
    elif fault == 'mask_empty':
        new_snapshot(scene, replace(snapshot, object_mask=np.zeros_like(snapshot.object_mask)))
    elif fault == 'mask_shape':
        new_snapshot(scene, replace(snapshot, object_mask=np.ones((1, 1), np.uint8)))
    elif fault == 'unmasked_depth':
        target = snapshot.target_depth_raw.copy(); target[0, 0] = .4
        new_snapshot(scene, replace(snapshot, target_depth_raw=target))
    elif fault == 'invented_depth':
        target = snapshot.target_depth_raw.copy(); target[50, 50] += .001
        new_snapshot(scene, replace(snapshot, target_depth_raw=target))
    elif fault == 'nonfinite_depth':
        depth = snapshot.depth_raw.copy(); depth[0, 0] = np.nan
        new_snapshot(scene, replace(snapshot, depth_raw=depth))
    elif fault == 'snapshot_failed':
        new_snapshot(scene, replace(snapshot, ok=False, failure_code='DEPTH_UNSTABLE'))
    with pytest.raises(ValueError):
        bind_precontact_observation_source(**scene)


@pytest.mark.parametrize('changes', [dict(min_size_m=True), dict(max_size_m=.001),
    dict(min_object_points=2), dict(min_support_points=3.5), dict(voxel_size_m=0.),
    dict(support_minimum_span_m=-1.), dict(outlier_std_ratio=np.nan)])
def test_original_configuration_is_strictly_typed_and_validated(scene, changes):
    with pytest.raises(ValueError):
        replace(scene['geometry_config'], **changes)
