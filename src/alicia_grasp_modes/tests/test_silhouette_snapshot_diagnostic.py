from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.vision.latest_only_inference import InferenceTicket
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.silhouette_snapshot_diagnostic import (
    SilhouetteSnapshotConfig, build_silhouette_snapshot_evidence)
from alicia_grasp_modes.tabletop import Config, segment


def bound_inputs():
    K = NS(fx=240., fy=240., cx=160., cy=120., depth_scale=.001)
    T = np.eye(4); C = np.eye(4); C[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    raw = np.full((240, 320), 300, np.uint16); raw[106:134, 134:186] = 280
    color = np.full((240, 320, 3), (20, 20, 180), np.uint8); color[106:134, 134:186] = 220
    depth = raw.astype(np.float32)*K.depth_scale
    config = Config(); camera = [K.fx, K.fy, K.cx, K.cy]
    initial = segment(depth, camera, T, config).candidates[0]
    refined = refine_candidate(color, depth, camera, T, initial, config)
    identity = TargetTrackIdentity.from_stream(7, 4); samples = []
    for ns in (3_100_000_000, 3_200_000_000, 3_300_000_000):
        position = NS(**dict(zip(('x', 'y', 'z'), refined.position_base)))
        obj = NS(header=NS(stamp=NS(to_nsec=lambda ns=ns: ns)), detected=True,
                 pose_base=NS(pose=NS(position=position)))
        samples.append(RgbdSample(color.copy(), raw.copy(), refined.mask.copy(), refined.bbox,
            obj, ns/1e9, 'camera_link', np.zeros(6), ns, identity.epoch, identity))
    snapshot = fuse_stable_samples(samples, require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01, erosion_px=2,
        depth_scale=.001, depth_min_m=.03, depth_max_m=2., mad_scale=3.5,
        mad_absolute_floor_m=.002, internal_hole_max_area_px=0)
    assert snapshot.ok
    selection = dict(mode='unknown', strategy='observe_return', generation=42, stamp_ns=1_000_000_000)
    ns = samples[-1].stamp_ns
    status = dict(state='ready', mode='unknown', target_lost=False, exact_rgbd=True,
        generation=42, selection_stamp_ns=selection['stamp_ns'],
        **dict.fromkeys(('source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'tf_lookup_stamp_ns'), ns),
        color_frame_id='camera_link', depth_frame_id='camera_link',
        projection_frame_convention='ros_camera_link', anchor_source_stamp_ns=samples[0].stamp_ns,
        anchor_position_base=initial.position_base.tolist(), geometry_metrics=deepcopy(initial.metrics))
    cfg = SilhouetteSnapshotConfig(.001, config, status)
    geometry = NS(ok=True, center_base=refined.position_base, size_xyz_m=np.array([.06, .04, .02]),
                  support_normal_base=np.array(initial.metrics['support_plane_base'][:3]),
                  support_offset_m=initial.metrics['support_plane_base'][3])
    ticket = InferenceTicket(8, 9, snapshot.stamp_sec, identity.epoch, (snapshot, None), 1.)
    return dict(sample=samples[-1], snapshot=snapshot, geometry=geometry,
        transform_base_optical=C, intrinsics=K, selection=selection, ticket=ticket,
        config=cfg, depth_uncertainty_m=.003)


def rectified_diagnostic_camera(args, activation=None):
    from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
    intrinsic = args['intrinsics']
    info = dict(width=320, height=240, model='none', coeffs=[0.]*5,
                **{key: getattr(intrinsic, key) for key in ('fx', 'fy', 'cx', 'cy')})
    mapper = PairedPinholeRectification(info)
    return dict(width=320, height=240, frame_id='camera_link', align_depth_to_color=True,
        depth_scale=intrinsic.depth_scale,
        **{key: getattr(intrinsic, key) for key in ('fx', 'fy', 'cx', 'cy')},
        runtime_profile_geometry=dict(geometry_intrinsics=mapper.output_intrinsics,
            rectification=mapper.contract, geometry_activation_stamp_ns=(
                args['sample'].stamp_ns-1 if activation is None else activation)))


@pytest.mark.parametrize('where', ['published_mask', 'search_background'])
def test_rectified_diagnostic_never_uses_padding_and_preserves_measurements(monkeypatch, where):
    import alicia_grasp_modes.silhouette_snapshot_diagnostic as diagnostic
    args = bound_inputs()
    args['config'] = replace(args['config'], camera_geometry=rectified_diagnostic_camera(args))
    valid = np.ones(args['sample'].depth_raw.shape, bool)
    valid[(120, 160) if where == 'published_mask' else (104, 132)] = False
    monkeypatch.setattr(diagnostic, 'rectification_rgb_valid_mask', lambda *_: valid)
    original = args['sample'].depth_raw.copy()
    report = build_silhouette_snapshot_evidence(**args)
    assert not report['ok'] and not report['execution_authority'] and not report['observation_authority']
    assert ('UNOBSERVED_RGB' in report['code'] or 'unobserved_rgb' in report.get('reason', ''))
    np.testing.assert_array_equal(args['sample'].depth_raw, original)


@pytest.mark.parametrize('offset,accepted', [(-1, True), (0, False), (1, False)])
def test_diagnostic_binds_original_frame_to_its_geometry_activation(offset, accepted):
    args = bound_inputs()
    camera = rectified_diagnostic_camera(args, args['sample'].stamp_ns+offset)
    args['config'] = replace(args['config'], camera_geometry=camera)
    report = build_silhouette_snapshot_evidence(**args)
    assert report['ok'] is accepted
    if not accepted:
        assert report['reason'] == 'rectification_source_predates_geometry_activation'


def test_diagnostic_cannot_pair_frame_with_different_camera_K():
    args = bound_inputs()
    camera = rectified_diagnostic_camera(args); camera['fx'] += 1
    args['config'] = replace(args['config'], camera_geometry=camera)
    report = build_silhouette_snapshot_evidence(**args)
    assert report['code'] == 'SILHOUETTE_SOURCE_CAMERA_GEOMETRY_MISMATCH'


def test_exact_original_source_and_status_reproduce_mask_without_relabeling_fused_data():
    args = bound_inputs(); source = args['sample']; before = [x.copy() for x in (
        source.color_bgr, source.depth_raw, source.object_mask)]
    report = build_silhouette_snapshot_evidence(**args)
    assert report['ok'], report
    assert report['baseline_mask_reproduced']
    assert report['snapshot_source_stamps_ns'] == [3_100_000_000, 3_200_000_000, 3_300_000_000]
    assert set(report['measurement']['source_stamps_ns'].values()) == {3_300_000_000}
    assert report['plane_provenance']['kind'] == 'same_source_perception_verified_model'
    assert report['plane_provenance']['anchor_source_stamp_ns'] == 3_100_000_000
    assert report['view_geometry']['target_geometry_source_stamps_ns'] == report['snapshot_source_stamps_ns']
    assert not report['observation_authority'] and not report['execution_authority']
    for actual, expected in zip((source.color_bgr, source.depth_raw, source.object_mask), before):
        np.testing.assert_array_equal(actual, expected)


def test_fused_depth_and_mask_are_never_used_as_single_source_inputs():
    args = bound_inputs()
    old = args['snapshot']
    changed = replace(old, depth_raw=np.full_like(old.depth_raw, 900),
                      object_mask=np.zeros_like(old.object_mask))
    args['snapshot'] = changed; args['ticket'] = replace(args['ticket'], payload=(changed, None))
    report = build_silhouette_snapshot_evidence(**args)
    assert report['ok'], report
    assert report['baseline_mask_reproduced']


@pytest.mark.parametrize('change', ['sample_stamp', 'sample_identity', 'sample_epoch', 'ticket_payload',
    'ticket_epoch', 'selection_stamp', 'rgb_changed', 'status_stamp', 'status_generation', 'status_tf'])
def test_wrong_source_or_request_never_produces_usable_evidence(change):
    args = bound_inputs(); sample = args['sample']
    if change == 'sample_stamp': sample.stamp_ns += 1
    if change == 'sample_identity': sample.target_identity = TargetTrackIdentity.from_stream(99, 4)
    if change == 'sample_epoch': sample.target_epoch += 1
    if change == 'ticket_payload': args['ticket'] = replace(args['ticket'], payload=(replace(args['snapshot']), None))
    if change == 'ticket_epoch': args['ticket'] = replace(args['ticket'], target_epoch=5)
    if change == 'selection_stamp': args['selection']['stamp_ns'] = 3_250_000_000
    if change == 'rgb_changed': sample.color_bgr[0, 0] = 0
    if change == 'status_stamp': args['config'].perception_status['source_stamp_ns'] += 1
    if change == 'status_generation': args['config'].perception_status['generation'] += 1
    if change == 'status_tf': args['config'].perception_status['tf_lookup_stamp_ns'] += 1
    report = build_silhouette_snapshot_evidence(**args)
    assert not report['ok']
    assert not report['execution_authority'] and not report['observation_authority']
    assert 'measurement' not in report


def test_one_pixel_nonreproduced_mask_is_rejected_instead_of_assumed_original():
    args = bound_inputs(); args['sample'].object_mask[120, 150] ^= 255
    report = build_silhouette_snapshot_evidence(**args)
    assert not report['ok'] and report['code'] == 'SILHOUETTE_BASELINE_MASK_NOT_REPRODUCED'
    assert report['differing_pixels'] == 1


def test_wrong_current_plane_is_not_replaced_with_fused_plane():
    args = bound_inputs()
    args['config'].perception_status['geometry_metrics']['support_plane_base'][3] += .015
    report = build_silhouette_snapshot_evidence(**args)
    assert not report['ok']
    assert 'measurement' not in report


def test_no_exact_original_sample_fails_without_resampling():
    args = bound_inputs(); args['sample'] = None
    report = build_silhouette_snapshot_evidence(**args)
    assert not report['ok'] and report['diagnostic_only']
