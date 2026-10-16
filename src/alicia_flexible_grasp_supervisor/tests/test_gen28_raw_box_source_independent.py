"""Independent real-fuser/remote integration: raw pixels, fused measured geometry.

All samples are generated calibrated ray measurements. No external experiment
files, ROS master, publisher or hardware are used by these tests.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib
from types import SimpleNamespace as NS

import numpy as np
import pytest

import test_remote_grasp6d_streaming as fixtures
from test_gen26_regular_box_inference_independent import ray_scene
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples
from alicia_flexible_grasp.vision.target_observation import TargetObservation, TargetTrackIdentity
from alicia_flexible_grasp.vision.object_geometry import deproject_depth

remote = fixtures.remote_node


def digest(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def real_fused_case(monkeypatch, *, erosion=2):
    scene = ray_scene()
    scene.K.depth_scale = .0001
    params = {'/grasp/unknown_require_measured_bilateral_contact': False}
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    monkeypatch.setattr(remote.rospy, 'get_param_cached', lambda name, default=None: params.get(name, default))
    ys, xs = np.nonzero(scene.mask)
    bbox = (int(xs.min()), int(ys.min()), int(np.ptp(xs)) + 1, int(np.ptp(ys)) + 1)
    samples = []
    for i in range(3):
        stamp = scene.stamp - (2-i)*100_000_000
        depth = scene.depth.copy()
        # Earlier images differ at real target depth, so a caller silently
        # substituting fused pixels cannot pass the raw-source hash assertions.
        if erosion and i < 2:
            depth[scene.mask > 0] += 1
        samples.append(RgbdSample(np.zeros(depth.shape+(3,), np.uint8), depth,
            scene.mask.copy(), bbox, NS(detected=True), stamp*1e-9,
            'camera_optical_frame', np.zeros(6), stamp, scene.identity.epoch, scene.identity))
    snapshot = fuse_stable_samples(samples, require_mask=True, min_mask_iou=.85,
        max_centroid_shift_px=5., max_joint_delta_rad=.01, erosion_px=erosion,
        depth_scale=.0001, depth_min_m=.03, depth_max_m=2., mad_scale=3.5,
        mad_absolute_floor_m=.002, internal_hole_max_area_px=25)
    assert snapshot.ok, snapshot.failure_reason
    points, _ = deproject_depth(snapshot.target_depth_raw, snapshot.target_depth_raw > 0,
        scene.K, .0001)
    measured = points @ scene.T[:3, :3].T + scene.T[:3, 3]
    geometry = replace(scene.geometry, object_points_base=measured)
    edge = min(bbox[0], bbox[1], snapshot.depth_raw.shape[1]-bbox[0]-bbox[2],
               snapshot.depth_raw.shape[0]-bbox[1]-bbox[3])
    observed = TargetObservation(scene.identity, snapshot.stamp_ns, 'base_link',
        geometry.object_points_base, geometry.support_normal_base,
        geometry.support_offset_m, bbox, snapshot.depth_raw.shape, edge,
        'instance_mask', 'unknown_small')
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node.near_field_planning_active = True
    node._stream_generation, node._near_field_phase_id = 4, 1
    node._current_stream_target_identity = lambda: scene.identity
    node._latest_target_observation = observed
    node._camera_intrinsics = lambda: scene.K
    return NS(node=node, scene=scene, snapshot=snapshot, geometry=geometry,
              params=params, observation=observed)


def prepare(case, snapshot=None):
    return case.node._prepare_regular_box_snapshot(snapshot or case.snapshot,
        case.geometry, case.scene.T)


def test_real_fuser_remote_qualifies_raw_triad_and_preserves_fused_measured_points(monkeypatch):
    c = real_fused_case(monkeypatch)
    raw = c.snapshot.original_sample
    assert not np.array_equal(raw.depth_raw, c.snapshot.depth_raw)
    assert not np.array_equal(raw.object_mask, c.snapshot.object_mask)
    original_points = c.geometry.object_points_base.copy()
    original_pixels = {name: getattr(c.snapshot, name).copy()
                       for name in ('depth_raw', 'target_depth_raw', 'object_mask', 'color_bgr')}
    planning = prepare(c)
    model = c.node._current_inferred_box()
    assert model is not None, c.node._regular_box_snapshot
    assert model.source['depth_sha256'] == digest(raw.depth_raw)
    assert model.source['mask_sha256'] == digest(raw.object_mask)
    assert model.source['color_sha256'] == digest(raw.color_bgr)
    assert model.source['stamp_ns'] == c.snapshot.stamp_ns
    assert model.source['target_identity'] == dict(epoch=c.scene.identity.epoch, track_id=c.scene.identity.track_id)
    np.testing.assert_array_equal(planning.object_points_base, original_points)
    np.testing.assert_array_equal(c.geometry.object_points_base, original_points)
    for name, value in original_pixels.items():
        np.testing.assert_array_equal(getattr(c.snapshot, name), value)
    # Inference remains an enclosure; it never appends invented side samples.
    assert c.node._regular_box_snapshot['result'].audit['measured_side_surfaces'] is False


@pytest.mark.parametrize('fault', [
    'stamp_ns', 'stamp_sec', 'identity', 'epoch', 'frame', 'color',
    'lineage', 'mask_digest', 'mask_digest_absent', 'mask_digest_count',
    'raw_mask_changed', 'raw_missing', 'raw_wrong_type'])
def test_actual_remote_rejects_wrong_or_missing_retained_source_without_fused_fallback(monkeypatch, fault):
    # Erosion=0 makes the fused source otherwise qualify. Rejection must come
    # from the source contract, not an unrelated malformed or cropped box.
    c = real_fused_case(monkeypatch, erosion=0)
    prepare(c)
    assert c.node._current_inferred_box() is not None
    s = c.snapshot
    if fault == 'stamp_ns': s = replace(s, original_sample=replace(s.original_sample, stamp_ns=s.stamp_ns+1))
    elif fault == 'stamp_sec': s = replace(s, original_sample=replace(s.original_sample, stamp_sec=s.stamp_sec+.01))
    elif fault == 'identity': s = replace(s, original_sample=replace(s.original_sample, target_identity=TargetTrackIdentity.from_stream(99, s.target_epoch)))
    elif fault == 'epoch': s = replace(s, original_sample=replace(s.original_sample, target_epoch=s.target_epoch+1))
    elif fault == 'frame': s = replace(s, original_sample=replace(s.original_sample, frame_id='another_optical_frame'))
    elif fault == 'color': s = replace(s, original_sample=replace(s.original_sample, color_bgr=np.ones_like(s.color_bgr)))
    elif fault == 'lineage': s = replace(s, sample_stamp_ns=s.sample_stamp_ns[:-1]+(s.stamp_ns+1,))
    elif fault == 'mask_digest': s = replace(s, sample_mask_sha256=s.sample_mask_sha256[:-1]+('f'*64,))
    elif fault == 'mask_digest_absent': s = replace(s, sample_mask_sha256=())
    elif fault == 'mask_digest_count': s = replace(s, sample_mask_sha256=s.sample_mask_sha256[:-1])
    elif fault == 'raw_mask_changed':
        mask = s.original_sample.object_mask.copy()
        mask.flat[np.flatnonzero(mask)[0]] = 0
        s = replace(s, original_sample=replace(s.original_sample, object_mask=mask))
    elif fault == 'raw_missing': s = replace(s, original_sample=None)
    else: s = replace(s, original_sample=NS(**vars(s.original_sample)))
    planning = prepare(c, s)
    assert c.node._current_inferred_box() is None
    assert planning is c.geometry
    result = c.node._regular_box_snapshot['result']
    assert result.ok is False and result.model is None
    assert result.code in ('BOX_INFERENCE_SOURCE_UNAVAILABLE', 'BOX_INFERENCE_SOURCE_MISMATCH')


@pytest.mark.parametrize('change', ['generation', 'phase', 'observation', 'policy'])
def test_raw_box_cannot_survive_subsequent_source_or_policy_change(monkeypatch, change):
    c = real_fused_case(monkeypatch)
    prepare(c)
    assert c.node._current_inferred_box() is not None
    if change == 'generation': c.node._stream_generation += 1
    elif change == 'phase': c.node._near_field_phase_id += 1
    elif change == 'observation': c.node._latest_target_observation = deepcopy(c.observation)
    else: c.params['/grasp/unknown_require_measured_bilateral_contact'] = True
    assert c.node._current_inferred_box() is None


def test_valid_fused_box_does_not_hide_invalid_actual_raw_depth(monkeypatch):
    c = real_fused_case(monkeypatch, erosion=0)
    prepare(c)
    assert c.node._current_inferred_box() is not None
    s = replace(c.snapshot, original_sample=replace(c.snapshot.original_sample,
        depth_raw=np.zeros_like(c.snapshot.depth_raw)))
    planning = prepare(c, s)
    assert planning is c.geometry
    assert c.node._current_inferred_box() is None
    assert c.node._regular_box_snapshot['result'].ok is False

