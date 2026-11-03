"""Actual producer source snapshots and executing ownership, without ROS motion."""
from copy import deepcopy
import hashlib
from unittest.mock import Mock

import numpy as np
import pytest

from test_locked_rgb_hold import locked, module
from test_observation_measurement_unknown import send, ack


def request_for(node, clock, params):
    node._observation_measurement_ack_pub = Mock()
    return dict(version=1, state='executing', motion_kind='observation',
        lease_id='face-lease', request_id='face-request', session_id='face-session',
        selection=deepcopy(params['/grasp_mode/selection']), target_epoch=1,
        target_track_id='target1', origin_plan_id='plan1',
        origin_snapshot_stamp_ns=node.anchor_source_stamp_ns, view_plan_id='plan2',
        view_source_stamp_ns=node.last_good_source_stamp_ns, view_index=1,
        plan_sha256='a'*64, control_epoch_ns=node._rgb_interest_epoch,
        deadline_ns=int((clock[0]+60)*1e9), stamp_ns=module.rospy.Time.now().to_nsec(),
        metric_geometry_authority=False, motion_authorized=False)


def sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


def test_real_hold_captures_same_source_immutable_depth_geometry_and_configuration(monkeypatch):
    node, _, feed, _, params = locked(monkeypatch)
    pair = feed(target_depth=.35)
    hold = node._locked_rgb_hold_reference
    source = hold['measured_face_source']
    raw = node.bridge.imgmsg_to_cv2(pair[1], desired_encoding='passthrough')
    assert source.source_ns == pair[1].header.stamp.to_nsec() == hold['source_stamp_ns']
    assert source.source_id == hold['source_id']
    assert source.encoding == '32FC1' and source.depth_scale == 1.
    assert source.depth_sha256 == sha(raw) == hold['source_depth_sha256']
    assert source.rgb_sha256 == hold['source_rgb_sha256']
    assert source.whole_mask_sha256 == hold['refined_mask_sha256']
    assert np.array_equal(source.raw_depth, raw)
    assert np.array_equal(source.plane, node.tracker.plane_base)
    assert np.array_equal(source.intrinsics, [params['/camera'][k] for k in ('fx','fy','cx','cy')])
    for array in (source.raw_depth, source.intrinsics, source.transform, source.plane):
        with pytest.raises(ValueError):
            array.setflags(write=True)
    with pytest.raises(AttributeError):
        source.raw_depth = raw.copy()
    saved = source.raw_depth.copy()
    pair[1].data = bytes(len(pair[1].data))
    assert np.array_equal(source.raw_depth, saved)
    assert source.depth_sha256 == sha(saved)


@pytest.mark.parametrize('encoding', ['16UC1', 'mono16', '32FC1'])
def test_depth_encoding_and_scale_are_bound_to_original_message(monkeypatch, encoding):
    node, _, feed, _, params = locked(monkeypatch)
    original_pair = feed(target_depth=.35)
    before = node._locked_rgb_hold_reference['measured_face_source']
    metric = before.metric_depth()
    camera = params['/camera']
    camera['depth_scale'] = .001
    raw = metric.copy() if encoding == '32FC1' else np.rint(metric/.001).astype(np.uint16)
    depth = node.bridge.cv2_to_imgmsg(raw, encoding=encoding)
    depth.header = deepcopy(original_pair[1].header)
    pair = (original_pair[0], depth)
    hold = node._prepare_locked_rgb_hold_reference(pair, node.tracker.last, camera,
        source_transform=before.transform)
    assert hold is not None
    source = hold['measured_face_source']
    assert source.encoding == encoding
    assert source.depth_scale == (1. if encoding == '32FC1' else .001)
    assert source.depth_sha256 == sha(raw)
    assert np.allclose(source.metric_depth(), metric, rtol=0., atol=1e-6)


def test_missing_explicit_source_transform_cannot_gain_common_face_proof(monkeypatch):
    node, _, feed, _, params = locked(monkeypatch)
    pair = feed(target_depth=.35)
    hold = node._prepare_locked_rgb_hold_reference(pair, node.tracker.last, params['/camera'])
    assert hold is not None
    assert 'measured_face_source' not in hold


@pytest.mark.parametrize('field', ['source_depth_sha256', 'source_rgb_sha256',
    'refined_mask_sha256', 'measured_face_source_binding_sha256', 'config_sha256'])
def test_mismatched_original_hold_binding_cannot_accept_executing(monkeypatch, field):
    node, clock, _, _, params = locked(monkeypatch)
    request = request_for(node, clock, params)
    node._locked_rgb_hold_reference[field] = 'f'*64
    send(node, request)
    assert not ack(node)['accepted']
    assert ack(node)['source_stamps_ns'] == []
    assert getattr(node, '_observation_measurement', None) is None


@pytest.mark.parametrize('mutation', ['manual', 'epoch', 'model', 'camera', 'hold',
    'measured_source', 'binding', 'source_expired', 'lease_expired', 'sdk_out_and_back'])
def test_unavailable_face_does_not_swallow_owner_changes_during_copy(monkeypatch, mutation):
    node, clock, _, joint, params = locked(monkeypatch)
    request = request_for(node, clock, params)
    hold = node._locked_rgb_hold_reference
    initial_good = node.last_good_source_stamp_ns
    calls = []
    def changed(*args, **kwargs):
        calls.append(args[0])
        kwargs['check_current']()
        if mutation == 'manual': params['/gui/joint_direct_mode'] = True
        elif mutation == 'epoch': node._rgb_interest_epoch += 1
        elif mutation == 'model': params['/robot_description'] = '<robot name="changed"/>'
        elif mutation == 'camera': params['/camera']['fx'] += 1
        elif mutation == 'hold': node._locked_rgb_hold_reference = dict(hold)
        elif mutation == 'measured_source': hold['measured_face_source'] = None
        elif mutation == 'binding': hold['measured_face_source_binding_sha256'] = 'f'*64
        elif mutation == 'source_expired': clock[0] += .801
        elif mutation == 'lease_expired': clock[0] += 61.
        else:
            joint(clock[0]-.02, [.005, 0., 0., 0., 0., 0.])
            joint(clock[0]-.01, [0.]*6)
        raise module.MeasuredFaceUnavailable('synthetic_face_insufficiency_after_change')
    monkeypatch.setattr(module, 'freeze_measured_face', changed)
    send(node, request)
    assert len(calls) == 1
    assert not ack(node)['accepted']
    assert getattr(node, '_observation_measurement', None) is None
    assert node.last_good_source_stamp_ns == initial_good
    assert 'face-lease' not in getattr(node, '_observation_measurement_seen', ())


def test_plain_face_insufficiency_keeps_legacy_identity_without_qualification(monkeypatch):
    node, clock, _, _, params = locked(monkeypatch)
    request = request_for(node, clock, params)
    def insufficient(*args, **kwargs):
        kwargs['check_current']()
        raise module.MeasuredFaceUnavailable('no_complete_measured_face')
    monkeypatch.setattr(module, 'freeze_measured_face', insufficient)
    send(node, request)
    assert ack(node)['accepted'] and not ack(node)['qualified']
    assert ack(node)['source_stamps_ns'] == []
    reference = node._observation_measurement['confirmation'].appearance_reference
    assert reference.measured_face is None
    assert node._observation_measurement['confirmation'].source_stamps_ns == []


def test_non_optional_face_error_is_not_converted_into_legacy_acceptance(monkeypatch):
    node, clock, _, _, params = locked(monkeypatch)
    request = request_for(node, clock, params)
    def broken(*args, **kwargs):
        raise ValueError('source_binding_broken')
    monkeypatch.setattr(module, 'freeze_measured_face', broken)
    send(node, request)
    assert not ack(node)['accepted'] and ack(node)['reason'] == 'source_binding_broken'
    assert getattr(node, '_observation_measurement', None) is None
