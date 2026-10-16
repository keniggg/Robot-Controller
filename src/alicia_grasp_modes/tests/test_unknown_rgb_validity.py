"""Real producer paths reject unobserved RGB without editing measured depth."""
import copy
import json

import numpy as np
import pytest

from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
from test_unknown_stream import module, rgb_interest_node
from test_rgb_handoff_stream import frozen_node, settled
from test_locked_rgb_hold import locked


def missing_target_rgb():
    valid = np.ones((240, 320), bool)
    valid[106:134, 148:173] = False
    return valid


def test_disabled_rectification_adds_no_rgb_keyword():
    assert module._rgb_source_kwargs({}, (240, 320)) == {}
    assert module._rgb_source_kwargs({'runtime_profile': {}}, (240, 320)) == {}


def test_actual_geometry_contract_decoded_by_producer_helper():
    intrinsics = dict(width=320, height=240, fx=400., fy=400., cx=160., cy=120.,
                      model='none', coeffs=[0.]*5)
    mapper = PairedPinholeRectification(intrinsics)
    profile = dict(geometry_intrinsics=mapper.output_intrinsics, rectification=mapper.contract)
    for key in ('runtime_profile', 'runtime_profile_geometry'):
        result = module._rgb_source_kwargs({key: profile}, (240, 320))
        assert result['rgb_valid'].dtype == bool and result['rgb_valid'].all()
    broken = copy.deepcopy(profile)
    broken['rectification']['rgb_valid_mask_sha256'] = '0'*64
    with pytest.raises(ValueError, match='rectification'):
        module._rgb_source_kwargs({'runtime_profile': broken}, (240, 320))


def test_initial_rgb_interest_cannot_freeze_padding_as_original_identity(monkeypatch):
    n, clock, feed, _, parameters = rgb_interest_node(monkeypatch)
    seen = []
    def domain(camera, shape):
        seen.append((camera, shape))
        return missing_target_rgb()
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', domain)
    for index in range(3):
        clock[0] = 10.+index*.6
        messages = feed()
        assert not n._rgb_observation_interest['observation_interest_ready']
        assert not n._rgb_reference_cache and n.tracker.anchor is None
        assert not n.pubs['object'].publish.call_args[0][0].detected
        depth = n.bridge.imgmsg_to_cv2(messages[1], 'passthrough')
        assert depth[0, 0] == np.float32(.4) and depth[120, 160] == 0
    assert len(seen) == 3
    assert all(camera is parameters['/camera'] and shape == (240, 320) for camera, shape in seen)


def test_initial_metric_refinement_rejects_padding_without_erasing_real_depth(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', lambda *_: missing_target_rgb())
    for _ in range(3):
        source = feed(target_depth=.38)
        assert not n.pubs['object'].publish.call_args[0][0].detected
        assert n.tracker.anchor is None and n.acquisition.samples == []
        native = n.bridge.imgmsg_to_cv2(source[1], 'passthrough')
        np.testing.assert_array_equal(native[106:134, 148:173], np.full((28, 25), .38, np.float32))
        clock[0] += .6
    status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert 'unobserved_rgb' in status['detail']


def test_cross_view_padding_cannot_match_or_replace_frozen_reference(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    reference = n._rgb_acquisition_active['reference']
    deadline = n._rgb_acquisition_active['deadline_ns']
    settled(n, clock)
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', lambda *_: missing_target_rgb())
    for _ in range(3):
        clock[0] += .2
        source = feed(target_depth=.38)
        assert not n.pubs['object'].publish.call_args[0][0].detected
        assert n._rgb_acquisition_active['reference'] is reference
        assert n._rgb_acquisition_active['deadline_ns'] == deadline
        assert n._rgb_handoff_status['reason'] == 'no_cross_view_rgb_match'
        assert n.tracker.anchor is None
        assert n.bridge.imgmsg_to_cv2(source[1], 'passthrough')[120, 160] == np.float32(.38)


def test_tracked_metric_refinement_never_uses_padding_as_current_instance(monkeypatch):
    n, clock, feed, _, _ = locked(monkeypatch)
    anchor = n.tracker.anchor
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', lambda *_: missing_target_rgb())
    source = feed(target_depth=.35)
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert n.tracker.anchor is anchor and not n.tracker.lost
    status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert 'rgb_depth_refinement_rejected' in status['detail'] and 'unobserved_rgb' in status['detail']
    assert n.bridge.imgmsg_to_cv2(source[1], 'passthrough')[120, 160] == np.float32(.35)


def test_short_depth_gap_does_not_claim_identity_from_unobserved_rgb(monkeypatch):
    n, clock, feed, _, _ = locked(monkeypatch)
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', lambda *_: missing_target_rgb())
    feed(target_depth=0.)
    status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert status['rgb_identity_hold']['state'] != 'holding_identity_only'
    assert 'no_unique_current_rgb_match' in status['rgb_identity_hold']['reason']
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_depth_only_candidate_cannot_seed_rgb_hold_with_unobserved_pixels(monkeypatch):
    n, _, feed, _, parameters = locked(monkeypatch)
    source = feed(target_depth=.35)
    depth_before = n.bridge.imgmsg_to_cv2(source[1], 'passthrough').copy()
    monkeypatch.setattr(module, 'rectification_rgb_valid_mask', lambda *_: missing_target_rgb())
    assert n._prepare_locked_rgb_hold_reference(source, n.tracker.last, parameters['/camera']) is None
    np.testing.assert_array_equal(n.bridge.imgmsg_to_cv2(source[1], 'passthrough'), depth_before)


def test_malformed_enabled_contract_cannot_fall_back_to_unbounded_rgb(monkeypatch):
    n, clock, feed, _, parameters = rgb_interest_node(monkeypatch)
    parameters['/camera']['runtime_profile'] = {'rectification': {'enabled': True}}
    for index in range(3):
        clock[0] = 10.+index*.6
        feed()
        assert not n._rgb_observation_interest['observation_interest_ready']
        assert not n._rgb_reference_cache and n.tracker.anchor is None


@pytest.mark.parametrize('offset,accepted', [(-1, True), (0, False), (1, False)])
@pytest.mark.parametrize('restored_native', [False, True])
def test_same_canvas_old_source_cannot_use_new_geometry_even_while_fresh(monkeypatch, offset, accepted, restored_native):
    from unittest.mock import Mock
    n, _, feed, _, parameters = rgb_interest_node(monkeypatch)
    mapper = PairedPinholeRectification(dict(width=320, height=240, fx=400., fy=400.,
        cx=160., cy=120., model='none', coeffs=[0.]*5))
    parameters['/camera']['runtime_profile'] = dict(geometry_intrinsics=mapper.output_intrinsics,
        rectification=mapper.contract, geometry_activation_stamp_ns=9_900_000_000+offset)
    if restored_native:
        parameters['/camera']['runtime_profile'].pop('rectification')
    segment = Mock(wraps=module.segment_with_support_fallback)
    monkeypatch.setattr(module, 'segment_with_support_fallback', segment)
    source = feed()
    assert source[0].header.stamp.to_nsec() == 9_900_000_000
    assert segment.call_count == int(accepted)
    if not accepted:
        status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
        assert 'rectification_source_predates_geometry_activation' in status['detail']
        assert not n._rgb_observation_interest['observation_interest_ready']
    assert n.bridge.imgmsg_to_cv2(source[1], 'passthrough')[0, 0] == np.float32(.4)
