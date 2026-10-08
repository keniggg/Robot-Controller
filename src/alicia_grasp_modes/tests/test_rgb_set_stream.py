"""Actual RGB/depth algorithms with fake ROS transport for set acquisition."""
import json
from dataclasses import replace

import numpy as np
import pytest
from std_msgs.msg import String

from test_unknown_stream import module, rgb_interest_node
from test_rgb_handoff_stream import settled


def set_node(monkeypatch):
    n, clock, _, joint, parameters = rgb_interest_node(monkeypatch)
    # Two independently visible, equally plausible interests. Actual geometry
    # is absent until native depth samples are explicitly supplied by a test.
    boxes = ((134, 106, 16, 28), (170, 106, 16, 28))
    def feed(depths=(0., 0.), visible=(True, True), q=None, measured_pixels=(None, None)):
        stamp = clock[0]-.1
        rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
        depth = np.full((240, 320), .4, np.float32)
        for box, measured_depth, shown, count in zip(boxes, depths, visible, measured_pixels):
            x, y, w, h = box
            if shown:
                rgb[y:y+h, x:x+w] = 20
                depth[y:y+h, x:x+w] = measured_depth
                if count is not None:
                    native = np.zeros((h, w), np.float32)
                    native.flat[:count] = measured_depth
                    depth[y:y+h, x:x+w] = native
        messages = (n.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                    n.bridge.cv2_to_imgmsg(depth, encoding='32FC1'))
        for message in messages:
            message.header.stamp = module.rospy.Time.from_sec(stamp)
            message.header.frame_id = n.camera_frame
        joint(stamp-.05, q); joint(stamp+.05, q)
        n.latest = messages
        n.process(None)
        return messages
    return n, clock, feed, parameters


def ready_set(monkeypatch):
    n, clock, feed, parameters = set_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    assert n._rgb_observation_interest['observation_interest_ready'], n._rgb_observation_interest
    assert len(n._rgb_observation_interest['rgb_interest_members']) == 2
    return n, clock, feed, parameters


def freeze_set(monkeypatch, mutate_request=None):
    n, clock, feed, parameters = ready_set(monkeypatch)
    interest = n._rgb_observation_interest
    request = dict(version=1, request_id='offline-set-request', source_id=interest['source_id'],
        selection=parameters['/grasp_mode/selection'], source_stamp_ns=str(interest['source_stamp_ns']),
        deadline_ns=str(int((clock[0]+100)*1e9)), max_observations=3,
        **{key:interest[key] for key in ('region_mask_sha256', 'source_rgb_sha256',
                                       'source_depth_sha256', 'observation_set_sha256')})
    if mutate_request is not None:
        mutate_request(request)
    n._rgb_reference_request_cb(String(data=json.dumps(request)))
    n._service_rgb_reference_request()
    return n, clock, feed, parameters, request


def detected(n):
    return n.pubs['object'].publish.call_args[0][0].detected


def test_initial_three_frames_preserve_two_rgb_interests_without_detected_geometry(monkeypatch):
    n, clock, feed, _ = set_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index; feed()
        interest = n._rgb_observation_interest
        assert interest['observation_interest_ready'] is (index == 2)
        assert len(interest['rgb_interest_members']) == 2
        assert not interest['single_rgb_hypothesis']
        assert not detected(n) and n.tracker.anchor is None
        assert not np.any(n.bridge.imgmsg_to_cv2(n.pubs['object_mask'].publish.call_args[0][0], 'mono8'))
    assert len(interest['interest_volumes']) == 2
    assert all(not v['complete_object_volume'] for v in interest['interest_volumes'])
    assert not interest['metric_geometry_authority'] and not interest['motion_authorized']
    n.pubs['object_pose_base'].publish.assert_not_called()


def test_set_freeze_ack_binds_members_and_exact_source_digest(monkeypatch):
    n, _, _, _, request = freeze_set(monkeypatch)
    ack = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert ack['status'] == 'frozen'
    assert ack['observation_set_sha256'] == request['observation_set_sha256']
    ref = n._rgb_acquisition_active['reference']
    assert isinstance(ref, module.RgbInterestSetReference) and len(ref.members) == 2
    assert len({m.mask_fingerprint for m in ref.members}) == 2
    assert len({m.volume.bbox_xywh for m in ref.members}) == 2
    assert not detected(n)


@pytest.mark.parametrize('field', ['observation_set_sha256', 'source_rgb_sha256',
                                 'source_depth_sha256', 'region_mask_sha256'])
def test_wrong_set_or_pixels_cannot_freeze(monkeypatch, field):
    n, *_ = freeze_set(monkeypatch, lambda request:request.update({field:'0'*64}))
    ack = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert ack['status'] == 'rejected' and n._rgb_acquisition_active is None


def test_missing_member_after_settle_cannot_reselect_remaining_depth_object(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    reference = n._rgb_acquisition_active['reference']
    settled(n, clock)
    for _ in range(4):
        clock[0] += .2; feed(depths=(.38, 0.), visible=(True, False))
        assert not detected(n) and n.tracker.anchor is None
        assert not n._rgb_observation_interest['observation_interest_ready']
    assert n._rgb_acquisition_active['reference'] is reference
    assert n._rgb_handoff_status['reason'] == 'no_cross_view_rgb_match'


def test_duplicate_rgb_proposals_after_settle_cannot_make_member_unique(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    original = module.propose_rgb_observation_regions
    def duplicate(*args, **kwargs):
        result = original(*args, **kwargs)
        return replace(result, candidates=result.candidates+(result.candidates[0],),
                       total_candidate_count=result.total_candidate_count+1)
    monkeypatch.setattr(module, 'propose_rgb_observation_regions', duplicate)
    settled(n, clock)
    for _ in range(3):
        clock[0] += .2; feed(depths=(.38, 0.))
    assert not detected(n) and n.tracker.anchor is None
    assert n._rgb_handoff_status['reason'] == 'cross_view_rgb_ambiguous'


def test_two_measured_members_never_pick_first_refined_as_target(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    settled(n, clock)
    for _ in range(4):
        clock[0] += .2; feed(depths=(.38, .38))
        assert not detected(n) and n.tracker.anchor is None
    assert n._rgb_handoff_status['reason'] == 'multiple_original_rgb_members_have_measured_geometry'
    assert n._rgb_observation_interest['observation_interest_ready']
    assert len(n._rgb_observation_interest['rgb_interest_members']) == 2


@pytest.mark.parametrize('member', [0, 1])
def test_only_unique_original_member_with_three_actual_depth_frames_can_handoff(monkeypatch, member):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    settled(n, clock)
    depths = [.0, .0]; depths[member] = .38
    for index in range(3):
        clock[0] += .2; feed(depths=depths)
        assert detected(n) is (index == 2), n._rgb_handoff_status
    assert n.tracker.anchor is not None and n._rgb_acquisition_active is None
    assert n._rgb_handoff_status['state'] == 'measured_ready'
    assert n.tracker.anchor.metrics['valid_points'] >= 120
    expected_x = (134 if member == 0 else 170)+7.5
    assert n.tracker.anchor.center_uv[0] == pytest.approx(expected_x, abs=1.)


def test_continuation_preserves_all_members_volumes_and_original_reference_set(monkeypatch):
    n, clock, feed, _, request = freeze_set(monkeypatch)
    reference = n._rgb_acquisition_active['reference']
    deadline = n._rgb_acquisition_active['deadline_ns']
    settled(n, clock)
    for _ in range(3):
        clock[0] += .2; feed()
    interest = n._rgb_observation_interest
    ack = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert ack['status'] == 'supplement_ready'
    assert len(interest['rgb_interest_members']) == len(interest['interest_volumes']) == 2
    assert len(interest['association_reference_volumes']) == 2
    assert interest['association_reference_set_sha256'] == request['observation_set_sha256']
    assert interest['observation_set_sha256'] == ack['observation_set_sha256']
    assert all(tuple(v['bbox_xywh']) == tuple(r['bbox_xywh'])
               for v, r in zip(interest['interest_volumes'], interest['rgb_interest_members']))
    assert not interest['single_rgb_hypothesis']
    settled(n, clock, phase='reserved', observation_index=2)
    clock[0] += .2; feed()
    assert n._rgb_acquisition_active['reference'] is reference
    assert n._rgb_acquisition_active['deadline_ns'] == deadline
    assert n._rgb_observation_interest['observation_interest_ready']
    assert n._rgb_observation_interest['association_reference_set_sha256'] == request['observation_set_sha256']


def test_switching_which_member_has_depth_restarts_measured_three_frame_confirmation(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    settled(n, clock)
    for depths in ((.38, 0.), (.38, 0.), (0., .38), (0., .38)):
        clock[0] += .2; feed(depths=depths)
        assert not detected(n) and n.tracker.anchor is None
    clock[0] += .2; feed(depths=(0., .38))
    assert detected(n) and n.tracker.anchor is not None


def test_rgb_set_cannot_replace_the_original_minimum_actual_depth_count(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    settled(n, clock)
    for _ in range(4):
        clock[0] += .2; feed(depths=(.38, 0.), measured_pixels=(119, None))
        assert not detected(n) and n.tracker.anchor is None
    assert n._rgb_acquisition_active is not None
    assert not n._rgb_handoff_status['metric_geometry_authority']


def test_original_raw_support_disagreement_blocks_all_set_handoff(monkeypatch):
    n, clock, feed, _, _ = freeze_set(monkeypatch)
    original = module.segment_with_support_fallback
    def changed(*args, **kwargs):
        result = original(*args, **kwargs)
        result.plane_base = result.plane_base+np.array([0., 0., 0., .019])
        return result
    monkeypatch.setattr(module, 'segment_with_support_fallback', changed)
    # Isolate the existing raw reference-plane audit after the independent
    # coverage gate, which is tested separately with actual depth above.
    monkeypatch.setattr(module, 'measured_support_coverage', lambda *args:dict(spatial_quadrants=4))
    settled(n, clock)
    clock[0] += .2; feed(depths=(.38, 0.))
    assert not detected(n) and n.tracker.anchor is None
    assert n._rgb_handoff_status['state'] == 'rejected'
    audit = n._rgb_handoff_status['support_audit']
    assert audit['support_reference_max_distance_m'] == pytest.approx(.019)
    assert audit['support_alignment_applied'] is False
