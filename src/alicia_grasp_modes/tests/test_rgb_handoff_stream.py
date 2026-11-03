"""Transport-only tests for freeze/settle handoff; no running ROS or robot."""
import json
from dataclasses import replace

import numpy as np
import pytest
from std_msgs.msg import String, Header

from test_unknown_stream import module, rgb_interest_node


def frozen_node(monkeypatch, mutate_request=None):
    n, clock, feed, joint, parameters = rgb_interest_node(monkeypatch)
    for index in range(3):
        clock[0] = 10.+.6*index
        feed()
    interest = n._rgb_observation_interest
    request = dict(version=1, request_id='offline-test-request',
        source_id=interest['source_id'], selection=parameters['/grasp_mode/selection'],
        source_stamp_ns=str(interest['source_stamp_ns']), deadline_ns=str(int((clock[0]+100)*1e9)),
        max_observations=3, **{key:interest[key] for key in ('region_mask_sha256',
            'source_rgb_sha256', 'source_depth_sha256', 'observation_set_sha256')})
    if mutate_request:
        mutate_request(request)
    n._rgb_reference_request_cb(String(data=json.dumps(request)))
    n._service_rgb_reference_request()
    return n, clock, feed, joint, parameters, request


def settled(n, clock, phase='awaiting_measured_target', observation_index=1):
    active = n._rgb_acquisition_active
    payload = dict(version=1, request_id=active['request_id'], source_id=active['source_id'],
        reference_id=active['reference_id'], phase=phase,
        settled_ns=str(int(clock[0]*1e9)), observation_index=observation_index)
    n._rgb_acquisition_status_cb(String(data=json.dumps(payload)))


def test_freeze_ack_retains_exact_original_pixels_and_reference(monkeypatch):
    n, _, _, _, _, request = frozen_node(monkeypatch)
    response = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert response['status'] == 'frozen'
    assert response['request_id'] == request['request_id']
    active = n._rgb_acquisition_active
    assert active['reference'].source_stamp_ns == int(request['source_stamp_ns'])
    assert active['reference_id'] == response['reference_id']
    assert len(n._rgb_reference_cache) == 0
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('field', ['region_mask_sha256', 'source_rgb_sha256', 'source_depth_sha256',
                                 'observation_set_sha256', 'source_id'])
def test_wrong_pixel_or_source_hash_cannot_freeze(monkeypatch, field):
    n, *_ = frozen_node(monkeypatch, lambda request:request.update({field:'0'*64}))
    response = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert response['status'] == 'rejected'
    assert n._rgb_acquisition_active is None


def test_frozen_or_executing_task_does_not_acquire_new_center_target(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    for phase in ('frozen', 'executing'):
        if phase == 'executing':
            settled(n, clock, phase)
        for _ in range(3):
            clock[0] += .2
            feed(target_depth=.38)
            assert n.tracker.anchor is None
            assert not n.pubs['object'].publish.call_args[0][0].detected
        if phase == 'frozen':
            assert n._rgb_observation_interest['observation_interest_ready']
            assert n._rgb_observation_interest['reason'] == 'frozen_reference_fresh_scene'
    assert n._rgb_handoff_status['reason'] == 'waiting_for_gateway_settled_receipt_and_new_frame'


def test_settled_handoff_requires_three_real_associated_frames_then_publishes(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    settled(n, clock)
    for index in range(3):
        clock[0] += .2
        feed(target_depth=.38)
        assert n.pubs['object'].publish.call_args[0][0].detected is (index == 2)
    assert n.tracker.anchor is not None
    status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
    assert status['detail'] == 'rgb_associated_measured_acquisition'
    assert status['rgb_acquisition_handoff']['state'] == 'measured_ready'
    assert n._rgb_acquisition_active is None


def test_same_object_rgb_without_depth_requests_next_observation_not_detection(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    original_reference = n._rgb_acquisition_active['reference']
    settled(n, clock)
    for index in range(3):
        clock[0] += .2
        feed()
        assert not n.pubs['object'].publish.call_args[0][0].detected
    ack = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert ack['status'] == 'supplement_ready'
    interest = n._rgb_observation_interest
    assert interest['observation_interest_ready']
    assert interest['rgb_track']['source_id'] == interest['source_id']
    assert interest['reference_id'] == ack['reference_id']
    assert interest['association_sha256'] == ack['association_sha256']
    assert n._rgb_acquisition_active['reference'] is original_reference
    assert interest['source_stamp_ns'] > original_reference.source_stamp_ns
    assert not interest['metric_geometry_authority'] and not interest['motion_authorized']


def test_second_observation_resets_rgb_confirmation_without_reanchoring(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    reference = n._rgb_acquisition_active['reference']
    settled(n, clock)
    for _ in range(3):
        clock[0] += .2; feed()
    assert n._rgb_observation_interest['observation_interest_ready']
    settled(n, clock, phase='reserved', observation_index=2)
    clock[0] += .2; feed()
    # Planning a second view still reports the first completed view, with fresh
    # same-stamp evidence instead of freezing a new identity at image centre.
    refreshed = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
    assert refreshed['status'] == 'supplement_ready' and refreshed['observation_index'] == 1
    assert int(refreshed['source_stamp_ns']) == n._rgb_observation_interest['source_stamp_ns']
    assert n._rgb_observation_interest['observation_interest_ready']
    settled(n, clock, phase='executing', observation_index=2)
    clock[0] += .2; feed()
    settled(n, clock, observation_index=2)
    for index in range(3):
        clock[0] += .2; feed()
        assert n._rgb_handoff_status['rgb_consistent_frames'] == index+1
    assert n._rgb_acquisition_active['reference'] is reference


def test_raw_19mm_support_error_rejects_before_rgb_or_three_dimensional_handoff(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    original = module.segment_with_support_fallback
    def shifted_plane(*args, **kwargs):
        result = original(*args, **kwargs)
        result.plane_base = result.plane_base + np.array([0., 0., 0., .019])
        return result
    monkeypatch.setattr(module, 'segment_with_support_fallback', shifted_plane)
    # Keep this test focused on the original cross-view plane audit: the live
    # independent fit/coverage have their own tests and are not relaxed here.
    monkeypatch.setattr(module, 'measured_support_coverage', lambda *args:dict(spatial_quadrants=4))
    settled(n, clock); clock[0] += .2; feed(target_depth=.38)
    assert n.tracker.anchor is None
    assert n._rgb_handoff_status['state'] == 'rejected'
    assert n._rgb_handoff_status['support_audit']['support_reference_max_distance_m'] == pytest.approx(.019)
    assert not n._rgb_observation_interest['observation_interest_ready']


@pytest.mark.parametrize('change', ['epoch', 'source', 'revoked', 'deadline'])
def test_reference_clears_on_actual_task_invalidations(monkeypatch, change):
    n, clock, feed, _, params, _ = frozen_node(monkeypatch)
    settled(n, clock)
    if change == 'epoch':
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
    elif change == 'source':
        params['/handeye']['new_mount'] = 'changed'
    elif change == 'revoked':
        settled(n, clock, phase='revoked')
    else:
        clock[0] += 101
    clock[0] += .2; feed()
    assert n._rgb_acquisition_active is None
    assert n.tracker.anchor is None


def test_gateway_revocation_during_refine_cannot_commit(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    settled(n, clock)
    for _ in range(2):
        clock[0] += .2; feed(target_depth=.38)
    original = module.refine_candidate
    def revoke(*args):
        result = original(*args)
        settled(n, clock, phase='revoked')
        return result
    monkeypatch.setattr(module, 'refine_candidate', revoke)
    clock[0] += .2; feed(target_depth=.38)
    assert n.tracker.anchor is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_task_deadline_expiring_during_refine_cannot_commit(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    settled(n, clock)
    for _ in range(2):
        clock[0] += .2; feed(target_depth=.38)
    original = module.refine_candidate
    def expire(*args):
        result = original(*args)
        n._rgb_acquisition_active['deadline_monotonic'] = clock[0]-.001
        return result
    monkeypatch.setattr(module, 'refine_candidate', expire)
    clock[0] += .2; feed(target_depth=.38)
    assert n.tracker.anchor is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_pose_movement_during_planning_retires_reference_without_new_center_acquisition(monkeypatch):
    n, clock, feed, _, _, _ = frozen_node(monkeypatch)
    clock[0] += .2
    feed(q=[.01, 0, 0, 0, 0, 0], target_depth=.38)
    assert n._rgb_acquisition_active is None and n.tracker.anchor is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


def test_cache_is_bounded_and_ambiguity_retires_preceding_freezable_sources(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for index in range(8):
        clock[0] = 10.+.6*index; feed()
        assert len(n._rgb_reference_cache) <= 2
    assert len(n._rgb_reference_cache) == 2
    original = module.propose_rgb_observation_regions
    monkeypatch.setattr(module, 'propose_rgb_observation_regions', lambda *args, **kwargs:
        replace(original(*args, **kwargs), selected_index=None, reason='ambiguous_rgb_regions'))
    clock[0] += .6; feed()
    assert len(n._rgb_reference_cache) == 0
