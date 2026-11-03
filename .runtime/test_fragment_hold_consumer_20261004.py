"""Offline producer/consumer contract checks; no ROS services or hardware I/O.

Synthetic RGB-D exercises the real producer segmenter/refiner and the emitted
ObjectPose/empty mask through the actual mode-aware remote callbacks. This is
not a replay of generation 8, whose original loss RGB-D was not recorded.
"""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from sensor_msgs.msg import JointState

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
import test_remote_modes as remote_tests
from test_locked_rgb_hold import (
    custom_frame, fragment_depth, multiple_fragment_depth, latest_status, locked,
    module as producer_module,
)


def producer_messages(monkeypatch, fragment_kind):
    node, clock, feed, joint, _params = locked(monkeypatch)
    good = node.pubs['object'].publish.call_args[0][0]
    anchor, last = node.tracker.anchor, node.tracker.last
    reference = node._locked_rgb_hold_reference
    last_good = node.last_good_source_stamp_ns
    fragment_count = 2 if fragment_kind == 2 else 1
    if fragment_kind in ('boundary', 'pending'):
        rgb = np.full((240, 320, 3), (60, 90, 190), np.uint8)
        rgb[106:134, 148:173] = 20
        depth = np.full((240, 320), .4, np.float32)
        depth[106:134, 148:173] = 0.
        if fragment_kind == 'boundary':
            depth[114:127, 163:174] = .35  # One native column beyond the RGB outline.
        else:
            depth[106:134, 148:173] = fragment_depth()
        source = (node.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                  node.bridge.cv2_to_imgmsg(depth, encoding='32FC1'))
        source_stamp = clock[0] - .1
        for message in source:
            message.header.stamp = producer_module.rospy.Time.from_sec(source_stamp)
            message.header.frame_id = node.camera_frame
        if fragment_kind == 'pending':
            # Actual gen10 span, with measured samples on each side; no pose
            # sample is invented inside the unqualified 200.04325 ms interval.
            source_ns = source[0].header.stamp.to_nsec()
            node._rgb_interest_feedback.clear()
            for offset in (-140279338, 59763912):
                ns = source_ns + offset
                message = JointState()
                message.header.stamp = producer_module.rospy.Time(ns // 10**9, ns % 10**9)
                message.header.frame_id = 'sdk_measured'
                message.name, message.position = list(producer_module.JOINT_NAMES), [0.] * 6
                node._rgb_interest_joint_cb(message)
        else:
            joint(source_stamp-.05)
            joint(source_stamp+.05)
        node.latest = source
        node.process(None)
    else:
        custom_frame(node, clock, joint, target_depth=(
            fragment_depth() if fragment_count == 1 else multiple_fragment_depth()))
    status = latest_status(node)
    if fragment_kind == 'pending':
        assert status['detail'] == 'source_qualification_pending'
        assert 'rgb_identity_hold' not in status
        qualification = status['source_qualification']
        assert qualification['state'] == 'pending_source_qualification'
        for key in ('source_qualified', 'identity_verified', 'reference_refreshed',
                    'metric_geometry_authority', 'motion_authorized'):
            assert qualification[key] is False
        assert qualification['feedback_rejection']['span_ns'] == 200043250
        assert qualification['feedback_rejection']['maximum_span_ns'] == 200000000
        assert node._locked_source_pending is not None
    else:
        hold = status['rgb_identity_hold']
        assert hold['state'] == 'holding_identity_only'
        assert hold['depth_gap_kind'] == ('rgb_boundary_consistent_shrinking_fragments'
            if fragment_kind == 'boundary' else 'unique_contained_shrinking_fragment'
            if fragment_count == 1 else 'disjoint_contained_shrinking_fragments')
        assert hold['fragment_count'] == fragment_count
        assert [row['measured_pixel_count'] for row in hold['fragment_evidence_list']] == (
            [143] if fragment_count == 1 else [110, 110])
        assert any(row['boundary_tolerance_used'] for row in hold['fragment_evidence_list']) == (fragment_kind == 'boundary')
        assert not hold['metric_geometry_authority'] and not hold['motion_authorized']
    assert not status['observation_interest']['observation_interest_ready']
    assert node.tracker.anchor is anchor and node.tracker.last is last
    assert not node.tracker.lost and node.first_loss is None
    assert node._locked_rgb_hold_reference is reference
    assert node.last_good_source_stamp_ns == last_good
    held = node.pubs['object'].publish.call_args[0][0]
    mask = node.pubs['object_mask'].publish.call_args[0][0]
    assert not held.detected and not np.any(node.bridge.imgmsg_to_cv2(mask))
    clock[0] += .2
    feed(target_depth=.35)
    recovered = node.pubs['object'].publish.call_args[0][0]
    assert recovered.detected and node.last_good_source_stamp_ns > last_good
    assert node.tracker.anchor is anchor and not node.tracker.lost
    if fragment_kind == 'pending':
        assert node._locked_source_pending is None
        assert latest_status(node)['source_qualification']['state'] == 'qualified_new_frame'
    return node, good, held, mask, recovered


@pytest.mark.parametrize('executing', [False, True])
@pytest.mark.parametrize('fragment_kind', [1, 2, 'boundary', 'pending'])
def test_producer_hold_invalidates_pending_authority_without_changing_frozen_execution(monkeypatch, executing, fragment_kind):
    producer, good, held, mask, recovered = producer_messages(monkeypatch, fragment_kind)
    remote, params = remote_tests.configured.__wrapped__(monkeypatch)
    remote_tests.select_unknown(params)
    assert remote.reset_mode_cb(None).success
    monkeypatch.setattr(remote_tests.module.rospy.Time, 'now',
                        staticmethod(lambda: remote_tests.module.rospy.Time(11)))
    remote.bridge = producer.bridge
    remote.object_cb(good)
    remote.streaming_enabled = True
    old_epoch = remote.target_instance_epoch
    old_ticket = SimpleNamespace(generation=remote._stream_generation, target_epoch=old_epoch)
    old_plan = SimpleNamespace(valid=True)
    old_legacy = SimpleNamespace(poses=[1])
    remote.latest_rich_plan, remote.latest_plan = old_plan, old_legacy
    remote._latest_geometry_estimate = object()
    remote.latest_target_cloud_base_xyz = np.ones((3, 3))
    remote._near_field_fused_surface = object()
    remote._latest_preview_proposal = object()
    remote._execution_authority_ticket = old_ticket
    remote.robot_execution_active = executing
    controller = SimpleNamespace(has_execution=executing, clear_execution=Mock())
    remote.execution_plan_controller = controller
    rich_count, legacy_count = len(remote.rich_plan_pub.messages), len(remote.plan_pub.messages)

    remote.mask_cb(mask)
    remote.object_cb(held)

    assert not remote.latest_object.detected
    assert remote.target_instance_epoch == old_epoch + 1
    assert remote._latest_geometry_estimate is None
    assert remote.latest_target_cloud_base_xyz is None
    assert remote._near_field_fused_surface is None
    assert remote._latest_preview_proposal is None
    assert not remote.latest_object_geometry.valid
    assert remote.latest_object_geometry.failure_reason.startswith('TARGET_LOST:')
    with pytest.raises(remote_tests.base.StreamResultCancelled, match='TARGET_EPOCH_STALE'):
        remote._require_stream_ticket_current(old_ticket)
    if executing:
        # Existing occlusion contract retains an already executing frozen plan;
        # the identity-only producer response supplies no replacement authority.
        assert remote.latest_rich_plan is old_plan and remote.latest_plan is old_legacy
        assert remote._execution_authority_ticket is old_ticket
        assert len(remote.rich_plan_pub.messages) == rich_count
        assert len(remote.plan_pub.messages) == legacy_count
        controller.clear_execution.assert_not_called()
    else:
        assert remote.latest_rich_plan is None and remote.latest_plan is None
        assert remote._execution_authority_ticket is None
        assert not remote.rich_plan_pub.messages[-1].valid
        assert not remote.plan_pub.messages[-1].poses
        controller.clear_execution.assert_called_once_with()

    remote.object_cb(recovered)
    assert remote.latest_object.detected and remote._latest_geometry_estimate is None
    assert remote._stream_ticket_cancellation_code_locked(old_ticket) == 'TARGET_EPOCH_STALE'
    if executing:
        assert remote.latest_rich_plan is old_plan and remote.latest_plan is old_legacy
    else:
        assert remote.latest_rich_plan is None and remote.latest_plan is None


def test_changed_raw_candidate_then_refinement_rejection_does_not_refresh_hold_reference(monkeypatch):
    node, clock, _feed, joint, _params = locked(monkeypatch)
    reference, last = node._locked_rgb_hold_reference, node.tracker.last
    last_good = node.last_good_source_stamp_ns
    original_refine = producer_module.refine_candidate

    def reject_refinement(*args, **kwargs):
        raise ValueError('rgb_instance_reaches_search_boundary')

    monkeypatch.setattr(producer_module, 'refine_candidate', reject_refinement)
    custom_frame(node, clock, joint, target_depth=.349)
    assert not node.tracker.lost and node.tracker.last is not last
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert node.last_good_source_stamp_ns == last_good
    assert node._locked_rgb_hold_reference is reference
    monkeypatch.setattr(producer_module, 'refine_candidate', original_refine)
    clock[0] += .1
    custom_frame(node, clock, joint, target_depth=fragment_depth())
    # Conservative existing behavior: a changed raw tracking candidate cannot
    # silently inherit the last published-good candidate's identity cache.
    assert node.tracker.lost and node._locked_rgb_hold_reference is None
    assert node.last_good_source_stamp_ns == last_good
    assert not node.pubs['object'].publish.call_args[0][0].detected
    assert latest_status(node)['rgb_identity_hold']['reason'] == 'locked_rgb_hold_reference_changed'
