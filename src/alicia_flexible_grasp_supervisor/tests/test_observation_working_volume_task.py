"""Run task observation acceptance with fake TF/messages; never command ROS."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

import test_grasp_task_sequence as sequence
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id

task = sequence.grasp_task_node


@pytest.fixture
def context(monkeypatch):
    fixture = sequence.GraspTaskSequenceTest()
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    plan = fixture._rich_plan(stamp_sec=9.)
    plan.diagnostic = task._FAR_FIELD_OBSERVATION_PLAN
    plan.plan_id = compute_plan_id(plan)
    node.active = True
    node._freeze_execution_plan(plan)
    node.latest_obj = fixture._object_at(.4, stamp_sec=9.9)
    node.latest_obj.confidence = .9
    node.latest_obj_time = node.latest_obj.header.stamp
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: task.rospy.Time(10)))
    monkeypatch.setattr(task.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(task.rospy, 'sleep', lambda duration: setattr(node, 'active', False))
    camera = dict(frame_id='camera_link', fx=400., fy=400., cx=320., cy=240.,
                  width=640, height=480, intrinsics_source='realsense_active_profile')
    params = {'/camera': camera}
    monkeypatch.setattr(task.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.header.stamp = node.latest_obj.header.stamp
    transform.child_frame_id = 'camera_link'
    transform.transform.translation.x = .4 - .2118
    transform.transform.translation.z = .2
    transform.transform.rotation.w = 1.
    calls = []
    def lookup(parent, child, stamp, timeout):
        calls.append((parent, child, stamp.to_nsec()))
        return deepcopy(transform)
    node.tf_buffer = NS(lookup_transform=lookup)
    node._current_camera_pose_base = mock.Mock(side_effect=AssertionError('latest TF must not be used'))
    node.set_state = mock.Mock()
    cfg = dict(observation_acceptance_policy='rgbd_working_volume_v1',
        observation_camera_target_range_check_enabled=True,
        observation_camera_target_retreat_correction_enabled=True,
        observation_camera_target_min_distance_m=.22,
        observation_camera_target_nominal_distance_m=.26,
        observation_camera_target_max_distance_m=.30,
        target_observation_validity_sec=1., target_max_drift_m=.02,
        min_object_confidence=.5)
    return node, plan, cfg, transform, params, calls


def test_actual_2118_mm_view_passes_at_exact_image_tf_without_correction(context):
    node, plan, cfg, _, _, calls = context
    result = node._observation_camera_target_range_result(plan, cfg)
    assert result.ok, result.reason
    assert calls == [('base_link', 'camera_link', 9900000000)]
    audit = node._last_observation_camera_range_evidence
    assert audit['target_center_radial_distance_m'] == pytest.approx(.2118)
    assert audit['minimum_depth_m'] == pytest.approx(.1718)
    assert audit['maximum_depth_m'] == pytest.approx(.2518)
    assert audit['source_stamp_ns'] == audit['camera_transform_stamp_ns'] == 9900000000
    assert audit['requires_independent_information_quality_check']
    assert audit['camera_intrinsics_source'] == 'realsense_active_profile'
    assert node._current_camera_pose_base.call_count == 0
    motion = mock.Mock(side_effect=AssertionError('range acceptance never commands motion'))
    for code in ('OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE', 'OBSERVATION_CAMERA_WORKING_VOLUME_INVALID'):
        assert node._maybe_execute_observation_camera_retreat(plan, cfg,
            task.PlanValidationResult(False, code, 'outside'), motion, motion) is False
    motion.assert_not_called()
    node.set_state.assert_not_called()


def test_source_time_snapshot_is_preserved_through_fresh_wait(context):
    node, plan, cfg, _, _, calls = context
    result = node._wait_for_fresh_observation_camera_target_range(plan, cfg, 9800000000)
    assert result.ok, result.reason
    assert calls == [('base_link', 'camera_link', 9900000000)]
    assert node._last_observation_camera_range_evidence['image_source_stamp_ns'] == 9900000000


def test_legacy_check_disabled_does_not_bypass_explicit_working_volume_policy(context):
    node, plan, cfg, _, _, _ = context
    cfg['observation_camera_target_range_check_enabled'] = False
    node.latest_obj = None
    assert not node._current_observation_view_reusable(plan, cfg)
    assert not node._wait_for_fresh_observation_camera_target_range(plan, cfg, 9800000000).ok


@pytest.mark.parametrize('fault', ['stale', 'future', 'missing_stamp', 'undetected', 'low_confidence',
    'nan_confidence', 'mismatched_track', 'drift', 'source_pose_mismatch', 'missing_tf', 'lookup_error',
    'tf_stamp', 'tf_static_zero', 'tf_parent', 'tf_child', 'tf_nan', 'tf_quaternion',
    'camera_frame', 'camera_intrinsics', 'camera_dimensions', 'optical_convention', 'relaxed_limits'])
def test_bad_live_evidence_rejects_reuse_and_range(context, fault):
    node, plan, cfg, transform, params, _ = context
    if fault == 'stale': node.latest_obj.header.stamp = task.rospy.Time(8)
    elif fault == 'future': node.latest_obj.header.stamp = task.rospy.Time(11)
    elif fault == 'missing_stamp':
        node.latest_obj.header.stamp = task.rospy.Time(0)
        node.latest_obj.pose_base.header.stamp = task.rospy.Time(0)
    elif fault == 'undetected': node.latest_obj.detected = False
    elif fault == 'low_confidence': node.latest_obj.confidence = .49
    elif fault == 'nan_confidence': node.latest_obj.confidence = float('nan')
    elif fault == 'mismatched_track':
        node.latest_target_geometry = deepcopy(plan.object_geometry)
        node.latest_target_geometry.target_track_id = 'g0-t2'
    elif fault == 'drift': node.latest_obj.pose_base.pose.position.x += .03
    elif fault == 'source_pose_mismatch': node.latest_obj.pose_base.header.stamp = task.rospy.Time(9)
    elif fault == 'missing_tf': node.tf_buffer = None
    elif fault == 'lookup_error': node.tf_buffer.lookup_transform = mock.Mock(side_effect=RuntimeError('missing'))
    elif fault == 'tf_stamp': transform.header.stamp = task.rospy.Time(9)
    elif fault == 'tf_static_zero': transform.header.stamp = task.rospy.Time(0)
    elif fault == 'tf_parent': transform.header.frame_id = 'tool0'
    elif fault == 'tf_child': transform.child_frame_id = 'camera_color_optical_frame'
    elif fault == 'tf_nan': transform.transform.translation.x = float('nan')
    elif fault == 'tf_quaternion': transform.transform.rotation.w = 0.
    elif fault == 'camera_frame': params['/camera']['frame_id'] = 'wrong_frame'
    elif fault == 'camera_intrinsics': del params['/camera']['fx']
    elif fault == 'camera_dimensions': params['/camera']['width'] = 640.
    elif fault == 'optical_convention': params['/perception/projection_frame_convention'] = 'opencv_optical'
    elif fault == 'relaxed_limits': params['/grasp_6d/remote/camera_visibility_min_depth_m'] = .06
    result = node._observation_camera_target_range_result(plan, cfg)
    assert not result.ok, fault
    assert not node._current_observation_view_reusable(plan, cfg), fault
    node._current_camera_pose_base.assert_not_called()


@pytest.mark.parametrize('translation,expected', [(.34, 'depth range'), (.1882, 'image margin')])
def test_whole_frozen_obb_must_fit_not_just_live_target_center(context, translation, expected):
    node, plan, cfg, transform, _, _ = context
    transform.transform.translation.x = translation
    if expected == 'image margin': transform.transform.translation.y = -.14
    result = node._observation_camera_target_range_result(plan, cfg)
    assert not result.ok and expected in result.reason
    assert result.code == 'OBSERVATION_CAMERA_WORKING_VOLUME_INVALID'


def test_measured_surface_drift_does_not_translate_frozen_obb(context):
    node, plan, cfg, _, _, _ = context
    node.latest_obj.pose_base.pose.position.x += .01
    result = node._observation_camera_target_range_result(plan, cfg)
    assert result.ok, result.reason
    audit = node._last_observation_camera_range_evidence
    assert audit['measured_target_drift_m'] == pytest.approx(.01)
    assert audit['minimum_depth_m'] == pytest.approx(.1718)


def test_old_image_with_new_receipt_does_not_meet_post_arrival_gate(context):
    node, plan, cfg, _, _, calls = context
    node.latest_obj_time = task.rospy.Time(10)
    result = node._wait_for_fresh_observation_camera_target_range(plan, cfg, 9950000000)
    assert not result.ok and 'OBSERVATION_CAMERA_SOURCE_OLD' in result.reason
    assert calls == []


def test_epoch_tf_rounding_preserves_actual_returned_stamp(context, monkeypatch):
    node, plan, cfg, transform, _, calls = context
    source_ns = 1785344323552103281
    source = task.rospy.Time(source_ns // 1000000000, source_ns % 1000000000)
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: task.rospy.Time.from_sec(source.to_sec()+.1)))
    node.latest_obj.header.stamp = source
    node.latest_obj.pose_base.header.stamp = source
    transform.header.stamp = task.rospy.Time(source_ns // 1000000000, source_ns % 1000000000 + 100)
    result = node._observation_camera_target_range_result(plan, cfg)
    assert result.ok, result.reason
    audit = node._last_observation_camera_range_evidence
    assert calls[-1][2] == source_ns
    assert audit['source_stamp_ns'] == source_ns
    assert audit['camera_transform_stamp_ns'] == source_ns + 100
    assert audit['camera_transform_stamp_delta_ns'] == 100
    transform.header.stamp = task.rospy.Time(source_ns // 1000000000, source_ns % 1000000000 + 1000)
    assert not node._observation_camera_target_range_result(plan, cfg).ok


def test_legacy_policy_retains_its_original_radial_interval(context):
    node, plan, cfg, _, _, calls = context
    cfg.pop('observation_acceptance_policy')
    pose = sequence.GraspTaskSequenceTest()._pose(.4-.2118, 0., .2)
    node._current_camera_pose_base = mock.Mock(return_value=pose)
    result = node._observation_camera_target_range_result(plan, cfg)
    assert not result.ok and result.code == 'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE'
    assert node._last_observation_camera_range_evidence['distance_m'] == pytest.approx(.2118)
    assert calls == []
