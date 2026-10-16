"""Correction targets retain selected-distance provenance and measured margins."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped

import test_grasp_task_sequence as sequence

task = sequence.grasp_task_node
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id


@pytest.fixture
def context(monkeypatch):
    fixture = sequence.GraspTaskSequenceTest()
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    plan = fixture._rich_plan(xs=[.15, .2, .3, .4], stamp_sec=9.)
    plan.diagnostic = task._FAR_FIELD_OBSERVATION_PLAN
    plan.plan_id = compute_plan_id(plan)
    node.active = True
    node._freeze_execution_plan(plan)
    node.latest_obj = fixture._object_at(.4, stamp_sec=9.9)
    node.latest_obj_time = node.latest_obj.header.stamp
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: task.rospy.Time(10)))
    params = {}
    monkeypatch.setattr(task.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    transform = TransformStamped()
    transform.header.frame_id = 'tool0'
    transform.header.stamp = plan.header.stamp
    transform.child_frame_id = 'camera_link'
    transform.transform.translation.x = .03
    transform.transform.rotation.w = 1.
    lookups = []
    def lookup(parent, child, stamp, timeout):
        lookups.append((parent, child, stamp.to_nsec()))
        return deepcopy(transform)
    node.tf_buffer = NS(lookup_transform=lookup)
    cfg = dict(observation_camera_target_retreat_correction_enabled=True,
        observation_camera_target_min_distance_m=.22,
        observation_camera_target_nominal_distance_m=.26,
        observation_camera_target_max_distance_m=.30)
    node.set_state = mock.Mock()
    node._current_tool_pose_base = lambda: fixture._pose(1., 2., 3.)
    node._last_observation_camera_range_evidence = dict(
        distance_m=.198779, camera_position_m=[.198779, 0., .2],
        target_position_m=[0., 0., .2])
    node._last_measured_endpoint_sample = dict(plan_id=plan.plan_id,
        plan_phase=task._FAR_FIELD_OBSERVATION_PLAN,
        position_error_vector_m=[-.01077, 0., 0.], orientation_error_rad=.02)
    node._plan_and_execute_pose = mock.Mock(return_value=True)
    node._validate_measured_observation_envelope = mock.Mock(return_value=True)
    return node, plan, cfg, transform, params, lookups


def run_correction(node, plan, cfg):
    preflight = mock.Mock(return_value=NS(success=True, message='strict path'))
    result = node._maybe_execute_observation_camera_retreat(plan, cfg,
        task.PlanValidationResult(False, 'OBSERVATION_CAMERA_TARGET_OUT_OF_RANGE', 'too close'),
        preflight, object())
    return result, preflight


@pytest.mark.parametrize('model', ['carton_segment:range', 'unknown_tabletop'])
def test_two_stage_and_unknown_use_selected_range_not_nominal(context, model):
    node, plan, cfg, _, _, lookups = context
    plan.model_choice = model
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    node._last_measured_endpoint_sample['plan_id'] = plan.plan_id
    result, preflight = run_correction(node, plan, cfg)
    assert result is True
    assert lookups == [('tool0', 'camera_link', 9000000000)]
    assert preflight.call_count == node._plan_and_execute_pose.call_count == 1
    correction, execute = preflight.call_args[0]
    assert execute is False
    margin = .01077 + 2. * .03 * np.sin(.02 / 2.)
    assert correction.pose.position.x == pytest.approx(1. + .22 + margin - .198779 + .01077)
    assert correction.pose.position.x < 1.06  # No unconditional return to the .26 preference.
    assert correction.pose.orientation == node._current_tool_pose_base().pose.orientation
    audit = node._last_observation_camera_correction_audit
    assert audit['selected_camera_target_distance_m'] == pytest.approx(.22)
    assert audit['correction_target_camera_distance_m'] == pytest.approx(.22 + margin)
    assert audit['range_margin_is_future_error_bound'] is False
    assert audit['post_move_measurement_required'] is True
    assert audit['correction_target_reference'] == audit['live_reference']
    assert audit['configured_nominal_camera_target_distance_m'] == .26
    assert audit['planned_reference'] == 'frozen_obb_center'
    assert audit['live_reference'] == ('visible_surface_median' if model == 'unknown_tabletop'
                                      else 'fresh_object_pose_base')
    assert audit['plan_id'] == plan.plan_id
    assert node._validate_measured_observation_envelope.call_count == 1
    assert node._last_observation_camera_range_evidence['distance_m'] < cfg['observation_camera_target_min_distance_m']


def test_selected_range_composes_observation_rotation_with_handeye_offset(context):
    node, plan, cfg, transform, _, _ = context
    plan.poses[0].orientation.z = np.sin(np.pi/4.)
    plan.poses[0].orientation.w = np.cos(np.pi/4.)
    plan.poses[0].position.x = .4
    plan.poses[0].position.y = -.25
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    selected, audit = node._bound_observation_camera_distance(plan, cfg)
    assert selected == pytest.approx(.22)
    assert audit['planned_camera_position_m'] == pytest.approx([.4, -.22, .2])
    assert transform.transform.translation.x == .03


@pytest.mark.parametrize('fault', ['unfrozen', 'revoked', 'mutated', 'contact_phase',
    'missing_tf', 'lookup_failed', 'tf_parent', 'tf_child', 'tf_stamp', 'tf_nan',
    'tf_zero_quaternion', 'wrong_base', 'wrong_tool', 'bad_timeout', 'bad_config',
    'inverted_band', 'out_of_band', 'current_frame'])
def test_missing_or_mismatched_reference_fails_before_any_preflight(context, fault):
    node, plan, cfg, transform, params, _ = context
    if fault == 'unfrozen': node._bound_execution_plan = None
    elif fault == 'revoked': node._execution_authority_revoked = True
    elif fault == 'mutated': plan.poses[0].position.x += .001
    elif fault == 'contact_phase':
        plan.diagnostic = task._CONTACT_EXECUTION_PLAN
        plan.plan_id = compute_plan_id(plan)
        node._freeze_execution_plan(plan)
        node._last_measured_endpoint_sample['plan_id'] = plan.plan_id
    elif fault == 'missing_tf': node.tf_buffer = None
    elif fault == 'lookup_failed': node.tf_buffer.lookup_transform = mock.Mock(side_effect=RuntimeError('missing'))
    elif fault == 'tf_parent': transform.header.frame_id = 'base_link'
    elif fault == 'tf_child': transform.child_frame_id = 'camera_optical'
    elif fault == 'tf_stamp': transform.header.stamp = task.rospy.Time(8)
    elif fault == 'tf_nan': transform.transform.translation.x = float('nan')
    elif fault == 'tf_zero_quaternion': transform.transform.rotation.w = 0.
    elif fault == 'wrong_base': params['/handeye/base_frame'] = 'world'
    elif fault == 'wrong_tool': params['/handeye/parent_frame'] = 'Link6'
    elif fault == 'bad_timeout': params['/handeye/tf_timeout_sec'] = float('nan')
    elif fault == 'bad_config': cfg['observation_camera_target_min_distance_m'] = float('nan')
    elif fault == 'inverted_band': cfg['observation_camera_target_max_distance_m'] = .21
    elif fault == 'out_of_band': transform.transform.translation.x = -.10
    elif fault == 'current_frame':
        actual = node._current_tool_pose_base()
        actual.header.frame_id = 'world'
        node._current_tool_pose_base = lambda: actual
    result, preflight = run_correction(node, plan, cfg)
    assert result is None
    preflight.assert_not_called()
    node._plan_and_execute_pose.assert_not_called()
    assert 'CORRECTION_' in node.set_state.call_args[0][1]


def test_finite_static_tf_and_only_numeric_band_boundary_are_admitted(context):
    node, plan, cfg, transform, _, _ = context
    transform.header.stamp = task.rospy.Time(0)
    transform.transform.translation.x += 5e-10
    selected, audit = node._bound_observation_camera_distance(plan, cfg)
    assert selected == cfg['observation_camera_target_min_distance_m']
    assert audit['handeye_source_stamp_ns'] == 0
    transform.transform.translation.x += 1e-7
    with pytest.raises(ValueError, match='outside'):
        node._bound_observation_camera_distance(plan, cfg)


def test_controller_failure_still_forbids_new_correction(context):
    node, plan, cfg, _, _, lookups = context
    node._last_observation_execution_failure = dict(plan_id=plan.plan_id, stage_label='observation')
    result, preflight = run_correction(node, plan, cfg)
    assert result is None and not lookups
    preflight.assert_not_called()
    node._plan_and_execute_pose.assert_not_called()


@pytest.mark.parametrize('delta_ns,accepted', [
    (1, True), (-47, True), (-114, True), (108, True),
    (-121, True), (121, True), (-122, False), (122, False),
    (-1000000, False), (33000000, False)])
def test_epoch_tf_quantization_does_not_become_a_stale_frame_tolerance(context, monkeypatch, delta_ns, accepted):
    node, plan, cfg, transform, _, _ = context
    # Exact source of the gen24 failure; TF returned source+1 ns on hardware.
    source_ns = 1790766117378127574
    source = task.rospy.Time(source_ns // 1000000000, source_ns % 1000000000)
    plan.header.stamp = source
    plan.object_geometry.header.stamp = source
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    now = task.rospy.Time(source.secs + 1, source.nsecs)
    node.latest_obj.header.stamp = now
    node.latest_obj.pose_base.header.stamp = now
    node.latest_obj_time = now
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(lambda: now))
    actual_ns = source_ns + delta_ns
    transform.header.stamp = task.rospy.Time(actual_ns // 1000000000, actual_ns % 1000000000)
    if accepted:
        distance, audit = node._bound_observation_camera_distance(plan, cfg)
        assert distance == pytest.approx(.22)
        assert audit['plan_source_stamp_ns'] == source_ns
        assert audit['handeye_source_stamp_delta_ns'] == delta_ns
        assert audit['handeye_stamp_quantization_bound_ns'] == 121
    else:
        with pytest.raises(ValueError, match='timestamp mismatch'):
            node._bound_observation_camera_distance(plan, cfg)
    # Source/model geometry and all pose checks are still read-only here.
    node._plan_and_execute_pose.assert_not_called()


@pytest.mark.parametrize('selected,current,orientation,error', [
    (.22, .19, 0., [0., 0., 0.]),
    (.22, .19, .02, [-.01077, 0., 0.]),
    (.30, .33, .02, [-.01077, 0., 0.]),
])
def test_live_correction_uses_interior_target_in_both_directions(
        context, selected, current, orientation, error):
    node, plan, cfg, _, _, _ = context
    plan.poses[0].position.x = .4 - .03 - selected
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    node._last_measured_endpoint_sample.update(plan_id=plan.plan_id,
        position_error_vector_m=error, orientation_error_rad=orientation)
    node._last_observation_camera_range_evidence.update(distance_m=current,
        camera_position_m=[current, 0., .2])
    result, preflight = run_correction(node, plan, cfg)
    assert result is True and preflight.call_count == 1
    audit = node._last_observation_camera_correction_audit
    margin = np.linalg.norm(error) + 2. * .03 * np.sin(orientation / 2.)
    expected = min(.30 - margin, max(.22 + margin, selected))
    assert audit['selected_camera_target_distance_m'] == pytest.approx(selected)
    assert audit['correction_target_camera_distance_m'] == pytest.approx(expected)
    assert preflight.call_args[0][0].pose.position.x == pytest.approx(1. + expected - current - error[0])
    assert audit['correction_direction'] == ('retreat' if current < .22 else 'approach')
    assert audit['post_move_measurement_required'] is True
    assert node._plan_and_execute_pose.call_count == 1
    assert node._validate_measured_observation_envelope.call_count == 1


@pytest.mark.parametrize('model', ['unknown_tabletop', 'carton_segment:range'])
@pytest.mark.parametrize('fault', ['missing_rotation', 'negative_rotation',
    'nonfinite_rotation', 'malformed_position', 'wrong_endpoint_plan',
    'wrong_endpoint_phase', 'excessive_margin', 'negative_frozen_mad'])
def test_invalid_margin_evidence_never_reaches_preflight(context, model, fault):
    node, plan, cfg, _, _, _ = context
    plan.model_choice = model
    if fault == 'negative_frozen_mad':
        plan.object_geometry.depth_mad_m = -.001
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    endpoint = node._last_measured_endpoint_sample
    endpoint['plan_id'] = plan.plan_id
    if fault == 'missing_rotation': del endpoint['orientation_error_rad']
    elif fault == 'negative_rotation': endpoint['orientation_error_rad'] = -.001
    elif fault == 'nonfinite_rotation': endpoint['orientation_error_rad'] = float('nan')
    elif fault == 'malformed_position': endpoint['position_error_vector_m'] = [0., 0.]
    elif fault == 'wrong_endpoint_plan': endpoint['plan_id'] = 'unrelated'
    elif fault == 'wrong_endpoint_phase': endpoint['plan_phase'] = task._CONTACT_EXECUTION_PLAN
    elif fault == 'excessive_margin': endpoint['position_error_vector_m'] = [.10, 0., 0.]
    result, preflight = run_correction(node, plan, cfg)
    assert result is None
    preflight.assert_not_called()
    node._plan_and_execute_pose.assert_not_called()
    assert 'CORRECTION_' in node.set_state.call_args[0][1]


def test_nonzero_frozen_depth_dispersion_is_used_and_bound(context):
    node, plan, cfg, _, _, _ = context
    plan.object_geometry.depth_mad_m = .002
    plan.plan_id = compute_plan_id(plan)
    node._freeze_execution_plan(plan)
    node._last_measured_endpoint_sample['plan_id'] = plan.plan_id
    result, _ = run_correction(node, plan, cfg)
    assert result is True
    audit = node._last_observation_camera_correction_audit
    assert audit['range_margin_frozen_depth_mad_m'] == pytest.approx(.002)
    assert audit['correction_target_camera_distance_m'] == pytest.approx(
        .22 + .01077 + 2. * .03 * np.sin(.02 / 2.) + .002)
    # A later modification of that source is still rejected by the bound-plan contract.
    plan.object_geometry.depth_mad_m = .003
    node._plan_and_execute_pose.reset_mock()
    result, preflight = run_correction(node, plan, cfg)
    assert result is None
    preflight.assert_not_called()
    node._plan_and_execute_pose.assert_not_called()


def test_one_successful_correction_does_not_override_failed_post_move_measurement():
    # Reuse the complete sequence fixture: it asserts FAILED, no near-field
    # request, and no contact stage. A successful correction merely requires a
    # second fresh measurement; it cannot change the final out-of-range result.
    with mock.patch.object(task.GraspTaskNode, '_maybe_execute_observation_camera_retreat',
            return_value=True) as correction:
        sequence.GraspTaskSequenceTest().test_far_field_range_failure_prevents_near_field_request()
    assert correction.call_count == 1
