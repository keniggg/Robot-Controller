"""Post-arrival image handoff, with no ROS master or motion clients."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import test_grasp_task_sequence as sequence

module = sequence.grasp_task_node


def setup_node(monkeypatch, now=12.):
    helper = sequence.GraspTaskSequenceTest()
    node = module.GraspTaskNode.__new__(module.GraspTaskNode)
    node.active = True
    node.latest_obj = helper._object(stamp_sec=now-.1)
    node.latest_obj_time = node.latest_obj.header.stamp
    node._latest_obj_received_monotonic = now
    node._latest_obj_received_source_ns = node.latest_obj.header.stamp.to_nsec()
    node._observed_target_matches_plan = lambda *args: True
    clock = [now]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: module.rospy.Time.from_sec(clock[0]))
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(module.rospy, 'sleep', lambda duration: clock.__setitem__(0, clock[0]+duration))
    monkeypatch.setattr(module.rospy, 'get_param', lambda _name, default=None: default)
    return node, helper, clock


def test_waits_for_post_arrival_source_then_publishes_exact_anchor_with_original_deadline(monkeypatch):
    node, helper, clock = setup_node(monkeypatch)
    plan = helper._rich_plan(stamp_sec=10.)
    # Recorded failure: previous frame was complete but preceded the final
    # compensation endpoint. Repeated polling cannot make it a new frame.
    old = deepcopy(node.latest_obj)
    polls = []
    def deliver(duration):
        clock[0] += duration
        polls.append(clock[0])
        if clock[0] >= 13.:
            node.latest_obj = helper._object(stamp_sec=12.1)
            node.latest_obj.header.stamp = module.rospy.Time(12, 100_000_000)
            node.latest_obj_time = node.latest_obj.header.stamp
            node._latest_obj_received_monotonic = clock[0]
            node._latest_obj_received_source_ns = node.latest_obj.header.stamp.to_nsec()
    monkeypatch.setattr(module.rospy, 'sleep', deliver)
    selected = node._wait_for_final_refine_reference(plan, 12_000_000_000, 32., .05)
    assert selected.header.stamp.to_nsec() == 12_100_000_000
    assert old.header.stamp.to_nsec() == 11_900_000_000
    assert polls and clock[0] < 32.
    phases = []
    node.near_field_phase_pub = SimpleNamespace(publish=phases.append)
    # A new callback may replace latest_obj between selection and publication;
    # the handoff must still bind the exact validated reference.
    node.latest_obj = helper._object(stamp_sec=13.01)
    node._set_near_field_active(True, force=True, budget_sec=20.,
        absolute_deadline_sec=32., reference_plan=plan, reference_object=selected)
    assert len(phases) == 1
    assert phases[0].reference_source_stamp.to_nsec() == 12_100_000_000
    assert phases[0].reference_center_valid
    assert phases[0].deadline.to_nsec() == 32_000_000_000
    assert phases[0].header.stamp.to_sec() > 12.


@pytest.mark.parametrize('fault', ['pre_arrival', 'future', 'latency', 'completion_age',
                                   'missing_receipt', 'receipt_source_mismatch',
                                   'identity', 'undetected', 'inactive'])
def test_bad_reference_never_starts_new_phase(monkeypatch, fault):
    node, helper, clock = setup_node(monkeypatch)
    if fault == 'pre_arrival': node.latest_obj = helper._object(stamp_sec=10.)
    if fault == 'future': node.latest_obj = helper._object(stamp_sec=14.)
    if fault == 'latency': node.latest_obj = helper._object(stamp_sec=10.5)
    if fault == 'completion_age': node._latest_obj_received_monotonic = 11.
    if fault == 'missing_receipt': del node._latest_obj_received_monotonic
    if fault == 'receipt_source_mismatch': node._latest_obj_received_source_ns += 1
    if fault == 'identity': node._observed_target_matches_plan = lambda *args: False
    if fault == 'undetected': node.latest_obj.detected = False
    if fault == 'inactive': node.active = False
    assert node._wait_for_final_refine_reference(None, 10_000_000_000, 12.3, .05) is None
    assert clock[0] <= 12.3+1e-9


def test_duplicate_detection_does_not_renew_completion_freshness(monkeypatch):
    node, _, clock = setup_node(monkeypatch)
    node._latest_obj_received_monotonic = 11.
    node.obj_cb(deepcopy(node.latest_obj))
    assert node._latest_obj_received_monotonic == 11.
    assert node._wait_for_final_refine_reference(None, 10_000_000_000, 12.3, .05) is None


def test_reference_uses_deployed_receiver_latency_without_renewing_source(monkeypatch):
    node, helper, clock = setup_node(monkeypatch)
    node.latest_obj = helper._object(stamp_sec=9.4)
    node._latest_obj_received_source_ns = node.latest_obj.header.stamp.to_nsec()
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None:
        5. if name == '/grasp_6d/remote/planning_snapshot_max_inference_latency_sec' else default)
    selected = node._wait_for_final_refine_reference(None, 9_000_000_000, 12.3, .05)
    assert selected is not None
    assert selected.header.stamp.to_nsec() == node.latest_obj.header.stamp.to_nsec()
    assert clock[0] == 12.
    # The same new message remains unusable if it predates physical arrival.
    assert node._wait_for_final_refine_reference(None, 10_000_000_000, 12.3, .05) is None


@pytest.mark.parametrize('limit', [0., -1., float('nan'), float('inf'), None, 'invalid'])
def test_invalid_receiver_latency_cannot_authorize_reference(monkeypatch, limit):
    node, _, _ = setup_node(monkeypatch)
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None:
        limit if name == '/grasp_6d/remote/planning_snapshot_max_inference_latency_sec' else default)
    assert node._wait_for_final_refine_reference(None, 10_000_000_000, 12.3, .05) is None


def test_missing_post_arrival_reference_exhausts_original_final_deadline_without_inference(monkeypatch):
    node, helper, clock = setup_node(monkeypatch)
    node.set_state = Mock()
    node._set_near_field_active = Mock()
    node._request_near_field_preview_stream = Mock()
    result = node._maybe_final_refine_grasp6d_plan({
        'near_field_strategy': 'single_snapshot_direct',
        'final_visual_refine_enabled': True,
        'final_visual_refine_timeout_sec': .3,
    }, {}, helper._rich_plan(stamp_sec=10.))
    assert result is None and clock[0] <= 12.3+1e-9
    node._set_near_field_active.assert_not_called()
    node._request_near_field_preview_stream.assert_not_called()
    assert 'FINAL_REFINE_REFERENCE_UNAVAILABLE' in node.set_state.call_args.args[1]


@pytest.mark.parametrize('delay_stage', ['none', 'service_wait', 'service_call'])
def test_strict_sequence_receives_remaining_original_phase_deadline(monkeypatch, delay_stage):
    node, helper, clock = setup_node(monkeypatch)
    node._near_field_phase_deadline_sec = 12.5
    waits, requests = [], []

    def wait_for_service(_name, timeout):
        waits.append(timeout)
        if delay_stage == 'service_wait':
            clock[0] = 12.6

    def invoke(*args):
        requests.append(args)
        if delay_stage == 'service_call':
            clock[0] = 12.6
        return SimpleNamespace(success=True, message='sequence verified')

    monkeypatch.setattr(module.rospy, 'wait_for_service', wait_for_service)
    monkeypatch.setattr(module.rospy, 'ServiceProxy', lambda *_args: invoke)
    result = node._check_final_refine_sequence(helper._rich_plan(stamp_sec=10.), {
        'near_field_strategy': 'single_snapshot_direct',
        'final_visual_refine_service_timeout_sec': 3.,
    })
    assert waits == [.5]
    if delay_stage == 'service_wait':
        assert requests == []
    else:
        assert len(requests) == 1
        assert requests[0][3].to_nsec() == 12_500_000_000
    assert result.ok == (delay_stage == 'none')
    if not result.ok:
        assert result.code == 'FINAL_REFINE_TIMEOUT'


@pytest.mark.parametrize('delay_stage', ['none', 'sequence', 'rebind_lock', 'simulation',
                                       'ros_clock', 'monotonic_only'])
def test_late_final_checks_cannot_authorize_refined_plan(monkeypatch, delay_stage):
    node, helper, clock = setup_node(monkeypatch)
    ros_override = [None]
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: module.rospy.Time.from_sec(
        clock[0] if ros_override[0] is None else ros_override[0]))
    current = helper._rich_plan(stamp_sec=10.)
    candidate = helper._rich_plan(stamp_sec=12.01)
    node.set_state = Mock()

    def fresh_reference(*_args):
        clock[0] = 12.02
        return helper._object(stamp_sec=12.01)

    node._wait_for_final_refine_reference = fresh_reference
    node._request_near_field_preview_stream = lambda _gcfg: True
    node._copy_near_field_preview_candidate = lambda *_args, **_kwargs: (
        module.PlanValidationResult(True), candidate)
    monkeypatch.setattr(module, 'build_bounded_final_visual_refinement',
        lambda *_args: (module.PlanValidationResult(True), candidate, {
            'translation_m': .001, 'yaw_rad': 0., 'observed_roll_pitch_change_rad': 0.}))
    monkeypatch.setattr(module, 'validate_calibration_centering_margin',
                        lambda *_args: module.PlanValidationResult(True))
    node._execution_checkpoint = lambda *_args: True

    def check_sequence(*_args):
        if delay_stage == 'sequence':
            clock[0] = 32.1
        elif delay_stage == 'ros_clock':
            ros_override[0] = 32.1
        elif delay_stage == 'monotonic_only':
            ros_override[0] = clock[0]
            clock[0] = 32.1
        return module.PlanValidationResult(True)

    def invoke_bound(_plan, _gcfg, _label, action):
        if delay_stage == 'rebind_lock':
            clock[0] = 32.1
        return module.PlanValidationResult(True), action()

    def simulate(*_args):
        if delay_stage == 'simulation':
            clock[0] = 32.1
        return True

    node._check_final_refine_sequence = check_sequence
    node._invoke_plan_bound_action = invoke_bound
    node._freeze_execution_plan = Mock(return_value=candidate)
    node._simulate_grasp6d_plan_if_required = simulate
    result = node._maybe_final_refine_grasp6d_plan({
        'near_field_strategy': 'single_snapshot_direct',
        'final_visual_refine_enabled': True,
        'final_visual_refine_timeout_sec': 20.,
    }, {}, current)
    if delay_stage == 'none':
        assert result is candidate
    else:
        assert result is None
        assert node.set_state.call_args.args[0] == module.GraspStages.FAILED
    if delay_stage in ('sequence', 'rebind_lock', 'ros_clock', 'monotonic_only'):
        node._freeze_execution_plan.assert_not_called()
