"""Failure diagnostics use new measured evidence and never resume motion."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from test_contact_linear_compensation import gateway_case
from alicia_flexible_grasp.robot import contact_linear_compensation as module
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q


FAILURE = 'execute failed from cached plan (cartesian): target xyz=(-0.130, -0.422, 0.112)'


def stationary_metadata(sample, *, epoch, model, end_ns):
    """Synthetic test window metadata; positions may come from a real fixture."""
    sample.update(measurement_kind='stationary_sdk_following_not_executed_endpoint',
        model_sha256=model, epoch_ns=epoch, joint_names=list(ARM_NAMES),
        sdk_stamp_ns=end_ns, accepted_stamp_ns=end_ns,
        stationary_window_start_ns=end_ns-300_000_000,
        stationary_window_end_ns=end_ns)
    return sample


def recorded_positions_with_synthetic_windows():
    data = json.loads((Path(__file__).parent/'fixtures/contact_no_response_20260927.json').read_text())
    # Exercise the real position arithmetic without claiming the fixture's
    # pre-result sample was captured in a new post-failure stationary window.
    epoch = data['epoch_ns']
    before = stationary_metadata(deepcopy(data['before']), epoch=epoch,
        model=data['model_sha256'], end_ns=epoch+1_000_000_000)
    after = stationary_metadata(deepcopy(data['after']), epoch=epoch,
        model=data['model_sha256'], end_ns=epoch+2_500_000_000)
    args = dict(epoch=epoch, model_sha256=data['model_sha256'],
        after_ns=epoch+2_000_000_000, now_sec=(epoch+2_500_000_000)*1e-9)
    return data, before, after, args


def test_recorded_joint2_transmitted_fifteen_counts_without_encoder_response():
    _, before, after, args = recorded_positions_with_synthetic_windows()
    original = deepcopy((before, after))
    report = module.contact_failure_following(before, after, **args)
    joint = report['joints'][1]
    assert joint['joint_name'] == 'Joint2'
    assert joint['transmitted_command_delta_counts'] == 15
    assert joint['measured_delta_counts'] == 0
    assert joint['initial_sdk_minus_encoder_counts'] == pytest.approx(17)
    assert joint['final_sdk_minus_encoder_counts'] == pytest.approx(32)
    assert joint['no_measured_response']
    assert report['maximum_final_following_error_rad'] == pytest.approx(32*Q)
    assert report['maximum_initial_following_error_rad'] < .035
    assert report['following_outside_original_contract']
    assert (before, after) == original
    json.dumps(report, allow_nan=False)


def test_real_recorded_post_failure_window_reports_joint2_without_authorizing_recovery():
    data = json.loads((Path(__file__).parent/'fixtures/contact_no_response_20260927.json').read_text())
    initial = deepcopy(data['before'])
    # The cached sample metadata is reconstructed here only for its earlier
    # window; the post-failure sample and its source window are real bag data.
    stationary_metadata(initial, epoch=data['epoch_ns'], model=data['model_sha256'],
        end_ns=min(initial['sdk_stamp_ns'], initial['accepted_stamp_ns']))
    final = data['post_failure_stationary']
    report = module.contact_failure_following(initial, final,
        epoch=data['epoch_ns'], model_sha256=data['model_sha256'],
        after_ns=1790491095840000000, now_sec=final['stamp_sec'])
    assert report['joints'][1]['transmitted_command_delta_counts'] == 15
    assert report['joints'][1]['measured_delta_counts'] == 0
    assert report['joints'][1]['no_measured_response']
    assert report['following_outside_original_contract']
    assert report['maximum_final_following_error_rad'] == pytest.approx(32*Q)


@pytest.mark.parametrize('fault', ['old', 'stale', 'epoch', 'model', 'nan', 'short_window', 'future'])
def test_invalid_post_failure_evidence_cannot_be_used(fault):
    _, before, after, args = recorded_positions_with_synthetic_windows()
    if fault == 'old': after = deepcopy(before)
    if fault == 'stale': args['now_sec'] += 1.
    if fault == 'epoch': after['epoch_ns'] += 1
    if fault == 'model': after['model_sha256'] = 'different'
    if fault == 'nan': after['sdk_positions_rad'][1] = float('nan')
    if fault == 'short_window': after['stationary_window_start_ns'] += 1
    if fault == 'future': args['now_sec'] -= 1.
    with pytest.raises(ValueError):
        module.contact_failure_following(before, after, **args)


@pytest.mark.parametrize('command,measured,missing', [(3, 0, False), (4, 1, True),
                                                    (-4, -1, True), (4, 2, False)])
def test_no_response_label_requires_sufficient_transmitted_movement(command, measured, missing):
    _, before, after, args = recorded_positions_with_synthetic_windows()
    after['sdk_positions_rad'][1] = before['sdk_positions_rad'][1]+command*Q
    after['accepted_positions_rad'][1] = before['accepted_positions_rad'][1]+measured*Q
    report = module.contact_failure_following(before, after, **args)
    assert report['joints'][1]['no_measured_response'] is missing


def failed_gateway(monkeypatch, fault=None):
    node, planner, plan, initial = gateway_case(monkeypatch)
    stationary_metadata(initial, epoch=1, model=initial['model_sha256'], end_ns=99_500_000_000)
    clock = {'now': 100.}
    monkeypatch.setattr(module.rospy, 'get_time', lambda: clock['now'])
    node._completed_command_reference = {'original': 'successful pregrasp only'}
    def snapshot(*args, **kwargs):
        if not kwargs.get('after_ns'):
            return deepcopy(initial)
        assert kwargs['after_ns'] == 100_000_000_000
        kwargs['context_validator']()
        if fault == 'sampling': raise ValueError('missing post-failure stationary samples')
        if fault == 'revoked_during_sampling':
            node._validate_pregrasp_context.side_effect = ValueError('manual authority revoked')
        after = stationary_metadata(deepcopy(initial), epoch=1,
            model=initial['model_sha256'], end_ns=100_500_000_000)
        after['sdk_positions_rad'][1] += 15*Q
        if fault == 'old': after = deepcopy(initial)
        if fault == 'epoch': after['epoch_ns'] += 1
        clock['now'] = 100.5
        if fault == 'stale': clock['now'] = 101.1
        return after
    node._endpoint_following_snapshot.side_effect = snapshot
    original_execute = planner._execute_cached_pose_plan.side_effect
    def execute(*args, **kwargs):
        original_execute(*args, **kwargs)
        if fault == 'revoked_before_sampling':
            node._validate_pregrasp_context.side_effect = ValueError('epoch changed')
        return False, FAILURE
    planner._execute_cached_pose_plan.side_effect = execute
    return node, planner, plan


def test_failed_motion_keeps_original_failure_and_adds_bound_read_only_diagnostic(monkeypatch):
    node, planner, plan = failed_gateway(monkeypatch)
    reference = node._completed_command_reference
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    response = node.handle_contact_approach(NS(plan=plan, execute=True))
    assert not response.success
    report = json.loads(response.message)
    assert report['code'] == 'CONTACT_LINEAR_COMPENSATION_FAILED'
    assert report['reason'] == FAILURE
    diagnostic = report['following_failure_diagnostic']
    assert diagnostic['status'] == 'available'
    assert diagnostic['joints'][1]['no_measured_response']
    for key in ('plan_id', 'goal_index', 'trajectory_sha256'):
        assert diagnostic[key] == report[key]
    assert diagnostic['diagnostic_only']
    for key in ('motion_authorized', 'recovery_authorized', 'real_grasp_success', 'hardware_cause_determined'):
        assert diagnostic[key] is False
    assert node._completed_command_reference is reference
    assert planner._execute_cached_pose_plan.call_count == 1
    assert node._ensure_trajectory_controllers_started.call_count == 1
    assert node._contact_linear_cache is None
    assert node._compensation_visual_context is None
    assert planner._last_pose_plan is None


@pytest.mark.parametrize('fault', ['sampling', 'old', 'epoch', 'stale',
                                  'revoked_before_sampling', 'revoked_during_sampling'])
def test_unavailable_diagnostic_preserves_original_rejection_without_old_feedback(monkeypatch, fault):
    node, planner, plan = failed_gateway(monkeypatch, fault)
    reference = deepcopy(node._completed_command_reference)
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    response = node.handle_contact_approach(NS(plan=plan, execute=True))
    assert not response.success
    report = json.loads(response.message)
    assert report['reason'] == FAILURE
    assert report['code'] == 'CONTACT_LINEAR_COMPENSATION_FAILED'
    diagnostic = report['following_failure_diagnostic']
    assert diagnostic['status'] == 'diagnostic_unavailable'
    assert diagnostic['reason']
    assert 'joints' not in diagnostic
    assert node._completed_command_reference == reference
    assert planner._execute_cached_pose_plan.call_count == 1


@pytest.mark.parametrize('message', ['CONTROLLER_REFERENCE_INVALID: missing',
                                   'execute cached plan exception: transport failed',
                                   'strict cached execute blocked: OBSERVATION_PATH_INVALID'])
def test_other_execution_rejections_keep_existing_output_and_do_not_sample(monkeypatch, message):
    node, planner, plan = failed_gateway(monkeypatch)
    assert node.handle_contact_approach(NS(plan=plan, execute=False)).success
    planner._execute_cached_pose_plan.side_effect = lambda *args, **kwargs: (False, message)
    response = node.handle_contact_approach(NS(plan=plan, execute=True))
    assert not response.success
    report = json.loads(response.message)
    assert report['reason'] == message
    assert 'following_failure_diagnostic' not in report
    assert node._endpoint_following_snapshot.call_count == 2
