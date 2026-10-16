"""Independent producer/runner regressions for unknown observation discovery.

Only offline service responses and clocks are controlled. The real candidate
loop, producer transaction, audit promotion and runner preview consumer run.
These tests assert scheduling/authority behavior; they do not prove physical
IK, collision clearance, tracking, or a successful hardware grasp.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from test_remote_modes import configured, base, module
from test_silhouette_observation_coverage import search_context
from test_silhouette_observation_view_source import view_context
from test_silhouette_observation_planning import context as promotion_context
from test_mode_runner import module as runner_module


def response(ok=False, code='MOVEIT_UNREACHABLE'):
    return (base.MoveItResult(ok, .3 if ok else 0., .2 if ok else 0.,
        'controlled response observation_screened_duration_sec=2.0',
        True if ok else None, True if ok else None,
        True if ok else None, True if ok else None,
        failure_code='' if ok else code), {}, '')


def install_checks(monkeypatch, node, variants, checked, clock, policy):
    def strict(pose, observation=False):
        assert observation is True
        index = next(i for i, row in enumerate(variants)
                     if row['sequence'].pregrasp is pose)
        checked.append(index)
        clock.value += .8
        return policy(len(checked), index)
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)


def runner_for(node, prepared, monkeypatch):
    runner = runner_module.ModeAwareGraspRunner()
    runner.selection = deepcopy(node._mode_selection)
    runner.expected_model = 'unknown_tabletop'
    runner.minimum_stamp_ns = prepared.snapshot.stamp_ns
    runner.plan_validity_sec = 120.
    monkeypatch.setattr(runner_module.rospy.Time, 'now', staticmethod(
        lambda: runner_module.rospy.Time.from_sec(prepared.snapshot.stamp_sec + .1)))
    assert runner.selection_current()
    return runner


def assert_no_authority(node, tmp_path):
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not (tmp_path / 'planning.json.execution').exists()


@pytest.mark.parametrize('first_success', [5, 7])
def test_later_original_view_can_supply_first_qualified_incumbent(
        configured, monkeypatch, tmp_path, first_success):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda call, index: response(call == first_success))
    result = node._accept_prediction(prepared)
    assert result['status'] == 'PREVIEW_READY'
    assert len(checked) >= first_success and len(set(checked)) == len(checked)
    runtime = node._stable_variant_runtime[(0, 0)]
    assert runtime['_observation_qualified_incumbent_ticket'] is prepared.ticket
    search = runtime['observation_orientation_search']
    assert search['success_first_search']
    assert search['optimization_budget_starts_after_incumbent']
    assert search['reachable_variant_count'] == 1
    preview = node.latest_preview_rich_plan
    assert preview.valid and len(preview.poses) == 1
    assert preview.candidate_source == 'silhouette_observation'
    assert not preview.contact_geometry_evidence and preview.required_open_width_m == 0.
    assert node._execution_promotion_audit_ready(preview)
    runner = runner_for(node, prepared, monkeypatch)
    runner.on_preview(preview)
    assert runner.preview.plan_id == preview.plan_id and not runner.terminal_reason


@pytest.mark.parametrize('failure_code', ['MOVEIT_UNREACHABLE', 'MOVEIT_COLLISION'])
def test_complete_real_rejections_remain_terminal(
        configured, monkeypatch, tmp_path, failure_code):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda *_: response(False, failure_code))
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_NO_SAFE_VIEW'
    assert sorted(checked) == list(range(len(variants)))
    search = node._stable_variant_runtime[(0, 0)]['observation_orientation_search']
    assert search['search_complete'] and search['pending_variant_count'] == 0
    assert search['unchecked_variant_count'] == 0
    assert node._observe_return_paused
    preview = node.latest_preview_rich_plan
    assert not preview.valid and preview.candidate_source == 'near_field_terminal'
    runner = runner_for(node, prepared, monkeypatch)
    runner.on_preview(preview)
    assert runner.terminal_reason.startswith('SILHOUETTE_OBSERVATION_NO_SAFE_VIEW:')
    assert_no_authority(node, tmp_path)


@pytest.mark.parametrize('failure_code,expected_checks', [
    ('OBSERVATION_PLANNING_TIMEOUT', 8), ('MOVEIT_CHECK_ERROR', 8), ('MOVEIT_TIMEOUT', 1)])
def test_unresolved_search_does_not_pause_producer_or_latch_runner_terminal(
        configured, monkeypatch, tmp_path, failure_code, expected_checks):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda call, _: response(False, failure_code if call == 1 else 'MOVEIT_UNREACHABLE'))
    runner = runner_for(node, prepared, monkeypatch)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_SEARCH_PENDING'
    assert len(checked) == expected_checks
    search = node._stable_variant_runtime[(0, 0)]['observation_orientation_search']
    assert not search['search_complete'] and search['pending_variant_count'] == 1
    assert not node._observe_return_paused
    pending = node.latest_preview_rich_plan
    assert not pending.valid and pending.candidate_source == 'observation_search_pending'
    runner.on_preview(pending)
    assert not runner.terminal_reason and runner.preview is None
    assert_no_authority(node, tmp_path)
    # A subsequent freshly checked transaction can satisfy the same runner;
    # no previous positive or negative path result is used as authority.
    checked.clear()
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda call, _: response(call == 5))
    result = node._accept_prediction(prepared)
    assert result['status'] == 'PREVIEW_READY' and len(checked) >= 5
    runner.on_preview(node.latest_preview_rich_plan)
    assert runner.preview is not None and not runner.terminal_reason


@pytest.mark.parametrize('with_incumbent', [False, True])
@pytest.mark.parametrize('clock_fault', ['reserve_exhausted', 'future_source'])
def test_inflight_result_cannot_renew_source_or_preserve_prior_incumbent(
        configured, monkeypatch, tmp_path, with_incumbent, clock_fault):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    source = NS(now=prepared.ticket.snapshot_stamp_sec + .1)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    node._stream_source_clock = lambda: source.now
    def policy(call, _):
        if call == (2 if with_incumbent else 1):
            source.now = prepared.ticket.snapshot_stamp_sec + (90. if clock_fault == 'reserve_exhausted' else -.01)
        return response(True)
    install_checks(monkeypatch, node, variants, checked, clock, policy)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_SEARCH_PENDING'
    assert len(checked) == (2 if with_incumbent else 1)
    runtime = node._stable_variant_runtime[(0, 0)]
    assert runtime['_observation_qualified_incumbent_ticket'] is None
    assert runtime['observation_orientation_search']['stop_reason'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'
    assert_no_authority(node, tmp_path)


@pytest.mark.parametrize('owner_change', ['stop', 'epoch'])
def test_actual_owner_change_inflight_cancels_even_after_qualified_incumbent(
        configured, monkeypatch, tmp_path, owner_change):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    def policy(call, _):
        if call == 2:
            if owner_change == 'stop':
                node.stop_streaming()
            else:
                node._advance_target_instance_epoch('CAMERA_STREAM_STALE', preserve_identity=False)
        return response(True)
    install_checks(monkeypatch, node, variants, checked, clock, policy)
    with pytest.raises(base.StreamResultCancelled):
        node._accept_prediction(prepared)
    assert len(checked) == 2
    assert_no_authority(node, tmp_path)


def test_original_comparison_window_exhaustion_uses_only_qualified_incumbent(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    def policy(call, _):
        if call == 2:
            clock.value += 8.1
        return response(call == 1)
    install_checks(monkeypatch, node, variants, checked, clock, policy)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'PREVIEW_READY' and len(checked) == 2
    search = node._stable_variant_runtime[(0, 0)]['observation_orientation_search']
    assert search['comparison_budget_reached'] and not search['search_complete']
    assert search['reachable_variant_count'] == 1 and search['unchecked_variant_count'] == 6
    report = node._active_execution_audit_report
    assert report['moveit_selection']['global_time_optimality_claimed'] is False
    assert node._execution_promotion_audit_ready(node.latest_preview_rich_plan)


@pytest.mark.parametrize('mutation', ['camera', 'view_limits'])
def test_real_frozen_source_change_after_search_before_publish_blocks_authority(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _, camera, _ = view_context(configured, monkeypatch, tmp_path)
    # Isolate full path response; retain the real source fingerprint guards.
    monkeypatch.setattr(node, '_observation_following_support_evaluation',
        lambda _p, result, _target: (result, {'ok': True}))
    rank = node._checked_observation_variant_rank
    injected = []
    def changed_rank(result, variant):
        ranked = rank(result, variant)
        injected.append(True)
        if mutation == 'camera':
            camera.fx += 1.
        else:
            configured[1]['/grasp_6d/remote/observation_camera_visibility_margin_px'] = 25.
        return ranked
    monkeypatch.setattr(node, '_checked_observation_variant_rank', changed_rank)
    with pytest.raises((ValueError, base.CandidateContractError, base.StreamResultCancelled)):
        node._accept_prediction(prepared)
    assert injected, 'must reach the qualified selection before mutating its frozen source'
    assert_no_authority(node, tmp_path)
    assert not any(plan.valid for plan in node.preview_rich_plan_pub.messages)


def test_pending_does_not_erase_an_existing_real_runner_terminal(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda *_: response(False, 'MOVEIT_TIMEOUT'))
    node._accept_prediction(prepared)
    runner = runner_for(node, prepared, monkeypatch)
    runner.terminal_reason = 'OBSERVE_RETURN_TARGET_CHANGED: real owner loss'
    runner.on_preview(node.latest_preview_rich_plan)
    assert runner.terminal_reason == 'OBSERVE_RETURN_TARGET_CHANGED: real owner loss'
    assert runner.preview is None


def test_repeated_pending_preserves_the_original_runner_wait_deadline(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    install_checks(monkeypatch, node, variants, checked, clock,
        lambda *_: response(False, 'OBSERVATION_PLANNING_TIMEOUT'))
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_SEARCH_PENDING'
    search = node._stable_variant_runtime[(0, 0)]['observation_orientation_search']
    assert search['pending_variant_count'] == 8 and not search['search_complete']
    runner = runner_for(node, prepared, monkeypatch)
    elapsed = NS(value=0., waits=0)
    monkeypatch.setattr(runner_module.time, 'monotonic', lambda: elapsed.value)
    monkeypatch.setattr(runner_module.rospy, 'is_shutdown', lambda: False)
    def wait(duration):
        elapsed.value += duration
        elapsed.waits += 1
        assert elapsed.waits <= 4, 'pending renewed the original wait deadline'
        runner.on_preview(node.latest_preview_rich_plan)
    monkeypatch.setattr(runner.condition, 'wait', wait)
    assert runner.wait_for_preview(1.) is None
    assert elapsed.value == 1. and elapsed.waits == 4
    assert not runner.terminal_reason and runner.preview is None
    assert not node._observe_return_paused
    assert_no_authority(node, tmp_path)


@pytest.mark.parametrize('mutation', ['camera', 'source_reserve'])
def test_audit_write_cannot_hide_change_before_execution_publication(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _, _ = promotion_context(configured, monkeypatch, tmp_path)
    changed, writes = [], []
    # Camera projection itself is exercised by the real frozen-source tests
    # above. This valid promotion fixture controls only its live predicate to
    # isolate the ordering across the original audit-write transaction.
    def require_frozen_view(_):
        if changed:
            raise ValueError('controlled frozen camera signature changed')
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', require_frozen_view)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    writer = node._write_gate_audit_report
    def changed_write(*args, **kwargs):
        result = writer(*args, **kwargs)
        if kwargs.get('reference_attr') == '_latest_execution_audit_reference':
            writes.append(True)
            if mutation == 'camera':
                changed.append(True)
            else:
                node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
        return result
    monkeypatch.setattr(node, '_write_gate_audit_report', changed_write)
    try:
        node._accept_prediction(prepared)
    except (ValueError, base.CandidateContractError, base.StreamResultCancelled) as exc:
        assert writes, 'fixture failed before audit mutation: %r' % exc
    assert writes, 'must reach original execution audit writer before mutation'
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not getattr(node.latest_rich_plan, 'valid', False)


@pytest.mark.parametrize('stage', ['legacy', 'rich'])
@pytest.mark.parametrize('mutation', ['camera', 'source_reserve'])
def test_partial_publication_retracts_source_invalidated_during_transport(
        configured, monkeypatch, tmp_path, stage, mutation):
    node, prepared, _, _ = promotion_context(configured, monkeypatch, tmp_path)
    changed, injected = [], []
    def require_frozen_view(_):
        if changed:
            raise ValueError('controlled frozen camera signature changed')
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', require_frozen_view)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    publisher = node.plan_pub if stage == 'legacy' else node.rich_plan_pub
    original = publisher.publish
    def publish(message):
        original(message)
        valid = bool(message.poses) if stage == 'legacy' else message.valid
        if valid and not injected:
            injected.append(True)
            if mutation == 'camera':
                changed.append(True)
            else:
                node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    monkeypatch.setattr(publisher, 'publish', publish)
    try:
        node._accept_prediction(prepared)
    except (ValueError, base.CandidateContractError, base.StreamResultCancelled) as exc:
        assert injected, 'fixture failed before publication mutation: %r' % exc
    assert injected
    assert node.rich_plan_pub.messages and not node.rich_plan_pub.messages[-1].valid
    assert node.plan_pub.messages and not node.plan_pub.messages[-1].poses
    assert not getattr(node.latest_rich_plan, 'valid', False)
    assert not node._promotion_controller().has_execution


@pytest.mark.parametrize('mutation', ['camera', 'source_reserve'])
def test_recovery_never_revives_invalid_previous_observation_authority(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _, _ = promotion_context(configured, monkeypatch, tmp_path)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    assert node._promotion_controller().has_execution and node.latest_rich_plan.valid
    token = node._capture_promotion_token_locked(prepared.ticket,
        prepared.request_invalidation_generation)
    assert token['previous_rich_plan'].valid
    before = len(node.rich_plan_pub.messages)
    if mutation == 'camera':
        def changed(_):
            raise ValueError('controlled frozen camera signature changed')
        monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', changed)
    else:
        node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    node._recover_aborted_execution_publish(token, 'controlled invalidation')
    assert len(node.rich_plan_pub.messages) > before
    assert all(not plan.valid for plan in node.rich_plan_pub.messages[before:])
    assert not getattr(node.latest_rich_plan, 'valid', False)
    assert not node._promotion_controller().has_execution
    assert not getattr(node, '_execution_authority_ticket', None)


def test_token_capture_rechecks_source_clock_after_view_parameter_reads(
        configured, monkeypatch, tmp_path):
    node, prepared, _, _ = promotion_context(configured, monkeypatch, tmp_path)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    def delayed_view_parameters(_):
        node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', delayed_view_parameters)
    with pytest.raises(base.CandidateContractError, match='reserve'):
        node._capture_promotion_token_locked(prepared.ticket,
            prepared.request_invalidation_generation)


@pytest.mark.parametrize('other_owner', ['newer_ticket', 'executing', 'new_generation'])
def test_old_recovery_does_not_clear_later_or_executing_authority(
        configured, monkeypatch, tmp_path, other_owner):
    node, prepared, _, _ = promotion_context(configured, monkeypatch, tmp_path)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    token = node._capture_promotion_token_locked(prepared.ticket,
        prepared.request_invalidation_generation)
    if other_owner == 'newer_ticket':
        newer = replace(prepared.ticket, request_id=prepared.ticket.request_id + 1,
                        snapshot_stamp_sec=prepared.ticket.snapshot_stamp_sec + .1)
        node._execution_authority_ticket = node._execution_authority_record(newer)
    elif other_owner == 'executing':
        node.robot_execution_active = True
    else:
        node._stream_generation += 1
    owned = deepcopy(node._execution_authority_ticket)
    plan_id = node.latest_rich_plan.plan_id
    before = len(node.rich_plan_pub.messages)
    node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    node._recover_aborted_execution_publish(token, 'controlled old source invalidation')
    assert len(node.rich_plan_pub.messages) == before
    assert node.latest_rich_plan.valid and node.latest_rich_plan.plan_id == plan_id
    assert node._execution_authority_ticket == owned
    assert node._promotion_controller().has_execution
