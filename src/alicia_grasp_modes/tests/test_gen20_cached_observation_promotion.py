"""Cached observations retain the original source, audit and publication checks."""
from copy import deepcopy
from dataclasses import replace

import pytest

from test_remote_modes import configured, base
from test_silhouette_observation_planning import context


def cached_context(configured, monkeypatch, tmp_path, ordinary=False, previous=False):
    node, prepared, checks, _ = context(configured, monkeypatch, tmp_path, ordinary=ordinary)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    assert node._execution_promotion_audit_ready(node.latest_preview_rich_plan)
    assert node._latest_preview_gate_audit_report['plan_id'] == node.latest_preview_rich_plan.plan_id
    if not previous:
        # Retain the real cached Preview and its audit, then request a new
        # execution transaction without re-running the path service.
        node._promotion_controller().clear_execution()
        node.latest_rich_plan = None
        node.latest_plan = None
        node._execution_authority_ticket = None
        node._active_execution_audit_report = None
        node._latest_execution_audit_reference = None
    node._promotion_now_sec = lambda: 14.
    assert node._promotion_controller().request_replan(False).code == 'REPLAN_REQUESTED'
    node.plan_pub.messages.clear()
    node.rich_plan_pub.messages.clear()
    return node, prepared, checks


@pytest.mark.parametrize('ordinary', [False, True])
def test_cached_observation_promotes_with_original_source_and_checked_path(
        configured, monkeypatch, tmp_path, ordinary):
    node, prepared, checks = cached_context(configured, monkeypatch, tmp_path, ordinary)
    checked_count = len(checks)
    original_preview = deepcopy(node.latest_preview_rich_plan)
    node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 89.999
    actual = node._try_promote_cached_preview_after_replan()
    assert actual.promote and actual.code == 'PROMOTED_CACHED_PREVIEW'
    assert node.latest_rich_plan.plan_id == original_preview.plan_id
    assert node.latest_rich_plan.header.stamp == prepared.stamp
    assert node._execution_authority_ticket == node._execution_authority_record(prepared.ticket)
    assert node._active_execution_audit_report['snapshot_stamp_sec'] == prepared.ticket.snapshot_stamp_sec
    assert node._execution_promotion_audit_ready(node.latest_rich_plan)
    assert len(checks) == checked_count


@pytest.mark.parametrize('ordinary', [False, True])
@pytest.mark.parametrize('age', [-.001, 90., 119.999, 120.])
def test_cached_observation_cannot_renew_original_source(
        configured, monkeypatch, tmp_path, ordinary, age):
    node, prepared, _ = cached_context(configured, monkeypatch, tmp_path, ordinary)
    node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + age
    actual = node._try_promote_cached_preview_after_replan()
    assert not actual.promote
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not getattr(node.latest_rich_plan, 'valid', False)
    assert not node._promotion_controller().has_execution
    assert not node._execution_promotion_audit_ready(node.latest_preview_rich_plan)


@pytest.mark.parametrize('ordinary', [False, True])
@pytest.mark.parametrize('stage', ['before', 'audit', 'legacy', 'rich'])
@pytest.mark.parametrize('mutation', ['camera', 'source_reserve'])
def test_cached_observation_rechecks_after_every_blocking_publication_boundary(
        configured, monkeypatch, tmp_path, ordinary, stage, mutation):
    node, prepared, _ = cached_context(configured, monkeypatch, tmp_path, ordinary)
    changed, injected = [], []
    original_view = node._require_diagnostic_observation_view_source
    def frozen_view(value):
        if changed:
            raise ValueError('controlled original camera changed')
        return original_view(value)
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', frozen_view)
    def change():
        if injected:
            return
        injected.append(True)
        if mutation == 'camera':
            changed.append(True)
        else:
            node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    if stage == 'before':
        change()
    elif stage == 'audit':
        writer = node._write_gate_audit_report
        def delayed_write(*args, **kwargs):
            result = writer(*args, **kwargs)
            if kwargs.get('reference_attr') == '_latest_execution_audit_reference':
                change()
            return result
        monkeypatch.setattr(node, '_write_gate_audit_report', delayed_write)
    else:
        publisher = node.plan_pub if stage == 'legacy' else node.rich_plan_pub
        publish = publisher.publish
        def delayed_publish(message):
            publish(message)
            if bool(message.poses) if stage == 'legacy' else message.valid:
                change()
        monkeypatch.setattr(publisher, 'publish', delayed_publish)
    actual = node._try_promote_cached_preview_after_replan()
    assert injected, 'must reach the intended boundary'
    assert not actual.promote
    if stage != 'rich':
        assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    else:
        assert not node.rich_plan_pub.messages[-1].valid
        assert not node.plan_pub.messages[-1].poses
    assert not getattr(node.latest_rich_plan, 'valid', False)
    assert not node._promotion_controller().has_execution
    assert not node._execution_promotion_audit_ready(node.latest_preview_rich_plan)


@pytest.mark.parametrize('mutation', ['runtime_missing', 'plan_content', 'plan_malformed',
    'request', 'phase', 'manual'])
def test_cached_observation_requires_exact_prepared_request_and_owner(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _ = cached_context(configured, monkeypatch, tmp_path)
    runtime = node._stable_variant_runtime[(0, 0)]
    if mutation == 'runtime_missing':
        runtime.pop('prepared')
    elif mutation == 'plan_content':
        node.latest_preview_rich_plan.poses[0].position.x += .001
    elif mutation == 'plan_malformed':
        node.latest_preview_rich_plan.poses[0].position.x = float('nan')
    elif mutation == 'request':
        runtime['prepared'] = replace(prepared,
            ticket=replace(prepared.ticket, request_id=prepared.ticket.request_id + 1))
    elif mutation == 'phase':
        node._observe_return_phase = 'return'
    else:
        node.stop_streaming()
    actual = node._try_promote_cached_preview_after_replan()
    assert not actual.promote
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not getattr(node.latest_rich_plan, 'valid', False)


@pytest.mark.parametrize('mutation', ['source_reserve', 'camera'])
def test_cached_rejection_cannot_restore_previous_revoked_observation(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _ = cached_context(configured, monkeypatch, tmp_path, previous=True)
    if mutation == 'source_reserve':
        node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
    else:
        def changed(_):
            raise ValueError('controlled original camera changed')
        monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', changed)
    actual = node._try_promote_cached_preview_after_replan()
    assert not actual.promote
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not getattr(node.latest_rich_plan, 'valid', False)
    assert not node._promotion_controller().has_execution
    assert not node._execution_promotion_audit_ready(node.latest_preview_rich_plan)
