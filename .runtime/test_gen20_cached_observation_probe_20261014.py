"""Independent, inert cached-publication probe; not a hardware test."""
from types import SimpleNamespace as NS
import pytest
from test_remote_modes import configured
from test_silhouette_observation_planning import context


@pytest.mark.parametrize('mutation', ['none', 'source_reserve', 'camera_contract'])
def test_cached_replan_rechecks_original_observation_authority(configured, monkeypatch, tmp_path, mutation):
    node, prepared, _, _ = context(configured, monkeypatch, tmp_path)
    node._configured_execution_plan_validity_sec = lambda: 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    assert node.latest_preview_rich_plan.valid and node.latest_rich_plan.valid
    old_count = len(node.rich_plan_pub.messages)
    checked = []
    # An immediate replan is intentionally throttled; cross the existing
    # cooldown before probing a new cached publication transaction.
    node._promotion_now_sec = lambda: 14.
    if mutation == 'source_reserve':
        now = prepared.ticket.snapshot_stamp_sec + 90.
        node._stream_source_clock = lambda: now
        node._promotion_now_sec = lambda: now
        node._execution_plan_validity_now_sec = lambda: now
        assert not node._observation_source_reserve_available(prepared.ticket)
    elif mutation == 'camera_contract':
        def changed(_):
            checked.append(True)
            raise ValueError('controlled camera configuration changed after cached preview')
        monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', changed)
    controller = node._promotion_controller()
    assert controller.request_replan(robot_active=False).code == 'REPLAN_REQUESTED'
    decision = node._try_promote_cached_preview_after_replan()
    print(dict(mutation=mutation, decision=decision.code, promoted=decision.promote,
        rich_new_count=len(node.rich_plan_pub.messages)-old_count,
        final_rich_valid=bool(node.latest_rich_plan and node.latest_rich_plan.valid),
        controller_execution_id=controller.execution_plan_id,
        invalid_camera_predicate_checked=bool(checked)))
    if mutation == 'none':
        assert decision.promote
    else:
        assert not decision.promote, 'cached observation bypassed revoked source/configuration'
        assert not node.latest_rich_plan or not node.latest_rich_plan.valid
