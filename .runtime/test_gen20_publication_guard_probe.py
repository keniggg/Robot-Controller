"""Offline probe of authority changes during the actual audit write."""
import pytest
from test_remote_modes import configured, base
from test_silhouette_observation_planning import context


@pytest.mark.parametrize('mutation', ['camera', 'source_reserve'])
def test_execution_audit_write_cannot_hide_observation_authority_change(
        configured, monkeypatch, tmp_path, mutation):
    node, prepared, _, _ = context(configured, monkeypatch, tmp_path)
    changed = []
    def require_frozen_view(_):
        if changed:
            raise ValueError('controlled frozen camera signature changed')
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', require_frozen_view)
    node.mujoco_selection_snapshot_reserve_sec = 30.
    monkeypatch.setattr(node, '_configured_execution_plan_validity_sec', lambda: 120.)
    writer = node._write_gate_audit_report
    writes = []
    def changed_write(*args, **kwargs):
        result = writer(*args, **kwargs)
        if kwargs.get('reference_attr') == '_latest_execution_audit_reference':
            writes.append(1)
            if mutation == 'camera':
                changed.append(1)
            else:
                node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + 90.
        return result
    monkeypatch.setattr(node, '_write_gate_audit_report', changed_write)
    try:
        node._accept_prediction(prepared)
    except (ValueError, base.CandidateContractError, base.StreamResultCancelled) as exc:
        assert writes, 'source fixture failed before probe injection: %r' % exc
    assert writes, 'probe must reach the original execution audit write'
    assert not any(plan.valid for plan in node.rich_plan_pub.messages), (
        'execution authority published after frozen source/config changed during audit write')
