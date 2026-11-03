"""Pending measurement cannot cross the original task submission boundary.

Uses the same three real owners as the finite transport tests, with inert
transport and an action spy. No plan validator or admission gate is replaced.
"""
from copy import deepcopy

import pytest

from alicia_grasp_modes.station_compute_wait_task import PENDING
import test_gen63_station_source_overlap as contract_test


def test_pending_overlap_and_incomplete_recovery_reject_actual_plan_submission(monkeypatch):
    c = contract_test.owners(monkeypatch)
    task = c.task
    original_plan = deepcopy(task._bound_execution_plan)
    original_id = task._bound_execution_plan_id
    original_digest = task._bound_execution_plan_digest
    phase_deadline = task._near_field_phase_deadline_sec
    acknowledgments = (c.producer._observation_measurement_ack_pub.publish.call_count,
                       len(c.node._measurement_remote_ack_pub.messages))
    actions = []

    def action():
        actions.append('physical action callback reached')
        return True

    def unchanged_authority():
        assert task._bound_execution_plan == original_plan
        assert task._bound_execution_plan_id == original_id
        assert task._bound_execution_plan_digest == original_digest
        cached = task.latest_grasp6d_plan
        assert cached.valid and cached.diagnostic == 'FAR_FIELD_OBSERVATION_PLAN'
        assert cached.plan_id == original_id
        assert task._execution_plan_digest(cached) == original_digest
        assert task._near_field_phase_deadline_sec == phase_deadline
        assert not getattr(task.latest_grasp6d_preview_plan, 'valid', False)
        assert actions == []
        assert acknowledgments == (
            c.producer._observation_measurement_ack_pub.publish.call_count,
            len(c.node._measurement_remote_ack_pub.messages))

    def pending_entries_reject():
        contract_test.assert_nonmetric_wait(c)
        contact = deepcopy(c.plan)
        contact.diagnostic = 'CONTACT_EXECUTION_PLAN'
        for plan in (c.plan, contact):
            result, response = task._invoke_plan_bound_action(
                plan, {}, 'strict pregrasp', action)
            assert not result.ok and result.code == PENDING
            assert result.reason == 'waiting for a new independently measured frame'
            assert response is None
        with pytest.raises(ValueError, match=PENDING):
            task._freeze_execution_plan(c.plan)
        with pytest.raises(ValueError):
            task._freeze_execution_plan(contact)
        retry = task._before_near_field_contact_freeze(contact, c.plan, {})
        assert retry == dict(retry=True,
            minimum_stamp_ns=max(1, c.plan.header.stamp.to_nsec()+1),
            phase_deadline_sec=phase_deadline)
        unchanged_authority()

    row, source, _ = contract_test.overlap_frame(c, monkeypatch)
    bad_ns = source[0].header.stamp.to_nsec()
    contract_test.wire(c, row)
    pending_entries_reject()
    assert bad_ns not in task._station_contact_sources

    ready = contract_test.fresh_frame(c)
    contract_test.wire(c, ready, ('status', 'object'))
    pending_entries_reject()
    assert bad_ns not in task._station_contact_sources

    contract_test.wire(c, ready, ('mask',))
    assert not c.node._station_compute_pending and not task._station_task_record['pending']
    assert task.latest_obj.header.stamp == ready['object'].header.stamp
    assert bad_ns not in task._station_contact_sources
    result, response = task._invoke_plan_bound_action(
        c.plan, {}, 'strict pregrasp', action)
    assert not result.ok and result.code == PENDING
    assert result.reason == 'old observation is historical while waiting for contact'
    assert response is None
    unchanged_authority()
