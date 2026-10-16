"""A measured station may consume failure while its metric frame is pending.

The real task wait and owner checks run with inert ROS transport and clocks.
No test treats a planning failure as a pose or motion authorization.
"""
from copy import deepcopy
import json
from unittest.mock import Mock

import pytest

from alicia_grasp_modes.observation_search_failure import CODE, SOURCE, POLICY, REASON
from test_station_compute_wait_task import (
    station, configured, module, base, frame, feedback, deliver, advance, at_ns)


def offer(c):
    c.now[0] = 188_050_000_000
    c.mono[0] = 98.
    feedback(c, 187_980_000_000)
    feedback(c, 188_020_000_000)
    f = frame(c, 188_000_000_000)
    deliver(c, f)
    contract = f['status']['station_compute_wait']
    receipt = dict(version=1, policy=POLICY, execution_authority=False,
        reason=REASON, binding=deepcopy(contract['binding']),
        phase_deadline_ns=contract['phase_deadline_ns'],
        session_deadline_ns=contract['session_deadline_ns'],
        request=dict(request_id=87, generation=7,
            target_epoch=contract['binding']['target_epoch'], snapshot_stamp_ns=188_000_000_000),
        emitted_ns=c.now[0], remaining_sec=1.95, estimated_strict_sec=1.65,
        publication_reserve_sec=.9, attempted_count=39, unchecked_count=22, timeout_count=39)
    p = base.Grasp6DPlan()
    p.header.stamp = at_ns(receipt['emitted_ns'])
    p.header.frame_id = 'base_link'
    p.valid = False
    p.candidate_source = SOURCE
    p.target_track_id = contract['binding']['target_track_id']
    p.diagnostic = CODE + ': ' + json.dumps(receipt, sort_keys=True)
    c.node.latest_grasp6d_preview_plan = p
    return p, receipt


@pytest.mark.parametrize('pending', [False, True])
def test_actual_wait_consumes_bound_failure_without_waiting_for_new_geometry(station, pending):
    c = station
    preview, _ = offer(c)
    c.node.latest_grasp6d_preview_plan = None
    c.node.grasp6d_preview_plan_cb(preview)
    assert c.node.latest_grasp6d_preview_plan == preview
    if pending:
        advance(c, 188_900_000_000)
        assert c.node._station_task_record['pending'] and c.node.latest_obj is None
    frozen = deepcopy(c.node._bound_execution_plan)
    contract = deepcopy(c.node._station_task_record['contract'])
    states = []
    c.node.set_state = lambda *args: states.append(args)
    c.node._set_near_field_preview_stream = Mock(return_value=True)
    c.node._freeze_execution_plan = Mock(side_effect=AssertionError('failure cannot freeze'))
    move, execute = Mock(), Mock()
    cfg = dict(near_field_replan_enabled=True, near_field_replan_required=True,
        near_field_strategy='single_snapshot_direct')
    assert c.node._maybe_rebind_near_field_grasp6d_plan(
        cfg, {}, c.plan, move_pose=move, strict_execute_pose=execute) is None
    assert states[-1][0] == base.GraspStages.FAILED
    assert states[-1][1].startswith(CODE + ':')
    assert 'TARGET_LOST' not in states[-1][1]
    assert 'attempted=39 unchecked=22 timeouts=39' in states[-1][1]
    c.node._set_near_field_preview_stream.assert_called_once_with(cfg, False)
    c.node._freeze_execution_plan.assert_not_called()
    move.assert_not_called()
    execute.assert_not_called()
    assert c.node._bound_execution_plan == frozen
    assert c.node._station_task_record['contract'] == contract
    assert not c.node._execution_authority_revoked


@pytest.mark.parametrize('fault', [
    'foreign_selection', 'foreign_session', 'foreign_phase', 'foreign_epoch',
    'deadline', 'header', 'valid', 'poses', 'plan_id', 'contact', 'width',
    'code', 'source', 'frame', 'version_bool', 'nan_budget', 'not_exhausted',
    'wrong_remaining', 'negative_count', 'more_timeouts_than_checks',
    'request_epoch', 'old_request', 'extra_field', 'bad_json', 'wrong_track'])
def test_malformed_or_foreign_receipt_cannot_classify_current_wait(station, fault):
    c = station
    p, receipt = offer(c)
    if fault == 'foreign_selection': receipt['binding']['selection']['generation'] += 1
    elif fault == 'foreign_session': receipt['binding']['session_id'] = 'other'
    elif fault == 'foreign_phase': receipt['binding']['phase_id'] += 1
    elif fault == 'foreign_epoch': receipt['binding']['control_epoch_ns'] += 1
    elif fault == 'deadline': receipt['phase_deadline_ns'] += 1
    elif fault == 'header': p.header.stamp = at_ns(receipt['emitted_ns']+1)
    elif fault == 'valid': p.valid = True
    elif fault == 'poses': p.poses = [c.plan.object_geometry.pose_base]
    elif fault == 'plan_id': p.plan_id = 'looks-executable'
    elif fault == 'contact': p.contact_geometry_evidence = 'invented'
    elif fault == 'width': p.required_open_width_m = .04
    elif fault == 'source': p.candidate_source = 'near_field_terminal'
    elif fault == 'frame': p.header.frame_id = 'other'
    elif fault == 'wrong_track': p.target_track_id = 'other'
    elif fault == 'version_bool': receipt['version'] = True
    elif fault == 'nan_budget': receipt['estimated_strict_sec'] = float('nan')
    elif fault == 'not_exhausted': receipt['estimated_strict_sec'] = .1
    elif fault == 'wrong_remaining': receipt['remaining_sec'] = 1.
    elif fault == 'negative_count': receipt['unchecked_count'] = -1
    elif fault == 'more_timeouts_than_checks': receipt['timeout_count'] = 40
    elif fault == 'request_epoch': receipt['request']['target_epoch'] += 1
    elif fault == 'old_request': receipt['request']['snapshot_stamp_ns'] = 99_000_000_000
    elif fault == 'extra_field': receipt['motion_authorized'] = True
    p.diagnostic = CODE + ': ' + json.dumps(receipt, sort_keys=True)
    if fault == 'bad_json': p.diagnostic = CODE + ': {'
    if fault == 'code': p.diagnostic = 'OTHER_FAILURE: ' + json.dumps(receipt)
    assert c.node._near_field_wait_failure(c.plan, {}) is None
    assert not c.node._execution_authority_revoked


@pytest.mark.parametrize('change', ['manual', 'selection', 'phase', 'epoch', 'moved', 'contact'])
def test_current_owner_changes_cannot_accept_old_budget_failure(station, change):
    c = station
    offer(c)
    if change == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif change == 'selection': c.params[module.SELECTION_PARAM]['generation'] += 1
    elif change == 'phase': c.node._near_field_phase_id += 1
    elif change == 'epoch': c.node._following_epoch_ns += 1
    elif change == 'moved': feedback(c, c.now[0], .01)
    elif change == 'contact': c.plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    result = c.node._near_field_wait_failure(c.plan, {})
    if change == 'moved':
        # The real feedback callback already retired the station; retain its
        # original failure rather than misclassifying it as search exhaustion.
        assert result.code == 'TARGET_LOST' and result.reason == 'station_task_pose_changed'
    else:
        assert result is None


def test_real_target_loss_precedes_search_budget_diagnostic(station):
    c = station
    offer(c)
    c.node._unknown_target_loss = dict(selection=deepcopy(c.selection),
        reason='TARGET_LOST: independently observed target moved')
    result = c.node._near_field_wait_failure(c.plan, {})
    assert result.code == 'TARGET_LOST'
    assert result.reason == 'independently observed target moved'


def test_expired_station_is_not_reopened_by_budget_diagnostic(station):
    c = station
    offer(c)
    advance(c, 190_000_000_000)
    assert c.node._station_task_retired
    assert c.node._station_task_record is None
    result = c.node._near_field_wait_failure(c.plan, {})
    assert result.code == 'TARGET_LOST'
    assert c.node._execution_authority_revoked
