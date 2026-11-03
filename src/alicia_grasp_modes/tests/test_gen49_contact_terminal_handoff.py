"""Actual Task callbacks/wait consume a bound failure once despite clock noise.

RGB-D ownership witnesses come from the original three-topic join. Transport,
clocks and SDK feedback are inert providers; no path or physical success is
claimed. The unchanged execution wait, identity checks and stop path run.
"""
from copy import deepcopy
from itertools import permutations
import json
from unittest.mock import Mock

import pytest

from alicia_grasp_modes.observation_search_failure import CONTACT_CODE, CONTACT_POLICY
from test_station_compute_wait_task import (
    station, configured, module, base, frame, feedback, deliver, advance, at_ns)


def offer(c, order=('object', 'mask', 'status')):
    stamps = [100_000_000_000, 100_200_000_000, 100_400_000_000]
    for stamp in stamps:
        c.now[0] = stamp+50_000_000
        c.mono[0] = 10.+(c.now[0]-100_050_000_000)/1e9
        feedback(c, stamp-20_000_000)
        feedback(c, stamp+20_000_000)
        deliver(c, frame(c, stamp), order)
    ledger = c.node._station_contact_sources
    assert all(stamp in ledger for stamp in stamps)
    witnesses = [deepcopy(ledger[stamp]) for stamp in stamps]
    contract = c.node._station_task_record['contract']
    receipt = dict(version=1, policy=CONTACT_POLICY, execution_authority=False,
        metric_geometry_authority=False, code=CONTACT_CODE,
        reason='all 19 checked contact candidates failed original endpoint collision checks',
        binding=deepcopy(contract['binding']),
        phase_deadline_ns=contract['phase_deadline_ns'],
        session_deadline_ns=contract['session_deadline_ns'],
        request=dict(request_id=22, generation=7, target_epoch=4,
                     snapshot_stamp_ns=stamps[-1]), emitted_ns=c.now[0],
        sources=witnesses, current=deepcopy(witnesses[-1]),
        search=dict(checked_count=19, shortlist_count=19, reachable_count=0,
                    selected=False, terminated_early=False), audit_sha256='e'*64)
    p = base.Grasp6DPlan()
    p.header.stamp = at_ns(receipt['emitted_ns']); p.header.frame_id = 'base_link'
    p.valid = False; p.candidate_source = 'station_contact_search_terminal'
    p.target_track_id = contract['binding']['target_track_id']
    p.diagnostic = CONTACT_CODE+': '+json.dumps(receipt, sort_keys=True, separators=(',', ':'))
    return p, receipt


def clock_preview(c):
    p = base.Grasp6DPlan()
    p.header.stamp = at_ns(c.now[0]); p.header.frame_id = 'base_link'
    p.valid = False; p.candidate_source = 'near_field_pending'
    p.diagnostic = 'TARGET_PROCESSING_PENDING: waiting for a new independently measured frame'
    return p


@pytest.mark.parametrize('order', list(permutations(('object', 'mask', 'status'))))
@pytest.mark.parametrize('delivery', ['terminal_then_clock', 'clock_then_terminal', 'duplicates', 'delayed_terminal'])
def test_bound_terminal_reaches_actual_task_stop_across_pending_message_order(station, order, delivery):
    c = station; terminal, _receipt = offer(c, order)
    if delivery in ('terminal_then_clock', 'duplicates'):
        c.node.grasp6d_preview_plan_cb(terminal)
    advance(c, 101_300_000_000)
    assert c.node._station_task_record['pending'] and c.node.latest_obj is None
    c.node.grasp6d_preview_plan_cb(clock_preview(c))
    if delivery in ('clock_then_terminal', 'delayed_terminal', 'duplicates'):
        c.node.grasp6d_preview_plan_cb(deepcopy(terminal))
    if delivery == 'duplicates':
        c.node.grasp6d_preview_plan_cb(clock_preview(c))
        c.node.grasp6d_preview_plan_cb(deepcopy(terminal))
    frozen_plan = deepcopy(c.node._bound_execution_plan)
    contract = deepcopy(c.node._station_task_record['contract'])
    states = []; c.node.set_state = lambda *args: states.append(args)
    c.node._set_near_field_preview_stream = Mock(return_value=True)
    c.node._freeze_execution_plan = Mock(side_effect=AssertionError('failure cannot freeze a plan'))
    move, execute = Mock(), Mock()
    cfg = dict(near_field_replan_enabled=True, near_field_replan_required=True,
               near_field_strategy='single_snapshot_direct')
    assert c.node._maybe_rebind_near_field_grasp6d_plan(
        cfg, {}, c.plan, move_pose=move, strict_execute_pose=execute) is None
    assert states[-1][0] == base.GraspStages.FAILED
    assert states[-1][1].startswith(CONTACT_CODE+':')
    assert 'TARGET_LOST' not in states[-1][1]
    c.node._set_near_field_preview_stream.assert_called_once_with(cfg, False)
    c.node._freeze_execution_plan.assert_not_called()
    move.assert_not_called(); execute.assert_not_called()
    assert c.node._bound_execution_plan == frozen_plan
    assert c.node._station_task_record['contract'] == contract
    assert not c.node._execution_authority_revoked


@pytest.mark.parametrize('fault', ['selection', 'session', 'phase', 'epoch', 'deadline',
    'source_digest', 'source_order', 'source_missing', 'source_future', 'unknown_current',
    'unchecked', 'reachable', 'early_exit', 'selected', 'header', 'track', 'authority',
    'poses', 'plan_id', 'opening', 'contact_evidence', 'unbound_legacy'])
def test_foreign_incomplete_or_executable_terminal_cannot_end_current_wait(station, fault):
    c = station; p, receipt = offer(c)
    if fault == 'selection': receipt['binding']['selection']['generation'] += 1
    elif fault == 'session': receipt['binding']['session_id'] = 'other'
    elif fault == 'phase': receipt['binding']['phase_id'] += 1
    elif fault == 'epoch': receipt['binding']['control_epoch_ns'] += 1
    elif fault == 'deadline': receipt['phase_deadline_ns'] += 1
    elif fault == 'source_digest': receipt['sources'][0]['object_sha256'] = '0'*64
    elif fault == 'source_order': receipt['sources'].reverse()
    elif fault == 'source_missing': receipt['sources'].pop(0)
    elif fault == 'source_future': receipt['request']['snapshot_stamp_ns'] += 1
    elif fault == 'unknown_current': receipt['current']['mask']['sha256'] = '0'*64
    elif fault == 'unchecked': receipt['search']['checked_count'] -= 1
    elif fault == 'reachable': receipt['search']['reachable_count'] = 1
    elif fault == 'early_exit': receipt['search']['terminated_early'] = True
    elif fault == 'selected': receipt['search']['selected'] = True
    elif fault == 'header': p.header.stamp = at_ns(receipt['emitted_ns']+1)
    elif fault == 'track': p.target_track_id = 'other'
    elif fault == 'authority': receipt['execution_authority'] = True
    elif fault == 'poses': p.poses = [c.plan.object_geometry.pose_base]
    elif fault == 'plan_id': p.plan_id = 'invented'
    elif fault == 'opening': p.required_open_width_m = .04
    elif fault == 'contact_evidence': p.contact_geometry_evidence = 'invented'
    p.diagnostic = CONTACT_CODE+': '+json.dumps(receipt, sort_keys=True)
    if fault == 'unbound_legacy': p.diagnostic = CONTACT_CODE+': original unbound text'
    c.node.grasp6d_preview_plan_cb(p)
    advance(c, 101_300_000_000)
    c.node.grasp6d_preview_plan_cb(clock_preview(c))
    assert c.node._near_field_wait_failure(c.plan, {}) is None
    assert not c.node._execution_authority_revoked


@pytest.mark.parametrize('fault', ['manual', 'selection', 'phase', 'epoch', 'moved', 'actual_target_loss'])
def test_current_control_or_target_revocation_keeps_priority_over_old_failure(station, fault):
    c = station; p, _ = offer(c)
    c.node.grasp6d_preview_plan_cb(p)
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'selection': c.params[module.SELECTION_PARAM]['generation'] += 1
    elif fault == 'phase': c.node._near_field_phase_id += 1
    elif fault == 'epoch': c.node._following_epoch_ns += 1
    elif fault == 'moved': feedback(c, c.now[0], .01)
    elif fault == 'actual_target_loss':
        c.node._unknown_target_loss = dict(selection=deepcopy(c.selection),
            reason='TARGET_LOST: independently observed target moved')
    result = c.node._near_field_wait_failure(c.plan, {})
    assert result is None or result.code != CONTACT_CODE


def test_phase_deadline_does_not_get_renewed_by_replayed_failure(station):
    c = station; p, _ = offer(c)
    c.node.grasp6d_preview_plan_cb(p)
    advance(c, 190_000_000_000)
    assert c.node._station_task_retired
    c.node.grasp6d_preview_plan_cb(deepcopy(p))
    result = c.node._near_field_wait_failure(c.plan, {})
    assert result is not None and result.code == 'TARGET_LOST'
    assert c.node._station_task_record is None and c.node._execution_authority_revoked


def actor_terminal(c):
    """Negative receipt built from original actors' independently joined rows."""
    contract = c.task._station_task_record['contract']
    witnesses = [deepcopy(c.task._station_contact_sources[stamp])
                 for stamp in c.ticket.payload[0].sample_stamp_ns]
    now = base.rospy.Time.now().to_nsec()
    receipt = dict(version=1, policy=CONTACT_POLICY, execution_authority=False,
        metric_geometry_authority=False, code=CONTACT_CODE,
        reason='recorded negative search input used only to exercise message ownership',
        binding=deepcopy(contract['binding']),
        phase_deadline_ns=contract['phase_deadline_ns'],
        session_deadline_ns=contract['session_deadline_ns'],
        request=dict(request_id=int(c.ticket.request_id), generation=int(c.ticket.generation),
                     target_epoch=int(c.ticket.target_epoch),
                     snapshot_stamp_ns=int(c.ticket.payload[0].stamp_ns)),
        emitted_ns=now, sources=witnesses, current=deepcopy(witnesses[-1]),
        search=dict(checked_count=19, shortlist_count=19, reachable_count=0,
                    selected=False, terminated_early=False), audit_sha256='e'*64)
    p = base.Grasp6DPlan()
    p.header.stamp = at_ns(now); p.header.frame_id = 'base_link'
    p.valid = False; p.candidate_source = 'station_contact_search_terminal'
    p.target_track_id = contract['binding']['target_track_id']
    p.diagnostic = CONTACT_CODE+': '+json.dumps(receipt, sort_keys=True)
    return p, receipt


@pytest.mark.parametrize('when', ['before_audit', 'after_audit'])
@pytest.mark.parametrize('fault', ['none', 'foreign_owner', 'bad_json', 'legacy_text', 'missing_sources', 'executable'])
def test_delayed_typed_terminal_cannot_erase_original_admitted_contact(monkeypatch, tmp_path, when, fault):
    from test_gen11_contact_compute_admission_independent import (
        context, computed, audit, copy_candidate, freeze, retirement)
    c = context(monkeypatch)
    p, receipt = actor_terminal(c)
    if fault == 'foreign_owner': receipt['binding']['session_id'] = 'foreign'
    elif fault == 'missing_sources': receipt.pop('sources')
    p.diagnostic = CONTACT_CODE+': '+json.dumps(receipt, sort_keys=True)
    if fault == 'bad_json': p.diagnostic = CONTACT_CODE+': {'
    elif fault == 'legacy_text': p.diagnostic = CONTACT_CODE+': old unbound terminal text'
    elif fault == 'executable': p.valid = True; p.poses = deepcopy(c.contact.poses)
    computed(c)
    original = deepcopy(c.task._station_contact_proposal)
    assert original == c.contact
    if when == 'before_audit':
        c.task.grasp6d_preview_plan_cb(deepcopy(p))
    audit(c, tmp_path)
    # Duplicate late delivery remains inert even after atomic audit acceptance.
    c.task.grasp6d_preview_plan_cb(deepcopy(p))
    c.task.grasp6d_preview_plan_cb(deepcopy(p))
    assert c.task._station_contact_proposal == original
    assert c.task._near_field_wait_failure(c.observation, c.gcfg) is None
    validation, plan = copy_candidate(c)
    assert validation.ok, (validation.code, validation.reason)
    assert plan == c.contact
    frozen = freeze(c)
    entered = retirement(c, frozen)
    assert len(entered) == 1


def test_original_valid_contact_supersedes_prior_failure_without_new_motion_request(monkeypatch, tmp_path):
    from test_gen11_contact_compute_admission_independent import (
        context, computed, audit, copy_candidate, freeze, retirement)
    c = context(monkeypatch)
    p, _ = actor_terminal(c)
    c.task.grasp6d_preview_plan_cb(p)
    assert c.task._station_contact_search_terminal is not None
    computed(c)
    assert c.task._station_contact_search_terminal is None
    audit(c, tmp_path)
    c.task.grasp6d_preview_plan_cb(deepcopy(p))
    validation, plan = copy_candidate(c)
    assert validation.ok, (validation.code, validation.reason)
    assert plan == c.contact
    frozen = freeze(c)
    entered = retirement(c, frozen)
    assert len(entered) == 1
