"""Real builder/finalizer/task audit handoff with inert physical checking.

Synthetic surfaces are unit inputs, not recorded grasp success. The dedicated
sealed-record case below reports only fields actually preserved in gen13.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.active_observation_policy import evidence_digest, registered_information_gain
from alicia_flexible_grasp.robot.observation_view_limits import MINIMUM_OBSERVATION_IMAGE_MARGIN_PX
from alicia_grasp_modes.runner_contract import (
    observe_return_audit_error, supplemental_observation_audit_error)
from test_remote_modes import configured, base
from test_silhouette_observation_planning import context as planner_context
from test_observe_return_task import context as task_context


def continuation(configured, monkeypatch, tmp_path):
    # The task fixture installs its own parameter transport. Restore the
    # planner's existing fixture before its real selection/phase transaction.
    monkeypatch.setattr(base.rospy, 'get_param', lambda key, default=None: configured[1].get(key, default))
    node, prepared, checks, variant = planner_context(configured, monkeypatch, tmp_path, ordinary=True)
    identity = prepared.snapshot.target_identity
    initial = node._observe_return_initial_stamp
    failure_stamp = initial + 1_000_000
    before = NS(identity=identity, points_base=np.array([[0., y, .02] for y in (.001, .011, .021)]),
                view_stamps_ns=(initial,))
    after = NS(identity=identity, points_base=np.vstack((before.points_base, before.points_base+[.02, 0, 0])),
               view_stamps_ns=(initial, failure_stamp), view_indices=np.array([0, 0, 0, 1, 1, 1]))
    gain = registered_information_gain(before, after, NS(ok=True), depth_uncertainty_m=.002)
    failure = dict(request_id=prepared.ticket.request_id-1, generation=prepared.ticket.generation,
        target_epoch=identity.epoch, snapshot_stamp_ns=failure_stamp,
        contact_search_code='NEAR_FIELD_NO_REACHABLE_CANDIDATE', selected=False,
        terminated_early=False, checked_count=2, shortlist_count=2, reachable_count=0,
        locally_valid=dict(entered=3, passed=2, rejected=1), contact_or_motion_authorized=False)
    opportunity = {key: failure[key] for key in ('request_id', 'generation', 'target_epoch',
        'snapshot_stamp_ns', 'contact_search_code', 'checked_count')}
    opportunity.update(policy='current_request_independent_bilateral_view_gap',
        missing_surface_direction=deepcopy(node._observe_return_view_gaps[0]), contact_or_motion_authorized=False)
    session = dict(version=2, phase='observing', selection=deepcopy(node._mode_selection),
        session_id='a'*64, origin_plan_id='b'*24, origin_snapshot_stamp_ns=initial,
        view_index=1, view_plan_id='b'*24, view_source_stamp_ns=initial,
        target_epoch=identity.epoch, target_track_id=identity.track_id,
        max_observations=3, deadline_ns=initial+180_000_000_000)
    pending = {key: session[key] for key in ('session_id', 'origin_plan_id', 'origin_snapshot_stamp_ns',
        'target_epoch', 'target_track_id', 'max_observations', 'deadline_ns')}
    pending.update(view_index=2, previous_view_index=1, failed_search=failure, visual_opportunity=opportunity,
        previous_gain=gain, previous_gain_sha256=evidence_digest(gain), contact_or_motion_authorized=False)
    pending['view_request_id'] = evidence_digest(pending)
    node._observe_return_session = session
    node._observe_return_pending_supplement = pending
    node._observe_return_trigger_evidence = deepcopy(opportunity)
    # Actual task activity blocks generic auto-promotion; the separately typed
    # preview audit must nevertheless reach the existing task handoff.
    node.robot_execution_active = True
    return node, prepared, checks, variant


def build(configured, monkeypatch, tmp_path):
    node, prepared, checks, variant = continuation(configured, monkeypatch, tmp_path)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'PREVIEW_READY'
    assert checks and all(observation for _, observation in checks)
    assert (tmp_path/'planning.json').exists(), 'selected supplement lost its audit during non-promotion'
    report = json.loads((tmp_path/'planning.json').read_text())
    assert report['observe_return']['phase'] == 'supplemental_observation'
    return node, prepared, report


def check(node, report):
    return supplemental_observation_audit_error(report, session=node._observe_return_session,
        pending_request=node._observe_return_pending_supplement, now_ns=int(node._ros_source_clock_ns()))


def test_selected_supplement_writes_audit_without_generic_promotion_and_task_consumes_it(
        configured, task_context, monkeypatch, tmp_path):
    node, prepared, report = build(configured, monkeypatch, tmp_path)
    assert check(node, report) == ''
    assert observe_return_audit_error(report) == 'observe_return phase is not an initial attempt entry'
    assert observe_return_audit_error(report, allow_returned=True)
    assert not node._latest_promotion_decision.promote
    assert report['pipeline_funnel']['stage_counts']['promoted']['passed'] == 0
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not (tmp_path/'planning.json.execution').exists()
    assert node.gate_audit_pub.messages
    consumer = task_context.node
    consumer._mode_bound_selection = deepcopy(node._mode_selection)
    consumer._observe_return_session = deepcopy(node._observe_return_session)
    consumer._observe_return_audit_cb(node.gate_audit_pub.messages[-1])
    plan = node.latest_preview_rich_plan
    loaded = consumer._observe_return_reports[plan.plan_id]
    assert loaded == report
    metadata = consumer._observe_return_supplement_report(plan, loaded)
    assert metadata['view_index'] == 2
    assert metadata['deadline_ns'] == node._observe_return_session['deadline_ns']
    consumer._observe_return_session['session_id'] = 'foreign'
    with pytest.raises(ValueError, match='SUPPLEMENTAL_AUDIT_INVALID'):
        consumer._observe_return_supplement_report(plan, loaded)


@pytest.mark.parametrize('fault', [
    'missing_session', 'v1', 'returned', 'selection', 'missing_pending', 'pending_digest',
    'session_id', 'origin_plan', 'origin_stamp', 'epoch', 'track', 'view_index',
    'previous_view', 'max_views', 'deadline', 'expired', 'old_source', 'future_source',
    'gain_digest', 'gain_failed', 'failure_incomplete', 'failure_checked', 'opportunity',
    'role', 'source_binding', 'gap', 'predicted_gain', 'strict', 'fov', 'support', 'initial_phase'])
def test_supplement_rejects_changed_context_or_missing_original_gate(
        configured, monkeypatch, tmp_path, fault):
    node, _, report = build(configured, monkeypatch, tmp_path)
    metadata = report['observe_return']
    session = node._observe_return_session
    if fault == 'missing_session': node._observe_return_session = None
    elif fault == 'v1': session['version'] = 1
    elif fault == 'returned': session['phase'] = 'returned'
    elif fault == 'selection': session['selection']['generation'] += 1
    elif fault == 'missing_pending': node._observe_return_pending_supplement = None
    elif fault == 'pending_digest': node._observe_return_pending_supplement['view_request_id'] = '0'*64
    elif fault == 'session_id': session['session_id'] = 'foreign'
    elif fault == 'origin_plan': session['origin_plan_id'] = 'f'*24
    elif fault == 'origin_stamp': session['origin_snapshot_stamp_ns'] += 1
    elif fault == 'epoch': session['target_epoch'] += 1
    elif fault == 'track': session['target_track_id'] += '-other'
    elif fault == 'view_index': session['view_index'] = 2
    elif fault == 'previous_view': metadata['previous_view_index'] = 0
    elif fault == 'max_views': session['max_observations'] = 1
    elif fault == 'deadline': session['deadline_ns'] += 1
    elif fault == 'expired': node._ros_source_clock_ns = lambda: session['deadline_ns']
    elif fault == 'old_source': report['snapshot_stamp_ns'] = metadata['failed_search']['snapshot_stamp_ns']
    elif fault == 'future_source': node._ros_source_clock_ns = lambda: report['snapshot_stamp_ns']-1
    elif fault == 'gain_digest': metadata['previous_gain_sha256'] = '0'*64
    elif fault == 'gain_failed': metadata['previous_gain']['registration_ok'] = False
    elif fault == 'failure_incomplete': metadata['failed_search']['terminated_early'] = True
    elif fault == 'failure_checked': metadata['failed_search']['checked_count'] = 0
    elif fault == 'opportunity': metadata['visual_opportunity']['request_id'] += 1
    elif fault == 'role': report['selected']['candidate_role'] = 'contact'
    elif fault == 'source_binding': report['selected']['observation_source_binding']['snapshot_stamp_ns'] -= 1
    elif fault == 'gap': metadata['measured_missing_sides'] = []
    elif fault == 'predicted_gain': report['selected']['observation_view']['observe_return_missing_side_gain']['ok'] = False
    elif fault == 'strict': report['selected']['moveit']['collision_free'] = False
    elif fault == 'fov': report['selected']['observation_view']['working_volume']['minimum_image_edge_distance_px'] = MINIMUM_OBSERVATION_IMAGE_MARGIN_PX - 1.
    elif fault == 'support': report['selected']['observation_envelope']['minimum_support_clearance_m'] = .002
    elif fault == 'initial_phase': metadata['phase'] = 'observation_prefix'
    assert check(node, report)


def test_invalid_pending_still_never_publishes_a_successful_audit_or_execution(
        configured, monkeypatch, tmp_path):
    node, prepared, _, _ = continuation(configured, monkeypatch, tmp_path)
    node._observe_return_pending_supplement['view_request_id'] = '0'*64
    node._accept_prediction(prepared)
    # The existing generic finalizer may retain a preview, but an invalid
    # continuation must never acquire the audit needed for task admission.
    assert not (tmp_path/'planning.json').exists()
    assert not node.gate_audit_pub.messages
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_real_gen13_record_preserves_successful_supplement_trigger_but_no_supplement_audit():
    path = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
        'full_latest_ros_20261009T003407Z/gen13_handoff_analysis/arrival_and_terminal.json')
    if not path.exists(): pytest.skip('sealed local gen13 evidence is unavailable')
    record = json.loads(path.read_text())
    # Validate the genuine record's shape in the dedicated evidence report;
    # private pending/gain were never emitted and are not reconstructed here.
    assert record['arrival_passed'] is True
    assert record['supplement_required_event']['message'].startswith('OBSERVE_RETURN_SUPPLEMENTAL_VIEW_REQUIRED ')
    assert record['supplement_preview_count'] == 5
    assert len(record['sessions']) == 1
    assert record['sessions'][0]['message']['view_index'] == 1
    assert record['gate_audit_count'] == 4
    assert not any(row['message'].get('observe_return', {}) and
        row['message']['observe_return'].get('phase') == 'supplemental_observation'
        for row in record['gate_audit_summaries'])
    assert record['private_state_limitations']
    assert record['contact_or_close_started'] is False
    assert record['physical_grasp_verified'] is False
