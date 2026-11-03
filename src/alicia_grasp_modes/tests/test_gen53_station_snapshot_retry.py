"""Real stationary source callbacks, worker cleanup, polling and Task wait.

The analytic RGB-D producer and all station ownership/freshness gates are
original. Planning outcomes are explicit simulated zero-candidate results;
this file does not claim a new feasible grasp or a historical physical replay.
"""
from copy import deepcopy
import json
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from alicia_grasp_modes import station_snapshot_retry as policy
import test_gen35_minimal_chain_replay as chain
import test_gen41_pending_handoff_chain as wire
from test_gen38_cuboid_chain import cuboid_scene
from test_gen39_registration_chain import _install_real_reached_source_adapter
from test_gen42_geometry_lock_handoff import _fresh_triplet
from test_task_modes import base as task_base


class _StationReady(BaseException):
    pass


@pytest.fixture(autouse=True)
def forbid_external(monkeypatch):
    import socket
    import rospy
    def forbidden(*args, **kwargs):
        raise chain.UnsimulatedExternalCall('GEN53 unsimulated boundary '+repr(args))
    for name in ('connect', 'connect_ex', 'sendto', 'bind', 'listen'):
        monkeypatch.setattr(socket.socket, name, forbidden)
    monkeypatch.setattr(socket, 'create_connection', forbidden)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
                 'wait_for_service', 'set_param', 'delete_param'):
        monkeypatch.setattr(rospy, name, forbidden)


def _context(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'runtime_logs'))
    monkeypatch.setattr(chain, 'ray_scene', cuboid_scene)
    _install_real_reached_source_adapter(monkeypatch)
    c = chain.arrived_same_scene(monkeypatch)
    original = chain.station_samples
    def station_only(context):
        original(context)
        raise _StationReady()
    with monkeypatch.context() as patch:
        patch.setattr(chain, 'station_samples', station_only)
        with pytest.raises(_StationReady):
            chain.same_source_contact(c)
    c.gen41_sdk_provider = []
    return c


def _report(n, ticket):
    """Explicit computation-output provider, not reconstructed historical geometry."""
    return dict(grasp_mode_selection=deepcopy(n._mode_selection),
        request_id=ticket.request_id, generation=ticket.generation,
        target_epoch=ticket.target_epoch, snapshot_stamp_ns=ticket.payload[0].stamp_ns,
        selected=None, outcome=dict(code=policy.FAILURE, valid_plan=False, preview_valid=False),
        moveit_selection=dict(checked_count=0, shortlist_count=0, reachable_count=0,
                              terminated_early=False),
        pipeline_funnel=dict(stage_counts=dict(locally_valid=dict(entered=12, passed=0, rejected=12)),
            rejection_counts=dict(BILATERAL_SURFACE_EVIDENCE_MISSING=12)),
        remote_diagnostics=dict(tabletop_geometry=dict(
            failure_code='BILATERAL_SURFACE_EVIDENCE_MISSING', plan_phase='CONTACT_EXECUTION_PLAN',
            contact_execution_gate_deferred=False, materialized_count=0,
            regular_box_inference=dict(code='BOX_TOP_CONTOUR_INCOMPLETE'),
            rejection_counts=dict(BILATERAL_SURFACE_EVIDENCE_MISSING=32))))


@pytest.mark.parametrize('order', [('status', 'object', 'mask'), ('mask', 'object', 'status')])
@pytest.mark.parametrize('delivery', ['normal', 'duplicates_and_delayed'])
@pytest.mark.parametrize('second_outcome', ['terminal', 'expired', 'cancelled'])
def test_original_worker_gets_one_new_three_frame_window_then_task_keeps_second_failure(
        monkeypatch, tmp_path, order, delivery, second_outcome):
    c = _context(monkeypatch, tmp_path)
    n, task, first = c.remote, c.task, c.ticket
    saved = (n._near_field_fused_surface, n._near_field_reference_view, n._observe_return_saved)
    phase = (n._stream_generation, n._near_field_phase_id, n._near_field_phase_deadline_sec,
             deepcopy(n._observe_return_session), n._current_stream_target_identity())
    watermark = n.last_submitted_stamp_ns
    released = threading.Event(); second_finished = threading.Event()
    errors, releases, notices, tickets = [], [], [], []
    original_release = n._release_expired_direct_snapshot_locked
    original_publish = n.preview_rich_plan_pub.publish

    def publish(msg):
        original_publish(msg)
        task.grasp6d_preview_plan_cb(deepcopy(msg))
        if msg.candidate_source == policy.SOURCE:
            notices.append(deepcopy(msg))
            if delivery == 'duplicates_and_delayed':
                task.grasp6d_preview_plan_cb(deepcopy(msg))

    def prepare(ticket):
        n._require_stream_ticket_current(ticket)
        tickets.append(ticket)
        if ticket is not first and second_outcome == 'cancelled':
            raise n._observe_return_original.StreamResultCancelled('TARGET_PROCESSING_PENDING')
        return NS(ticket=ticket, snapshot=ticket.payload[0], remote_failure_code='',
                  remote_performance={}, remote_diagnostics={}, ros_prepare_ms=0.)

    def accept(prepared):
        n._active_gate_audit_report = _report(n, prepared.ticket)
        n._latest_gate_audit_reference = dict(report_sha256='a'*64)
        n._publish_direct_near_field_terminal(prepared, policy.FAILURE,
                                            'explicit simulated incomplete surface result')
        return dict(status=policy.FAILURE, funnel={})

    def release(ticket, status):
        assert n._stream_condition._is_owned() and not n._stream_worker_busy
        result = original_release(ticket, status)
        releases.append((ticket.request_id, result))
        (released if ticket is first else second_finished).set()
        return result

    monkeypatch.setattr(n.preview_rich_plan_pub, 'publish', publish)
    monkeypatch.setattr(n, '_prepare_and_predict', prepare)
    monkeypatch.setattr(n, '_accept_prediction', accept)
    monkeypatch.setattr(n, '_release_expired_direct_snapshot_locked', release)
    def run():
        try:
            n._stream_worker_main()
        except BaseException as exc:
            errors.append(exc)
    worker = threading.Thread(target=run, name='gen53-original-worker')
    n._stream_worker = worker; worker.start()
    try:
        assert released.wait(10.), errors
        assert not errors and releases == [(first.request_id, True)]
        assert len(notices) == 1
        result, candidate = task._copy_near_field_preview_candidate(c.origin, 0, {})
        assert result.code == policy.CODE and not result.terminal_for_request and candidate is None
        assert n._poll_direct_snapshot() is False
        with n._stream_condition:
            assert not original_release(first, policy.FAILURE), 'duplicate cleanup reopened slot'
        assert watermark == n.last_submitted_stamp_ns
        assert all(a is b for a, b in zip(saved,
            (n._near_field_fused_surface, n._near_field_reference_view, n._observe_return_saved)))
        if second_outcome == 'expired':
            # Explicit old-result-clock provider: source age is genuinely
            # greater than this test's inference completion lifetime.
            n.inference_coordinator._result_max_age_sec = .05
        for index in range(3):
            row = _fresh_triplet(c)
            wire._deliver(c, row, order)
            if delivery == 'duplicates_and_delayed':
                wire._deliver(c, c.station_rows[-1], tuple(reversed(order)))
                task.grasp6d_preview_plan_cb(deepcopy(notices[0]))
            assert n._poll_direct_snapshot() is (index == 2)
        assert second_finished.wait(10.), errors
        assert not errors and len(tickets) == 2
        assert releases[-1][1] is False and len(notices) == 1
        assert n._station_snapshot_retry_limit['consumed_request_id'] == tickets[1].request_id
        assert all(stamp > watermark for stamp in tickets[1].payload[0].sample_stamp_ns)
        assert phase == (n._stream_generation, n._near_field_phase_id, n._near_field_phase_deadline_sec,
                         n._observe_return_session, n._current_stream_target_identity())
        if delivery == 'duplicates_and_delayed':
            task.grasp6d_preview_plan_cb(deepcopy(notices[0]))
        if second_outcome == 'terminal':
            states = []
            task.set_state = lambda *args: states.append(args)
            task._set_near_field_preview_stream = Mock(return_value=True)
            task._freeze_execution_plan = Mock(side_effect=AssertionError('failure cannot freeze'))
            motion, execute = Mock(), Mock()
            config = dict(near_field_replan_enabled=True, near_field_replan_required=True,
                          near_field_strategy='single_snapshot_direct')
            # The consumer starts while this result is current. Advancing three
            # additional frames first would intentionally put it outside the
            # new request's 0.20 s freshness window, testing a different path.
            minimum = max(c.origin.header.stamp.to_nsec()+1,
                task_base.rospy.Time.now().to_nsec()-200_000_000,
                task_base.rospy.Time.from_sec(task._near_field_phase_started_sec).to_nsec())
            result, candidate = task._copy_near_field_preview_candidate(c.origin, minimum, config)
            assert result.code == policy.FAILURE and candidate is None, dict(
                result=repr(result), minimum_stamp_ns=minimum,
                preview=repr(getattr(task, 'latest_grasp6d_preview_plan', None)),
                retry_notice=repr(task._station_snapshot_retry_notice))
            polls = []
            def bounded_poll(duration):
                polls.append(float(duration))
                c.clock[0] += float(duration)  # Advance both explicit clock providers.
                assert len(polls) <= 2, 'terminal consumption entered a repeated wait: '+repr(states)
            with monkeypatch.context() as polling:
                polling.setattr(task_base.rospy, 'sleep', bounded_poll)
                assert task._maybe_rebind_near_field_grasp6d_plan(config, {}, c.origin,
                    move_pose=motion, strict_execute_pose=execute) is None
            assert states[-1][0] == task_base.GraspStages.FAILED
            assert states[-1][1].startswith(policy.FAILURE+':')
            assert polls == []
            task._freeze_execution_plan.assert_not_called()
            motion.assert_not_called(); execute.assert_not_called()
        # Test the admission limit after the current terminal has been consumed,
        # without making the terminal stale before its first consumer poll.
        for _ in range(3):
            row = _fresh_triplet(c)
            wire._deliver(c, row, order)
            assert n._poll_direct_snapshot() is False, 'a third input window was admitted'
        if second_outcome != 'terminal':
            result, candidate = task._copy_near_field_preview_candidate(c.origin, 0, {})
            assert result.code == policy.CODE and not result.terminal_for_request and candidate is None
        assert n._pipeline_counters['submitted'] == 2
        assert len(c.motion) == 1
        (tmp_path/'gen53_station_retry_handoff.json').write_text(json.dumps(dict(
            order=order, delivery=delivery, second_outcome=second_outcome, releases=releases,
            new_sources=list(tickets[1].payload[0].sample_stamp_ns), old_watermark=watermark,
            physical_grasp_verified=False, planning_outcome_provider='explicit simulated zero-candidate audit',
            source_producer='original analytic RGB-D producer', extra_motion_count=0), indent=2)+'\n')
    finally:
        n.shutdown_streaming_worker(timeout_sec=3.)
        worker.join(3.)
        assert not worker.is_alive()


@pytest.mark.parametrize('fault', ['box_conflict', 'sweep_collision', 'strict_checked',
    'local_pass', 'incomplete_search', 'stale_audit', 'unsupported_model'])
def test_conflict_or_path_failure_is_never_reclassified_as_new_input(fault):
    ticket = NS(request_id=18, generation=7, target_epoch=4,
                payload=(NS(stamp_ns=1792334066357594251),))
    prepared = NS(ticket=ticket, snapshot=ticket.payload[0], remote_failure_code='')
    node = NS(_mode_selection=dict(mode='unknown', strategy='observe_return'))
    report = _report(node, ticket)
    if fault == 'box_conflict':
        report['remote_diagnostics']['tabletop_geometry']['regular_box_inference'].update(
            code='BOX_MEASURED_GEOMETRY_CONFLICT', maximum_observed_model_excess_m=.003182664492)
    elif fault == 'sweep_collision': report['pipeline_funnel']['rejection_counts']['GRIPPER_SWEEP_COLLISION']=1
    elif fault == 'strict_checked': report['moveit_selection'].update(checked_count=1, shortlist_count=1)
    elif fault == 'local_pass': report['pipeline_funnel']['stage_counts']['locally_valid'].update(passed=1,rejected=11)
    elif fault == 'incomplete_search': report['moveit_selection']['terminated_early']=True
    elif fault == 'stale_audit': report['request_id']-=1
    else: report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['code']='BOX_INPUT_INVALID'
    assert not policy.completed_missing_evidence(report, prepared, node._mode_selection)


@pytest.mark.parametrize('fault', ['manual', 'epoch', 'phase', 'target',
    'watermark', 'monotonic_expired', 'busy_worker', 'other_ticket'])
def test_original_owner_checks_or_exact_cleanup_cas_prevent_retry(monkeypatch, tmp_path, fault):
    c = _context(monkeypatch, tmp_path)
    n, ticket = c.remote, c.ticket
    prepared = NS(ticket=ticket, snapshot=ticket.payload[0], remote_failure_code='')
    n._active_gate_audit_report = _report(n, ticket)
    n._latest_gate_audit_reference = dict(report_sha256='a'*64)
    n._stream_worker_busy = True
    assert n._station_queue_snapshot_retry(prepared, policy.FAILURE)
    n._stream_worker_busy = False
    if fault == 'manual': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'epoch': n._measurement_remote_epoch_ns += 1
    elif fault == 'phase': n._near_field_phase_id += 1
    elif fault == 'target': n.target_instance_epoch += 1
    elif fault == 'watermark': n.last_submitted_stamp_ns += 1
    elif fault == 'monotonic_expired': n._station_snapshot_retry['monotonic_deadline'] = -1.
    elif fault == 'busy_worker': n._stream_worker_busy = True
    elif fault == 'other_ticket': ticket = deepcopy(ticket)
    old_slot = n._direct_near_field_submission_generation
    with n._stream_condition:
        assert not n._release_expired_direct_snapshot_locked(ticket, policy.FAILURE)
    if fault in ('phase', 'watermark', 'monotonic_expired', 'busy_worker', 'other_ticket'):
        assert n._direct_near_field_submission_generation == old_slot
    assert len(c.motion) == 1
