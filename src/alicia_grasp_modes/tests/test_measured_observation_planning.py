"""Offline ordinary observation preflight; no ROS master or action execution.

Synthetic measured surfaces exercise the production geometric predicates and
phase transaction. They are unit inputs, never evidence of a physical grasp.
"""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_observe_return_remote import setup_observe_return
from test_observe_return_visual import measured_gap_fixture
from test_reachable_observation_integration import recorded_inputs, recorded_proposal
from test_tabletop_geometry_candidates import (
    GRIPPER, TabletopGeometryConfig, SurfaceView, fused_surface_from_view,
    generate_tabletop_proposals)


def preflight_context(configured, monkeypatch):
    node, params, clock = setup_observe_return(configured)
    # start_streaming above owns the real phase contract. Its source clock
    # and ROS validity clock must share the same synthetic time domain.
    assert node._near_field_phase_id == 1
    assert node._near_field_phase_deadline_sec == 70.
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(
        lambda: base.rospy.Time(clock[0] // 10**9, clock[0] % 10**9)))
    node.tabletop_geometry_config = TabletopGeometryConfig()
    node.gripper_geometry = GRIPPER
    identity = node._current_stream_target_identity()
    ns = node._active_multiview_surface()[0].view_stamps_ns[0]
    rotation, center, template, camera, _ = measured_gap_fixture(include_negative=False)
    view = SurfaceView(identity, ns, template.points_base,
        np.array([0., 0., 1.]), 0., 12)
    surface = fused_surface_from_view(view)
    node._near_field_fused_surface = surface
    node._near_field_reference_view = view
    geometry = NS(ok=True, center_base=center, axes_base=rotation,
        size_xyz_m=np.array([.045, .030, .020]), support_normal_base=np.array([0., 0., 1.]))
    generated = generate_tabletop_proposals(surface.points_base, center, rotation,
        geometry.size_xyz_m, np.zeros(3), geometry.support_normal_base,
        node.tabletop_geometry_config, surface, node.gripper_geometry)
    assert not generated.proposals
    diagnostics = dict(materialized_count=0, plan_phase='CONTACT_EXECUTION_PLAN',
        contact_execution_gate_deferred=False,
        surface_evidence_rejections=deepcopy(generated.surface_evidence_rejections))
    snapshot = NS(stamp_ns=ns, target_identity=identity, object_msg=NS(detected=True))
    ticket = NS(request_id=76, generation=node._stream_generation, target_epoch=identity.epoch)
    transform = np.eye(4); transform[:3, 3] = camera
    # The request lifetime guard is tested separately below; this unit fixture
    # supplies its current request without starting the background worker.
    guards = []
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', lambda value: guards.append(value))
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [
        NS(depth_uncertainty_m=.002, execution_position_error_m=0.)])
    return node, ticket, snapshot, geometry, transform, (), diagnostics, guards


def test_real_missing_side_preflight_preserves_identity_without_claiming_contact_search(
        configured, monkeypatch):
    node, *args, guards = preflight_context(configured, monkeypatch)
    identity = node._current_stream_target_identity()
    surface = node._active_multiview_surface()[0]
    diagnostics = deepcopy(args[-1])
    assert node._maybe_prepare_observation_from_contact_geometry(*args)
    assert node._observe_return_phase == 'observation_prefix'
    assert node._current_stream_target_identity() == identity
    assert node._observe_return_saved[0] is surface
    assert node._observe_return_view_gaps and guards == [args[0], args[0]]
    proof = node._observe_return_trigger_evidence
    assert proof['policy'] == 'current_request_measured_view_deficit_preflight'
    assert proof['request_id'] == args[0].request_id
    assert proof['snapshot_stamp_ns'] == args[1].stamp_ns
    assert proof['source_view_stamps_ns'] == list(surface.view_stamps_ns)
    assert proof['target_epoch'] == identity.epoch
    assert proof['target_track_id'] == identity.track_id
    assert proof['contact_search_completed'] is False
    assert proof['learned_candidates_requested'] is False
    assert proof['tabletop_geometry_preflight_completed'] is True
    assert proof['selected'] is False
    assert proof['contact_or_motion_authorized'] is False
    assert len(proof['evidence_sha256']) == 64
    assert args[-1] == diagnostics
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


@pytest.mark.parametrize('fault', [
    'direct', 'carton', 'two_stage', 'observing_contact', 'returned_contact',
    'paused', 'diagnostic', 'session', 'far_field', 'candidate', 'materialized',
    'bool_count', 'missing_count', 'deferred', 'wrong_phase', 'preview',
    'execution_plan', 'execution_authority', 'executing', 'foreign_identity',
    'foreign_epoch', 'foreign_stamp', 'invalid_geometry', 'undetected',
    'no_measured_gap', 'invalid_uncertainty', 'missing_uncertainty'])
def test_preflight_does_not_divert_valid_contact_or_unqualified_inputs(
        configured, monkeypatch, fault):
    node, ticket, snapshot, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    if fault in ('direct', 'two_stage'): node._mode_selection['strategy'] = fault
    elif fault == 'carton': node._mode_selection['mode'] = 'carton'
    elif fault in ('observing_contact', 'returned_contact'): node._observe_return_phase = fault
    elif fault == 'paused': node._observe_return_paused = True
    elif fault == 'diagnostic': node._observe_return_diagnostic_observation_only = True
    elif fault == 'session': node._observe_return_session = {'phase': 'observing'}
    elif fault == 'far_field': node.near_field_planning_active = False
    elif fault == 'candidate': candidates = (object(),)
    elif fault == 'materialized': diagnostics['materialized_count'] = 1
    elif fault == 'bool_count': diagnostics['materialized_count'] = False
    elif fault == 'missing_count': del diagnostics['materialized_count']
    elif fault == 'deferred': diagnostics['contact_execution_gate_deferred'] = True
    elif fault == 'wrong_phase': diagnostics['plan_phase'] = 'FAR_FIELD_OBSERVATION_PLAN'
    elif fault == 'preview': node.latest_preview_rich_plan = NS(valid=True)
    elif fault == 'execution_plan': node.latest_rich_plan = NS(valid=True)
    elif fault == 'execution_authority': node._execution_authority_ticket = object()
    elif fault == 'executing': node.robot_execution_active = True
    elif fault == 'foreign_identity': snapshot.target_identity = base.TargetTrackIdentity.from_stream(99, 99)
    elif fault == 'foreign_epoch': ticket.target_epoch += 1
    elif fault == 'foreign_stamp': snapshot.stamp_ns += 1
    elif fault == 'invalid_geometry': geometry.ok = False
    elif fault == 'undetected': snapshot.object_msg.detected = False
    elif fault == 'no_measured_gap': diagnostics['surface_evidence_rejections'] = []
    elif fault == 'invalid_uncertainty':
        monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [NS(depth_uncertainty_m=float('nan'))])
    elif fault == 'missing_uncertainty': monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [])
    phase = node._observe_return_phase
    assert node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, candidates, diagnostics) is False
    assert node._observe_return_phase == phase
    assert node._observe_return_trigger_evidence is None


@pytest.mark.parametrize('guard_call', [1, 2])
def test_generation_change_during_preflight_cancels_without_view_transition(
        configured, monkeypatch, guard_call):
    node, *args, _ = preflight_context(configured, monkeypatch)
    count = [0]
    def current(_ticket):
        count[0] += 1
        if count[0] == guard_call:
            raise base.StreamResultCancelled('GENERATION_STALE')
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', current)
    with pytest.raises(base.StreamResultCancelled, match='GENERATION_STALE'):
        node._maybe_prepare_observation_from_contact_geometry(*args)
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_trigger_evidence is None


def test_default_base_preflight_preserves_other_planners(configured):
    node, _ = configured
    assert base.RemoteGrasp6DNode._maybe_prepare_observation_from_contact_geometry(
        node, None, None, None, None, (), {}) is False


def test_original_prepare_calls_preflight_before_learned_input_or_client(
        configured, monkeypatch):
    from alicia_flexible_grasp.vision.rgbd_snapshot import DepthQuality
    from alicia_flexible_grasp.vision.latest_only_inference import InferenceTicket
    node, old_ticket, measured, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    ns = measured.stamp_ns
    msg = base.ObjectPose(); msg.header.stamp = base.rospy.Time(ns // 10**9, ns % 10**9)
    msg.header.frame_id = 'camera_link'; msg.detected = True; msg.label = 'unknown'
    depth = np.ones((3, 3), np.uint16) * 250
    snapshot = base.SnapshotResult(True, '', '', np.zeros((3, 3, 3), np.uint8),
        depth, depth, np.ones((3, 3), np.uint8)*255, (0, 0, 3, 3), msg,
        ns/1e9, 'camera_link', DepthQuality(1, 9, 9, 1., .25, 0., .001),
        'instance_mask', ns, old_ticket.target_epoch, measured.target_identity, (ns,))
    config = NS(candidate_target_gate_enabled=True)
    ticket = InferenceTicket(old_ticket.request_id, old_ticket.generation,
        ns/1e9, old_ticket.target_epoch, (snapshot, config), 0.)
    monkeypatch.setattr(node, '_require_stream_ticket_current', lambda _: None)
    monkeypatch.setattr(node, '_require_graspnet_input_prerequisites', lambda *_: None)
    monkeypatch.setattr(node, '_capture_geometry_generation', lambda: 0)
    monkeypatch.setattr(node, '_refresh_runtime_params', lambda: None)
    monkeypatch.setattr(node, '_prepare_snapshot_geometry', lambda *_: (geometry, depth, transform))
    monkeypatch.setattr(node, '_generate_tabletop_candidates', lambda *a, **k: (candidates, diagnostics))
    monkeypatch.setattr(base, 'FrozenSnapshotCandidatePoseEstimator',
        lambda *a, **k: pytest.fail('contact candidate estimator entered after preflight'))
    with pytest.raises(base.StreamResultCancelled, match='OBSERVE_RETURN_MEASURED_VIEW_REQUIRED'):
        base.RemoteGrasp6DNode._prepare_and_predict(node, ticket)
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_trigger_evidence['learned_candidates_requested'] is False


def independent_context(configured, monkeypatch, tmp_path):
    from test_silhouette_observation_planning import context
    return context(configured, monkeypatch, tmp_path, ordinary=True)


def test_ordinary_observation_prepares_without_contact_seed_and_promotes_one_pose(
        configured, monkeypatch, tmp_path):
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    node, prepared, checks, _ = independent_context(configured, monkeypatch, tmp_path)
    assert prepared.candidates == () and prepared.tabletop_candidates == ()
    assert prepared.graspnet_input is None
    assert prepared.remote_diagnostics['candidate_role'] == 'measured_observation'
    assert prepared.remote_diagnostics['remote_grasp_inference_required'] is False
    actual = node._accept_prediction(prepared)
    assert actual['status'] == 'PREVIEW_READY'
    assert checks and all(observation for _, observation in checks)
    plan = node.latest_preview_rich_plan
    assert plan.candidate_source == 'measured_observation'
    assert plan.candidate_source_lineage == ['measured_observation']
    assert len(plan.poses) == 1
    assert plan.required_open_width_m == plan.candidate_width_m == 0.
    assert not plan.has_candidate_model_width and not plan.contact_geometry_evidence
    assert len(base.rich_plan_to_legacy(plan).poses) == 1
    report = node._active_execution_audit_report
    assert observe_return_audit_error(report) == ''
    assert not node._continuous_execution_schema_error(report, prepared.ticket)
    assert report['selected']['candidate_role'] == 'measured_observation'
    assert report['selected']['observation_source_binding']['request_id'] == prepared.ticket.request_id
    assert report['pipeline_funnel']['source_counts'] == {'measured_observation':
        dict(generated=1, locally_valid=1, stable=1, preview=1, promoted=1)}
    assert report['pipeline_funnel']['grasp_tracker_used'] is False
    assert report['observe_return']['trigger_evidence']['contact_search_completed'] is False
    assert report['observe_return']['diagnostic_observation_only'] is False


@pytest.mark.parametrize('failure', ['strict', 'no_endpoint', 'following', 'view'])
def test_independent_measured_view_failure_never_publishes_execution_or_contact(
        configured, monkeypatch, tmp_path, failure):
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    if failure == 'strict':
        monkeypatch.setattr(node, '_strict_moveit_evaluation', lambda *a, **k: (
            base.MoveItResult(False, 0., 0., 'collision', failure_code='MOVEIT_COLLISION'), {}, {}))
    elif failure == 'no_endpoint':
        monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: ((), ({'reason': 'endpoint CAD'},)))
    elif failure == 'following':
        monkeypatch.setattr(base.RemoteGrasp6DNode, '_observation_following_support_evaluation',
            lambda self, _p, _r, _t=None: (
                base.MoveItResult(False, 0., 0., 'following support', failure_code='FOLLOWING_SUPPORT_REJECTED'), {'ok': False}))
    else:
        monkeypatch.setattr(node, '_diagnostic_observation_view_evaluation',
            lambda *_: (_ for _ in ()).throw(ValueError('view rejected')))
    result = node._accept_prediction(prepared)
    assert result['status'].endswith('OBSERVATION_NO_SAFE_VIEW')
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not getattr(node, 'latest_rich_plan', None)


@pytest.mark.parametrize('fault', ['foreign_gap_identity', 'future_gap_stamp',
    'empty_gap_stamps', 'no_gaps', 'contact_candidate', 'tabletop_candidate',
    'graspnet_input', 'contact_role', 'requested_contact', 'checked_contact'])
def test_independent_source_rejects_unbound_or_contact_bearing_prepared_input(
        configured, monkeypatch, tmp_path, fault):
    from dataclasses import replace
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    if fault == 'foreign_gap_identity': node._observe_return_view_gaps[0]['target_track_id'] = 'another-target'
    elif fault == 'future_gap_stamp': node._observe_return_view_gaps[0]['source_view_stamps_ns'] = [prepared.snapshot.stamp_ns+1]
    elif fault == 'empty_gap_stamps': node._observe_return_view_gaps[0]['source_view_stamps_ns'] = []
    elif fault == 'no_gaps': node._observe_return_view_gaps = ()
    elif fault == 'contact_candidate': prepared = replace(prepared, candidates=(object(),))
    elif fault == 'tabletop_candidate': prepared = replace(prepared, tabletop_candidates=(object(),))
    elif fault == 'graspnet_input': prepared = replace(prepared, graspnet_input=object())
    else:
        diagnostics = base._thaw_request_evidence(prepared.remote_diagnostics)
        field, value = {'contact_role': ('candidate_role', 'contact'),
            'requested_contact': ('contact_candidates_requested', True),
            'checked_contact': ('contact_geometry_checked', True)}[fault]
        diagnostics[field] = value
        prepared = replace(prepared, remote_diagnostics=diagnostics)
    with pytest.raises(base.CandidateContractError):
        node._accept_prediction(prepared)
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_recorded_fk_is_available_without_contact_seed_or_passing_centered_view(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    from test_reachable_observation_integration import context, install_proposal
    node, _, _, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    install_proposal(node, monkeypatch, recorded_proposal)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: [
        NS(depth_uncertainty_m=.003, execution_position_error_m=0.)])
    monkeypatch.setattr(node, '_contact_stage_profile', lambda *a, **k: pytest.fail('future contact profile requested'))
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', lambda *a, **k: ((), ()))
    rows, rejected = node._far_field_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, reference, prepared)
    assert not rejected and len(rows) == 1
    sequence = rows[0]['sequence']
    assert set(vars(sequence)) == {'pregrasp', 'adaptive_stage_profile', 'observation_view_audit'}
    assert set(vars(sequence.adaptive_stage_profile)) == {'depth_uncertainty_m'}
    assert rows[0]['search_family'] == 'joint_space_fk'
    assert rows[0]['observation_envelope'].ok
    view = rows[0]['observation_view']
    assert view['working_volume']['ok'] and view['observe_return_missing_side_gain']['ok']
    assert view['information_utility']['prediction_only']


@pytest.mark.parametrize('fault', ['missing_trigger', 'future_trigger', 'selected_trigger',
    'claims_completed_search', 'trigger_digest', 'missing_source_binding',
    'foreign_request', 'foreign_source_stamp', 'foreign_target', 'empty_missing_sides',
    'mixed_source', 'contact_role', 'contact_width', 'contact_phase',
    'direct_strategy', 'missing_nondiagnostic_flag', 'weak_gain'])
def test_real_ordinary_producer_audit_cannot_be_rebound_or_gain_contact_authority(
        configured, monkeypatch, tmp_path, fault):
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    report = deepcopy(node._active_execution_audit_report)
    assert not observe_return_audit_error(report)
    meta = report['observe_return']; selected = report['selected']
    if fault == 'missing_trigger': del meta['trigger_evidence']
    elif fault == 'future_trigger': meta['trigger_evidence']['snapshot_stamp_ns'] = report['snapshot_stamp_ns']+1
    elif fault == 'selected_trigger': meta['trigger_evidence']['selected'] = True
    elif fault == 'claims_completed_search': meta['trigger_evidence']['contact_search_completed'] = True
    elif fault == 'trigger_digest': meta['trigger_evidence']['evidence_sha256'] = '0'*64
    elif fault == 'missing_source_binding': del selected['observation_source_binding']
    elif fault == 'foreign_request': selected['observation_source_binding']['request_id'] += 1
    elif fault == 'foreign_source_stamp': selected['observation_source_binding']['snapshot_stamp_ns'] += 1
    elif fault == 'foreign_target': selected['observation_source_binding']['target_track_id'] = 'other-target'
    elif fault == 'empty_missing_sides': meta['measured_missing_sides'] = []
    elif fault == 'mixed_source': selected['source_lineage'] += ['tabletop_geometry']
    elif fault == 'contact_role': selected['candidate_role'] = 'contact'
    elif fault == 'contact_width': selected['required_open_width_m'] = .04
    elif fault == 'contact_phase': meta['phase'] = 'initial_contact'
    elif fault == 'direct_strategy': report['execution_strategy'] = 'direct'
    elif fault == 'missing_nondiagnostic_flag': del meta['diagnostic_observation_only']
    else:
        gain = selected['observation_view']['observe_return_missing_side_gain']
        gain['predicted_new_side_gain_m'] = gain['depth_uncertainty_m']
    assert observe_return_audit_error(report)


def test_independent_request_replaces_expired_prior_comparison_state(
        configured, monkeypatch, tmp_path):
    node, prepared, checks, _ = independent_context(configured, monkeypatch, tmp_path)
    old = dict(deadline=1., ticket=object(), stop_reason='OBSERVATION_COMPARISON_BUDGET_REACHED')
    node._observation_information_comparison = old
    node._observation_comparison_clock = lambda: 100.
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    current = node._observation_information_comparison
    assert current is not old
    assert current['ticket'] is prepared.ticket
    assert current['deadline'] is None or current['deadline'] > 100.
    assert checks


@pytest.mark.parametrize('expires_during_geometry', [False, True])
def test_preflight_cannot_extend_an_expired_contact_phase(
        configured, monkeypatch, expires_during_geometry):
    node, *args, _ = preflight_context(configured, monkeypatch)
    times = iter([12., 14.] if expires_during_geometry else [14.])
    node._near_field_phase_id = 1
    node._near_field_phase_deadline_sec = 13.
    monkeypatch.setattr(node, '_execution_plan_validity_now_sec', lambda: next(times))
    assert node._maybe_prepare_observation_from_contact_geometry(*args) is False
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_trigger_evidence is None
