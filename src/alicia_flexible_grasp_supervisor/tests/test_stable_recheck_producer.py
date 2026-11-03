"""Synthetic producer-stage regression; no hardware or original R6 bag replay.

The real stable loop evaluates twelve measured height placements. Only the
geometry input and persistence transport are substituted; empty scored output,
completion ledger, selection, classification and audit builder are production.
"""
import ast
from copy import deepcopy
from dataclasses import replace
import threading
from types import SimpleNamespace

import pytest

from test_contact_height_placement import case
from test_remote_grasp6d_streaming import (
    ROOT, remote_node, CandidateObservation, CandidateTracker, TrackingConfig,
    SoftScoreWeights, MoveItResult, InferenceTicket, TargetTrackIdentity, configure_identity_handeye,
    attach_bilateral_surface, tabletop_geometry, tabletop_candidates_for, tabletop_gripper,
)
from alicia_flexible_grasp.grasp.stable_recheck_contract import (
    CODE, INCOMPLETE_CODE, completed_stable_recheck_proof_valid,
)


def producer(monkeypatch):
    node = remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    configure_identity_handeye(node)
    monkeypatch.setattr(remote_node.rospy, 'get_param', lambda _name, default=None: default)
    monkeypatch.setattr(remote_node.rospy, 'logwarn', lambda *_a, **_k: None)
    monkeypatch.setattr(remote_node.rospy, 'loginfo', lambda *_a, **_k: None)
    node.target_instance_epoch = 4
    node._target_identity_generation = 0
    node._stream_condition = threading.Condition(threading.RLock())
    node._stream_shutdown = threading.Event()
    node.streaming_enabled = True
    node._stream_generation = 1
    node._last_model_choice = 'unknown_tabletop'
    node.tracker = CandidateTracker(TrackingConfig(window_size=5, min_hits=3))
    node.grasp_config = dict(tool_approach_axis='z', pregrasp_distance_m=.08,
                            final_approach_offset_m=.015, lift_height_m=.05)
    node.gripper_geometry = tabletop_gripper()
    node.gripper_tool_jaw_axis = 'y'
    node.gripper_tool_finger_length_axis = 'z'
    node.gripper_physical_open_width_m = .05
    node.target_absolute_sanity_distance_m = .15
    node.soft_score_weights = SoftScoreWeights()
    node.camera_visibility_gate_enabled = False
    node.camera_visibility_diagnostic_enabled = False
    node.moveit_top_n = 5
    node.candidate_max_joint_delta_rad = 1.8
    node.near_field_strategy = 'single_snapshot_direct'
    node.near_field_direct_timeout_sec = 30.
    node.near_field_planning_active = True
    node._near_field_phase_id = 1
    node._near_field_phase_deadline_sec = 50.
    clock = [21.]
    node._execution_plan_validity_now_sec = lambda: clock[0]
    attach_bilateral_surface(node)
    geometry = tabletop_geometry()
    ticket = InferenceTicket(request_id=13, generation=1, snapshot_stamp_sec=20.13,
                             target_epoch=4, payload=None, submitted_monotonic_sec=20.)
    prepared = remote_node.PreparedPrediction(
        ticket=ticket,
        snapshot=SimpleNamespace(
            object_msg=SimpleNamespace(detected=True, label='carton'),
            target_identity=TargetTrackIdentity.from_stream(0, 4), stamp_ns=20_130_000_000),
        stamp=remote_node.rospy.Time.from_sec(20.13), geometry=geometry,
        pose_estimator=SimpleNamespace(transform_sha256='current-tf'),
        graspnet_input=None, candidates=(),
        remote_diagnostics={}, remote_performance={}, model_choice='unknown_tabletop',
        near_field=True, near_field_phase_id=1, near_field_phase_deadline_sec=50.)
    node._contact_height_evidence_limit = lambda: None
    normalized = node._normalize_tabletop_candidate(prepared, tabletop_candidates_for(geometry)[0])
    observations = tuple(CandidateObservation(
        request_id=13, snapshot_stamp_sec=20.13, target_epoch=4,
        target_track_id=prepared.snapshot.target_identity.track_id, target_label='carton',
        model_choice='unknown_tabletop', center_base_xyz=normalized.contact_center_base,
        tool0_position_xyz=normalized.T_base_tool0[:3, 3],
        quaternion_xyzw=remote_node.quaternion_from_matrix(normalized.T_base_tool0),
        approach_base_xyz=normalized.insertion_axis_base,
        required_open_width_m=normalized.required_open_width_m, model_width_m=None,
        model_score=None, geometry_margin_m=.003, pre_moveit_score=normalized.common_physical_cost,
        payload=replace(normalized, source_index=i), candidate_source='tabletop_geometry',
        source_lineage=('tabletop_geometry',)) for i in range(6))
    # A real measured narrow interval with an unchanged evidence budget. This
    # fixture is not claimed to be the unavailable R6 measured point cloud.
    placement_inputs = []
    def height_geometry(*args):
        placement_inputs.append(args)
        return case()
    node._contact_height_geometry = height_geometry
    node._contact_height_evidence_limit = lambda: .00460616416406
    return node, prepared, observations, placement_inputs, clock


def proof_for(node, prepared, local_count=6):
    return dict(request_id=prepared.ticket.request_id, generation=prepared.ticket.generation,
                target_epoch=prepared.ticket.target_epoch, snapshot_stamp_ns=prepared.snapshot.stamp_ns,
                contact_search_code=CODE, selected=False, terminated_early=False,
                checked_count=0, shortlist_count=0, reachable_count=0,
                locally_valid=dict(entered=local_count, passed=local_count, rejected=0),
                stable_recheck=node._stable_recheck_audit_for(prepared),
                contact_or_motion_authorized=False)


def test_real_twelve_variant_recheck_records_measured_rejections(monkeypatch):
    node, prepared, observations, calls, _ = producer(monkeypatch)
    scored = node._recheck_and_score_stable(
        prepared, node._single_request_stable_candidates(observations))
    assert scored == () and len(calls) == 12
    proof = proof_for(node, prepared)
    assert completed_stable_recheck_proof_valid(proof)
    ledger = proof['stable_recheck']
    assert ledger['completed_variant_count'] == ledger['rejected_count'] == 12
    assert len(ledger['variants']) == 12
    assert all(row['contact_height_placement']['before']['evaluated']
               for row in ledger['variants'])
    assert node._stable_recheck_audit_for(replace(prepared)) is None
    ledger['variants'].clear()
    assert len(node._stable_recheck_audit_for(prepared)['variants']) == 12
    assert not node._stable_variant_runtime


@pytest.mark.parametrize('fault', ['invalid_payload', 'unexpected_error', 'unevaluated_height'])
def test_incomplete_variant_cannot_prove_completed_search(monkeypatch, fault):
    node, prepared, observations, calls, _ = producer(monkeypatch)
    stable = node._single_request_stable_candidates(observations)
    if fault == 'invalid_payload':
        stable = (replace(stable[0], payload=None), *stable[1:])
    elif fault == 'unexpected_error':
        original = node._stable_variant_pose
        node._stable_variant_pose = lambda item, *args: (
            (_ for _ in ()).throw(ValueError('unexpected geometry error'))
            if item.track_id == 1 else original(item, *args))
    else:
        real = remote_node.place_contact_height
        def unevaluated(**kwargs):
            transform, evidence = real(**kwargs)
            evidence['before']['evaluated'] = False
            return transform, evidence
        monkeypatch.setattr(remote_node, 'place_contact_height', unevaluated)
    assert node._recheck_and_score_stable(prepared, stable) == ()
    ledger = node._stable_recheck_audit_for(prepared)
    assert ledger['incomplete_count'] >= 2 and not ledger['completed']
    assert not completed_stable_recheck_proof_valid(proof_for(node, prepared))


def test_cancel_during_real_variant_loop_never_commits_completed_ledger(monkeypatch):
    node, prepared, observations, calls, _ = producer(monkeypatch)
    original = node._contact_height_geometry
    def cancel_after_first(*args):
        result = original(*args)
        node._stream_shutdown.set()
        return result
    node._contact_height_geometry = cancel_after_first
    with pytest.raises(remote_node.StreamResultCancelled):
        node._recheck_and_score_stable(
            prepared, node._single_request_stable_candidates(observations))
    assert len(calls) == 1
    assert node._stable_recheck_audit_for(prepared) is None


def acceptance(monkeypatch, fault='none'):
    node, prepared, observations, calls, clock = producer(monkeypatch)
    if fault in ('partial_generated', 'partial_success', 'all_filtered'):
        observations = tuple(replace(item, model_choice='carton_segment') for item in observations)
    if fault in ('invalid_payload', 'partial_generated', 'partial_success'):
        observations = (replace(observations[0], payload=None), *observations[1:])
    if fault == 'remote_error':
        prepared = replace(prepared, remote_failure_code='WSL_UNAVAILABLE',
                           remote_failure_reason='synthetic endpoint error')
    node._activate_prepared_geometry = lambda _: True
    node._run_candidate_gate_audit = lambda *_a, **_k: {'summary': {}, 'rows': []}
    node._evaluate_local_candidates = lambda _: (observations, {
        'input_count': 6, 'stage_counts': {'locally_valid': dict(entered=6, passed=6, rejected=0)},
        'rejection_counts': {}, 'rejection_ratios': {}, 'primary_failure': None})
    node._check_moveit_stable_candidate = lambda _: pytest.fail('empty shortlist reached MoveIt')
    strict_calls = []
    def strict_unreachable(candidate):
        strict_calls.append(candidate)
        if fault == 'partial_success':
            return MoveItResult(True, .2, .1, 'synthetic strictly checked candidate',
                collision_free=True, within_joint_limits=True, ik_valid=True, planning_success=True)
        return MoveItResult(False, 0., 0., 'synthetic strict IK rejection',
                            failure_code='MOVEIT_UNREACHABLE')
    node._check_direct_registered_candidate = strict_unreachable
    node._test_strict_calls = strict_calls
    if fault == 'all_filtered':
        real_explicit = remote_node.evaluate_explicit_candidate
        def unsafe(**kwargs):
            return replace(real_explicit(**kwargs), ok=False,
                           failure_code='SUPPORT_COLLISION', failed_gate='support_clearance',
                           failure_reason='synthetic measured collision')
        monkeypatch.setattr(remote_node, 'evaluate_explicit_candidate', unsafe)
    node._observe_stream_prediction_failure = lambda *_a: None
    events = []
    if fault == 'partial_success':
        node._publish_selected_preview = lambda *_a, **_k: events.append(('preview', None))
    else:
        node._publish_selected_preview = lambda *_a, **_k: pytest.fail('rejection published a preview')
    node._cache_latest_preview_gate_audit_if_current = lambda _: None
    def memory_writer(report, *, commit_callback, **_kwargs):
        # Only the persistence transport is fake: production builder and commit
        # callback set authoritative report, and terminal must observe it.
        events.append(('persist', deepcopy(report)))
        reference = {'atomic_committed': True, 'sha256': 'synthetic-in-memory-only'}
        commit_callback(reference)
        events.append(('committed', deepcopy(node._active_gate_audit_report)))
        return reference
    node._write_gate_audit_report = memory_writer
    node._publish_bounded_gate_audit = lambda *_a, **_k: events.append(('published', None))
    node._observe_execution_candidate_invalid = lambda **_k: events.append(
        ('invalidated', deepcopy(node._active_gate_audit_report)))
    def terminal(_prepared, code, reason):
        events.append(('terminal', dict(code=code, reason=reason,
            report=deepcopy(node._active_gate_audit_report))))
    node._publish_direct_near_field_terminal = terminal
    if fault == 'deadline':
        original = node._contact_height_geometry
        def deadline_during_recheck(*args):
            value = original(*args)
            clock[0] = 50.
            return value
        node._contact_height_geometry = deadline_during_recheck
    result = node._accept_prediction(prepared)
    return node, prepared, result, calls, events


@pytest.mark.parametrize('fault,expected', [
    ('none', CODE), ('invalid_payload', INCOMPLETE_CODE),
    ('deadline', 'NEAR_FIELD_DIRECT_TIMEOUT'), ('remote_error', 'WSL_UNAVAILABLE'),
])
def test_actual_accept_empty_moveit_classifies_after_audit_commit(monkeypatch, fault, expected):
    node, prepared, result, calls, events = acceptance(monkeypatch, fault)
    assert result['status'] == expected
    assert len(calls) == (10 if fault == 'invalid_payload' else 12)
    names = [name for name, _ in events]
    assert names.index('persist') < names.index('committed') < names.index('terminal')
    report = events[-1][1]['report']
    assert events[-1][1]['code'] == expected
    assert report['outcome']['code'] == expected and not report['outcome']['valid_plan']
    assert report['selected'] is None and report['stable_evaluations'] == []
    assert report['moveit_selection']['checked_count'] == 0
    assert report['moveit_selection']['shortlist_count'] == 0
    assert report['stable_recheck'] == node._stable_recheck_audit_for(prepared)
    if fault == 'none':
        assert completed_stable_recheck_proof_valid(proof_for(node, prepared))
        assert 'no candidate reached MoveIt' in events[-1][1]['reason']
        assert names.index('invalidated') < names.index('terminal')
    if fault == 'deadline':
        assert report['moveit_selection']['terminated_early']


def test_base_task_recognizes_both_pre_moveit_terminal_codes():
    tree = ast.parse((ROOT / 'scripts/grasp_task_node.py').read_text())
    assigned = next(stmt for stmt in tree.body if isinstance(stmt, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == '_DIRECT_NEAR_FIELD_TERMINAL_CODES'
                for target in stmt.targets))
    codes = ast.literal_eval(assigned.value.args[0])
    assert CODE in codes and INCOMPLETE_CODE in codes and 'NEAR_FIELD_NO_PATH_CANDIDATE' in codes


def test_partial_generation_and_checked_rejections_do_not_prove_complete_search(monkeypatch):
    node, prepared, result, calls, events = acceptance(monkeypatch, 'partial_generated')
    ledger = node._stable_recheck_audit_for(prepared)
    assert ledger['generated_count'] == 10 and ledger['incomplete_count'] == 2
    assert not ledger['completed'] and calls == []
    assert node._test_strict_calls
    assert result['status'] == INCOMPLETE_CODE
    report = events[-1][1]['report']
    assert report['moveit_selection']['checked_count'] > 0
    assert report['moveit_selection']['reachable_count'] == 0
    assert events[-1][1]['code'] == INCOMPLETE_CODE
    assert not completed_stable_recheck_proof_valid(proof_for(node, prepared))


def test_generated_variants_filtered_before_moveit_are_not_path_failures(monkeypatch):
    node, prepared, result, calls, events = acceptance(monkeypatch, 'all_filtered')
    ledger = node._stable_recheck_audit_for(prepared)
    assert ledger['generated_count'] == 12 and ledger['incomplete_count'] == 0, ledger['variants'][0]
    assert ledger['completed'] and calls == [] and not node._test_strict_calls
    assert result['status'] == 'NEAR_FIELD_NO_PATH_CANDIDATE'
    report = events[-1][1]['report']
    assert report['moveit_selection']['checked_count'] == 0
    assert report['moveit_selection']['shortlist_count'] == 0
    assert events[-1][1]['code'] == 'NEAR_FIELD_NO_PATH_CANDIDATE'
    assert not completed_stable_recheck_proof_valid(proof_for(node, prepared))


def test_partial_recheck_does_not_discard_independently_checked_candidate(monkeypatch):
    node, prepared, result, calls, events = acceptance(monkeypatch, 'partial_success')
    ledger = node._stable_recheck_audit_for(prepared)
    assert ledger['generated_count'] == 10 and ledger['incomplete_count'] == 2
    assert node._test_strict_calls and calls == []
    assert result['status'] == 'PREVIEW_READY'
    assert [name for name, _ in events].count('preview') == 1
    assert not any(name == 'terminal' for name, _ in events)
    report = node._active_gate_audit_report
    assert report['selected'] is not None and report['outcome']['preview_valid']
    # A strict preview is still not a promoted or executed motion.
    assert not report['outcome']['valid_plan']
    assert not completed_stable_recheck_proof_valid(proof_for(node, prepared))
