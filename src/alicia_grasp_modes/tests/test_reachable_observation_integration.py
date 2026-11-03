"""FK proposals join the existing observation gates; no device access."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_remote_modes import configured, base, module
from test_observe_return_remote import setup_observe_return
from test_reachable_observation_views import recorded_inputs
from alicia_grasp_modes import reachable_observation_views as proposals


@pytest.fixture(scope='module')
def recorded_proposal(recorded_inputs):
    rows, audit = proposals.generate_reachable_observation_views(**recorded_inputs)
    assert rows, audit
    return rows[0]


def context(configured, monkeypatch, recorded_inputs):
    node, params, _ = setup_observe_return(configured)
    source = deepcopy(recorded_inputs['source'])
    binding = source['binding']; ns = binding['snapshot_stamp_ns']
    stamp = base.rospy.Time(ns // 10**9, ns % 10**9)
    geometry = NS(center_base=np.array(source['center_base']),
        axes_base=np.array(source['geometry']['R_base_obb']),
        size_xyz_m=np.array(source['geometry']['obb_size_xyz_m']),
        support_normal_base=np.array(source['geometry']['support_normal_base']),
        support_offset_m=source['geometry']['support_offset_m'])
    mount = np.array(source['tool_from_camera'])
    transform = np.array(source['base_from_tool'])
    reference = base.make_pose_stamped('base_link', transform[:3, 3],
        base.quaternion_from_matrix(transform), stamp=stamp)
    prepared = NS(geometry=geometry, stamp=stamp, snapshot=NS(stamp_ns=ns),
        pose_estimator=NS(T_base_camera_link=transform @ mount))
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    node._observe_return_diagnostic_observation_only = False
    node._observe_return_view_gaps = deepcopy(recorded_inputs['gaps'])
    node._observe_return_view_uncertainty_m = .003
    node._observe_return_directed_views = ()
    node._diagnostic_observation_view_source = source
    node.gripper_geometry = recorded_inputs['gripper']
    node.gripper_physical_open_width_m = .05
    node.gripper_tool_jaw_axis = '+Y'; node.gripper_tool_finger_length_axis = '+Z'
    node.grasp_config = {'observation_acceptance_policy': 'rgbd_working_volume_v1'}
    params['/robot_description'] = (Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log') /
        'user_grasp_20261008T090751Z/source_snapshot_review/gen28_fk_ik_100ms.model.urdf').read_text()
    # Source extraction/path revalidation have independent real-data tests.
    # Here the production adapter receives that already frozen source.
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', lambda p: source)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: mount)
    monkeypatch.setattr(node, '_camera_intrinsics', lambda: NS(**source['camera']))
    monkeypatch.setattr(node, '_observation_view_limit_configuration', lambda: source['view_limits'])
    monkeypatch.setattr(node, '_current_stream_target_identity', lambda:
        NS(epoch=binding['target_epoch'], track_id=binding['target_track_id']))
    profile = NS(depth_uncertainty_m=.003, pregrasp_distance_m=.04,
        approach_offset_m=.01, lift_height_m=.04)
    monkeypatch.setattr(node, '_contact_stage_profile', lambda *a, **k: profile)
    return node, params, source, prepared, reference


def install_proposal(node, monkeypatch, proposal):
    monkeypatch.setattr(node, '_joint_space_observation_candidates',
        lambda prepared: ((deepcopy(proposal),), {'execution_authority': False}))


def bind_task_target(node, source_binding, *, floor=.002):
    from test_observation_information_uncertainty import proposal_profiles, proposal_snapshot
    ns = source_binding['snapshot_stamp_ns']
    snapshot = proposal_snapshot(stamps=(ns-2, ns-1, ns))
    node._observe_return_trigger_evidence = node._observe_return_bind_predicted_gain(
        deepcopy(source_binding), proposal_profiles(floor=floor), snapshot)
    return node._observe_return_trigger_evidence


def test_real_measured_preflight_freezes_task_target_at_original_gap_source(configured, monkeypatch):
    from test_measured_observation_planning import preflight_context
    from test_observation_information_uncertainty import proposal_profiles, proposal_snapshot
    node, ticket, snapshot, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    temporal = proposal_snapshot(stamps=(snapshot.stamp_ns-2, snapshot.stamp_ns-1, snapshot.stamp_ns))
    snapshot.quality = temporal.quality; snapshot.sample_stamp_ns = temporal.sample_stamp_ns
    original_profiles = proposal_profiles()
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: original_profiles)
    assert node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, candidates, diagnostics)
    evidence = deepcopy(node._observe_return_trigger_evidence['predicted_gain_evidence'])
    assert evidence['source_binding']['request_id'] == ticket.request_id
    assert evidence['source_binding']['snapshot_stamp_ns'] == snapshot.stamp_ns
    assert node._observe_return_predicted_gain_target_m() == pytest.approx(.002)
    assert node._observe_return_view_uncertainty_m == pytest.approx(.003)
    # A later prepared source may have different contact requirements. It
    # cannot retroactively relabel the gap-source target or its visual noise.
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: proposal_profiles(floor=.004, support=.005))
    snapshot.quality = proposal_snapshot(.002).quality
    assert node._observe_return_predicted_gain_target_m() == pytest.approx(.002)
    assert node._observe_return_trigger_evidence['predicted_gain_evidence'] == evidence
    assert node._observe_return_view_uncertainty_m == pytest.approx(.003)
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


def test_same_source_task_target_flows_through_fk_sequence_final_gain_and_utility(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    from test_observation_information_uncertainty import proposal_profiles
    node, _, source, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    evidence = deepcopy(bind_task_target(node, source['binding']))
    install_proposal(node, monkeypatch, recorded_proposal)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: proposal_profiles(floor=.004))
    monkeypatch.setattr(node, '_contact_stage_profile', lambda *a, **k: pytest.fail('observation requested contact profile'))
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', lambda *a, **k: ((), ()))
    rows, rejected = node._far_field_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, reference, prepared)
    assert len(rows) == 1 and not rejected
    row = rows[0]; sequence = row['sequence']; view = row['observation_view']
    assert sequence.adaptive_stage_profile.depth_uncertainty_m == pytest.approx(.003)
    assert not any(hasattr(sequence, key) for key in ('approach', 'grasp', 'lift'))
    gain = view['observe_return_missing_side_gain']; utility = view['information_utility']
    assert gain['ok'] and gain['predicted_gain_target_m'] == pytest.approx(.002)
    assert gain['depth_uncertainty_m'] == pytest.approx(.003)
    assert utility['predicted_gain_target_m'] == pytest.approx(.002)
    assert utility['noise_px'] / utility['predicted_gain_target_px'] == pytest.approx(1.5)
    assert utility['certifies_new_view_perception'] is False
    assert row['observation_envelope'].ok
    assert row['observation_side_evidence']['observation_side_evidence_deficit_m'] == 0.
    assert view['working_volume']['ok'] and not view['proposal']['execution_authority']
    assert node._observe_return_trigger_evidence == evidence


def test_task_target_and_evidence_join_cache_key_without_renewing_original_budget(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    from test_observation_information_uncertainty import proposal_profiles
    node, params, source, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    bind_task_target(node, source['binding'])
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: prepared.stamp)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: proposal_profiles(floor=.004, support=.005))
    calls = []
    def generate(**kwargs):
        calls.append(kwargs)
        return (deepcopy(recorded_proposal),), dict(elapsed_sec=min(2.6, kwargs['max_seconds']),
            execution_authority=False, candidate_count=1)
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', generate)
    first, report = node._joint_space_observation_candidates(prepared)
    assert first and not report['cache_hit']
    assert calls[0]['predicted_gain_target_m'] == pytest.approx(.002)
    assert calls[0]['depth_uncertainty_m'] == pytest.approx(.003)
    assert calls[0]['side_depth_uncertainty_m'] == pytest.approx(.005)
    second, report = node._joint_space_observation_candidates(prepared)
    assert second and report['cache_hit'] and len(calls) == 1
    # Any change of the existing source contract invalidates cached proposals
    # while retaining the consumed 2.6 seconds of this request's 5-second cap.
    bind_task_target(node, source['binding'], floor=.004)
    third, report = node._joint_space_observation_candidates(prepared)
    assert third and not report['cache_hit'] and len(calls) == 2
    assert calls[1]['predicted_gain_target_m'] == pytest.approx(.004)
    assert calls[1]['max_seconds'] == pytest.approx(2.4)
    bind_task_target(node, source['binding'], floor=.005)
    fourth, report = node._joint_space_observation_candidates(prepared)
    assert not fourth and report['status'] == 'budget_exhausted' and len(calls) == 2


@pytest.mark.parametrize('kind', ['precision', 'silhouette'])
def test_bound_task_target_cannot_be_relabelled_as_precision_or_silhouette(
        configured, monkeypatch, recorded_inputs, kind):
    node, _, source, _, _ = context(configured, monkeypatch, recorded_inputs)
    assert node._observe_return_predicted_gain_target_m() is None
    bind_task_target(node, source['binding'])
    if kind == 'precision': node._observe_return_trigger = 'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'
    else: node._observe_return_diagnostic_observation_only = True
    with pytest.raises(ValueError):
        node._observe_return_predicted_gain_target_m()


def task_target_runner_report(configured, monkeypatch, tmp_path):
    from test_measured_observation_planning import independent_context
    from test_observation_information_uncertainty import proposal_profiles, proposal_snapshot
    from alicia_grasp_modes.observation_information_uncertainty import PREDICTED_GAIN_POLICY
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    assert node._accept_prediction(prepared)['status'] == 'PREVIEW_READY'
    report = deepcopy(node._active_execution_audit_report)
    trigger = report['observe_return']['trigger_evidence']; ns = trigger['snapshot_stamp_ns']
    report['observe_return']['trigger_evidence'] = node._observe_return_bind_predicted_gain(
        trigger, proposal_profiles(), proposal_snapshot(stamps=(ns-2, ns-1, ns)))
    gain = report['selected']['observation_view']['observe_return_missing_side_gain']
    # A serialization-contract fixture, not a claim about this path endpoint:
    # the task predicts 2.4 mm while actual visual novelty still requires 3 mm.
    gain.update(predicted_new_side_gain_m=.0024, depth_uncertainty_m=.003,
        predicted_gain_target_m=.002, predicted_gain_policy=PREDICTED_GAIN_POLICY,
        prediction_only=True, certifies_new_view_perception=False)
    assert observe_return_audit_error(report) == ''
    return report


@pytest.mark.parametrize('fault', ['target_missing', 'target_lower', 'target_higher',
    'policy', 'gain_equal_target', 'claims_perception', 'claims_measurement',
    'source_request', 'profile_target', 'evidence_missing'])
def test_runner_requires_same_typed_source_target_without_promoting_prediction_to_measurement(
        configured, monkeypatch, tmp_path, fault):
    from alicia_grasp_modes.runner_contract import observe_return_audit_error
    from alicia_grasp_modes.active_observation_policy import evidence_digest
    report = task_target_runner_report(configured, monkeypatch, tmp_path)
    gain = report['selected']['observation_view']['observe_return_missing_side_gain']
    trigger = report['observe_return']['trigger_evidence']
    if fault == 'target_missing': del gain['predicted_gain_target_m']
    elif fault == 'target_lower': gain['predicted_gain_target_m'] = .001
    elif fault == 'target_higher': gain['predicted_gain_target_m'] = .003
    elif fault == 'policy': gain['predicted_gain_policy'] = 'noise'
    elif fault == 'gain_equal_target': gain['predicted_new_side_gain_m'] = .002
    elif fault == 'claims_perception': gain['certifies_new_view_perception'] = True
    elif fault == 'claims_measurement': gain['prediction_only'] = False
    else:
        if fault == 'source_request': trigger['predicted_gain_evidence']['source_binding']['request_id'] += 1
        elif fault == 'profile_target': trigger['predicted_gain_evidence']['contact_overlap_requirements_m'][0] = .004
        else: del trigger['predicted_gain_evidence']
        trigger.pop('evidence_sha256')
        trigger['evidence_sha256'] = evidence_digest(trigger)
    assert observe_return_audit_error(report)


def test_lower_prediction_target_does_not_start_supplement_after_actual_no_gain(configured, monkeypatch):
    from test_observe_return_supplement_remote import continuation
    node, prepared = continuation(configured, monkeypatch)
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    bind_task_target(node, dict(request_id=prepared.ticket.request_id,
        generation=prepared.ticket.generation, target_epoch=prepared.ticket.target_epoch,
        target_track_id=prepared.snapshot.target_identity.track_id,
        snapshot_stamp_ns=prepared.snapshot.stamp_ns))
    assert node._observe_return_predicted_gain_target_m() == pytest.approx(.002)
    node._observe_return_step_gain['ok'] = False
    before = node._active_multiview_surface()[0]
    assert not node._observe_return_prepare_supplement(prepared, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert node._observe_return_phase == 'observing_contact'
    assert node._observe_return_pending_supplement is None
    assert node._active_multiview_surface()[0] is before


@pytest.mark.parametrize('entry', ['completed_supplement', 'yield_supplement', 'mixed_initial_fallback'])
def test_positive_temporal_evidence_joins_real_mixed_failure_opportunity_sources(
        configured, monkeypatch, entry):
    from dataclasses import asdict
    from test_observation_information_uncertainty import proposal_profiles, proposal_snapshot
    from test_observe_return_supplement_remote import continuation
    from test_contact_search_yield_remote import ready, search, CODE
    if entry == 'yield_supplement':
        node, prepared, clock, candidates = ready(configured, monkeypatch)
    else:
        node, prepared = continuation(configured, monkeypatch)
    ns = prepared.snapshot.stamp_ns
    temporal = proposal_snapshot(stamps=(ns-2, ns-1, ns))
    prepared.snapshot.quality = temporal.quality
    prepared.snapshot.sample_stamp_ns = temporal.sample_stamp_ns
    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles'] = [
        asdict(p) for p in proposal_profiles()]
    code = 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE'
    if entry == 'yield_supplement':
        selection, calls, _ = search(node, prepared, clock, candidates, monkeypatch, seconds=[15., 15.])
        assert selection.termination_reason == CODE and calls == [1, 2]
        code = CODE
    if entry == 'mixed_initial_fallback':
        node._observe_return_phase = 'initial_contact'
        node._observe_return_session = None
    # Use the production creator, without pre-inserting a missing binding key.
    opportunity = node._observe_return_mixed_visual_failure(prepared, code)
    assert opportunity is not None
    if entry == 'mixed_initial_fallback':
        assert node._observe_return_fallback(prepared, visual_opportunity=opportunity)
    else:
        deadline = node._observe_return_session['deadline_ns']
        assert node._observe_return_prepare_supplement(prepared, code)
        assert node._observe_return_session['deadline_ns'] == deadline
        assert not node._observe_return_pending_supplement['contact_or_motion_authorized']
    assert node._observe_return_phase == 'observation_prefix'
    assert node._observe_return_predicted_gain_target_m() == pytest.approx(.002)
    assert node._observe_return_view_uncertainty_m == pytest.approx(.003)
    bound = node._observe_return_trigger_evidence['predicted_gain_evidence']['source_binding']
    assert bound['request_id'] == prepared.ticket.request_id
    assert bound['snapshot_stamp_ns'] == ns
    assert bound['target_track_id'] == prepared.snapshot.target_identity.track_id
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


@pytest.mark.parametrize('fault', ['nonfinite_combined', 'nonfinite_execution', 'negative_perception'])
def test_positive_task_target_cannot_bypass_original_visual_profile_validation(
        configured, monkeypatch, fault):
    from dataclasses import asdict
    from test_measured_observation_planning import preflight_context
    from test_observation_information_uncertainty import proposal_profiles, proposal_snapshot
    node, ticket, snapshot, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    temporal = proposal_snapshot(stamps=(snapshot.stamp_ns-2, snapshot.stamp_ns-1, snapshot.stamp_ns))
    snapshot.quality = temporal.quality; snapshot.sample_stamp_ns = temporal.sample_stamp_ns
    profiles = [asdict(p) for p in proposal_profiles()]
    if fault == 'nonfinite_combined': profiles[0]['depth_uncertainty_m'] = float('nan')
    elif fault == 'nonfinite_execution': profiles[0]['execution_position_error_m'] = float('inf')
    else: profiles[0]['depth_uncertainty_m'] = 0.
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: profiles)
    assert not node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, candidates, diagnostics)
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_trigger_evidence is None


@pytest.mark.parametrize('computed_zero', [False, True])
@pytest.mark.parametrize('fault', ['nonfinite_combined', 'nonfinite_execution', 'negative_perception'])
def test_fallback_with_valid_task_scale_but_invalid_visual_split_has_no_new_target(
        configured, monkeypatch, computed_zero, fault):
    from dataclasses import asdict
    from test_rgbd_snapshot import sample, stable_mask, fuse
    from test_observe_return_supplement_remote import continuation
    from test_observation_information_uncertainty import proposal_profiles
    node, prepared = continuation(configured, monkeypatch)
    node._observe_return_phase = 'initial_contact'; node._observe_return_session = None
    ns = prepared.snapshot.stamp_ns
    samples = []
    for index, value in enumerate((2200, 2200, 2200) if computed_zero else (2199, 2200, 2201)):
        stamp = ns-(2-index)*30_000_000
        item = sample(value, stable_mask(), stamp*1e-9)
        item.stamp_ns = stamp; item.target_identity = prepared.snapshot.target_identity
        item.target_epoch = prepared.snapshot.target_identity.epoch
        samples.append(item)
    temporal = fuse(samples)
    assert temporal.ok and temporal.quality.depth_repeatability_sample_count > 0
    assert (temporal.quality.depth_repeatability_m == 0.) is computed_zero
    prepared.snapshot.quality = temporal.quality
    prepared.snapshot.sample_stamp_ns = temporal.sample_stamp_ns
    profiles = [asdict(p) for p in proposal_profiles(repeatability=temporal.quality.depth_repeatability_m)]
    assert all(p['contact_overlap_requirement_m'] == .002 for p in profiles)
    if fault == 'nonfinite_combined': profiles[0]['depth_uncertainty_m'] = float('nan')
    elif fault == 'nonfinite_execution': profiles[0]['execution_position_error_m'] = float('inf')
    else: profiles[0]['depth_uncertainty_m'] = 0.
    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles'] = profiles
    opportunity = node._observe_return_mixed_visual_failure(prepared, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert opportunity is not None
    node._observe_return_fallback(prepared, visual_opportunity=opportunity)
    assert node._observe_return_predicted_gain_target_m() is None
    assert 'predicted_gain_evidence' not in (node._observe_return_trigger_evidence or {})
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)


@pytest.mark.parametrize('physical_gate', ['optical_side', 'arrival_error_box'])
def test_lower_task_target_still_rejects_real_inadmissible_recorded_endpoints(monkeypatch, physical_gate):
    original = proposals.generate_reachable_observation_views
    calls = []
    def targeted(**kwargs):
        # The arrival fixture first finds an old nominal endpoint; keep that
        # original control unchanged, then test its rejection with target 2mm.
        if physical_gate == 'optical_side' or calls:
            kwargs['predicted_gain_target_m'] = .002
        calls.append(deepcopy(kwargs.get('predicted_gain_target_m')))
        return original(**kwargs)
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', targeted)
    if physical_gate == 'optical_side':
        from test_reachable_observation_optical_side import test_claimed_solver_success_cannot_return_gen14_original_side_deficient_endpoint
        test_claimed_solver_success_cannot_return_gen14_original_side_deficient_endpoint(monkeypatch)
    else:
        from test_reachable_observation_arrival_reserve import test_nominal_solver_success_cannot_override_arrival_reserve
        test_nominal_solver_success_cannot_override_arrival_reserve(monkeypatch)
    assert calls[-1] == .002


def test_unavailable_fk_backend_keeps_existing_cartesian_view_with_task_target(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    from test_observation_information_uncertainty import proposal_profiles
    node, params, source, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    bind_task_target(node, source['binding'])
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: prepared.stamp)
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: proposal_profiles())
    T = np.asarray(recorded_proposal['T_base_tool0'])
    pose = base.make_pose_stamped('base_link', T[:3, 3], base.quaternion_from_matrix(T), stamp=prepared.stamp)
    sequence = NS(pregrasp=pose, adaptive_stage_profile=NS(depth_uncertainty_m=.003), observation_view_audit={})
    existing = dict(sequence=sequence, search_family='camera_orientation_lattice',
        preference_index=7, observation_translation_delta_m=.05)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', lambda *a, **k: ((existing,), ()))
    def unavailable():
        raise ImportError('isolated missing numerical backend')
    monkeypatch.setattr(proposals, '_backend', unavailable)
    rows, rejected = node._far_field_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, reference, prepared)
    assert len(rows) == 1 and rows[0]['sequence'] is sequence
    assert rows[0]['search_family'] == 'camera_orientation_lattice'
    assert node._joint_space_observation_cache['audit']['status'] == 'backend_unavailable'
    assert rows[0]['observation_view']['working_volume']['ok']
    gain = rows[0]['observation_view']['observe_return_missing_side_gain']
    assert gain['ok'] and gain['predicted_gain_target_m'] == pytest.approx(.002)
    assert gain['depth_uncertainty_m'] == pytest.approx(.003)


def test_noncentered_fk_joins_original_gates_without_changing_contact_family(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    node, _, source, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    install_proposal(node, monkeypatch, recorded_proposal)
    profile = node._contact_stage_profile(None, None)
    sequence = NS(pregrasp=reference, approach=deepcopy(reference), grasp=deepcopy(reference),
        lift=deepcopy(reference), adaptive_stage_profile=profile, observation_view_audit={})
    existing = (dict(sequence=sequence, preference_index=500),)
    rows, rejected = node._joint_space_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, prepared, existing)
    assert len(rows) == 1 and not rejected
    variant = rows[0]; result = variant['sequence']
    assert result is not sequence and sequence.pregrasp is reference
    for name in ('approach', 'grasp', 'lift'):
        assert getattr(result, name) == getattr(sequence, name)
    assert vars(result.adaptive_stage_profile) == vars(profile)
    np.testing.assert_allclose(base.pose_matrix(result.pregrasp), recorded_proposal['T_base_tool0'], atol=1e-12)
    assert result.pregrasp.header.stamp == prepared.stamp
    assert variant['preference_index'] == 501 and variant['search_family'] == 'joint_space_fk'
    assert variant['observation_view']['center_residual_m'] > .001
    assert variant['observation_view']['construction'] == 'measured_joint_space_fk'
    assert variant['observation_envelope'].ok
    assert variant['observation_side_evidence']['observation_side_evidence_deficit_m'] == 0.
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', lambda *a, **k: ((), ()))
    passed, rejected = node._far_field_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, reference, prepared)
    assert len(passed) == 1 and not rejected
    view = passed[0]['observation_view']
    assert view['working_volume']['ok'] and view['observe_return_missing_side_gain']['ok']
    assert view['information_utility']['prediction_only']
    assert not view['proposal']['execution_authority']
    runtime = dict(observation_variants=passed)
    node._activate_far_field_observation_variant(runtime, passed[0], 1)
    assert runtime['observation_orientation_search']['joint_space_fk_proposals_used']
    assert runtime['observation_view']['source_stamp_ns'] == source['binding']['snapshot_stamp_ns']


@pytest.mark.parametrize('fault', ['side', 'support', 'target', 'gain', 'baseline', 'volume'])
def test_fk_proposal_cannot_bypass_original_endpoint_and_view_gates(
        configured, monkeypatch, recorded_inputs, recorded_proposal, fault):
    node, _, _, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    install_proposal(node, monkeypatch, recorded_proposal)
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_far_field_observation_variants', lambda *a, **k: ((), ()))
    if fault == 'side':
        monkeypatch.setattr(node, '_observation_side_evidence', lambda *a: {'observation_side_evidence_deficit_m': .001})
    elif fault in ('support', 'target'):
        monkeypatch.setattr(node, '_observation_envelope_gate', lambda *a:
            base.ObservationEnvelopeResult(False, 'OBSERVATION_'+fault.upper()+'_COLLISION', 'injected collision', -.001))
    elif fault == 'gain':
        node._observe_return_view_gaps = ()
    elif fault == 'baseline':
        monkeypatch.setattr(node, '_frozen_observation_translation_delta_m', lambda *a: .001)
    else:
        monkeypatch.setattr(node, '_camera_intrinsics', lambda: NS(fx=5000., fy=5000., cx=320., cy=240., width=640, height=480))
    passed, rejected = node._far_field_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, reference, prepared)
    assert not passed and len(rejected) == 1
    assert rejected[0]['search_family'] == 'joint_space_fk'


@pytest.mark.parametrize('phase', ['initial_contact', 'observing_contact', 'returned_contact', 'direct', 'carton', 'silhouette'])
def test_new_generator_is_not_entered_outside_ordinary_missing_side_prefix(configured, monkeypatch, phase):
    node, _, _ = setup_observe_return(configured)
    node._observe_return_phase = phase
    node._observe_return_trigger = 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    if phase in ('direct', 'carton', 'silhouette'):
        node._observe_return_phase = 'observation_prefix'
        if phase == 'direct': node._mode_selection['strategy'] = 'direct'
        elif phase == 'carton': node._mode_selection['mode'] = 'carton'
        else: node._observe_return_diagnostic_observation_only = True
    monkeypatch.setattr(module.rospy, 'get_param', lambda *a: pytest.fail('outside phase read ROS'))
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', lambda **k: pytest.fail('outside phase ran solver'))
    assert node._joint_space_observation_candidates(object()) == ((), {})


def test_zero_angle_fk_still_spends_shared_budget_and_preserves_roll_fallback():
    from test_remote_grasp6d_streaming import streaming_node, MutableClock, snapshot, remote_node
    node = streaming_node(clock=MutableClock(50.), start_worker=False)
    node.execution_plan_validity_sec = 120.
    try:
        node.start_streaming(); node.submit_stream_snapshot(snapshot(49.8))
        ticket = node._stream_worker_ticket
        node.mujoco_selection_snapshot_reserve_sec = 30.
        elapsed = MutableClock(0.); node._observation_tilt_clock = elapsed
        variants = []
        for index in range(8):
            pose = remote_node.PoseStamped(); pose.pose.position.x = index
            variants.append(dict(sequence=NS(pregrasp=pose), camera_tilt_offset_deg=0.,
                camera_roll_offset_deg=0., preference_index=index,
                search_family='joint_space_fk' if index < 7 else 'camera_orientation_lattice'))
        runtime = dict(prepared=NS(ticket=ticket), grasp_pose=variants[0]['sequence'].pregrasp,
            observation_sequence=variants[0]['sequence'], observation_variants=tuple(variants), soft_evidence={})
        node._stable_variant_runtime = {(3, 1): runtime}; checked = []
        def strict(pose, observation=False):
            checked.append(int(pose.pose.position.x)); elapsed.value += .1
            return remote_node.MoveItResult(False, 0., 0., 'no path', failure_code='MOVEIT_UNREACHABLE'), {}, ''
        node._strict_moveit_evaluation = strict
        node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert checked == [0, 1, 2, 3, 4, 5, 7]
        node._check_moveit_stable_candidate(NS(track_id=3, variant_index=1))
        assert checked == [0, 1, 2, 3, 4, 5, 7, 7]
        assert node._observation_tilt_search_budget['checked_count'] == 6
    finally:
        node.shutdown_streaming_worker()


def test_proposal_cache_regenerates_after_movement_without_renewing_budget(
        configured, monkeypatch, recorded_inputs, recorded_proposal):
    node, params, _, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: prepared.stamp)
    calls = []
    def generate(**kwargs):
        calls.append(kwargs)
        return (deepcopy(recorded_proposal),), {'elapsed_sec': min(2.6, kwargs['max_seconds']),
            'execution_authority': False, 'candidate_count': 1}
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', generate)
    first, report = node._joint_space_observation_candidates(prepared)
    assert first and not report['cache_hit']
    second, report = node._joint_space_observation_candidates(prepared)
    assert second and report['cache_hit'] and len(calls) == 1
    assert calls[0]['joint_error_bounds_rad'] == pytest.approx(recorded_inputs['joint_error_bounds_rad'])
    # Actual seed movement beyond one SDK count still requires a new search.
    node.latest_joint_state.position[0] += .002
    third, _ = node._joint_space_observation_candidates(prepared)
    assert third and len(calls) == 2 and calls[1]['max_seconds'] == pytest.approx(2.4)
    node.latest_joint_state.position[0] += .002
    fourth, report = node._joint_space_observation_candidates(prepared)
    assert not fourth and report['status'] == 'budget_exhausted' and len(calls) == 2


def test_actual_frozen_source_reaches_real_helper_and_keeps_cache_identity(
        configured, monkeypatch, recorded_inputs):
    from types import MappingProxyType
    node, params, source, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    frozen = base._freeze_request_evidence(source)
    assert isinstance(frozen['binding'], MappingProxyType)
    assert not frozen['points_base'].flags.writeable
    node._diagnostic_observation_view_source = frozen
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', lambda p: frozen)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: prepared.stamp)
    first, report = node._joint_space_observation_candidates(prepared)
    assert first and report['status'] in ('complete', 'budget_exhausted'), report
    assert not report['cache_hit'] and not report['execution_authority']
    for row in first:
        assert row['binding'] == dict(frozen['binding']) == source['binding']
        assert not row['complete_path_checked'] and not row['execution_authority']
        for gate in ('missing_side_view_gain', 'working_volume', 'observation_envelope',
                     'conditional_endpoint_support', 'measured_support_projection'):
            assert row['endpoint_checks'][gate]['ok']
    cache = node._joint_space_observation_cache
    used = cache['used_sec']
    assert cache['source'] is frozen and cache['prepared'] is prepared
    # Caller changes to a returned proposal cannot rewrite cached/frozen evidence.
    first[0]['binding']['target_track_id'] = 'changed-output-only'
    second, report = node._joint_space_observation_candidates(prepared)
    assert second and report['cache_hit']
    assert second[0]['binding'] == source['binding']
    assert cache['used_sec'] == used and cache['source'] is frozen
    assert node._diagnostic_observation_view_source is frozen
    for name in ('points_base', 'corners_base', 'tool_from_camera'):
        np.testing.assert_array_equal(frozen[name], source[name])
        assert not frozen[name].flags.writeable


@pytest.mark.parametrize('fault', ['stale_joint', 'nonfinite_joint', 'changed_model', 'changed_q'])
def test_bad_or_changed_generation_inputs_cannot_supply_fk_proposals(
        configured, monkeypatch, recorded_inputs, recorded_proposal, fault):
    node, params, _, prepared, _ = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {name: {'trajectory': .2, 'goal': .035} for name in names}
    node.latest_joint_state = NS(name=names, position=list(recorded_inputs['current_joint_positions']),
        header=NS(stamp=prepared.stamp))
    now = base.rospy.Time.from_sec(prepared.stamp.to_sec()+(.6 if fault == 'stale_joint' else 0.))
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: now)
    if fault == 'nonfinite_joint': node.latest_joint_state.position[0] = float('nan')
    calls = []
    def generate(**kwargs):
        calls.append(True)
        if fault == 'changed_model': params['/robot_description'] += '\n'
        if fault == 'changed_q': node.latest_joint_state.position[0] += .002
        return (deepcopy(recorded_proposal),), {'elapsed_sec': .1}
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', generate)
    rows, report = node._joint_space_observation_candidates(prepared)
    assert not rows
    assert bool(calls) == (fault in ('changed_model', 'changed_q'))
    if calls: assert report['status'] == 'inputs_changed'
