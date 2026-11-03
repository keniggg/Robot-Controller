"""Information resolution is not a commanded endpoint residual or contact margin."""
from copy import deepcopy
from dataclasses import asdict
from types import MappingProxyType, SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes.observation_information_uncertainty import observation_information_uncertainty_m as visual_noise
from alicia_grasp_modes.active_observation_policy import registered_information_gain
from alicia_flexible_grasp.grasp.adaptive_stage_profiles import AdaptiveStageLimits, derive_adaptive_stage_profiles
from test_remote_modes import configured


def profiles():
    return derive_adaptive_stage_profiles(object_height_m=.023, depth_repeatability_m=.001,
        finger_length_m=.06, support_clearance_m=.003, limits=AdaptiveStageLimits(),
        execution_position_error_m=.01055195)


def test_real_contact_profiles_keep_every_distance_and_error_component_unchanged():
    contact = profiles(); before = [asdict(p) for p in contact]
    assert visual_noise(contact) == pytest.approx(.003)
    assert all(p.depth_uncertainty_m == pytest.approx(.01355195) for p in contact)
    assert all(p.execution_position_error_m == pytest.approx(.01055195) for p in contact)
    assert [asdict(p) for p in contact] == before
    frozen = tuple(MappingProxyType(row) for row in before)
    assert visual_noise(frozen) == pytest.approx(.003)


@pytest.mark.parametrize('row', [
    {}, {'depth_uncertainty_m': .013}, {'execution_position_error_m': .01},
    {'depth_uncertainty_m': float('nan'), 'execution_position_error_m': .01},
    {'depth_uncertainty_m': .013, 'execution_position_error_m': float('inf')},
    {'depth_uncertainty_m': True, 'execution_position_error_m': 0.},
    {'depth_uncertainty_m': .013, 'execution_position_error_m': False},
    {'depth_uncertainty_m': '.013', 'execution_position_error_m': .01},
    {'depth_uncertainty_m': .003, 'execution_position_error_m': -.01},
    {'depth_uncertainty_m': .003, 'execution_position_error_m': .01},
    {'depth_uncertainty_m': .003, 'execution_position_error_m': .003},
    {'depth_uncertainty_m': 0., 'execution_position_error_m': 0.},
])
def test_missing_or_invalid_split_never_defaults_execution_error_to_zero(row):
    with pytest.raises(ValueError): visual_noise([row])


def test_empty_profiles_fail_and_largest_current_perception_term_is_used():
    with pytest.raises(ValueError): visual_noise([])
    assert visual_noise([NS(depth_uncertainty_m=.02, execution_position_error_m=.017),
        NS(depth_uncertainty_m=.012, execution_position_error_m=.007)]) == pytest.approx(.005)


def test_registered_supplement_carries_visual_term_without_editing_contact_diagnostics(configured, monkeypatch):
    from test_observe_return_supplement_remote import continuation
    node, prepared = continuation(configured, monkeypatch)
    contact = [asdict(p) for p in profiles()]
    prepared.remote_diagnostics['tabletop_geometry']['adaptive_stage_profiles'] = contact
    before = deepcopy(contact)
    assert node._observe_return_prepare_supplement(prepared, 'NEAR_FIELD_NO_HARD_SAFE_CANDIDATE')
    assert node._observe_return_view_uncertainty_m == pytest.approx(.003)
    assert contact == before


def test_ordinary_sequence_and_final_gain_share_visual_term_without_contact_poses(configured, monkeypatch, tmp_path):
    from test_measured_observation_planning import independent_context
    from test_remote_modes import base
    from alicia_grasp_modes import observe_return_remote as remote
    node, prepared, _, _ = independent_context(configured, monkeypatch, tmp_path)
    contact = profiles(); before = [asdict(p) for p in contact]
    monkeypatch.setattr(node, '_adaptive_stage_profiles', lambda *_: contact)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    reference = node._frozen_observation_reference_pose(prepared)
    sequence = node._make_observation_sequence(reference, prepared.geometry, [0,0,-1],
        prepared.snapshot, reference, .26)
    assert sequence.adaptive_stage_profile.depth_uncertainty_m == pytest.approx(.003)
    assert not any(hasattr(sequence, name) for name in ('approach', 'grasp', 'lift'))
    captured = []
    def gain(gaps, camera, uncertainty):
        captured.append(uncertainty)
        return dict(ok=False, code='OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED')
    monkeypatch.setattr(remote, 'missing_side_view_gain', gain)
    node._observe_return_view_gain(sequence)
    assert captured == [pytest.approx(.003)]
    assert [asdict(p) for p in contact] == before


def test_actual_registered_gain_callback_receives_same_visual_term(configured, monkeypatch):
    from test_observe_return_supplement_remote import v2_session, direct_observation, module
    from alicia_grasp_modes import observe_return_remote as remote
    node, _, clock, _ = v2_session(configured, monkeypatch)
    node._observe_return_view_uncertainty_m = visual_noise(profiles())
    observation = direct_observation(node, 19_000_000_000)
    monkeypatch.setattr(node, '_resolve_reached_view_observation', lambda _: observation)
    captured = []; original = remote.registered_information_gain
    def actual(*args, **kwargs):
        captured.append(kwargs['depth_uncertainty_m'])
        return original(*args, **kwargs)
    monkeypatch.setattr(remote, 'registered_information_gain', actual)
    phase = NS(active=True, phase_id=1, header=NS(stamp=module.rospy.Time.from_sec(20.),
        frame_id='base_link'), deadline=module.rospy.Time.from_sec(70.))
    clock[0] = 21_000_000_000
    node.near_field_state_cb(phase)
    assert node._latest_registration_evidence.result.ok
    assert captured == [pytest.approx(.003)]
    assert node._observe_return_step_gain['depth_uncertainty_m'] == pytest.approx(.003)


@pytest.mark.parametrize('fault', ['below_noise', 'only_two', 'old_provenance', 'registration', 'identity', 'source_chain'])
def test_visual_resolution_keeps_registered_gain_noise_and_source_gates(fault):
    from test_active_observation_policy import surfaces
    distance = .0029 if fault == 'below_noise' else .01
    extra = [[distance, y, .02] for y in (.001, .011, .021)]
    if fault == 'only_two': extra = extra[:2]
    before, after = surfaces(extra); registration = NS(ok=True)
    if fault == 'old_provenance': after.view_indices[:] = 0
    if fault == 'registration': registration.ok = False
    if fault == 'identity': after.identity = NS(epoch=5, track_id='foreign')
    if fault == 'source_chain': after.view_stamps_ns = (9, 20)
    if fault in ('registration', 'identity', 'source_chain'):
        with pytest.raises(ValueError):
            registered_information_gain(before, after, registration, depth_uncertainty_m=visual_noise(profiles()))
    else:
        result = registered_information_gain(before, after, registration, depth_uncertainty_m=visual_noise(profiles()))
        assert not result['ok'] and result['minimum_novel_voxels'] == 3
        assert result['contact_or_motion_authorized'] is False


def proposal_snapshot(repeatability=.00014825999625463736, stamps=(10, 20, 30)):
    from alicia_flexible_grasp.vision.rgbd_snapshot import DepthQuality
    return NS(stamp_ns=stamps[-1], sample_stamp_ns=stamps, quality=DepthQuality(
        fused_frames=len(stamps), mask_area=300, valid_depth_points=250,
        valid_depth_ratio=250./300., depth_median_m=.21, depth_mad_m=.0025,
        depth_repeatability_m=repeatability))


def proposal_profiles(*, height=.023, repeatability=.00014825999625463736,
        support=.003, execution=.01055195, floor=.002, scale=2.):
    return derive_adaptive_stage_profiles(object_height_m=height,
        depth_repeatability_m=repeatability, finger_length_m=.06,
        support_clearance_m=support, execution_position_error_m=execution,
        limits=AdaptiveStageLimits(contact_overlap_min_m=floor, depth_uncertainty_scale=scale))


def proposal_target(contact, snapshot):
    # Import at call time so red runs retain the old suite and expose exactly
    # the missing behavior, instead of turning it into a collection failure.
    from alicia_grasp_modes.observation_information_uncertainty import observation_predicted_gain_target_m
    return observation_predicted_gain_target_m(contact, snapshot)


@pytest.mark.parametrize('height', [.008, .023, .064])
@pytest.mark.parametrize('support,execution', [(.003, 0.), (.003, .01), (.007, .01)])
def test_task_target_is_independent_of_object_height_support_and_execution_margin(
        height, support, execution):
    contact = proposal_profiles(height=height, support=support, execution=execution)
    before = [asdict(p) for p in contact]
    snapshot = proposal_snapshot()
    assert proposal_target(contact, snapshot) == pytest.approx(.002)
    assert visual_noise(contact) == pytest.approx(support)
    assert all(p.depth_uncertainty_m == pytest.approx(support+execution) for p in contact)
    assert [asdict(p) for p in contact] == before
    frozen = tuple(MappingProxyType(row) for row in before)
    assert proposal_target(frozen, snapshot) == pytest.approx(.002)


@pytest.mark.parametrize('repeatability,scale,floor,expected', [
    (.00014825999625463736, 2., .002, .002),
    (.0015, 2., .002, .003), (.003, 2., .002, .006),
    (.001, 3., .002, .003), (.00014825999625463736, 2., .004, .004),
])
def test_task_target_tracks_existing_contact_requirement_without_clipping_to_visual_noise(
        repeatability, scale, floor, expected):
    contact = proposal_profiles(repeatability=repeatability, scale=scale, floor=floor)
    assert proposal_target(contact, proposal_snapshot(repeatability)) == pytest.approx(expected)
    assert all(p.contact_overlap_requirement_m == pytest.approx(expected) for p in contact)


def test_task_target_uses_largest_explicit_profile_requirement_without_mutation():
    rows = [asdict(p) for p in proposal_profiles()]
    rows[-1]['contact_overlap_requirement_m'] = .0045
    before = deepcopy(rows)
    assert proposal_target(rows, proposal_snapshot()) == pytest.approx(.0045)
    assert rows == before


@pytest.mark.parametrize('value', [0., -.001, float('nan'), float('inf'), True, False, '.002', None])
def test_explicit_invalid_contact_requirement_never_enables_a_lower_target(value):
    rows = [asdict(p) for p in proposal_profiles()]
    rows[-1]['contact_overlap_requirement_m'] = value
    with pytest.raises(ValueError):
        proposal_target(rows, proposal_snapshot())


@pytest.mark.parametrize('fault', ['empty_profiles', 'missing_contact', 'missing_quality',
    'missing_repeatability', 'zero_repeatability', 'single_frame', 'missing_stamps', 'missing_stamp'])
def test_unmeasured_or_missing_task_target_keeps_legacy_branch(fault):
    rows = [asdict(p) for p in proposal_profiles()]
    snapshot = proposal_snapshot()
    if fault == 'empty_profiles': rows = []
    elif fault == 'missing_contact': del rows[-1]['contact_overlap_requirement_m']
    elif fault == 'missing_quality': del snapshot.quality
    elif fault == 'missing_repeatability':
        values = vars(snapshot.quality).copy(); del values['depth_repeatability_m']
        snapshot.quality = NS(**values)
    elif fault == 'zero_repeatability': snapshot = proposal_snapshot(0.)
    elif fault == 'single_frame': snapshot = proposal_snapshot(stamps=(30,))
    elif fault == 'missing_stamps': del snapshot.sample_stamp_ns
    else: del snapshot.stamp_ns
    assert proposal_target(rows, snapshot) is None


@pytest.mark.parametrize('fault', ['duplicate_stamp', 'count_mismatch', 'future_stamp',
    'nonpositive_stamp', 'bool_stamp', 'bad_repeatability', 'failed_snapshot'])
def test_contradictory_source_evidence_cannot_enable_a_lower_task_target(fault):
    snapshot = proposal_snapshot()
    if fault == 'duplicate_stamp': snapshot.sample_stamp_ns = (10, 10, 30)
    elif fault == 'count_mismatch': snapshot.sample_stamp_ns = (10, 30)
    elif fault == 'future_stamp': snapshot.sample_stamp_ns = (10, 20, 31)
    elif fault == 'nonpositive_stamp': snapshot.sample_stamp_ns = (0, 20, 30)
    elif fault == 'bool_stamp': snapshot.sample_stamp_ns = (True, 20, 30)
    elif fault == 'failed_snapshot': snapshot.ok = False
    else: snapshot.quality = NS(**dict(vars(snapshot.quality), depth_repeatability_m=float('nan')))
    with pytest.raises(ValueError):
        proposal_target(proposal_profiles(), snapshot)


def test_lower_prediction_target_cannot_turn_subnoise_registered_points_into_progress():
    from test_active_observation_policy import surfaces
    target = proposal_target(proposal_profiles(), proposal_snapshot())
    assert target == pytest.approx(.002)
    before, after = surfaces([[.0029, y, .02] for y in (.001, .011, .021)])
    noise = visual_noise(proposal_profiles())
    result = registered_information_gain(before, after, NS(ok=True), depth_uncertainty_m=noise)
    assert noise == pytest.approx(.003)
    assert not result['ok']
    assert result['novelty_distance_m'] == pytest.approx(.003)
    assert result['contact_or_motion_authorized'] is False


def test_lower_prediction_target_does_not_reduce_existing_distinct_view_tolerance():
    from alicia_grasp_modes.active_observation_policy import distinct_view
    target = proposal_target(proposal_profiles(), proposal_snapshot())
    noise = visual_noise(proposal_profiles())
    seen = [[.2, 0., 0.]]
    assert target == pytest.approx(.002) and noise == pytest.approx(.003)
    # This ray difference lies between the original 2.5mm voxel floor and
    # retained 3mm information uncertainty, so replacing noise with target
    # would incorrectly make this already-visited view appear distinct.
    assert distinct_view([.2, .0028, 0.], [0., 0., 0.], seen, target)
    assert not distinct_view([.2, .0028, 0.], [0., 0., 0.], seen, noise)


def proposal_binding():
    return dict(request_id=5, generation=2, target_epoch=4,
        target_track_id='source-track', snapshot_stamp_ns=30)


def frozen_proposal_evidence():
    from alicia_grasp_modes.observation_information_uncertainty import observation_predicted_gain_evidence
    return observation_predicted_gain_evidence(proposal_profiles(), proposal_snapshot(), proposal_binding())


def test_task_prediction_evidence_freezes_source_requirements_without_aliasing_or_authority():
    from alicia_grasp_modes.observation_information_uncertainty import (
        observation_predicted_gain_evidence, bound_observation_predicted_gain_target_m, PREDICTED_GAIN_POLICY)
    profiles = [asdict(p) for p in proposal_profiles()]
    snapshot = proposal_snapshot(); snapshot.sample_stamp_ns = list(snapshot.sample_stamp_ns)
    binding = proposal_binding()
    evidence = observation_predicted_gain_evidence(profiles, snapshot, binding)
    before = deepcopy(evidence)
    assert evidence['policy'] == PREDICTED_GAIN_POLICY
    assert evidence['predicted_gain_target_m'] == pytest.approx(.002)
    assert evidence['prediction_only'] is True
    assert evidence['measured_registration_required'] is True
    assert evidence['contact_or_motion_authorized'] is False
    assert bound_observation_predicted_gain_target_m(evidence, binding) == pytest.approx(.002)
    profiles[0]['contact_overlap_requirement_m'] = .02
    snapshot.sample_stamp_ns[0] = 1
    binding['request_id'] += 1
    assert evidence == before
    assert bound_observation_predicted_gain_target_m(evidence, proposal_binding()) == pytest.approx(.002)


@pytest.mark.parametrize('field', ['request_id', 'generation', 'target_epoch', 'target_track_id', 'snapshot_stamp_ns'])
def test_task_prediction_target_cannot_move_to_another_trigger_source(field):
    from alicia_grasp_modes.observation_information_uncertainty import bound_observation_predicted_gain_target_m
    evidence = frozen_proposal_evidence(); binding = proposal_binding()
    binding[field] = 'foreign' if field == 'target_track_id' else binding[field]+1
    with pytest.raises(ValueError):
        bound_observation_predicted_gain_target_m(evidence, binding)


@pytest.mark.parametrize('fault', ['policy', 'target_lower', 'target_higher', 'missing_target',
    'empty_requirements', 'invalid_requirement', 'different_requirement', 'zero_repeatability',
    'duplicate_stamps', 'single_frame', 'not_prediction', 'no_registration', 'motion_authority'])
def test_malformed_or_relabelled_prediction_evidence_is_not_authority(fault):
    from alicia_grasp_modes.observation_information_uncertainty import bound_observation_predicted_gain_target_m
    evidence = frozen_proposal_evidence()
    if fault == 'policy': evidence['policy'] = 'predicted_noise'
    elif fault == 'target_lower': evidence['predicted_gain_target_m'] = .001
    elif fault == 'target_higher': evidence['predicted_gain_target_m'] = .003
    elif fault == 'missing_target': del evidence['predicted_gain_target_m']
    elif fault == 'empty_requirements': evidence['contact_overlap_requirements_m'] = []
    elif fault == 'invalid_requirement': evidence['contact_overlap_requirements_m'][0] = float('nan')
    elif fault == 'different_requirement': evidence['contact_overlap_requirements_m'][0] = .004
    elif fault == 'zero_repeatability': evidence['depth_repeatability_m'] = 0.
    elif fault == 'duplicate_stamps': evidence['sample_stamp_ns'] = [10, 10, 30]
    elif fault == 'single_frame': evidence.update(sample_stamp_ns=[30], fused_frames=1)
    elif fault == 'not_prediction': evidence['prediction_only'] = False
    elif fault == 'no_registration': evidence['measured_registration_required'] = False
    else: evidence['contact_or_motion_authorized'] = True
    with pytest.raises(ValueError):
        bound_observation_predicted_gain_target_m(evidence, proposal_binding())


def test_freezing_task_prediction_cannot_bind_profiles_to_a_different_snapshot():
    from alicia_grasp_modes.observation_information_uncertainty import observation_predicted_gain_evidence
    binding = proposal_binding(); binding['snapshot_stamp_ns'] += 1
    with pytest.raises(ValueError):
        observation_predicted_gain_evidence(proposal_profiles(), proposal_snapshot(), binding)


@pytest.mark.parametrize('fault', ['changed_target', 'changed_profiles', 'bad_policy', 'invalid_then_missing'])
def test_binding_cannot_replace_or_repair_preexisting_invalid_task_evidence(fault):
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    bind = ObserveReturnRemoteMixin._observe_return_bind_predicted_gain
    profiles = proposal_profiles(); snapshot = proposal_snapshot()
    trigger = bind(proposal_binding(), profiles, snapshot)
    if fault == 'changed_target': trigger['predicted_gain_evidence']['predicted_gain_target_m'] = .001
    elif fault == 'changed_profiles': profiles = proposal_profiles(floor=.004)
    elif fault == 'bad_policy': trigger['predicted_gain_evidence']['policy'] = 'noise'
    else:
        trigger['predicted_gain_evidence']['predicted_gain_target_m'] = 0.
        del snapshot.quality
    before = deepcopy(trigger)
    with pytest.raises(ValueError): bind(trigger, profiles, snapshot)
    assert trigger == before


def test_same_frozen_task_evidence_can_repeat_without_new_receipts():
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    from alicia_grasp_modes.active_observation_policy import evidence_digest
    bind = ObserveReturnRemoteMixin._observe_return_bind_predicted_gain
    trigger = proposal_binding(); trigger['evidence_sha256'] = evidence_digest(trigger)
    first = bind(trigger, proposal_profiles(), proposal_snapshot())
    assert bind(first, proposal_profiles(), proposal_snapshot()) == first
    payload = deepcopy(first); digest = payload.pop('evidence_sha256')
    assert digest == evidence_digest(payload)


def test_missing_current_quality_cannot_retain_an_earlier_lower_prediction_target():
    from alicia_grasp_modes.observe_return_remote import ObserveReturnRemoteMixin
    from alicia_grasp_modes.active_observation_policy import evidence_digest
    bind = ObserveReturnRemoteMixin._observe_return_bind_predicted_gain
    trigger = proposal_binding(); trigger['evidence_sha256'] = evidence_digest(trigger)
    first = bind(trigger, proposal_profiles(), proposal_snapshot())
    snapshot = proposal_snapshot(); del snapshot.quality
    legacy = bind(first, proposal_profiles(), snapshot)
    assert 'predicted_gain_evidence' not in legacy
    assert first['predicted_gain_evidence']['predicted_gain_target_m'] == pytest.approx(.002)
    payload = deepcopy(legacy); digest = payload.pop('evidence_sha256')
    assert digest == evidence_digest(payload)


@pytest.mark.parametrize('case', ['computed_zero', 'computed_positive', 'no_repeated_pixels', 'single_frame'])
def test_real_fusion_distinguishes_measured_zero_from_unmeasured_default_zero(case):
    from test_rgbd_snapshot import sample, stable_mask, fuse
    from alicia_grasp_modes.observation_information_uncertainty import (
        observation_predicted_gain_evidence, bound_observation_predicted_gain_target_m)
    mask = stable_mask()
    if case == 'computed_zero':
        shaped = np.full((20, 30), 2200, dtype=np.uint16)
        shaped += np.arange(30, dtype=np.uint16)[np.newaxis, :]
        samples = [sample(shaped, mask, ns) for ns in (1., 1.03, 1.06)]
    elif case == 'computed_positive':
        samples = [sample(value, mask, ns) for value, ns in zip((2199, 2200, 2201), (1., 1.03, 1.06))]
    elif case == 'single_frame':
        samples = [sample(2200, mask, 1.)]
    else:
        # Each pixel has one real depth observation, yet an even-window
        # median remains positive. Fused validity alone cannot prove that
        # temporal repeatability was actually calculated at repeated pixels.
        parity = np.indices((20, 30)).sum(axis=0) % 2 == 0
        first = np.where(parity, 2200, 0).astype(np.uint16)
        second = np.where(parity, 0, 2200).astype(np.uint16)
        samples = [sample(first, mask, 1.), sample(second, mask, 1.03)]
    for item in samples:
        item.stamp_ns = int(round(item.stamp_sec*1e9))
    snapshot = fuse(samples)
    assert snapshot.ok and snapshot.quality.valid_depth_points > 0
    quality = snapshot.quality
    assert quality.fused_frames == len(samples)
    assert len(set(snapshot.sample_stamp_ns)) == len(samples)
    contact = proposal_profiles(repeatability=quality.depth_repeatability_m)
    if case in ('computed_zero', 'computed_positive'):
        assert quality.depth_repeatability_sample_count == quality.valid_depth_points*3
        if case == 'computed_zero':
            assert quality.depth_repeatability_m == 0. and quality.depth_mad_m > 0.
        else:
            assert quality.depth_repeatability_m == pytest.approx(.00014826)
        assert proposal_target(contact, snapshot) == pytest.approx(.002)
        binding = dict(proposal_binding(), snapshot_stamp_ns=snapshot.stamp_ns,
            target_epoch=snapshot.target_identity.epoch, target_track_id=snapshot.target_identity.track_id)
        evidence = observation_predicted_gain_evidence(contact, snapshot, binding)
        assert evidence['depth_repeatability_m'] == quality.depth_repeatability_m
        assert evidence['depth_repeatability_sample_count'] == quality.depth_repeatability_sample_count
        assert bound_observation_predicted_gain_target_m(evidence, binding) == pytest.approx(.002)
        assert evidence['prediction_only'] and not evidence['contact_or_motion_authorized']
    else:
        assert quality.depth_repeatability_m == 0.
        assert quality.depth_repeatability_sample_count == 0
        assert proposal_target(contact, snapshot) is None
    # A zero temporal MAD never means zero actual novelty or side margin.
    assert visual_noise(contact) == pytest.approx(.003)


@pytest.mark.parametrize('count', [1, -1, True, False, '2', 2.0, float('nan')])
def test_explicit_invalid_temporal_support_count_cannot_authorize_task_target(count):
    from dataclasses import replace
    snapshot = proposal_snapshot(0.)
    snapshot.quality = replace(snapshot.quality, depth_repeatability_sample_count=count)
    with pytest.raises(ValueError): proposal_target(proposal_profiles(repeatability=0.), snapshot)


def test_known_temporal_support_cannot_claim_single_frame_repeatability():
    from dataclasses import replace
    snapshot = proposal_snapshot(0., stamps=(30,))
    snapshot.quality = replace(snapshot.quality, depth_repeatability_sample_count=2)
    with pytest.raises(ValueError): proposal_target(proposal_profiles(repeatability=0.), snapshot)


def test_legacy_positive_repeatability_does_not_need_a_fabricated_support_count():
    snapshot = proposal_snapshot()
    values = vars(snapshot.quality).copy(); del values['depth_repeatability_sample_count']
    snapshot.quality = NS(**values)
    assert proposal_target(proposal_profiles(), snapshot) == pytest.approx(.002)
    snapshot.quality.depth_repeatability_sample_count = 0
    assert proposal_target(proposal_profiles(), snapshot) == pytest.approx(.002)
