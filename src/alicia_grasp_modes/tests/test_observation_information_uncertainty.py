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
