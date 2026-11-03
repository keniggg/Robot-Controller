"""Mode-layer expansion and truthful rejection evidence; no device access."""
import numpy as np
import pytest
from test_remote_modes import configured, base
from test_observe_return_remote import setup_observe_return
from test_observe_return_visual import directed_expansion_fixture
from test_observation_information_selection import information_context, candidate_view, qualify_variants


def test_mode_missing_side_branch_enables_geometry_expansion(configured):
    node, _, _ = setup_observe_return(configured)
    gap, rotation, normal = directed_expansion_fixture(46.)
    identity = node._current_stream_target_identity()
    node._observe_return_phase = 'observation_prefix'
    node._observe_return_view_gaps = (dict(gap, target_epoch=identity.epoch,
        target_track_id=identity.track_id),)
    node._observe_return_view_uncertainty_m = .003
    node._observe_return_diagnostic_observation_only = False
    node.grasp_config = {'observation_camera_target_nominal_distance_m': .26}
    node.observation_camera_tilt_offsets_deg = (10.,20.,30.)
    node._tool_from_camera_matrix = lambda: np.eye(4)
    transform = np.eye(4); transform[:3,:3] = rotation
    reference = base.make_pose_stamped('base_link', gap['initial_camera_origin_base_m'],
        base.quaternion_from_matrix(transform), stamp=base.rospy.Time(1))
    variants = node._observation_reference_tilt_variants(reference, normal)
    expanded = [metadata for _, metadata in node._observe_return_directed_views
        if metadata.get('proposal_tier') == 'geometry_directed_expansion']
    assert variants and expanded
    assert min(item['camera_rotation_offset_deg'] for item in expanded) == pytest.approx(46.)
    assert all(item['observation_only'] and not item['contact_or_motion_authorized'] for item in expanded)
    assert node.observation_camera_tilt_offsets_deg == (10.,20.,30.)
    node._observe_return_phase = 'initial_contact'
    assert all(v['camera_tilt_offset_deg'] <= 30. for v in
        node._observation_reference_tilt_variants(reference, normal))


@pytest.mark.parametrize('stage', ['information_gain', 'view_baseline', 'working_volume'])
def test_later_rejection_retains_passed_cad_clearance(configured, monkeypatch, stage):
    node, geometry, _, _ = information_context(configured, monkeypatch)
    variant = candidate_view(geometry, .001 if stage == 'information_gain' else .012,
        distance=.04 if stage == 'working_volume' else .3)
    variant['observation_envelope'] = base.ObservationEnvelopeResult(True, '', '', .019)
    variant['camera_target_distance_m'] = .3
    if stage == 'view_baseline': variant['observation_translation_delta_m'] = .001
    passing, rejected = qualify_variants(node, geometry, [variant], monkeypatch)
    assert not passing and len(rejected) == 1
    row = rejected[0]
    assert row['rejection_stage'] == stage
    assert row['minimum_support_clearance_m'] == .019
    assert row['observation_envelope']['ok'] is True
    summary = base.summarize_observation_view_rejections(rejected)
    assert summary['rejection_stage_counts'] == {stage: 1}
    assert 'OBSERVATION_SUPPORT_COLLISION' not in summary['failure_code_counts']
