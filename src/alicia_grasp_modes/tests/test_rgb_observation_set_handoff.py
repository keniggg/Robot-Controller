"""Set membership never becomes one RGB object or bypasses raw support checks."""
from dataclasses import replace

import numpy as np
import pytest

import alicia_grasp_modes.rgb_observation_set_handoff as subject
from alicia_grasp_modes.rgb_observation_volume import build_rgb_observation_volume
from test_rgb_observation_handoff import STAMP, T, candidate, frame, plane_depth, region


def fixture(regions=None, *, missing_depth=False):
    regs = (region((280, 220, 24, 24)), region((342, 245, 20, 20))) if regions is None else regions
    first = frame(regions=regs, missing_depth=missing_depth)
    volumes = tuple(build_rgb_observation_volume(bbox_xywh=reg.bbox_xywh,
        image_shape=first.bgr.shape[:2], camera_matrix=first.camera_matrix,
        T_base_camera=first.T_base_camera,
        support_plane_base=first.support_provenance.support_plane_base,
        support_provenance=first.support_provenance, foreground_max_m=.12,
        source_stamp_ns=first.source_stamp_ns, working_depth_range_m=(.07, .7)) for reg in regs)
    ref = subject.freeze_rgb_interest_set(first, regs, volumes, support_ring_outer_radius_px=170.)
    return ref, regs, first, volumes


def test_freeze_retains_independent_masks_volumes_and_only_projection_envelope():
    ref, regs, first, volumes = fixture()
    assert len(ref.members) == 2
    assert tuple(m.volume for m in ref.members) == volumes
    assert ref.members[0].mask_fingerprint != ref.members[1].mask_fingerprint
    assert ref.volume.bbox_xywh == (280., 220., 82., 45.)
    assert not hasattr(ref, 'mask')
    assert not hasattr(ref, 'target_id')
    assert ref.source_id == first.source_id
    assert ref.source_stamp_ns == first.source_stamp_ns
    assert ref.source_image_fingerprint == first.image_fingerprint
    assert ref.source_depth_fingerprint == first.depth_fingerprint
    assert ref.generation == first.generation and ref.epoch_ns == first.epoch_ns
    assert ref.selection_stamp_ns == first.selection_stamp_ns
    assert ref.observation_only and not ref.motion_authorized
    assert not ref.metric_geometry_authority and ref.synthetic_measured_points_added == 0
    assert not ref.volume.complete_object_volume


def test_no_depth_is_still_only_a_set_of_rgb_observation_interests():
    ref, regs, _, _ = fixture(missing_depth=True)
    current = frame(STAMP+100_000_000, regions=regs, missing_depth=True)
    result = subject.associate_rgb_interest_set(ref, current, regs)
    assert result.associated and result.reason == 'rgb_interest_set_associated'
    assert result.regions == regs
    assert all(a.measured_candidate is None for a in result.member_associations)
    assert not result.motion_authorized and not result.metric_geometry_authority
    assert result.synthetic_measured_points_added == 0
    assert np.isnan(current.depth_m[regs[0].mask != 0]).all()


def test_every_member_keeps_its_fixed_domain_and_requires_current_support_coverage():
    ref, regs, _, _ = fixture((region((280, 220, 24, 24)), region((420, 220, 24, 24))))
    domains = [m.support_consistency_domain_base.copy() for m in ref.members]
    assert not np.array_equal(domains[0], domains[1])
    moved = T.copy(); moved[0, 3] -= .1
    current = frame(STAMP+100_000_000, T_camera=moved, regions=regs)
    result = subject.associate_rgb_interest_set(ref, current, ())
    assert 'current_support_domain_not_covered' in result.reason
    assert not result.associated and not result.projected_volume_pixels
    audits = [dict(m.support_audit) for m in result.member_associations]
    assert audits[0]['support_current_domain_covered_vertices'] == 8
    assert audits[1]['support_current_domain_covered_vertices'] < 8
    combined = dict(result.support_audit)
    assert combined['support_audit_scope'] == 'first_member_compatibility_fields'
    assert combined['support_failing_member_indices'] == (1,)
    assert combined['support_current_domain_covered_vertices'] == 8
    assert combined['support_member_audits'][1]['audit'] == audits[1]
    assert all(np.array_equal(m.support_consistency_domain_base, d) for m, d in zip(ref.members, domains))
    assert not result.metric_geometry_authority and not result.motion_authorized


def test_second_member_plane_disagreement_remains_visible_when_first_member_passes():
    ref, regs, _, _ = fixture((region((280, 220, 24, 24)), region((420, 220, 24, 24))))
    angle = np.deg2rad(3.)
    normal = np.array([np.sin(angle), 0., np.cos(angle)])
    offset = -normal @ ref.members[0].support_consistency_domain_base.mean(axis=0)
    current = frame(STAMP+100_000_000, regions=regs, plane=tuple(normal)+(offset,))
    current = replace(current, depth_m=plane_depth(current))
    result = subject.associate_rgb_interest_set(ref, current, regs)
    assert 'support_plane_local_distance_inconsistent' in result.reason
    assert not result.associated and not result.projected_volume_pixels
    audit = dict(result.support_audit)
    assert audit['support_reference_max_distance_m'] < .004
    assert audit['support_audit_scope'] == 'first_member_compatibility_fields'
    assert audit['support_failing_member_indices'] == (1,)
    members = audit['support_member_audits']
    assert tuple(row['member_index'] for row in members) == (0, 1)
    assert members[1]['audit']['support_reference_max_distance_m'] > .007
    assert members[1]['reason'].startswith('support_inconsistent:')
    assert members[0]['audit'] == dict(result.member_associations[0].support_audit)


def test_member_domain_mutation_cannot_hide_behind_unchanged_set_fingerprint():
    ref, regs, _, _ = fixture()
    member = ref.members[1]
    object.__setattr__(member, 'support_consistency_domain_base',
        member.support_consistency_domain_base[:4])
    result = subject.associate_rgb_interest_set(ref, frame(STAMP+100_000_000, regions=regs), regs)
    assert 'fixed_support_domain_changed' in result.reason
    assert not result.associated and not result.projected_volume_pixels


def test_proposal_order_does_not_change_frozen_member_order():
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs)
    result = subject.associate_rgb_interest_set(ref, current, tuple(reversed(regs)))
    assert result.associated
    assert tuple(a.rgb_region_index for a in result.member_associations) == (1, 0)
    assert result.regions == regs


def test_distinct_measured_candidates_are_preserved_not_merged_or_selected():
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs)
    measured = tuple(candidate(r, current) for r in regs)
    result = subject.associate_rgb_interest_set(ref, current, regs, measured[::-1])
    assert result.associated
    assert tuple(a.measured_candidate_index for a in result.member_associations) == (1, 0)
    assert all(a.measured_candidate is m for a, m in zip(result.member_associations, measured))
    assert not hasattr(result, 'measured_candidate')
    assert not result.metric_geometry_authority


def test_missing_member_does_not_shrink_set_or_reselect_image_center():
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs[:1])
    result = subject.associate_rgb_interest_set(ref, current, regs[:1])
    assert not result.associated and result.regions == ()
    assert result.reason == 'no_cross_view_rgb_match'
    assert len(result.member_associations) == 2
    assert result.member_associations[0].associated
    assert not result.member_associations[1].associated


def test_empty_proposals_still_audit_all_support_and_project_independent_members():
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs)
    result = subject.associate_rgb_interest_set(ref, current, ())
    assert result.reason == 'no_cross_view_rgb_match' and not result.associated
    assert len(result.projected_volume_pixels) == 8
    assert all(len(a.projected_volume_pixels) == 8 for a in result.member_associations)
    assert all(dict(a.support_audit)['support_reference_fixed'] for a in result.member_associations)
    assert dict(result.support_audit)['support_alignment_applied'] is False


@pytest.mark.parametrize('height', [-.0071, .0071, .020])
def test_even_empty_proposals_cannot_hide_raw_support_height_failure(height):
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs, plane=(0., 0., 1., height))
    result = subject.associate_rgb_interest_set(ref, current, ())
    assert result.reason.startswith('support_inconsistent:')
    assert not result.associated and not result.projected_volume_pixels
    assert all(not a.associated for a in result.member_associations)


def test_raw_support_rotation_above_four_degrees_is_not_aligned():
    ref, regs, _, _ = fixture()
    angle = np.deg2rad(4.1)
    current = frame(STAMP+100_000_000, regions=regs,
        plane=(np.sin(angle), 0., np.cos(angle), 0.))
    result = subject.associate_rgb_interest_set(ref, current, regs)
    assert result.reason.startswith('support_inconsistent:') and not result.associated
    assert dict(result.support_audit)['support_alignment_applied'] is False


@pytest.mark.parametrize('changes', [dict(source_id='different'), dict(generation=2),
    dict(epoch_ns=101), dict(selection_stamp_ns=201)])
def test_source_changes_do_not_rebind_set(changes):
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs, **changes)
    result = subject.associate_rgb_interest_set(ref, current, regs)
    assert result.reason == 'source_context_changed' and not result.associated


@pytest.mark.parametrize('index', [0, 4, True, 1.0])
def test_view_budget_is_original_one_to_three(index):
    ref, regs, _, _ = fixture()
    result = subject.associate_rgb_interest_set(ref, frame(STAMP+100_000_000, regions=regs),
        regs, observation_index=index)
    assert result.reason == 'observation_budget_exhausted' and not result.associated


def test_duplicate_member_and_count_mismatch_are_rejected():
    _, regs, first, volumes = fixture()
    with pytest.raises(ValueError, match='duplicate_member'):
        subject.freeze_rgb_interest_set(first, (regs[0], regs[0]), (volumes[0], volumes[0]),
            support_ring_outer_radius_px=170.)
    with pytest.raises(ValueError, match='members_invalid'):
        subject.freeze_rgb_interest_set(first, regs, volumes[:1], support_ring_outer_radius_px=170.)


@pytest.mark.parametrize('count', [0, 5])
def test_set_size_has_explicit_one_to_four_bound(count):
    _, regs, first, volumes = fixture()
    with pytest.raises(ValueError, match='members_invalid'):
        subject.freeze_rgb_interest_set(first, (regs[0],)*count, (volumes[0],)*count,
            support_ring_outer_radius_px=170.)


def test_one_member_interface_is_valid_without_changing_original_single_api():
    ref, regs, _, _ = fixture((region(),))
    result = subject.associate_rgb_interest_set(ref, frame(STAMP+100_000_000), regs)
    assert len(ref.members) == 1 and result.associated


def test_volume_for_wrong_member_bbox_is_rejected_before_freezing():
    _, regs, first, volumes = fixture()
    with pytest.raises(ValueError, match='volume_source_mismatch'):
        subject.freeze_rgb_interest_set(first, regs, volumes[::-1], support_ring_outer_radius_px=170.)


def test_reference_fingerprint_binds_order_and_member_source_data():
    ref, regs, first, volumes = fixture()
    duplicate = subject.freeze_rgb_interest_set(first, regs, volumes, support_ring_outer_radius_px=170.)
    reversed_ref = subject.freeze_rgb_interest_set(first, regs[::-1], volumes[::-1],
        support_ring_outer_radius_px=170.)
    assert ref.set_fingerprint == duplicate.set_fingerprint
    assert ref.set_fingerprint != reversed_ref.set_fingerprint
    with pytest.raises(ValueError, match='source_mismatch'):
        subject.RgbInterestSetReference((ref.members[0], replace(ref.members[1], generation=2)))


def test_reference_array_mutation_cannot_reuse_frozen_set_fingerprint():
    ref, regs, _, _ = fixture()
    # Frozen NumPy buffers can be forcibly reopened by a caller; detect this.
    ref.members[0].histogram.setflags(write=True)
    ref.members[0].histogram.flat[0] += .01
    result = subject.associate_rgb_interest_set(ref, frame(STAMP+100_000_000, regions=regs), regs)
    assert result.reason == 'rgb_interest_set_reference_mutated' and not result.associated


@pytest.mark.parametrize('collision', ['rgb', 'measured'])
def test_individually_successful_members_cannot_claim_same_current_evidence(monkeypatch, collision):
    ref, regs, _, _ = fixture()
    current = frame(STAMP+100_000_000, regions=regs)
    measured = tuple(candidate(r, current) for r in regs)
    originals = tuple(subject.associate_rgb_interest(m, current, regs, measured,
        observation_index=1) for m in ref.members)
    assert all(a.associated for a in originals)
    if collision == 'rgb':
        invalid = replace(originals[1], rgb_region_index=originals[0].rgb_region_index)
    else:
        invalid = replace(originals[1], measured_candidate_index=originals[0].measured_candidate_index)
    def associate(member, *args, **kwargs):
        return originals[0] if member is ref.members[0] else invalid
    monkeypatch.setattr(subject, 'associate_rgb_interest', associate)
    result = subject.associate_rgb_interest_set(ref, current, regs, measured)
    assert result.reason == 'rgb_interest_set_%s_not_one_to_one'%collision
    assert not result.associated and result.regions == ()


def test_new_camera_pose_reprojects_each_frozen_volume_without_center_switch():
    ref, regs, _, _ = fixture()
    shifted = tuple(region((r.bbox_xywh[0]-16, *r.bbox_xywh[1:])) for r in regs)
    transform = T.copy(); transform[0, 3] += .02
    current = frame(STAMP+100_000_000, regions=shifted, T_camera=transform)
    result = subject.associate_rgb_interest_set(ref, current, shifted[::-1])
    assert result.associated and result.regions == shifted
    assert tuple(a.rgb_region_index for a in result.member_associations) == (1, 0)
