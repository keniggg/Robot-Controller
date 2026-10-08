"""No cross-view centre switching, synthetic depth, or hidden support alignment."""
from dataclasses import replace

import cv2
import numpy as np
import pytest

from alicia_grasp_modes.rgb_observation_handoff import (
    RgbHandoffFrame, HandoffPolicy, RgbMeasuredHandoff,
    freeze_rgb_interest, associate_rgb_interest)
from alicia_grasp_modes.rgb_observation_region import RgbObservationRegion
from alicia_grasp_modes.rgb_observation_volume import (
    SupportPlaneProvenance, build_rgb_observation_volume)
from alicia_grasp_modes.tabletop import Candidate, _measured_support_footprint


K = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])
T = np.array([[0., -1., 0., .1], [0., 0., 1., -.2], [-1., 0., 0., .5], [0., 0., 0., 1.]])
PLANE = (0., 0., 1., 0.)
STAMP = 1_000_000_000


def region(box=(300, 220, 40, 40), shape='rect'):
    mask = np.zeros((480, 640), np.uint8)
    x, y, w, h = box
    if shape == 'rect':
        mask[y:y+h, x:x+w] = 1
    elif shape == 'circle':
        cv2.circle(mask, (x+w//2, y+h//2), min(w, h)//2-1, 1, -1)
    elif shape == 'triangle':
        cv2.fillConvexPoly(mask, np.array([[x+w//2, y], [x+w-1, y+h-1], [x, y+h-1]]), 1)
    v, u = np.nonzero(mask)
    box = cv2.boundingRect(mask)
    return RgbObservationRegion(mask, box, (float(u.mean()), float(v.mean())), len(u),
        .99, .8, 60., 0., 0., ('test_rgb',), None)


def frame(stamp=STAMP, *, regions=None, colours=None, plane=PLANE, T_camera=None,
          missing_depth=False, source_id='frozen-calibration-epoch-selection', q=None, **kwargs):
    regions = (region(),) if regions is None else regions
    colours = [(40, 70, 180)]*len(regions) if colours is None else colours
    bgr = np.full((480, 640, 3), 210, np.uint8)
    depth = np.full((480, 640), .5, np.float32)
    for reg, colour in zip(regions, colours):
        bgr[reg.mask != 0] = colour
        depth[reg.mask != 0] = np.nan if missing_depth else .49
    q = np.zeros(6) if q is None else np.asarray(q)
    evidence = SupportPlaneProvenance(stamp, plane, 'frame-%d'%stamp, 2000, .95, .0003, 4)
    args = dict(bgr=bgr, depth_m=depth, camera_matrix=K,
        T_base_camera=T if T_camera is None else T_camera, support_provenance=evidence,
        source_id=source_id, generation=1, epoch_ns=100, selection_stamp_ns=200,
        source_stamp_ns=stamp, tf_stamp_ns=stamp, now_ns=stamp+10_000_000,
        feedback_records=((stamp-10_000_000, q), (stamp+10_000_000, q)),
        support_ring_outer_radius_px=170.)
    args.update(kwargs)
    return RgbHandoffFrame(**args)


def reference(first=None, reg=None):
    first = frame() if first is None else first
    reg = region() if reg is None else reg
    volume = build_rgb_observation_volume(bbox_xywh=reg.bbox_xywh,
        image_shape=first.bgr.shape[:2], camera_matrix=first.camera_matrix,
        T_base_camera=first.T_base_camera, support_plane_base=first.support_provenance.support_plane_base,
        support_provenance=first.support_provenance, foreground_max_m=.12,
        source_stamp_ns=first.source_stamp_ns, working_depth_range_m=(.07, .7))
    return freeze_rgb_interest(first, reg, volume, support_ring_outer_radius_px=170.)


def candidate(reg=None, stamp_frame=None):
    reg = region() if reg is None else reg
    f = frame() if stamp_frame is None else stamp_frame
    u, v = reg.centroid_xy
    d = float(f.depth_m[int(v), int(u)])
    cam = np.array([d, -(u-320)*d/400, -(v-240)*d/400])
    base = f.T_base_camera[:3, :3] @ cam + f.T_base_camera[:3, 3]
    points = base + np.array([[-.02, -.02, 0.], [-.02, .02, 0.], [.02, .02, 0.], [.02, -.02, 0.]])
    return Candidate(reg.mask.astype(np.uint8)*255, np.array(reg.centroid_xy), reg.bbox_xywh,
        cam, base, np.array([.04, .04, .01]), d, .9,
        dict(support_plane_base=list(f.support_provenance.support_plane_base), valid_points=reg.area_px),
        _measured_support_footprint(points, f.support_provenance.support_plane_base))


def associate(ref=None, f=None, regs=None, candidates=(), **kwargs):
    args = dict(observation_index=1); args.update(kwargs)
    return associate_rgb_interest(reference() if ref is None else ref,
        frame(STAMP+100_000_000) if f is None else f,
        (region(),) if regs is None else regs, candidates, **args)


@pytest.mark.parametrize('colour', [(1, 1, 1), (245, 245, 245), (0, 220, 0), (255, 30, 70)])
def test_colour_independent_association_including_no_object_depth(colour):
    original = frame(colours=[colour], missing_depth=True)
    ref = reference(original)
    current = frame(STAMP+100_000_000, colours=[colour], missing_depth=True)
    result = associate(ref, current)
    assert result.associated and result.reason == 'rgb_only_associated'
    assert result.measured_candidate is None
    assert not result.metric_geometry_authority and not result.motion_authorized
    assert result.synthetic_measured_points_added == 0
    assert np.all(np.isnan(current.depth_m[region().mask != 0]))


def test_camera_translation_changes_pixels_not_original_identity():
    original = reference()
    shifted = region((284, 220, 40, 40))
    distractor = region((330, 220, 24, 24))
    transform = T.copy(); transform[0, 3] += .02
    current = frame(STAMP+100_000_000, regions=(distractor, shifted),
                    colours=[(180, 60, 20), (40, 70, 180)], T_camera=transform)
    result = associate(original, current, (distractor, shifted))
    assert result.rgb_region_index == 1
    assert dict(result.support_audit)['support_alignment_applied'] is False


def test_identical_alternative_inside_projected_volume_is_ambiguous():
    first = region((300, 220, 40, 40))
    # An oblique translation projects the uncertain height slab to a wider ROI.
    transform = T.copy(); transform[0, 3] += .1
    left = region((195, 224, 12, 12)); right = region((216, 224, 12, 12))
    current = frame(STAMP+100_000_000, regions=(left, right), T_camera=transform)
    result = associate(reference(frame(regions=(first,)), first), current, (left, right))
    assert result.reason == 'cross_view_rgb_ambiguous'
    assert not result.associated


def test_wrong_colour_inside_volume_is_not_same_object():
    current = frame(STAMP+100_000_000, colours=[(180, 70, 20)])
    result = associate(f=current)
    assert result.reason == 'no_cross_view_rgb_match'


def test_same_colour_outside_volume_is_not_selected_even_at_image_center():
    transform = T.copy(); transform[0, 3] += .1
    current = frame(STAMP+100_000_000, T_camera=transform)
    assert associate(f=current).reason == 'no_cross_view_rgb_match'


@pytest.mark.parametrize('shift', [.019, -.020])
def test_raw_cross_view_support_disagreement_is_reported_without_alignment(shift):
    current = frame(STAMP+100_000_000, plane=(0., 0., 1., shift))
    result = associate(f=current)
    assert result.reason.startswith('support_inconsistent:')
    assert dict(result.support_audit)['support_reference_max_distance_m'] == pytest.approx(abs(shift))
    assert dict(result.support_audit)['support_reference_fixed'] is True
    assert not result.associated


def test_support_angle_disagreement_is_not_hidden_by_projection():
    angle = np.deg2rad(5.)
    result = associate(f=frame(STAMP+100_000_000, plane=(np.sin(angle), 0., np.cos(angle), 0.)))
    assert 'support_plane_normal_inconsistent' in result.reason
    assert dict(result.support_audit)['support_reference_angle_deg'] == pytest.approx(5.)


@pytest.mark.parametrize('field,value', [('source_id', 'changed'), ('generation', 2),
    ('epoch_ns', 101), ('selection_stamp_ns', 201)])
def test_context_change_invalidates_original_interest(field, value):
    current = replace(frame(STAMP+100_000_000), **{field: value})
    assert associate(f=current).reason == 'source_context_changed'


@pytest.mark.parametrize('args', [dict(tf_stamp_ns=STAMP-1), dict(now_ns=STAMP+800_000_001),
    dict(feedback_records=((STAMP-110_000_000, [0.]*6), (STAMP+110_000_000, [0.]*6))),
    dict(feedback_records=((STAMP-10_000_000, [0.]*6),))])
def test_exact_frame_tf_freshness_and_200ms_feedback_are_required(args):
    with pytest.raises(ValueError):
        frame(**args)


def test_reference_uses_real_support_points_not_constructed_volume():
    f = frame(missing_depth=True)
    ref = reference(f)
    assert len(ref.measured_support_footprint_base) >= 3
    # No support measurements even if a caller claims an accepted plane.
    missing = np.full_like(f.depth_m, np.nan)
    with pytest.raises(ValueError, match='actual_support_coverage'):
        reference(replace(f, depth_m=missing))


def test_reference_arrays_are_frozen_and_copies_of_inputs():
    f = frame(); ref = reference(f)
    for array in (f.bgr, f.depth_m, f.T_base_camera, f.camera_matrix,
                  ref.histogram, ref.contour, ref.measured_support_footprint_base,
                  ref.support_consistency_domain_base):
        assert not array.flags.writeable
    reg = region(); another = reference(f, reg)
    before = another.mask_fingerprint
    reg.mask[:] = 0
    assert another.mask_fingerprint == before


def plane_depth(f):
    """Analytic measured-depth fixture for an independently tilted support."""
    equation = f.T_base_camera.T @ np.asarray(f.support_provenance.support_plane_base)
    v, u = np.indices(f.depth_m.shape)
    oy = (v-f.camera_matrix[1, 2])/f.camera_matrix[1, 1]
    ox = (u-f.camera_matrix[0, 2]-f.camera_matrix[0, 1]*oy)/f.camera_matrix[0, 0]
    return (-equation[3]/(equation[0]-ox*equation[1]-oy*equation[2])).astype(np.float32)


def test_fixed_domain_keeps_all_original_vertices_separate_from_real_support():
    ref = reference(frame(missing_depth=True))
    vertices = np.asarray(ref.volume.vertices_base_m)
    plane = np.asarray(ref.volume.support_plane_base)
    expected = vertices-(vertices @ plane[:3]+plane[3])[:, None]*plane[:3]
    assert np.array_equal(ref.support_consistency_domain_base, expected)
    assert expected.shape == (8, 3)
    assert np.ptp(ref.measured_support_footprint_base[:, 0]) > 5*np.ptp(expected[:, 0])
    assert ref.source_stamp_ns == STAMP
    assert not ref.metric_geometry_authority and not ref.motion_authorized


def test_original_ring_edge_disagreement_is_audited_but_local_domain_can_agree():
    ref = reference(frame(missing_depth=True))
    angle = np.deg2rad(1.)
    normal = np.array([np.sin(angle), 0., np.cos(angle)])
    offset = .0028-normal @ ref.support_consistency_domain_base.mean(axis=0)
    current = frame(STAMP+100_000_000, plane=tuple(normal)+(offset,), missing_depth=True)
    depth = plane_depth(current); depth[region().mask != 0] = np.nan
    current = replace(current, depth_m=depth)
    old_domain = ref.support_consistency_domain_base.copy()
    result = associate(ref, current)
    audit = dict(result.support_audit)
    assert result.associated and result.measured_candidate is None
    assert audit['support_reference_max_distance_m'] < .004
    assert audit['support_original_ring_max_distance_m'] > .004
    assert not audit['support_original_ring_consistent']
    assert 'local_distance_inconsistent' in audit['support_original_ring_rejection']
    assert audit['support_original_domain_covered_vertices'] == 8
    assert audit['support_current_domain_covered_vertices'] == 8
    assert audit['support_coverage_is_vertex_measurement'] is False
    assert np.array_equal(ref.support_consistency_domain_base, old_domain)
    assert ref.source_stamp_ns == STAMP and result.frame.source_stamp_ns == STAMP+100_000_000
    assert not result.metric_geometry_authority and not result.motion_authorized


def test_domain_edges_cannot_be_replaced_by_matching_center():
    reg = region((260, 180, 120, 120))
    ref = reference(frame(regions=(reg,)), reg)
    angle = np.deg2rad(3.9)
    normal = np.array([np.sin(angle), 0., np.cos(angle)])
    offset = -normal @ ref.support_consistency_domain_base.mean(axis=0)
    current = frame(STAMP+100_000_000, regions=(reg,), plane=tuple(normal)+(offset,))
    current = replace(current, depth_m=plane_depth(current))
    assert abs(ref.support_consistency_domain_base.mean(axis=0) @ normal+offset) < 1e-12
    result = associate(ref, current, (reg,))
    assert 'support_plane_local_distance_inconsistent' in result.reason
    assert dict(result.support_audit)['support_reference_max_distance_m'] > .004
    assert not result.associated and not result.projected_volume_pixels


def test_original_domain_requires_measured_ring_coverage_at_freeze():
    reg = region((510, 220, 24, 24))
    with pytest.raises(ValueError, match='original_support_domain_not_covered'):
        reference(frame(regions=(reg,)), reg)


@pytest.mark.parametrize('change', ['shrink', 'shift', 'nonfinite', 'drop_vertex'])
def test_derived_domain_cannot_be_tampered_or_recropped(change):
    ref = reference()
    domain = ref.support_consistency_domain_base.copy()
    if change == 'shrink':
        domain = domain.mean(axis=0)+.5*(domain-domain.mean(axis=0))
    elif change == 'shift':
        domain[:, 0] += .001
    elif change == 'nonfinite':
        domain[0, 0] = np.nan
    else:
        domain = domain[:-1]
    object.__setattr__(ref, 'support_consistency_domain_base', domain)
    result = associate(ref)
    assert 'fixed_support_domain_changed' in result.reason
    assert not result.associated and not result.projected_volume_pixels


@pytest.mark.parametrize('outer', [None, 110.])
def test_current_ring_must_be_known_and_cannot_expand_to_recover_coverage(outer):
    reg = region((430, 220, 24, 24))
    ref = reference(frame(regions=(reg,)), reg)
    current = frame(STAMP+100_000_000, regions=(reg,), support_ring_outer_radius_px=outer)
    result = associate(ref, current, (reg,))
    expected = 'current_support_ring_required' if outer is None else 'current_support_domain_not_covered'
    assert expected in result.reason
    assert not result.associated and not result.projected_volume_pixels


def test_missing_current_support_is_not_evidence_of_agreement():
    current = frame(STAMP+100_000_000)
    current = replace(current, depth_m=np.full_like(current.depth_m, np.nan))
    result = associate(f=current)
    assert 'actual_support_coverage_insufficient' in result.reason
    assert not result.associated


def test_nonplanar_pixels_do_not_supply_missing_support_coverage():
    reg = region((430, 220, 24, 24))
    ref = reference(frame(regions=(reg,)), reg)
    current = frame(STAMP+100_000_000, regions=(reg,))
    # Three populated quadrants still pass the original count/spatial gates.
    # Only far-right non-planar pixels could cover this fixed member domain.
    depth = current.depth_m.copy(); depth[:, 400:] -= .02
    result = associate(ref, replace(current, depth_m=depth), (reg,))
    assert 'current_support_domain_not_covered' in result.reason
    assert dict(result.support_audit)['support_current_domain_covered_vertices'] < 8


def test_reference_ring_argument_must_match_actual_frame_provenance():
    first = frame(support_ring_outer_radius_px=140.)
    with pytest.raises(ValueError, match='reference_support_ring_mismatch'):
        reference(first)


@pytest.mark.parametrize('outer', [True, np.bool_(True), 0., 70., 401., np.nan, 170+0j, '170'])
def test_malformed_current_ring_is_rejected_at_frame_creation(outer):
    with pytest.raises(ValueError, match='invalid_support_ring'):
        frame(support_ring_outer_radius_px=outer)


@pytest.mark.parametrize('index', [0, 4, True, 1.0])
def test_observation_budget_is_explicit_and_bounded(index):
    assert associate(observation_index=index).reason == 'observation_budget_exhausted'


def test_candidate_work_is_bounded_not_silently_truncated():
    assert associate(regs=[region()]*17).reason == 'association_candidate_budget_exhausted'


def test_association_returns_exact_original_measured_candidate_reference():
    f = frame(STAMP+100_000_000); c = candidate(stamp_frame=f)
    result = associate(f=f, candidates=(c,))
    assert result.measured_candidate is c
    assert result.measured_candidate_index == 0
    assert not result.metric_geometry_authority


def test_two_overlapping_measured_candidates_are_ambiguous():
    f = frame(STAMP+100_000_000)
    result = associate(f=f, candidates=(candidate(stamp_frame=f), candidate(stamp_frame=f)))
    assert result.reason == 'measured_candidate_ambiguous'
    assert not result.associated


def test_original_initial_acquisition_requires_three_associated_real_frames():
    ref = reference(); handoff = RgbMeasuredHandoff()
    for index in range(1, 4):
        f = frame(STAMP+index*100_000_000)
        raw = candidate(stamp_frame=f); refined = candidate(stamp_frame=f)
        result = handoff.observe(associate(ref, f, candidates=(raw,)), raw, refined)
        assert result.ready == (index == 3)
        assert result.candidate is (refined if index == 3 else None)
    assert len(result.confirming_stamps_ns) == 3
    assert not result.motion_authorized


def test_depth_holes_cannot_be_overridden_by_candidate_point_count():
    ref = reference(); handoff = RgbMeasuredHandoff()
    f = frame(STAMP+100_000_000, missing_depth=True)
    raw = candidate(); refined = candidate()
    result = handoff.observe(associate(ref, f, candidates=(raw,)), raw, refined)
    assert result.reason == 'handoff_insufficient_actual_depth_pixels'
    assert not result.ready


def test_mutating_raw_candidate_or_substituting_new_center_candidate_is_rejected():
    ref = reference(); f = frame(STAMP+100_000_000); raw = candidate(stamp_frame=f)
    association = associate(ref, f, candidates=(raw,))
    handoff = RgbMeasuredHandoff()
    assert handoff.observe(association, candidate(stamp_frame=f), candidate(stamp_frame=f)).reason == 'handoff_requires_associated_measured_candidate'
    raw.position_base[2] += .01
    assert handoff.observe(association, raw, candidate(stamp_frame=f)).reason == 'handoff_measured_candidate_mutated'


def test_ambiguity_resets_acquisition_before_new_confirmation():
    ref = reference(); handoff = RgbMeasuredHandoff()
    for index in (1, 2):
        f = frame(STAMP+index*100_000_000); c = candidate(stamp_frame=f)
        assert not handoff.observe(associate(ref, f, candidates=(c,)), c, c).ready
    assert not handoff.observe(associate(ref, frame(STAMP+300_000_000), regs=())).ready
    for index in (4, 5, 6):
        f = frame(STAMP+index*100_000_000); c = candidate(stamp_frame=f)
        assert handoff.observe(associate(ref, f, candidates=(c,)), c, c).ready == (index == 6)


def test_motion_duplicate_or_new_view_does_not_reuse_confirmed_frames():
    ref = reference(); handoff = RgbMeasuredHandoff()
    for index in (1, 2, 3):
        f = frame(STAMP+index*100_000_000); c = candidate(stamp_frame=f)
        association = associate(ref, f, candidates=(c,))
        result = handoff.observe(association, c, c)
    assert result.ready
    assert handoff.observe(association, c, c).reason == 'handoff_nonmonotonic_source'
    for index in (4, 5):
        f = frame(STAMP+index*100_000_000); c = candidate(stamp_frame=f)
        assert not handoff.observe(associate(ref, f, candidates=(c,)), c, c).ready
    f = frame(STAMP+600_000_000, q=[.005, 0, 0, 0, 0, 0]); c = candidate(stamp_frame=f)
    assert not handoff.observe(associate(ref, f, candidates=(c,)), c, c).ready
    f = frame(STAMP+700_000_000); c = candidate(stamp_frame=f)
    assert not handoff.observe(associate(ref, f, candidates=(c,), observation_index=2), c, c).ready


@pytest.mark.parametrize('key,value', [('maximum_candidates', True), ('maximum_observations', 4),
    ('min_projected_fraction', .4), ('minimum_measured_mask_iou', True)])
def test_policy_does_not_accept_bool_or_unbounded_work(key, value):
    with pytest.raises(ValueError, match='policy_invalid'):
        HandoffPolicy(**{key:value})
