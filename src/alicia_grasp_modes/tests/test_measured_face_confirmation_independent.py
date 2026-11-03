"""A real common-face proof cannot grant complete current-object identity.

Synthetic RGB/depth/TF, actual segmentation, source-local plane extraction,
ORB/dense identity and ObservationArrivalConfirmation; no verdict stubs/ROS.
"""
import time

import numpy as np
import pytest

from alicia_grasp_modes.observation_appearance import (
    FrozenObservationAppearance, match_observation_appearance)
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.tabletop import segment, segment_with_support_fallback, _matching_candidates
from test_measured_face_independent import flat_scene, freeze


def complete_scene(*, neighbours=False):
    scene = flat_scene()
    scene['whole'] = np.zeros((480, 640), np.uint8)
    scene['whole'][180:300, (180 if neighbours else 260):(300 if neighbours else 380)] = 1
    scene['valid'] = np.ones(scene['whole'].shape, bool)
    # Independent random texture remains inside a compact measured target;
    # the rest of the image is actual observed table in RGB and depth.
    rng = np.random.RandomState(774)
    gray = rng.randint(16, 104, scene['whole'].shape).astype(np.uint8)
    scene['rgb'] = np.repeat(gray[..., None], 3, axis=2)
    scene['rgb'][scene['whole'] == 0] = (80, 110, 180)
    scene['depth'] = np.full(scene['whole'].shape, .4, np.float32)
    scene['depth'][scene['whole'] > 0] = .35
    scene['intrinsics'] = [2500. if neighbours else 600.] * 2 + [320., 240.]
    yy, xx = np.nonzero(scene['whole'])
    target = np.array([.35, -(xx.mean()-320.)*.35/scene['intrinsics'][0],
                       -(yy.mean()-240.)*.35/scene['intrinsics'][1]])
    result = segment(scene['depth'], scene['intrinsics'], scene['transform'], scene['config'],
                     target_position_base=target)
    assert len(result.candidates) == 1
    anchor = result.candidates[0]
    refined = refine_candidate(scene['rgb'], scene['depth'], scene['intrinsics'],
                               scene['transform'], anchor, scene['config'])
    scene['whole'] = (refined.mask > 0).astype(np.uint8)
    scene['mask'] = scene['whole'].copy()
    face = freeze(scene)
    hist, contour = _appearance(scene['rgb'], scene['whole'])
    reference = FrozenObservationAppearance(scene['rgb'], scene['whole'], scene['valid'],
        100, hist, contour, measured_face=face)
    confirmation = ObservationArrivalConfirmation(scene['config'], anchor, anchor,
        scene['plane'], hist, contour, .5, appearance_reference=reference)
    current = scene['rgb'].copy()
    current[scene['whole'] > 0] = np.rint(current[scene['whole'] > 0].astype(float)*.52).astype(np.uint8)
    return scene, current, anchor, confirmation


def real_common_face_proof(scene, rgb, confirmation, *, source=101):
    reference = confirmation.appearance_reference
    context = dict(reference=reference, source_ns=source,
        deadline_monotonic=time.monotonic()+5., check_current=lambda: None,
        measured_face_cache={}, measured_face_frame={k: scene[k] for k in
            ('depth', 'intrinsics', 'transform', 'plane', 'config')})
    evidence = match_observation_appearance(rgb, scene['whole'],
        confirmation.histogram, confirmation.contour, rgb_valid=scene['valid'],
        appearance_context=context)
    assert evidence['appearance_match'] and evidence['code'] == 'COMMON_MEASURED_FACE_DENSE_CORROBORATED', evidence
    assert evidence['colour_distance'] > .4
    assert not evidence['metric_geometry_authority'] and not evidence['motion_authorized']
    return evidence


def actual_current_result(scene, anchor):
    result = segment_with_support_fallback(scene['depth'], scene['intrinsics'],
        scene['transform'], scene['config'], anchor, anchor, scene['plane'])
    assert result.metrics['support_current_audit_available'] is True
    return result


def observe(confirmation, result, scene, rgb, source):
    return confirmation.observe(result, rgb, scene['depth'], scene['intrinsics'],
        scene['transform'], source, rgb_valid=scene['valid'],
        appearance_deadline_monotonic=time.monotonic()+5.,
        appearance_check_current=lambda: None)


def test_real_common_face_and_complete_geometry_still_require_three_frames():
    scene, rgb, anchor, confirmation = complete_scene()
    real_common_face_proof(scene, rgb, confirmation)
    result = actual_current_result(scene, anchor)
    for index in range(3):
        qualified, _candidate, _tracker, evidence = observe(confirmation, result, scene, rgb, 101+index)
        assert qualified is (index == 2)
        assert evidence['source_stamps_ns'] == list(range(101, 102+index))
        assert evidence['appearance_identity'][-1]['code'] == 'COMMON_MEASURED_FACE_DENSE_CORROBORATED'
        assert not evidence['metric_geometry_authority'] and not evidence['motion_authorized']


@pytest.mark.parametrize('case', ['different_object_position', 'missing_native_body'])
def test_passing_real_face_cannot_replace_original_complete_geometry(case):
    scene, rgb, anchor, confirmation = complete_scene()
    # Seed two genuinely qualified-but-incomplete frames; a rejected third
    # must clear them instead of promoting a local appearance proof.
    original = actual_current_result(scene, anchor)
    for source in (101, 102):
        assert observe(confirmation, original, scene, rgb, source)[0] is False
    before = (anchor.mask.copy(), anchor.position_base.copy(), anchor.support_footprint_base.copy())
    if case == 'different_object_position':
        # Same complete texture on the same physical plane, but current exact
        # TF places the body 30 mm from the immutable 25 mm association bound.
        scene['transform'] = scene['transform'].copy()
        scene['transform'][1, 3] = .03
    else:
        # RGB still observes the complete face. Only one 16x16 actual depth
        # patch remains; no missing depth is fabricated by common-face proof.
        scene['depth'][scene['whole'] > 0] = 0.
        scene['depth'][232:248, 312:328] = .35
    depth_before = scene['depth'].copy()
    real_common_face_proof(scene, rgb, confirmation, source=103)
    current = actual_current_result(scene, anchor)
    assert not _matching_candidates(current.candidates, anchor, anchor, scene['config'])
    with pytest.raises(ValueError, match='^arrival_unique_original_identity_required$'):
        observe(confirmation, current, scene, rgb, 103)
    assert confirmation.source_stamps_ns == [] and confirmation.last_evidence is None
    assert confirmation.last_appearance_evidence == []
    np.testing.assert_array_equal(scene['depth'], depth_before)
    for actual, expected in zip((anchor.mask, anchor.position_base, anchor.support_footprint_base), before):
        np.testing.assert_array_equal(actual, expected)


def test_passing_one_complete_face_cannot_discard_actual_matching_neighbour():
    scene, rgb, anchor, confirmation = complete_scene(neighbours=True)
    scene['depth'][180:300, 310:430] = .35
    rgb[180:300, 310:430] = (20, 160, 20)
    real_common_face_proof(scene, rgb, confirmation)
    current = actual_current_result(scene, anchor)
    matches = _matching_candidates(current.candidates, anchor, anchor, scene['config'])
    assert len(current.candidates) == len(matches) == 2
    before = [candidate.mask.copy() for candidate in current.candidates]
    with pytest.raises(ValueError, match='^matching_fragment_original_gate_rejected: rgb_instance_has_insufficient_current_spatial_support$'):
        observe(confirmation, current, scene, rgb, 101)
    assert confirmation.source_stamps_ns == [] and confirmation.last_evidence is None
    # The first actual native refinement really did prove the common face.
    # A second competitor's original spatial gate still rejects the complete
    # operation; the first positive verdict never selects a sole winner.
    assert len(confirmation.last_appearance_evidence) == 1
    assert confirmation.last_appearance_evidence[0]['code'] == 'COMMON_MEASURED_FACE_DENSE_CORROBORATED'
    assert confirmation.last_appearance_evidence[0]['appearance_match']
    assert len(current.candidates) == 2
    for candidate, mask in zip(current.candidates, before):
        np.testing.assert_array_equal(candidate.mask, mask)
