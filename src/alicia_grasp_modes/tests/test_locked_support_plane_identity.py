"""Locked support provenance survives floating-point normalization roundoff."""
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'src'))
from alicia_grasp_modes import tabletop as t
from alicia_grasp_modes.rgb_instance import refine_candidate


# Generation 27's recorded acquisition plane has normal norm 1 - one ULP.
# The following scenes are synthetic; these coefficients alone are recorded.
RECORDED_REFERENCE = np.array([
    -.030537463025834762, .034107498699356954,
    .9989515212884051, .0063876970390733655])
K = (240., 240., 160., 120.)


def scene():
    unit = t._normalized_plane(RECORDED_REFERENCE)
    normal = unit[:3]
    axis = np.cross(normal, [1., 0., 0.])
    axis /= np.linalg.norm(axis)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((-normal, axis, np.cross(-normal, axis)))
    transform[:3, 3] = (.4-unit[3])*normal
    depth = np.full((240, 320), .4, np.float32)
    depth[106:134, 148:173] = .35
    tracker = t.TargetTracker()
    anchor, reason = tracker.choose(t.segment(depth, K, transform), depth.shape)
    assert anchor is not None, reason
    return depth, transform, tracker


@pytest.mark.parametrize('missing_current_support', [False, True])
def test_locked_segmentation_keeps_exact_recorded_plane_provenance(missing_current_support):
    depth, transform, tracker = scene()
    reference = RECORDED_REFERENCE.copy()
    reference.setflags(write=False)
    assert not np.array_equal(reference, t._normalized_plane(reference))
    if missing_current_support:
        v, u = np.indices(depth.shape)
        radius = np.hypot(u-160., v-120.)
        depth[(radius >= 30.) & (radius <= 95.)] = 0.
    result = t.segment_with_support_fallback(depth, K, transform,
        anchor=tracker.anchor, last=tracker.last, reference_plane_base=reference)
    assert result.metrics['support_source'] == 'locked_base_plane'
    assert result.metrics['support_current_audit_available'] is not missing_current_support
    assert len(result.candidates) == 1
    np.testing.assert_array_equal(result.plane_base, reference)
    np.testing.assert_array_equal(result.candidates[0].metrics['support_plane_base'],
                                  t._normalized_plane(reference))
    np.testing.assert_array_equal(reference, RECORDED_REFERENCE)
    assert not np.shares_memory(result.plane_base, reference)


@pytest.mark.parametrize('scale', [.25, 3.7])
def test_locked_plane_segmentation_and_refinement_keep_metric_height_units(scale):
    depth, transform, tracker = scene()
    arguments = dict(target_position_base=tracker.anchor.position_base,
                     fit_current_support=False)
    original = t.segment(depth, K, transform, plane_base=RECORDED_REFERENCE, **arguments)
    scaled = t.segment(depth, K, transform, plane_base=RECORDED_REFERENCE*scale, **arguments)
    assert len(original.candidates) == len(scaled.candidates) == 1
    np.testing.assert_array_equal(scaled.candidates[0].mask, original.candidates[0].mask)
    np.testing.assert_array_equal(scaled.candidates[0].position_base, original.candidates[0].position_base)
    assert scaled.candidates[0].metrics['median_height_m'] == pytest.approx(
        original.candidates[0].metrics['median_height_m'], abs=1e-15)
    # RGB refinement consumes the candidate metric plane as a unit normal;
    # retaining the provenance scale there would turn a 50 mm height into
    # 185 mm at scale 3.7 and wrongly reject the same measured object.
    rgb = np.full(depth.shape+(3,), (60, 90, 190), np.uint8)
    rgb[106:134, 148:173] = 20
    refined_original = refine_candidate(rgb, depth, K, transform,
                                        original.candidates[0], t.Config())
    refined_scaled = refine_candidate(rgb, depth, K, transform,
                                      scaled.candidates[0], t.Config())
    np.testing.assert_array_equal(refined_scaled.mask, refined_original.mask)
    np.testing.assert_array_equal(refined_scaled.position_base, refined_original.position_base)
    assert refined_scaled.metrics['valid_points'] == refined_original.metrics['valid_points']


@pytest.mark.parametrize('plane,reason', [
    ([0., 0., 0., .1], 'invalid_support_plane_normal'),
    ([0., np.nan, 1., .1], 'invalid_support_plane_coefficients'),
    ([0., 0., 1.], 'invalid_support_plane_coefficients')])
def test_locked_plane_validation_is_preserved(plane, reason):
    depth, transform, tracker = scene()
    with pytest.raises(ValueError, match=reason):
        t.segment(depth, K, transform, plane_base=plane,
                  target_position_base=tracker.anchor.position_base, fit_current_support=False)
