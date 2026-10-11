"""A locked target may use existing support rings without changing authority."""
from dataclasses import asdict
from unittest.mock import patch

import numpy as np
import pytest

from alicia_grasp_modes import tabletop as t


K = (240., 240., 160., 120.)


def scene():
    depth = np.full((240, 320), .30, np.float32)
    depth[112:129, 146:175] = .282
    transform = np.eye(4)
    tracker = t.TargetTracker()
    result = t.segment(depth, K, transform)
    anchor, reason = tracker.choose(result, depth.shape)
    assert anchor is not None, reason
    return depth, transform, tracker, anchor


def cluttered(depth, outer):
    result = depth.copy()
    v, u = np.indices(depth.shape)
    radius = np.hypot(u-160., v-120.)
    # Two surfaces individually occupy less than 70% of the larger ring.
    # The next ring contains only the original measured table.
    result[radius > outer * .5] = .25
    return result


@pytest.mark.parametrize('outer', [140, 110])
def test_locked_audit_and_refit_use_same_bounded_ring_and_immutable_reference(outer):
    depth, transform, tracker, anchor = scene()
    plane, footprint = tracker.plane_base.copy(), anchor.support_footprint_base.copy()
    target_position = anchor.position_base.copy()
    bad = cluttered(depth, outer)
    if outer == 110:
        # Make the middle annulus itself nonplanar so neither larger ring has
        # a qualifying majority; this leaves the 70--110 ring unchanged.
        v, u = np.indices(depth.shape);radius=np.hypot(u-160.,v-120.)
        outside = radius > 55.
        bad[outside] = np.random.RandomState(26).uniform(.20,.27,int(outside.sum()))
    audit = t._audit_current_support(bad, K, transform, tracker.config, anchor, plane)
    assert audit['support_current_audit_available'] is True
    assert audit['support_current_audit_ring_outer_radius_px'] == outer * .5
    assert audit['support_current_audit_inlier_ratio'] >= .70
    assert audit['support_current_audit_residual_m'] <= .002
    assert audit['support_current_audit_quadrants'] >= 3
    refit = t.segment(bad, K, transform, tracker.config,
        target_position_base=target_position, plane_base=plane,
        target_footprint_base=footprint)
    assert refit.metrics['support_ring_outer_radius_px'] == outer * .5
    assert refit.metrics['support_reference_max_distance_m'] <= .004
    assert refit.metrics['support_reference_angle_deg'] <= 4.
    assert np.array_equal(anchor.position_base,target_position)
    assert np.array_equal(anchor.support_footprint_base,footprint)
    assert np.array_equal(tracker.plane_base,plane)
    assert tracker.anchor is anchor and tracker.last is anchor and not tracker.lost


@pytest.mark.parametrize('path', ['audit','refit'])
def test_qualified_large_ring_positive_contradiction_never_tries_smaller_ring(path):
    depth, transform, tracker, anchor = scene()
    original = t._fit_plane
    with patch.object(t,'_fit_plane',wraps=original) as fit:
        with pytest.raises(ValueError,match='support_plane_local_distance_inconsistent'):
            if path == 'audit':
                t._audit_current_support(depth+.008,K,transform,tracker.config,anchor,tracker.plane_base)
            else:
                t.segment(depth+.008,K,transform,tracker.config,
                    target_position_base=anchor.position_base,plane_base=tracker.plane_base,
                    target_footprint_base=anchor.support_footprint_base)
        assert fit.call_count == 1


@pytest.mark.parametrize('reason', ['insufficient_support_depth','support_plane_not_found',
                                  'invalid_depth_shape'])
def test_other_fit_errors_never_search_smaller_ring(reason):
    depth, transform, tracker, anchor = scene()
    with patch.object(t,'_fit_plane',side_effect=ValueError(reason)) as fit:
        if reason == 'invalid_depth_shape':
            with pytest.raises(ValueError,match=reason):
                t._audit_current_support(depth,K,transform,tracker.config,anchor,tracker.plane_base)
        else:
            audit=t._audit_current_support(depth,K,transform,tracker.config,anchor,tracker.plane_base)
            assert not audit['support_current_audit_available']
            assert audit['support_current_audit_reason'] == reason
        assert fit.call_count == 1


def test_locked_fit_attempts_are_bounded_and_all_original_configuration_is_preserved():
    depth, transform, tracker, anchor = scene()
    with patch.object(t,'_fit_plane',side_effect=ValueError('support_plane_inlier_ratio_low')) as fit:
        result=t._audit_current_support(depth,K,transform,tracker.config,anchor,tracker.plane_base)
        assert not result['support_current_audit_available']
        assert fit.call_count == 3
        for call in fit.call_args_list:
            assert asdict(call.args[1]) == asdict(t.Config())
        assert [len(c.args[0]) for c in fit.call_args_list] == sorted(
            [len(c.args[0]) for c in fit.call_args_list],reverse=True)


@pytest.mark.parametrize('fault',['residual','coverage'])
def test_first_qualifying_fit_quality_failure_cannot_be_hidden_by_smaller_ring(fault):
    depth, transform, tracker, anchor = scene()
    plane=np.array([-1.,0.,0.,.30])
    residual=.00201 if fault=='residual' else .001
    with patch.object(t,'_fit_plane',return_value=(plane,.8,residual,1800)) as fit:
        with patch.object(t,'_support_quadrants',return_value=2 if fault=='coverage' else 4):
            audit=t._audit_current_support(depth,K,transform,tracker.config,anchor,tracker.plane_base)
    assert not audit['support_current_audit_available']
    assert fit.call_count == 1
