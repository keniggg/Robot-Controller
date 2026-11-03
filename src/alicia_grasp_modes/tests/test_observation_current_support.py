"""Measured current support removes old-plane table foreground after arrival."""
from copy import deepcopy
from unittest.mock import patch

import numpy as np
import pytest

from alicia_grasp_modes import tabletop as table
import test_tabletop as tabletop_fixture
from test_observation_arrival_station import completed
from test_observation_measurement_unknown import next_frame
from test_locked_rgb_hold import latest_status, module


def shifted_table():
    helper = tabletop_fixture.TabletopTests()
    depth, transform = helper.scene([(.293, 0., 0.)])
    tracker = table.TargetTracker()
    tracker.choose(table.segment(depth, helper.intrinsics, transform), depth.shape)
    depth -= .003
    depth[:10, :10] = depth[:10, -10:] = .30
    depth[-10:, :10] = depth[-10:, -10:] = .30
    # Connected table texture lies 1.5 mm above the CURRENT measured table,
    # but 4.5 mm above the immutable reference. Old tracking still matches it.
    depth[127:136, 151:169] = .2955
    return depth, transform, tracker, helper.intrinsics


def test_current_support_removes_connected_table_patch_without_replacing_identity():
    depth, transform, tracker, k = shifted_table()
    anchor, plane = tracker.anchor, tracker.plane_base.copy()
    args = (depth, k, transform, tracker.config, anchor, anchor, plane)
    default = table.segment_with_support_fallback(*args)
    matching = table._matching_candidates(default.candidates, anchor, anchor, tracker.config)
    assert len(matching) == 1
    assert np.any(matching[0][1].mask[128:136, 151:169])
    assert default.metrics['support_current_audit_available']
    measured = table.segment_with_support_fallback(*args, prefer_current_support=True)
    matching = table._matching_candidates(measured.candidates, anchor, anchor, tracker.config)
    assert len(matching) == 1
    assert not np.any(matching[0][1].mask[128:136, 151:169])
    assert measured.metrics['support_source'] == 'current_ring_fit'
    assert measured.metrics['support_refit_reason'] == 'qualified_current_support_measurement'
    assert measured.metrics['support_reference_max_distance_m'] <= .004
    assert tracker.anchor is anchor and tracker.last is anchor
    np.testing.assert_array_equal(tracker.plane_base, plane)


def test_current_preference_never_refits_away_two_old_eligible_identities():
    depth, transform, tracker, k = shifted_table()
    neighbour = deepcopy(tracker.anchor)
    neighbour.position_base += [0., .015, 0.]
    ambiguous = table.Segmentation([tracker.anchor, neighbour], tracker.plane_base.copy(), {})
    with patch.object(table, 'segment', return_value=ambiguous) as measured:
        result = table.segment_with_support_fallback(depth, k, transform, tracker.config,
            tracker.anchor, tracker.last, tracker.plane_base, prefer_current_support=True)
    assert measured.call_count == 1
    assert len(table._matching_candidates(result.candidates, tracker.anchor, tracker.last, tracker.config)) == 2
    assert tracker.choose(result, depth.shape)[1] == 'ambiguous_locked_instances'


def test_current_preference_cannot_supply_missing_independent_support():
    depth, transform, tracker, k = shifted_table()
    with patch.object(table, '_fit_plane', side_effect=ValueError('support_plane_inlier_ratio_low')):
        result = table.segment_with_support_fallback(depth, k, transform, tracker.config,
            tracker.anchor, tracker.last, tracker.plane_base, prefer_current_support=True)
    assert result.metrics['support_current_audit_available'] is False
    assert result.metrics['support_source'] == 'locked_base_plane'
    assert not result.metrics.get('support_refit_attempted', False)


@pytest.mark.parametrize('change', ['manual', 'out_and_back', 'camera'])
def test_marker_invalidated_during_current_fit_cannot_publish(monkeypatch, change):
    n, clock, _, joint, params, _ = completed(monkeypatch)
    stamp, last = n.last_good_source_stamp_ns, n.tracker.last
    original = module.segment_with_support_fallback
    def changed(*args, **kwargs):
        assert kwargs.get('prefer_current_support') is True
        result = original(*args, **kwargs)
        if change == 'manual':
            params['/gui/joint_direct_mode'] = True
        elif change == 'camera':
            params['/camera']['fx'] += 1
        else:
            joint(clock[0], [.005, 0., 0., 0., 0., 0.])
            joint(clock[0]+.01, [0.]*6)
        return result
    monkeypatch.setattr(module, 'segment_with_support_fallback', changed)
    next_frame(n, clock, joint)
    assert not n.pubs['object'].publish.call_args[0][0].detected, latest_status(n)
    assert n.last_good_source_stamp_ns == stamp and n.tracker.last is last
    assert 'observation_arrival_retry' not in (n._locked_rgb_hold_reference or {})


def test_arrival_then_normal_frames_keep_current_support_and_immutable_plane(monkeypatch):
    n, clock, _, joint, _, marker = completed(monkeypatch)
    plane, anchor = n.tracker.plane_base.copy(), n.tracker.anchor
    assert n.tracker.last.metrics['support_source'] == 'current_ring_fit'
    for _ in range(3):
        next_frame(n, clock, joint)
        assert n.pubs['object'].publish.call_args[0][0].detected, latest_status(n)
        assert n.tracker.last.metrics['support_source'] == 'current_ring_fit'
        assert n._locked_rgb_hold_reference['observation_arrival_retry'] is marker
        np.testing.assert_array_equal(n.tracker.plane_base, plane)
        assert n.tracker.anchor is anchor
