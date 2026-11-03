"""Original acquisition binds stationary current support; no ROS or motion."""
import numpy as np
import pytest
from std_msgs.msg import Header

from test_unknown_stream import rgb_interest_node
from test_locked_rgb_hold import locked as legacy_locked, custom_frame, latest_status, module
from test_support_measurement_hold import support_gap_depth


def locked(monkeypatch):
    return legacy_locked(monkeypatch, initial_station=True)


def test_only_original_three_frame_acquisition_creates_fixed_station(monkeypatch):
    n, clock, feed, _, _ = rgb_interest_node(monkeypatch)
    for _ in range(2):
        feed(target_depth=.35)
        assert n._locked_rgb_hold_reference is None
        clock[0] += .2
    feed(target_depth=.35)
    ref = n._locked_rgb_hold_reference
    marker = ref['initial_station_current_support']
    assert marker['source_stamp_ns'] == n.anchor_source_stamp_ns
    assert not marker['q_source'].flags.writeable
    # Original 16 px at 640x480 scales to 8 in this 320x240 fixture.
    assert latest_status(n)['geometry_metrics']['rgb_search_initial_margin_px'] == 8
    original_q, original_plane = marker['q_source'].copy(), n.tracker.plane_base.copy()
    for _ in range(3):
        clock[0] += .15
        feed(target_depth=.35)
        assert latest_status(n)['state'] == 'ready'
        current = n._locked_rgb_hold_reference
        assert current is not ref and current['initial_station_current_support'] is marker
        assert current['source_stamp_ns'] > marker['source_stamp_ns']
        assert latest_status(n)['geometry_metrics']['support_source'] == 'current_ring_fit'
        assert latest_status(n)['geometry_metrics']['rgb_search_initial_margin_px'] == 8
        np.testing.assert_array_equal(marker['q_source'], original_q)
        np.testing.assert_array_equal(n.tracker.plane_base, original_plane)


def test_support_gap_recovers_current_fit_with_original_rgb_search(monkeypatch):
    n, clock, feed, joint, _ = locked(monkeypatch)
    marker = n._locked_rgb_hold_reference['initial_station_current_support']
    anchor, plane = n.tracker.anchor, n.tracker.plane_base.copy()
    custom_frame(n, clock, joint, depth_image=support_gap_depth())
    assert latest_status(n)['source_qualification']['state'] == 'pending_current_support'
    clock[0] += .15
    feed(target_depth=.35)
    status = latest_status(n)
    assert status['state'] == 'ready', status
    assert status['source_qualification']['state'] == 'recovered_unique_measured_frame'
    assert status['geometry_metrics']['support_source'] == 'current_ring_fit'
    assert status['geometry_metrics']['rgb_search_initial_margin_px'] == 8
    assert n._locked_rgb_hold_reference['initial_station_current_support'] is marker
    assert n.tracker.anchor is anchor
    np.testing.assert_array_equal(n.tracker.plane_base, plane)


@pytest.mark.parametrize('change', ['manual', 'out_and_back', 'cumulative_drift', 'camera',
                                  'config', 'epoch', 'expired'])
def test_invalid_station_never_falls_back_or_rearms_from_old_acquisition(monkeypatch, change):
    n, clock, _, joint, params = locked(monkeypatch)
    marker = n._locked_rgb_hold_reference['initial_station_current_support']
    if change == 'cumulative_drift':
        for q in (.002, .0035):
            custom_frame(n, clock, joint, target_depth=.35, q=[q, 0., 0., 0., 0., 0.])
            assert latest_status(n)['state'] == 'ready'
            clock[0] += .15
    if change == 'manual': params['/gui/joint_direct_mode'] = True
    elif change == 'camera': params['/camera']['fx'] += 1
    elif change == 'config': n.config.support_threshold_m = .0019
    elif change == 'epoch':
        # The original epoch callback retires the whole old reference.
        n._rgb_interest_epoch_cb(Header(stamp=module.rospy.Time(2), frame_id='sdk_reference_epoch'))
    elif change == 'expired': clock[0] += 1.
    elif change == 'out_and_back':
        joint(clock[0], [.005, 0., 0., 0., 0., 0.]); joint(clock[0]+.01, [0.]*6)
    original = module.segment_with_support_fallback
    preferences = []
    def observed(*a, **k):
        preferences.append(k.get('prefer_current_support', False))
        return original(*a, **k)
    monkeypatch.setattr(module, 'segment_with_support_fallback', observed)
    custom_frame(n, clock, joint, target_depth=.35,
                 q=[.005, 0., 0., 0., 0., 0.] if change == 'cumulative_drift' else None)
    assert not any(preferences)
    if change != 'epoch':
        assert not n.pubs['object'].publish.call_args[0][0].detected
    assert not (n._locked_rgb_hold_reference or {}).get('initial_station_current_support')
    assert marker['source_stamp_ns'] == n.anchor_source_stamp_ns


@pytest.mark.parametrize('stage', ['segment', 'refine', 'new_hold', 'commit'])
@pytest.mark.parametrize('change', ['manual', 'out_and_back', 'camera', 'expired'])
def test_inflight_revalidation_prevents_current_fit_commit(monkeypatch, stage, change):
    n, clock, feed, joint, params = locked(monkeypatch)
    anchor, last, stamp = n.tracker.anchor, n.tracker.last, n.last_good_source_stamp_ns
    def invalidate():
        if change == 'manual': params['/gui/joint_direct_mode'] = True
        elif change == 'camera': params['/camera']['fx'] += 1
        # Timely normal work may finish after the OLD reference deadline.
        # This negative case expires the CURRENT measured source itself.
        elif change == 'expired': clock[0] += .81
        else:
            joint(clock[0], [.006, 0., 0., 0., 0., 0.]); joint(clock[0]+.01, [0.]*6)
    owner, name = ((module, 'segment_with_support_fallback') if stage == 'segment' else
                   (module, 'refine_candidate') if stage == 'refine' else
                   (n, '_prepare_locked_rgb_hold_reference') if stage == 'new_hold' else
                   (n, '_require_initial_station_support_reference'))
    original = getattr(owner, name)
    count = [0]
    def changed(*a, **k):
        count[0] += 1
        if stage == 'commit' and count[0] == 3: invalidate()
        result = original(*a, **k)
        if stage != 'commit': invalidate()
        return result
    monkeypatch.setattr(owner, name, changed)
    feed(target_depth=.35)
    assert not n.pubs['object'].publish.call_args[0][0].detected, latest_status(n)
    assert n.tracker.anchor is anchor and n.tracker.last is last
    assert n.last_good_source_stamp_ns == stamp


def test_missing_bracket_cannot_fall_back_or_claim_qualified_support_pending(monkeypatch):
    n, clock, _, joint, _ = locked(monkeypatch)
    marker = n._locked_rgb_hold_reference['initial_station_current_support']
    original = module.segment_with_support_fallback
    calls = []
    def observed(*a, **k):
        calls.append(k.get('prefer_current_support', False)); return original(*a, **k)
    monkeypatch.setattr(module, 'segment_with_support_fallback', observed)
    custom_frame(n, clock, joint, target_depth=.35, bracket=False)
    assert not calls and n.tracker.lost
    assert n._locked_source_pending is None
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert marker['invalidated_reason'] == 'accepted_feedback_does_not_bracket_source'


def test_absent_marker_preserves_original_locked_plane_path(monkeypatch):
    n, _, feed, _, _ = locked(monkeypatch)
    n._locked_rgb_hold_reference.pop('initial_station_current_support')
    feed(target_depth=.35)
    assert latest_status(n)['state'] == 'ready'
    assert latest_status(n)['geometry_metrics']['support_source'] == 'locked_base_plane'
    assert 'initial_station_current_support' not in n._locked_rgb_hold_reference


def test_existing_support_gap_still_cannot_downgrade_missing_bracket_to_wait(monkeypatch):
    n, clock, _, joint, _ = locked(monkeypatch)
    custom_frame(n, clock, joint, depth_image=support_gap_depth())
    assert n._locked_source_pending['kind'] == 'support_measurement_gap'
    clock[0] += .1
    custom_frame(n, clock, joint, depth_image=support_gap_depth(), bracket=False)
    assert n.tracker.lost and n._locked_source_pending is None
    assert not n.pubs['object'].publish.call_args[0][0].detected


@pytest.mark.parametrize('strategy', ['direct', 'two_stage'])
def test_other_strategies_keep_original_acquisition_and_locked_plane(monkeypatch, strategy):
    n, clock, feed, _, params = rgb_interest_node(monkeypatch)
    params['/grasp_mode/selection']['strategy'] = strategy
    for _ in range(4):
        feed(target_depth=.35)
        clock[0] += .15
    assert latest_status(n)['state'] == 'ready'
    assert 'initial_station_current_support' not in n._locked_rgb_hold_reference
    assert latest_status(n)['geometry_metrics']['support_source'] == 'locked_base_plane'
    assert latest_status(n)['geometry_metrics']['rgb_search_initial_margin_px'] == 8
