"""Authorized support offset policy; warnings retain all other evidence gates."""
import numpy as np
import pytest

from alicia_grasp_modes.tabletop import Config, _validate_support_plane_update


def audit(shift, config=None, tilt=0.):
    angle = np.deg2rad(tilt)
    reference = np.array([0., 0., 1., 0.])
    current = np.array([np.sin(angle), 0., np.cos(angle), -shift])
    footprint = np.array([[-.02, -.02, 0.], [.02, -.02, 0.], [.02, .02, 0.], [-.02, .02, 0.]])
    plane, metrics = _validate_support_plane_update(reference, current, np.zeros(3),
        config or Config(), footprint)
    np.testing.assert_array_equal(reference, [0., 0., 1., 0.])
    np.testing.assert_allclose(plane, current, rtol=0., atol=1e-15)
    return metrics


@pytest.mark.parametrize('shift,warning', [(0., False), (.003999, False), (.004, False),
    (.004001, True), (.0058, True), (.007, True), (-.0058, True)])
def test_warning_and_hard_boundaries_preserve_measured_plane(shift, warning):
    metrics = audit(shift)
    assert metrics['support_reference_max_distance_m'] == pytest.approx(abs(shift))
    assert metrics['support_reference_distance_warning'] is warning
    assert metrics['support_reference_warning_distance_m'] == .004
    assert metrics['support_reference_hard_distance_m'] == .007


@pytest.mark.parametrize('shift', [.007001, -.007001, .010])
def test_above_seven_mm_remains_a_positive_contradiction(shift):
    with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
        audit(shift)


def test_four_degree_normal_and_stricter_caller_limit_remain_active():
    assert audit(0., tilt=4.)['support_reference_angle_deg'] == pytest.approx(4.)
    with pytest.raises(ValueError, match='support_plane_normal_inconsistent'):
        audit(.005, tilt=4.01)
    assert audit(.001, Config(support_max_local_shift_m=.002))['support_reference_distance_warning'] is False
    with pytest.raises(ValueError, match='support_plane_local_distance_inconsistent'):
        audit(.002001, Config(support_max_local_shift_m=.002))
    assert Config().support_threshold_m == .002
    assert Config().support_min_ratio == .70
    assert Config().foreground_min_m == .004


def test_warning_band_requalifies_real_arrival_frames_and_is_recorded(monkeypatch):
    from test_observation_measurement_unknown import frozen, arrival, next_frame, ack
    from test_locked_rgb_hold import latest_status, module
    n, clock, feed, joint, params, request = frozen(monkeypatch)
    anchor = n.tracker.anchor
    original_plane = n.tracker.plane_base.copy()
    arrival(n, clock, joint, request)
    depth = np.full((240, 320), .4058, np.float32)
    depth[106:134, 148:173] = .3558
    for index in range(3):
        source = next_frame(n, clock, joint, depth_image=depth)
        status = latest_status(n)
        assert status['support_plane_consistency_policy'] == dict(support_max_local_shift_m=.007,
            support_local_shift_warning_m=.004, support_max_normal_change_deg=4.)
        warnings = status['support_plane_warnings']
        assert any(warning['source_stamp_ns'] == source[0].header.stamp.to_nsec()
                   and warning['support_reference_max_distance_m'] == pytest.approx(.0058, abs=1e-6)
                   for warning in warnings)
        assert status['observation_measurement']['state'] == ('qualified' if index == 2 else 'requalifying')
        assert n.tracker.anchor is anchor and not n.tracker.lost
        np.testing.assert_array_equal(n.tracker.plane_base, original_plane)
    assert ack(n)['qualified'] is True
    assert module.rospy.logwarn_throttle.called


def test_warning_band_never_overrides_failed_rgb_or_hard_support_gate(monkeypatch):
    from test_observation_measurement_unknown import frozen, arrival, next_frame, ack
    from test_locked_rgb_hold import latest_status
    n, clock, feed, joint, params, request = frozen(monkeypatch)
    arrival(n, clock, joint, request)
    depth = np.full((240, 320), .4058, np.float32)
    depth[106:134, 148:173] = .3558
    for _ in range(3):
        next_frame(n, clock, joint, depth_image=depth, rgb_missing=True)
    assert latest_status(n)['observation_measurement']['state'] == 'requalifying'
    assert not ack(n)['qualified']
    assert n._observation_measurement['confirmation'].source_stamps_ns == []
    depth += .002
    next_frame(n, clock, joint, depth_image=depth)
    assert 'support_plane_local_distance_inconsistent' in latest_status(n)['observation_measurement']['reason']
    assert n._observation_measurement['confirmation'].source_stamps_ns == []

