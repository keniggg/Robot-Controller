"""Offline camera-view checks; no ROS graph, controller or collision mutation."""
import json
import itertools

import numpy as np
import pytest
import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk, wire_digest
from alicia_flexible_grasp.robot.observation_view_guard import (
    ObservationViewError, validate_observation_target_trajectory,
    validate_observation_support_projection, validate_observation_view_trajectory,
)

K = (400., 400., 320., 240.)
CENTER = np.array([.3, 0., 0.])


def corners(center=CENTER):
    return np.asarray(list(itertools.product((-.002, .002), repeat=3))) + center


def fk(axis='0 0 1'):
    return SerialUrdfFk('<robot name="view"><joint name="J1" type="revolute">'
        '<parent link="base_link"/><child link="tool0"/><axis xyz="'+axis+'"/>'
        '</joint></robot>', ['J1'])


def point(q, t=0., velocity=0., acceleration=0.):
    p = JointTrajectoryPoint()
    p.positions = [q]
    p.velocities = [] if velocity is None else [velocity]
    p.accelerations = [] if acceleration is None else [acceleration]
    p.time_from_start = rospy.Duration.from_sec(t)
    return p


def plan(q=0., duration=1.):
    p = RobotTrajectory()
    p.joint_trajectory.joint_names = ['J1']
    p.joint_trajectory.points = [point(0.), point(q, duration)]
    return p


def check(p=None, desired=None, target=None, model=None, **kw):
    return validate_observation_target_trajectory(
        plan() if p is None else p, fk() if model is None else model,
        np.eye(4), corners() if target is None else target, K, 640, 480,
        point(0.) if desired is None else desired, **kw)


def measured_support(half=False):
    # Explicit test data simulating measured pixels, not guard-generated points.
    points = []
    for u in range(210, 431, 4):
        for v in range(130, 351, 4):
            if half and v >= 240:
                continue
            if 72. <= np.hypot(u-320., v-240.) <= 160.:
                points.append([.3, -(u-320.)*.3/400., -(v-240.)*.3/400.])
    return np.asarray(points)


def test_small_actual_spline_is_sampled_without_mutating_inputs():
    p = plan(.04)
    before = wire_digest(p)
    hold = point(0.)
    report = check(p, desired=hold)
    assert report['ok'] and report['sample_count'] > 3
    assert report['maximum_sample_interval_sec'] <= .025
    assert report['maximum_leaf_joint_span_rad'] <= .02
    assert not report['certifies_continuous_visibility']
    assert not report['certifies_collision_tracking_calibration_or_occlusion']
    assert not report['endpoint_support_checked']
    assert report['bridge_start_joint_positions_rad'] == [0.]
    assert before == wire_digest(p)
    assert hold.positions == [0.]


def test_visible_knots_hide_cubic_bridge_excursion():
    p = plan()
    p.joint_trajectory.points[1] = point(0., 1., -6., None)
    with pytest.raises(ObservationViewError, match='image margin') as captured:
        check(p)
    assert captured.value.audit['segment'] == 'bridge'
    assert 0. < captured.value.audit['trajectory_time_sec'] < 1.


def test_controller_bridge_uses_real_hold_instead_of_dropped_t0():
    with pytest.raises(ObservationViewError, match='image margin') as captured:
        check(plan(), desired=point(.9))
    assert captured.value.audit['trajectory_time_sec'] == 0.
    assert captured.value.audit['joint_positions_rad'] == [.9]
    json.dumps(captured.value.audit, allow_nan=False)


def test_roll_180_has_visible_endpoints_but_invisible_midpath():
    target = corners(np.array([.3, .105, .105]))
    # Inverted image at endpoint alone is not forbidden: both endpoints fit.
    for q in (0., np.pi):
        p = plan(q)
        p.joint_trajectory.points[0] = point(q)
        assert check(p, desired=point(q), target=target, model=fk('1 0 0'), roi_margin_px=36.)['ok']
    with pytest.raises(ObservationViewError, match='image margin'):
        check(plan(np.pi), target=target, model=fk('1 0 0'), roi_margin_px=36.)


def test_full_wrapper_requires_real_support_and_reports_independent_scopes():
    p = plan()
    report = validate_observation_view_trajectory(p, fk(), np.eye(4), corners(), CENTER,
        measured_support(), K, 640, 480, point(0.))
    assert report['ok']
    assert report['endpoint_support']['qualifying_quadrants'] == 4
    assert report['endpoint_support']['requires_arrival_measured_support_check']
    with pytest.raises(ObservationViewError, match='unavailable'):
        validate_observation_view_trajectory(p, fk(), np.eye(4), corners(), CENTER,
            None, K, 640, 480, point(0.))


def test_two_quadrants_are_rejected_despite_many_real_points():
    with pytest.raises(ObservationViewError, match='spatial coverage') as captured:
        validate_observation_support_projection(np.eye(4), measured_support(half=True), CENTER, K, 640, 480)
    assert captured.value.audit['support_pixels'] > 120
    assert captured.value.audit['qualifying_quadrants'] == 2


def test_duplicate_support_points_cannot_fabricate_coverage():
    duplicates = np.repeat(measured_support()[:1], 2000, axis=0)
    with pytest.raises(ObservationViewError, match='support pixels') as captured:
        validate_observation_support_projection(np.eye(4), duplicates, CENTER, K, 640, 480)
    assert captured.value.audit['support_pixels'] == 1


@pytest.mark.parametrize('case', ['names', 'stamp', 'time', 'nan', 'derivative', 'moving',
    'missing_acceleration', 'samples', 'depth_budget', 'time_budget', 'corners', 'mount'])
def test_malformed_or_unresolved_trajectory_fails_closed(case):
    p, desired, kw = plan(.1), point(0.), {}
    if case == 'names': p.joint_trajectory.joint_names = ['wrong']
    if case == 'stamp': p.joint_trajectory.header.stamp = rospy.Time(1)
    if case == 'time': p.joint_trajectory.points[1].time_from_start = rospy.Duration(0)
    if case == 'nan': p.joint_trajectory.points[1].positions = [float('nan')]
    if case == 'derivative': p.joint_trajectory.points[1].velocities = [0., 0.]
    if case == 'moving': desired.velocities = [.01]
    if case == 'missing_acceleration': desired.accelerations = []
    if case == 'samples': kw['max_samples'] = 2
    if case == 'depth_budget': kw['max_depth'] = 0
    if case == 'time_budget':
        ticks = iter([0., 1., 2., 3., 4., 5.])
        kw.update(max_seconds=.1, clock=lambda: next(ticks))
    if case == 'corners': kw['target'] = np.zeros((8, 3))
    if case == 'mount':
        with pytest.raises(ObservationViewError, match='rigid transform'):
            validate_observation_target_trajectory(p, fk(), np.zeros((4, 4)), corners(), K, 640, 480, desired)
        return
    with pytest.raises(ObservationViewError) as captured:
        check(p, desired=desired, **kw)
    assert captured.value.audit['ok'] is False


@pytest.mark.parametrize('k,w,h', [((0., 400., 320., 240.),640,480), (K,0,480),
    (K,640,480.5), ((400.,400.,320.,float('nan')),640,480)])
def test_invalid_camera_inputs(k, w, h):
    with pytest.raises(ObservationViewError):
        validate_observation_target_trajectory(plan(), fk(), np.eye(4), corners(), k, w, h, point(0.))


@pytest.mark.parametrize('points', [None, [], [[float('nan'),0.,0.]], [[1.,2.]]])
def test_missing_or_malformed_support(points):
    with pytest.raises(ObservationViewError):
        validate_observation_support_projection(np.eye(4), points, CENTER, K, 640, 480)


def test_target_depth_gate_and_behind_camera_reject():
    for center in (np.array([-.3, 0., 0.]), np.array([.06, 0., 0.]), np.array([.51, 0., 0.])):
        with pytest.raises(ObservationViewError, match='depth range'):
            check(target=corners(center))


def test_three_by_three_intrinsics_and_support_rotation():
    k = np.array([[400.,0.,320.], [0.,400.,240.], [0.,0.,1.]])
    transform = fk('1 0 0')([np.pi])
    report = validate_observation_support_projection(transform, measured_support(), CENTER, k, 640, 480)
    assert report['qualifying_quadrants'] == 4
