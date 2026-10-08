"""Support predictions reorder existing views without changing admission."""
from itertools import product
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_support_projection
from alicia_grasp_modes.observation_support_schedule import (
    MAX_WITNESS_POINTS, ObservationSupportSchedule, support_schedule_priority)
from test_remote_modes import base, configured
from test_observation_information_scheduler import scheduler_context, hints_for


def source_from_points(points):
    center = np.array([.28, 0., 0.])
    return dict(points_base=points, center_base=center,
        corners_base=center + np.array(list(product((-.01, .01), repeat=3))),
        tool_from_camera=np.eye(4),
        camera=dict(fx=400., fy=400., cx=320., cy=240., width=640, height=480),
        view_limits=dict(effective_visibility_limits=dict(
            roi_margin_px=36., minimum_depth_m=.07, maximum_depth_m=.5)))

def points_at_pixels(uv):
    uv = np.asarray(uv)
    return np.column_stack((np.full(len(uv), .3),
        (.3/400.)*(320.-uv[:, 0]), (.3/400.)*(240.-uv[:, 1])))

def full_projection(source, pose):
    camera = source['camera']
    return validate_observation_support_projection(pose @ source['tool_from_camera'],
        source['points_base'], source['center_base'],
        [camera[k] for k in ('fx', 'fy', 'cx', 'cy')], camera['width'], camera['height'],
        target_corners_base=source['corners_base'], **source['view_limits']['effective_visibility_limits'])


def test_sufficient_witness_uses_original_points_and_counts_are_lower_bounds():
    rng = np.random.RandomState(31)
    points = points_at_pixels(rng.uniform([80., 60.], [560., 420.], (74098, 2)))
    source = source_from_points(points)
    schedule = ObservationSupportSchedule(source)
    exact = points[np.linspace(0, len(points)-1, MAX_WITNESS_POINTS, dtype=np.int64)]
    assert np.array_equal(schedule.points, exact)
    for y in (0., -.005, .005):
        pose = np.eye(4); pose[1, 3] = y
        actual = schedule.evaluate(pose)
        full = full_projection(source, pose)
        assert actual['status'] == 'sufficient'
        assert actual['projection']['support_pixels'] <= full['support_pixels']
        assert all(n <= full['quadrant_pixels'][key]
            for key, n in actual['projection']['quadrant_pixels'].items())
        assert actual['projection_counts_are_lower_bounds']
        assert actual['execution_authority'] is False
        assert actual['requires_full_actual_path_and_endpoint_checks']


def test_failed_sparse_witness_is_unknown_even_when_full_set_passes():
    points = np.tile([.3, 0., 0.], (4000, 1))
    indices = np.linspace(0, len(points)-1, MAX_WITNESS_POINTS, dtype=np.int64)
    other = np.setdiff1d(np.arange(len(points)), indices)
    uv = np.array(list(product(np.arange(200., 441., 8.), np.arange(120., 361., 8.))))
    points[other[:len(uv)]] = points_at_pixels(uv)
    source = source_from_points(points)
    schedule = ObservationSupportSchedule(source)
    assert full_projection(source, np.eye(4))['ok']
    result = schedule.evaluate(np.eye(4))
    assert result['status'] == 'unknown' and not result['full_set_evaluated']
    assert result['projection']['ok'] is False
    assert len(schedule.points) == 2048  # No full-cloud retry, synthetic completion or pruning.


def test_nonfinite_original_point_outside_witness_cannot_claim_sufficient():
    points = np.tile([.3, 0., 0.], (4000, 1))
    points[2] = np.nan  # Index 2 is excluded by this deterministic witness.
    assert 2 not in np.linspace(0, 3999, MAX_WITNESS_POINTS, dtype=np.int64)
    with pytest.raises(ValueError):
        ObservationSupportSchedule(source_from_points(points))


def test_full_small_cloud_can_report_insufficient_without_removing_candidate():
    schedule = ObservationSupportSchedule(source_from_points(np.tile([.3, 0., 0.], (120, 1))))
    row = schedule.evaluate(np.eye(4))
    assert row['status'] == 'insufficient' and row['full_set_evaluated']
    assert schedule.report()['candidates_removed'] == 0
    assert support_schedule_priority(row) > support_schedule_priority(dict(status='unknown'))


def test_full_pose_cache_does_not_alias_translation_or_return_mutable_audit():
    source = source_from_points(points_at_pixels(np.array(list(product(range(180, 460, 4), range(100, 380, 4))))))
    schedule = ObservationSupportSchedule(source)
    original = schedule.evaluate(np.eye(4))
    original['projection']['ok'] = 'caller mutation'
    assert schedule.evaluate(np.eye(4))['projection']['ok'] is True
    pose = np.eye(4); pose[0, 3] = .27
    second = schedule.evaluate(pose)
    assert second['status'] == 'unknown'
    assert second['pose_sha256'] != original['pose_sha256']
    assert schedule.report()['unique_pose_count'] == 2


def test_supported_view_without_ik_hint_precedes_unproven_high_utility_hint(configured, monkeypatch, tmp_path):
    node, _, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path, [(1.1, .99), (.01, .1)])
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_:
        (hints_for(variants, [.01, None]), {}))
    result = node._order_unknown_observation_checks(tuple(variants))
    assert result == (variants[1], variants[0])
    rows = [v['observation_view']['check_scheduling_metrics']['measured_support_schedule'] for v in result]
    assert [r['status'] for r in rows] == ['sufficient', 'unknown']
    assert runtime['unknown_observation_scheduling']['pruned_candidate_count'] == 0
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_finite_ik_hint_budget_prioritizes_supported_view_too(configured, monkeypatch, tmp_path):
    node, prepared, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(1.1, .99), (.01, .1)])
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
    state = JointState(name=['Joint%d' % i for i in range(1, 7)], position=[0.] * 6)
    state.header.stamp = base.rospy.Time(9, 900000000)
    node.latest_joint_state = state
    clock = [0.]; calls = []
    node._observation_endpoint_hint_clock = lambda: clock[0]
    def service(request):
        calls.append(request)
        clock[0] += 3.1
        return NS(error_code=NS(val=-31))
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *_a, **_k: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: service)
    _, report = node._observation_endpoint_search_hints(tuple(variants), prepared)
    assert len(calls) == 1 and calls[0].ik_request.pose_stamped == variants[1]['sequence'].pregrasp
    assert report['maximum_start_budget_sec'] == 3. and report['maximum_queries'] == 96
    assert report['candidates_removed'] == 0
    assert report['measured_support_schedule']['status_counts'] == dict(sufficient=1, unknown=1, insufficient=0)


@pytest.mark.parametrize('change', ['camera', 'handeye', 'limits', 'prepared', 'points'])
def test_cached_support_hint_cannot_bypass_source_revalidation(configured, monkeypatch, tmp_path, change):
    node, prepared, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .5)])
    schedule = node._observation_support_schedule(prepared)
    schedule.evaluate(base.pose_matrix(variants[0]['sequence'].pregrasp))
    if change == 'camera':
        node._camera_intrinsics().fx += 1.
    elif change == 'handeye':
        matrix = np.eye(4); matrix[0, 3] = .001
        monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: matrix)
    elif change == 'limits':
        configured[1]['/grasp_6d/remote/camera_visibility_margin_px'] = 40.
    elif change == 'prepared':
        node._diagnostic_observation_view_prepared = object()
    else:
        source = base._thaw_request_evidence(node._diagnostic_observation_view_source)
        source['points_base'][0, 0] += .001
        node._diagnostic_observation_view_source = base._freeze_request_evidence(source)
    with pytest.raises(ValueError):
        node._observation_support_schedule(prepared)


def test_rebinding_same_request_creates_new_cache(configured, monkeypatch, tmp_path):
    node, prepared, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .5)])
    first = node._observation_support_schedule(prepared)
    first.evaluate(base.pose_matrix(variants[0]['sequence'].pregrasp))
    node._diagnostic_observation_view_source = base._freeze_request_evidence(
        base._thaw_request_evidence(node._diagnostic_observation_view_source))
    second = node._observation_support_schedule(prepared)
    assert second is not first and second.report()['unique_pose_count'] == 0
