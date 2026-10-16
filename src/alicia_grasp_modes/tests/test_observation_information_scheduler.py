"""Information-aware scheduling uses real source guards, without ROS motion."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState

from test_remote_modes import configured, base
from test_unknown_observation_view_source import generic_context
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint
from alicia_grasp_modes.silhouette_observation_planning import INFORMATION_ORDER_POLICY


def scheduler_context(configured, monkeypatch, tmp_path, specifications):
    node, prepared, _, _, fk = generic_context(configured, monkeypatch, tmp_path)
    variants = []
    for index, (angle, utility) in enumerate(specifications):
        matrix = fk([angle, 0., 0., 0., 0., 0.])
        pose = base.make_pose_stamped('base_link', matrix[:3, 3],
            base.quaternion_from_matrix(matrix), stamp=prepared.stamp)
        information = dict(utility=utility, gain_px=utility * 10., noise_px=1., prediction_only=True)
        sequence = NS(pregrasp=pose, observation_view_audit=dict(
            orientation_source='measured_missing_side_geodesic', information_utility=information))
        variants.append(dict(sequence=sequence, observation_view=deepcopy(sequence.observation_view_audit),
            observation_envelope=base.ObservationEnvelopeResult(True, '', '', .02),
            camera_roll_offset_deg=index * 15., camera_tilt_offset_deg=angle,
            camera_target_distance_m=.26 + index * .02, preference_index=index))
    runtime = dict(prepared=prepared, grasp_pose=variants[0]['sequence'].pregrasp)
    node._active_unknown_observation_check_runtime = runtime
    return node, prepared, variants, runtime


def hints_for(variants, displacements):
    return {transform_fingerprint(base.pose_matrix(variant['sequence'].pregrasp)):
        dict(status='endpoint_found', joint_max_delta_rad=delta, joint_l1_delta_rad=delta * 2.)
        for variant, delta in zip(variants, displacements) if delta is not None}


def test_high_information_can_precede_smaller_joint_delta(configured, monkeypatch, tmp_path):
    node, _, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path, [(0.01, .1), (.2, .8)])
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_:
        (hints_for(variants, [.01, .5]), dict(execution_authority=False)))
    result = tuple(node._order_unknown_observation_checks(tuple(variants)))
    assert result == (variants[1], variants[0])
    for variant in result:
        audit = variant['observation_view']
        assert audit['check_scheduling_policy'] == INFORMATION_ORDER_POLICY
        metrics = audit['check_scheduling_metrics']
        assert metrics['information_prediction_only'] is True
        assert metrics['motion_proxy_is_duration'] is False
        assert metrics['motion_cost_proxy_kind'] == 'maximum_joint_displacement_rad'
        assert audit['endpoint_schedule_proves_joint_motion'] is False
    assert runtime['unknown_observation_scheduling']['pruned_candidate_count'] == 0


def test_equal_utility_prefers_smaller_ik_motion_and_retains_unknown(configured, monkeypatch, tmp_path):
    node, _, variants, _ = scheduler_context(configured, monkeypatch, tmp_path,
        [(.01, .8), (.2, .8), (.3, .95)])
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_:
        (hints_for(variants, [.5, .1, None]), {}))
    result = tuple(node._order_unknown_observation_checks(tuple(variants)))
    assert result == (variants[1], variants[0], variants[2])
    assert result[-1]['observation_view']['check_scheduling_metrics']['endpoint_ik_hint'] == {}


def test_unknown_ik_uses_frozen_geometry_cost_and_keeps_utility_evidence(configured, monkeypatch, tmp_path):
    node, prepared, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .1), (.2, .8)])
    variants[1]['sequence'].observation_view_audit.pop('information_utility')
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_: ({}, {}))
    result = tuple(node._order_unknown_observation_checks(tuple(variants)))
    assert result == (variants[0], variants[1])
    metrics = variants[1]['observation_view']['check_scheduling_metrics']
    assert metrics['current_camera_target_distance_m'] == pytest.approx(.28)
    assert metrics['geometric_motion_proxy'] == pytest.approx(.2)
    assert variants[1]['observation_view']['information_utility']['utility'] == .8
    assert result[0]['observation_view']['check_scheduling_reference']['snapshot_stamp_ns'] == prepared.snapshot.stamp_ns


@pytest.mark.parametrize('utility', [None, {}, {'utility': True}, {'utility': float('nan')},
    {'utility': float('inf')}, {'utility': -.1}, {'utility': 1.01}, {'utility': '0.8'}])
def test_missing_or_invalid_information_never_calls_ik(configured, monkeypatch, tmp_path, utility):
    node, _, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .5)])
    variants[0]['observation_view']['information_utility'] = utility
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_: pytest.fail('IK before information validation'))
    with pytest.raises(base.CandidateContractError):
        node._order_unknown_observation_checks(tuple(variants))


def test_zero_utility_is_retained_at_tail_and_audit_is_strict_json(configured, monkeypatch, tmp_path):
    node, _, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path, [(.01, 0.), (.2, .8)])
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_: ({}, {}))
    result = tuple(node._order_unknown_observation_checks(tuple(variants)))
    assert result == (variants[1], variants[0])
    assert result[-1]['observation_view']['check_scheduling_metrics']['information_motion_rank'][0] is None
    json.dumps([v['observation_view'] for v in result], allow_nan=False)
    json.dumps(runtime['unknown_observation_scheduling'], allow_nan=False)


def test_probe_budget_tries_information_first_instead_of_nominal_distance(configured, monkeypatch, tmp_path):
    node, prepared, variants, _ = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .1), (.2, .8)])
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
    state = JointState(name=['Joint%d' % i for i in range(1, 7)], position=[0.] * 6)
    state.header.stamp = base.rospy.Time(9, 900000000)
    node.latest_joint_state = state
    clock = [0.]
    node._observation_endpoint_hint_clock = lambda: clock[0]
    calls = []
    def service(request):
        calls.append(request)
        clock[0] += 3.1  # One in-flight query exhausts the existing start budget.
        return NS(error_code=NS(val=-31))
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: service)
    hints, report = node._observation_endpoint_search_hints(tuple(variants), prepared)
    assert len(calls) == 1
    assert calls[0].ik_request.pose_stamped == variants[1]['sequence'].pregrasp
    assert calls[0].ik_request.avoid_collisions is True
    assert report['maximum_start_budget_sec'] == 3. and report['maximum_queries'] == 96
    assert report['probe_order_policy'] == 'measured_support_then_best_information_then_distance_direction_coverage'
    assert report['information_prediction_only'] is True and report['motion_proxy_is_duration'] is False
    assert report['candidates_removed'] == 0 and report['status'] == 'budget_exhausted'
    assert next(iter(hints.values()))['status'] == 'unknown'
    assert not any(message.valid for message in node.rich_plan_pub.messages)
