"""Real hint/dispatcher replay of GEN14 poses; IK transport is deliberately inert."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from sensor_msgs.msg import JointState

from test_remote_modes import configured, base
from test_observation_information_scheduler import scheduler_context
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint

FIXTURE = Path(__file__).parent / 'fixtures/gen14_endpoint_hint_coverage.json'


def recorded_context(configured, monkeypatch, tmp_path, positive_index=None):
    data = json.loads(FIXTURE.read_text())
    node, prepared, _, runtime = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .8)])
    mount = np.asarray(data['tool_from_camera'])
    source = dict(node._require_diagnostic_observation_view_source(prepared))
    source['tool_from_camera'] = mount
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', lambda _: source)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: mount)
    variants, witnesses, indices = [], {}, {}
    for row in data['rows']:
        matrix = np.asarray(row['base_from_tool'])
        pose = base.make_pose_stamped('base_link', matrix[:3, 3], base.quaternion_from_matrix(matrix), stamp=prepared.stamp)
        info = deepcopy(row['information_utility']); info['recorded_index'] = row['candidate_index']
        view = dict(information_utility=info)
        variants.append(dict(sequence=NS(pregrasp=pose, observation_view_audit=deepcopy(view)),
            observation_view=view, camera_target_distance_m=row['camera_target_distance_m'],
            camera_tilt_offset_deg=row['camera_tilt_offset_deg'], camera_roll_offset_deg=row['camera_roll_offset_deg'],
            preference_index=row['preference_index'],
            observation_envelope=base.ObservationEnvelopeResult(True, '', '', row['observation_envelope']['minimum_support_clearance_m'])))
        fingerprint = transform_fingerprint(base.pose_matrix(pose))
        witnesses[fingerprint] = deepcopy(row['metrics']['measured_support_schedule'])
        indices[fingerprint] = row['candidate_index']
    support = NS(evaluate=lambda pose: deepcopy(witnesses[transform_fingerprint(pose)]),
        report=lambda: deepcopy(data['scheduler_report']['measured_support_schedule']))
    monkeypatch.setattr(node, '_observation_support_schedule', lambda _: support)
    # Replay frozen information inputs; production hint sorting, pose fingerprint,
    # service requests, budget, cache and downstream dispatch remain real.
    original_utility = node._observation_information_utility
    class RecordedUtility(float):
        pass
    def utility(variant):
        value = RecordedUtility(original_utility(variant))
        value.recorded_index = indices[transform_fingerprint(base.pose_matrix(variant['sequence'].pregrasp))]
        return value
    monkeypatch.setattr(node, '_observation_information_utility', utility)
    def information(_metrics, info, _distance):
        row = data['rows'][info.recorded_index]['metrics']
        return {k: deepcopy(v) for k, v in row.items() if k not in (
            'endpoint_ik_hint', 'measured_support_schedule', 'information_motion_rank')}
    monkeypatch.setattr(node, '_observation_information_geometry', information)
    seed = data['joint_seed']
    state = JointState(name=seed['name'], position=seed['position'])
    state.header.stamp = base.rospy.Time(9, 900000000)
    node.latest_joint_state = state
    configured[1]['/robot_description'] = 'inert-recorded-scheduler-model'
    monkeypatch.setattr(base.rospy.Time, 'now', staticmethod(lambda: base.rospy.Time(10)))
    clock = [100.]; calls = []
    node._observation_endpoint_hint_clock = lambda: clock[0]
    def service(request):
        index = indices[transform_fingerprint(base.pose_matrix(request.ik_request.pose_stamped))]
        calls.append((index, deepcopy(request)))
        clock[0] += .042  # 72 starts in the unchanged 3 s budget, as in GEN14.
        if index == positive_index:
            solved = deepcopy(state); solved.position = [q + .1 for q in solved.position]
            return NS(error_code=NS(val=1), solution=NS(joint_state=solved))
        return NS(error_code=NS(val=-31))
    monkeypatch.setattr(base.rospy, 'wait_for_service', lambda *_args, **_kwargs: None)
    monkeypatch.setattr(base.rospy, 'ServiceProxy', lambda *_: service)
    return node, prepared, variants, runtime, data, calls, witnesses, clock


def test_recorded_72_query_budget_covers_low_tilts_and_distances(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, data, calls, _, _ = recorded_context(configured, monkeypatch, tmp_path)
    before = [base.pose_matrix(v['sequence'].pregrasp).copy() for v in variants]
    ordered = tuple(node._order_unknown_observation_checks(variants))
    checked = [index for index, _ in calls]
    queried_tilts = {round(data['rows'][index]['camera_tilt_offset_deg'], 3) for index in checked}
    assert queried_tilts == {27.606, 28.09, 45.0}, checked
    assert len(checked) == len(set(checked)) == 72
    assert len({round(data['rows'][i]['camera_target_distance_m'], 6) for i in checked[:12]}) >= 3
    assert checked[0] == 45  # Preserve the best information-ranked first probe.
    assert any(data['rows'][i]['camera_tilt_offset_deg'] < 30. for i in checked[:4])
    assert len(ordered) == 220 and {id(v) for v in ordered} == {id(v) for v in variants}
    assert all(np.array_equal(base.pose_matrix(v['sequence'].pregrasp), b) for v, b in zip(variants, before))
    for _, request in calls:
        assert request.ik_request.avoid_collisions
        assert request.ik_request.timeout.to_sec() <= .02
        assert request.ik_request.robot_state.joint_state == node.latest_joint_state
    report = runtime['unknown_observation_scheduling']['endpoint_ik_search_hint']
    assert report['maximum_start_budget_sec'] == 3. and report['maximum_queries'] == 96
    assert report['hinted_count'] == 0 and report['candidates_removed'] == 0
    assert not report['execution_authority']
    node._observation_endpoint_search_hints(tuple(reversed(variants)), prepared)
    assert len(calls) == 72  # No budget renewal on another scheduling call.
    assert not any(message.valid for message in node.rich_plan_pub.messages)


def test_newly_probed_low_tilt_positive_hint_leads_but_cannot_bypass_strict_failure(configured, monkeypatch, tmp_path):
    # Index 0 is low-tilt and was unqueried in the real run; this positive reply
    # is controlled evidence of the interface, not a claim that real IK succeeds.
    node, prepared, variants, runtime, _, calls, _, clock = recorded_context(
        configured, monkeypatch, tmp_path, positive_index=0)
    runtime.update(observation_variants=tuple(variants), observation_sequence=variants[0]['sequence'], soft_evidence={})
    candidate = NS(track_id=71, variant_index=0)
    node._stable_variant_runtime = {(71, 0): runtime}
    checked = []
    def strict(pose, observation=False):
        checked.append(next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose))
        clock[0] += .85
        return base.MoveItResult(False, 0., 0., 'controlled collision rejection', failure_code='MOVEIT_COLLISION'), {}, {}
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_source_reserve_available', lambda _: True)
    node._observation_tilt_clock = lambda: clock[0]
    outcome = node._check_moveit_stable_candidate(candidate)
    assert any(index == 0 for index, _ in calls)
    assert checked[0] == 0
    assert len(checked) == 4 and len(set(checked)) == 4
    assert not outcome.reachable and runtime['_observation_qualified_incumbent_ticket'] is None
    assert not any(message.valid for message in node.rich_plan_pub.messages)


def test_probe_coverage_preserves_original_support_tiers(configured, monkeypatch, tmp_path):
    node, prepared, variants, _, data, calls, witnesses, clock = recorded_context(configured, monkeypatch, tmp_path)
    # Force an information-leading real pose into the unknown tier; it must not
    # jump ahead of sufficient evidence just to improve geometric coverage.
    fingerprint = transform_fingerprint(base.pose_matrix(variants[45]['sequence'].pregrasp))
    witnesses[fingerprint]['status'] = 'unknown'
    node._observation_endpoint_search_hints(variants, prepared)
    assert calls[0][0] != 45 and all(
        witnesses[transform_fingerprint(base.pose_matrix(request.ik_request.pose_stamped))]['status'] == 'sufficient'
        for _, request in calls)

