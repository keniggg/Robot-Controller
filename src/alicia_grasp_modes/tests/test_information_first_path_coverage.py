"""Ordinary supplemental search coverage; recorded geometry is not a new path proof."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib
import json

import numpy as np
import pytest

from test_remote_modes import configured, base
from test_observation_information_scheduler import scheduler_context, hints_for
from alicia_grasp_modes.silhouette_observation_order import (
    diagnostic_coverage_indices, transform_fingerprint, INFORMATION_COVERAGE_POLICY)
from alicia_grasp_modes.silhouette_observation_planning import INFORMATION_ORDER_POLICY

FIXTURE = Path(__file__).parent / 'fixtures/gen6_information_search_coverage.json'


def test_recorded_gen6_preserves_220_poses_and_spreads_first_checks():
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == '6f2d55d7732850b64ac4e2883b5bb39891800c92e38701aa8c77a3812636a7fa'
    data = json.loads(FIXTURE.read_text())
    rows, checked = data['rows'], data['checked']
    assert len(rows) == 220 and len(checked) == 4
    assert {r['camera_roll_offset_deg'] for r in checked} == {-15.}
    assert {r['camera_tilt_offset_deg'] for r in checked} == {45.}
    assert max(r['camera_target_distance_m'] for r in checked) - min(r['camera_target_distance_m'] for r in checked) < .002
    assert data['scheduler_report']['endpoint_ik_search_hint']['hinted_count'] == 0
    mount = np.asarray(data['tool_from_camera'])
    assert transform_fingerprint(mount) == data['tool_from_camera_sha256']
    before = json.dumps(rows, sort_keys=True)
    distances = [r['camera_target_distance_m'] for r in rows]
    axes = [(np.asarray(r['base_from_tool']) @ mount)[:3, 0] for r in rows]
    order = diagnostic_coverage_indices(distances, axes, .26)
    assert sorted(order) == list(range(220))
    assert json.dumps(rows, sort_keys=True) == before
    first = [distances[i] for i in order[:4]]
    assert min(first) < .19 and max(first) > .38
    assert first[0] == pytest.approx(.285)
    assert len(set(np.round(first, 9))) == 4
    assert all(r['observation_envelope']['ok'] for r in rows)
    assert data['recorded_execution_authority'] is False


def ordinary_search(configured, monkeypatch, tmp_path, positive_hint=False):
    node, prepared, variants, runtime = scheduler_context(configured, monkeypatch, tmp_path,
        [(.01 + i * .01, .8) for i in range(8)])
    distances = (.46, .37, .46, .46, .27, .196, .285, .37)
    for index, variant in enumerate(variants):
        variant['camera_target_distance_m'] = distances[index]
        variant['camera_tilt_offset_deg'] = 45.
        variant['observation_side_evidence'] = {}
    node.grasp_config['observation_camera_target_nominal_distance_m'] = .26
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_:
        (hints_for(variants, [.01] + [None] * 7) if positive_hint else {}, {}))
    sequence = variants[0]['sequence']
    runtime.update(sequence=sequence, observation_sequence=sequence,
        observation_variants=tuple(variants), soft_evidence={})
    node._stable_variant_runtime = {(31, 1): runtime}
    candidate = NS(track_id=31, variant_index=1, candidate_source='tabletop_geometry')
    # Exercise the original strict loop; deterministic responses substitute
    # only read-only planning services and their independent following proof.
    clock = NS(value=0.)
    node._observation_tilt_clock = lambda: clock.value
    node._observation_comparison_clock = lambda: clock.value
    checked = []
    def strict(pose, observation=False):
        assert observation
        index = next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)
        checked.append(index); clock.value += .8
        return base.MoveItResult(False, 0., 0., 'controlled plan rejection',
            failure_code='MOVEIT_UNREACHABLE'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_commit_strict_moveit_evaluation', lambda _, evaluation: evaluation[0])
    return node, prepared, variants, runtime, candidate, clock, checked


def test_ordinary_discovery_covers_finite_set_beyond_soft_optimization_budget(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path)
    outcome = node._check_moveit_stable_candidate(candidate)
    assert not outcome.reachable and outcome.failure_code == 'MOVEIT_UNREACHABLE'
    assert checked[:4] == [4, 5, 0, 6] and sorted(checked) == list(range(8))
    assert node._observation_tilt_search_budget['checked_count'] == 8
    assert node._observation_tilt_search_budget['discovery_checked_count'] == 8
    assert node._observation_tilt_search_budget['used_sec'] == pytest.approx(6.4)
    assert runtime['observation_orientation_search']['search_complete'] is True
    coverage = runtime['unknown_observation_scheduling']['first_incumbent_coverage']
    assert coverage['policy'] == INFORMATION_COVERAGE_POLICY
    assert coverage['after_incumbent_policy'] == INFORMATION_ORDER_POLICY
    assert not coverage['full_path_incumbent_found'] and not coverage['execution_authority']
    assert len(runtime['observation_orientation_search']['variant_results']) == len(variants)
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_ordinary_qualified_incumbent_restores_information_order_and_original_count_cap(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path)
    def strict(pose, observation=False):
        index = next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)
        checked.append(index); clock.value += .8
        ok = index == 5
        return base.MoveItResult(ok, .3, .2, 'controlled observation_screened_duration_sec=2.0',
            True if ok else None, True if ok else None, True if ok else None, True if ok else None,
            failure_code='' if ok else 'MOVEIT_UNREACHABLE'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_following_support_evaluation', lambda p, r, target: (r, {'ok': True}))
    # Avoid serializing the controlled endpoint's incomplete physical fixture;
    # selection still comes from the unmodified strict-loop implementation.
    monkeypatch.setattr(node, '_activate_far_field_observation_variant', lambda *args:
        runtime.update(observation_orientation_search={'object_yaw_used': True}))
    outcome = node._check_moveit_stable_candidate(candidate)
    assert outcome.reachable
    assert checked == [4, 5, 0, 1, 2, 3, 6, 7]
    assert runtime['_observation_qualified_incumbent_ticket'] is prepared.ticket
    assert runtime['unknown_observation_scheduling']['first_incumbent_coverage']['full_path_incumbent_found']
    budget = node._observation_tilt_search_budget
    assert budget['discovery_checked_count'] == 2
    assert budget['checked_count']-budget['comparison_start_count'] == 6


def test_ordinary_failed_following_does_not_switch_from_coverage(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path)
    def strict(pose, observation=False):
        checked.append(next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)); clock.value += .8
        return base.MoveItResult(True, .3, .2, 'controlled endpoint'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_following_support_evaluation', lambda p, r, target:
        (replace(r, reachable=False, failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'), {'ok': False}))
    outcome = node._check_moveit_stable_candidate(candidate)
    assert not outcome.reachable and checked[:4] == [4, 5, 0, 6]
    assert sorted(checked) == list(range(8))
    assert runtime['_observation_qualified_incumbent_ticket'] is None


def test_ordinary_failed_positive_ik_hint_enters_coverage(configured, monkeypatch, tmp_path):
    node, _, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path, positive_hint=True)
    node._check_moveit_stable_candidate(candidate)
    assert checked[:4] == [0, 4, 5, 6] and sorted(checked) == list(range(8))
    coverage = runtime['unknown_observation_scheduling']['first_incumbent_coverage']
    assert coverage['positive_hint_first_original_index'] == 0
    assert not coverage['full_path_incumbent_found']


def test_ordinary_generator_revalidates_source_before_next_dispatch(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, *_ = ordinary_search(configured, monkeypatch, tmp_path)
    schedule = node._order_unknown_observation_checks(variants)
    assert next(schedule) is variants[4]
    def revoked(_):
        raise base.CandidateContractError('REFERENCE_INVALID', 'revoked')
    monkeypatch.setattr(node, '_require_unknown_observation_prepared', revoked)
    with pytest.raises(base.CandidateContractError):
        next(schedule)
    assert runtime.get('_observation_qualified_incumbent_ticket') is None


def test_multiple_positive_hints_cannot_starve_distance_coverage_after_first_rejection(configured, monkeypatch, tmp_path):
    node, _, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path)
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_:
        (hints_for(variants, [.01, .02, .03, .04, None, None, None, None]), {}))
    node._check_moveit_stable_candidate(candidate)
    assert checked[:4] == [0, 4, 5, 6] and sorted(checked) == list(range(8))
    assert not runtime['unknown_observation_scheduling']['first_incumbent_coverage']['full_path_incumbent_found']


def test_best_positive_hint_with_complete_path_restores_original_order_immediately(configured, monkeypatch, tmp_path):
    node, prepared, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path, positive_hint=True)
    def strict(pose, observation=False):
        index = next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)
        checked.append(index); clock.value += .8
        return base.MoveItResult(index == 0, .3, .2, 'controlled observation_screened_duration_sec=2.0',
            failure_code='' if index == 0 else 'MOVEIT_UNREACHABLE'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_following_support_evaluation', lambda p, r, target: (r, {'ok': True}))
    monkeypatch.setattr(node, '_activate_far_field_observation_variant', lambda *args:
        runtime.update(observation_orientation_search={'object_yaw_used': True}))
    outcome = node._check_moveit_stable_candidate(candidate)
    assert outcome.reachable and checked == [0, 1, 2, 3, 4, 5, 6]
    assert runtime['_observation_qualified_incumbent_ticket'] is prepared.ticket
    budget = node._observation_tilt_search_budget
    assert budget['discovery_checked_count'] == 1
    assert budget['checked_count']-budget['comparison_start_count'] == 6
    assert runtime['observation_orientation_search']['unchecked_variant_count'] == 1


def test_positive_hint_following_rejection_still_enters_coverage(configured, monkeypatch, tmp_path):
    node, _, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path, positive_hint=True)
    def strict(pose, observation=False):
        checked.append(next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)); clock.value += .8
        return base.MoveItResult(True, .3, .2, 'controlled endpoint success'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_following_support_evaluation', lambda p, r, target:
        (replace(r, reachable=False, failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'), {'ok': False}))
    assert not node._check_moveit_stable_candidate(candidate).reachable
    assert checked[:4] == [0, 4, 5, 6] and sorted(checked) == list(range(8))
    assert runtime['_observation_qualified_incumbent_ticket'] is None


def test_recorded_gen6_real_scheduler_preserves_support_tiers_and_all_original_poses(configured, monkeypatch, tmp_path):
    # Replay actual frozen poses, information ranks, support witnesses and
    # absent IK hints through the production dispatcher. The source wrapper
    # is controlled; this does not requalify the old frame or any new path.
    data = json.loads(FIXTURE.read_text())
    rows = data['rows']
    node, prepared, _, runtime = scheduler_context(configured, monkeypatch, tmp_path, [(.01, .8)])
    mount = np.asarray(data['tool_from_camera'])
    source = dict(node._require_diagnostic_observation_view_source(prepared))
    source['tool_from_camera'] = mount
    monkeypatch.setattr(node, '_require_diagnostic_observation_view_source', lambda _: source)
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: mount)
    variants, witnesses = [], {}
    for row in rows:
        matrix = np.asarray(row['base_from_tool'])
        pose = base.make_pose_stamped('base_link', matrix[:3, 3], base.quaternion_from_matrix(matrix), stamp=prepared.stamp)
        view = dict(information_utility=deepcopy(row['information_utility']))
        envelope = row['observation_envelope']
        variants.append(dict(sequence=NS(pregrasp=pose, observation_view_audit=deepcopy(view)),
            observation_view=view, camera_target_distance_m=row['camera_target_distance_m'],
            camera_tilt_offset_deg=row['camera_tilt_offset_deg'],
            camera_roll_offset_deg=row['camera_roll_offset_deg'], preference_index=row['preference_index'],
            observation_envelope=base.ObservationEnvelopeResult(True, '', '', envelope['minimum_support_clearance_m'])))
        witnesses[transform_fingerprint(base.pose_matrix(pose))] = row['measured_support_schedule']
    support = NS(evaluate=lambda pose: deepcopy(witnesses[transform_fingerprint(pose)]),
        report=lambda: data['scheduler_report']['measured_support_schedule'])
    monkeypatch.setattr(node, '_observation_support_schedule', lambda _: support)
    recorded_metrics = iter(row['metrics'] for row in rows)
    monkeypatch.setattr(node, '_observation_information_geometry', lambda *_: deepcopy(next(recorded_metrics)))
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', lambda *_: ({}, {'hinted_count': 0}))
    before = [base.pose_matrix(v['sequence'].pregrasp).copy() for v in variants]
    actual = tuple(node._order_unknown_observation_checks(variants))
    ids = {id(v): i for i, v in enumerate(variants)}
    actual_indices = [ids[id(v)] for v in actual]
    assert sorted(actual_indices) == list(range(220))
    assert actual_indices[:4] == [28, 96, 122, 3]
    assert [v['camera_target_distance_m'] for v in actual[:4]] == pytest.approx(
        [.285, .1869780140086508, .38302198599134923, .19004609768144137])
    assert all(rows[i]['measured_support_schedule']['status'] == 'sufficient' for i in actual_indices[:200])
    assert all(rows[i]['measured_support_schedule']['status'] == 'unknown' for i in actual_indices[200:])
    assert all(np.array_equal(base.pose_matrix(v['sequence'].pregrasp), initial)
        for v, initial in zip(variants, before))
    coverage = runtime['unknown_observation_scheduling']['first_incumbent_coverage']
    assert coverage['dispatched_original_indices'] == actual_indices
    assert not coverage['execution_authority'] and not coverage['full_path_incumbent_found']
