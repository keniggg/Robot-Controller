"""Exact endpoint scheduling plus controlled original strict-loop regressions.

No new path for the recorded poses is claimed. The synthetic ROS-response
cases exercise strict/following/commit authority and success-first scheduling,
not physical IK. Comparison quotas begin only after a full qualified path.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib
import json

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_silhouette_observation_planning import context
from alicia_grasp_modes.silhouette_observation_order import (
    diagnostic_coverage_indices, transform_fingerprint, COVERAGE_POLICY)


FIXTURE = Path(__file__).parent / 'fixtures/gen9_silhouette_coverage.json'


def test_recorded_47_keep_all_original_poses_and_cover_distances_before_repeats():
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == '69aa5cc9944a32dc376f2cd374fc26e7fe1340e652858faf996cca8b07e425e4'
    data = json.loads(FIXTURE.read_text())
    rows = sorted(data['observation_candidates'], key=lambda r:
        r['observation_view']['check_scheduling_index'])
    assert len(rows) == 47
    assert all(r['observation_envelope']['ok'] for r in rows)
    assert all(r['observation_view']['check_scheduling_metrics']['endpoint_ik_hint'][
        'error_code'] == -31 for r in rows)
    checked = [r for r in data['observation_orientation_search']['variant_results'] if r['checked']]
    assert len(checked) == 4
    assert len({round(r['camera_target_distance_m'], 9) for r in checked}) == 2
    # Recovered original TF must match the original source hash; no current
    # TF or numerically different static-calibration reconstruction is used.
    mount = np.asarray(data['source_transforms']['tool_from_camera'], dtype=float)
    assert transform_fingerprint(mount) == data['source']['captured_audit']['tool_camera_sha256']
    before = json.dumps(rows, sort_keys=True)
    distances = [r['camera_target_distance_m'] for r in rows]
    axes = [(np.asarray(r['base_from_tool']) @ mount)[:3, 0] for r in rows]
    order = diagnostic_coverage_indices(distances, axes, .26)
    assert sorted(order) == list(range(47))
    assert json.dumps(rows, sort_keys=True) == before
    actual = [round(distances[i], 9) for i in order[:5]]
    assert actual == [round(x, 9) for x in (.26982230422230874, .19618807360917923,
        .4626238527816416, .28500000000000003, .3738119263908207)]
    assert len(set(actual[:4])) == 4  # Original time budget may still admit only four.
    assert all(not rows[i]['observation_view']['endpoint_schedule_proves_joint_motion'] for i in order)


def test_direction_coverage_uses_optical_axis_not_roll_labels():
    # Two different rolls around X share [1,0,0]; a distinct optical axis
    # gets a turn before returning to that view direction at another distance.
    axes = [(1, 0, 0), (1, 0, 0), (0, 1, 0), (1, 0, 0)]
    order = diagnostic_coverage_indices([.26, .20, .20, .40], axes, .26)
    assert order == (0, 2, 3, 1)


def search_context(configured, monkeypatch, tmp_path, *, hints=False):
    node, prepared, _, variant = context(configured, monkeypatch, tmp_path)
    reference = base.pose_matrix(variant['sequence'].pregrasp)
    variants = []
    for index, distance in enumerate((.46, .37, .46, .46, .27, .196, .285, .37)):
        v = deepcopy(variant)
        a = .02 * (index + 1)
        r = np.eye(4); r[:3, :3] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]
        r[:3, 3] = [.002 * index, 0, 0]
        matrix = reference @ r
        v['sequence'].pregrasp = base.make_pose_stamped('base_link', matrix[:3, 3],
            base.quaternion_from_matrix(matrix), stamp=prepared.stamp)
        v.update(camera_target_distance_m=distance, camera_tilt_offset_deg=45., preference_index=index)
        variants.append(v)
    monkeypatch.setattr(node, '_far_field_observation_variants', lambda *_: (tuple(variants), ()))
    monkeypatch.setattr(node, '_tool_from_camera_matrix', lambda: np.eye(4))
    def endpoint_hints(rows, current):
        assert current is prepared
        if hints:
            key = transform_fingerprint(base.pose_matrix(variants[0]['sequence'].pregrasp))
            return {key: dict(status='endpoint_found', joint_max_delta_rad=.1, joint_l1_delta_rad=.2)}, {}
        return {}, dict(hinted_count=0, execution_authority=False)
    monkeypatch.setattr(node, '_observation_endpoint_search_hints', endpoint_hints)
    clock = NS(value=0.)
    node._observation_tilt_clock = lambda: clock.value
    node._observation_comparison_clock = lambda: clock.value
    checked = []
    def strict(pose, observation=False):
        assert observation is True
        index = next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)
        checked.append(index); clock.value += .8
        return base.MoveItResult(False, 0., 0., 'controlled original service rejection',
            failure_code='MOVEIT_UNREACHABLE'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    return node, prepared, variants, checked, clock


def test_real_mode_strict_loop_covers_all_original_views_before_negative_conclusion(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'SILHOUETTE_OBSERVATION_NO_SAFE_VIEW'
    assert checked[:4] == [4, 5, 0, 6]
    assert len(checked) == len(variants) and set(checked) == set(range(len(variants)))
    budget = node._observation_tilt_search_budget
    assert budget['checked_count'] == 8 and budget['used_sec'] == pytest.approx(6.4)
    assert budget['discovery_checked_count'] == 8
    runtime = node._stable_variant_runtime[(0, 0)]
    evidence = runtime['unknown_observation_scheduling']['first_incumbent_coverage']
    assert evidence['policy'] == COVERAGE_POLICY and not evidence['full_path_incumbent_found']
    assert not evidence['execution_authority'] and not any(p.valid for p in node.rich_plan_pub.messages)
    assert len(runtime['observation_orientation_search']['variant_results']) == len(variants)


def test_first_full_qualified_path_restores_original_comparison_order(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    def strict(pose, observation=False):
        index = next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose)
        checked.append(index); clock.value += .8
        ok = index == 5
        return base.MoveItResult(ok, .3, .2,
            'controlled response observation_screened_duration_sec=2.0',
            True if ok else None, True if ok else None, True if ok else None, True if ok else None,
            failure_code='' if ok else 'MOVEIT_UNREACHABLE'), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    result = node._accept_prediction(prepared)
    assert result['status'] == 'PREVIEW_READY'
    assert checked == [4, 5, 0, 1, 2, 3]
    runtime = node._stable_variant_runtime[(0, 0)]
    assert runtime['_observation_qualified_incumbent_ticket'] is prepared.ticket
    assert runtime['unknown_observation_scheduling']['first_incumbent_coverage']['full_path_incumbent_found']
    assert node._observation_tilt_search_budget['checked_count'] == 6
    assert node._observation_tilt_search_budget['comparison_start_count'] == 2


def test_failed_following_gate_does_not_create_incumbent(configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    def strict(pose, observation=False):
        checked.append(next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose))
        clock.value += .8
        return base.MoveItResult(True, .3, .2, 'controlled endpoint success', True, True, True, True), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    monkeypatch.setattr(node, '_observation_following_support_evaluation', lambda p, r, target:
        (replace(r, reachable=False, failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'), {'ok': False}))
    node._accept_prediction(prepared)
    assert checked[:4] == [4, 5, 0, 6]
    assert len(checked) == len(variants) and set(checked) == set(range(len(variants)))
    assert node._stable_variant_runtime[(0, 0)]['_observation_qualified_incumbent_ticket'] is None
    assert not any(p.valid for p in node.rich_plan_pub.messages)


def test_positive_ik_hint_retains_complete_original_order(configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path, hints=True)
    node._accept_prediction(prepared)
    assert checked == list(range(len(variants)))
    assert 'first_incumbent_coverage' not in node._stable_variant_runtime[(0, 0)]['unknown_observation_scheduling']


def test_ticket_invalidated_during_coverage_check_cannot_become_incumbent(
        configured, monkeypatch, tmp_path):
    node, prepared, variants, checked, clock = search_context(configured, monkeypatch, tmp_path)
    runtimes = []
    def strict(pose, observation=False):
        checked.append(next(i for i, v in enumerate(variants) if v['sequence'].pregrasp is pose))
        runtimes.append(node._stable_variant_runtime[(0, 0)])
        node.stop_streaming()
        return base.MoveItResult(True, .3, .2,
            'controlled response observation_screened_duration_sec=2.0',
            True, True, True, True), {}, ''
    monkeypatch.setattr(node, '_strict_moveit_evaluation', strict)
    with pytest.raises(base.StreamResultCancelled):
        node._accept_prediction(prepared)
    assert checked == [4]
    assert runtimes[0]['_observation_qualified_incumbent_ticket'] is None
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    assert not (tmp_path / 'planning.json.execution').exists()


@pytest.mark.parametrize('phase', ['initial_contact', 'observing_contact', 'direct', 'carton'])
def test_direct_and_non_diagnostic_phase_never_enter_new_order(configured, monkeypatch, phase):
    node, _ = configured
    node._mode_selection = dict(mode='unknown', strategy='observe_return')
    node._observe_return_phase = phase
    node._observe_return_paused = False
    node._observe_return_diagnostic_observation_only = True
    node.grasp_config = {}
    if phase == 'direct':
        node._mode_selection['strategy'] = 'direct'; node._observe_return_phase = 'observation_prefix'
    elif phase == 'carton':
        node._mode_selection['mode'] = 'carton'; node._observe_return_phase = 'observation_prefix'
    monkeypatch.setattr(node, '_order_unknown_observation_checks', lambda *_: pytest.fail('diagnostic scheduler entered'))
    assert node._order_observation_checks(()) == ()
