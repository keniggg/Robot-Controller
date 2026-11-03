"""A measured upper interior point still needs every original admission check.

Callback cases reuse the existing stubbed registered-geometry and strict
service fixtures. The production retry, height evaluator, evidence freezing,
plan hashing, recipe binding and deadline/cancellation flow execute unchanged.
Recorded finite-CAD and continuous-curve qualification is separate evidence.
"""
from copy import deepcopy
import gzip
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from alicia_flexible_grasp.grasp.contact_height_placement import contact_height_window_proposals
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, GRIPPER_CONTRACT_TOLERANCE_M)


@pytest.fixture
def frozen_height():
    data = json.loads(gzip.decompress(
        (ROOT / 'tests/fixtures/gen54_g53_height_window.json.gz').read_bytes()))
    assert data['source_audit_sha256'] == 'a3a4b24421979d13d6476f94154d120580ae6efa04e22a313953a8774baad38e'
    geometry = data['geometry']
    geometry['gripper'] = GripperGeometry(**geometry['gripper'])
    for name in ('actual_T_base_tool0', 'planned_T_base_tool0', 'contact_center_base',
                 'contact_height_axis_base', 'support_normal_base'):
        geometry[name] = np.asarray(geometry[name], dtype=float)
    return geometry, data['translation_evidence_limit_m']


def test_recorded_upper_interior_preserves_three_original_points_and_full_evidence(frozen_height):
    geometry, limit = frozen_height
    before = deepcopy(geometry)
    old = contact_height_window_proposals(translation_evidence_limit_m=limit, **geometry)
    new = contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=GRIPPER_CONTRACT_TOLERANCE_M, **geometry)
    assert len(old) == 3 and len(new) == 4
    for (old_pose, old_audit), (new_pose, new_audit) in zip(old, new[:3]):
        np.testing.assert_array_equal(old_pose, new_pose)
        assert old_audit == new_audit
    pose, audit = new[-1]
    upper = audit['allowed_normal_translation_upper_m']
    assert audit['normal_translation_m'] == pytest.approx(upper-limit-GRIPPER_CONTRACT_TOLERANCE_M)
    assert audit['normal_translation_m'] > max(a['normal_translation_m'] for _, a in old)
    assert audit['after']['ok'] and audit['after']['translation_evidence_limit_m'] == limit
    assert audit['after']['minimum_contact_overlap_m'] == .002
    assert audit['after']['minimum_support_clearance_m'] == .003
    assert audit['motion_authorized'] is False and audit['target_relocated'] is False
    np.testing.assert_array_equal(pose[:3, :3], np.asarray(before['actual_T_base_tool0'])[:3, :3])
    for name in geometry:
        if name != 'gripper':
            np.testing.assert_array_equal(geometry[name], before[name])


@pytest.mark.parametrize('added_band_half_height_m', [.0005, .001])
def test_changed_measured_target_height_changes_the_proposal(frozen_height, added_band_half_height_m):
    geometry, limit = frozen_height
    old = contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=GRIPPER_CONTRACT_TOLERANCE_M, **geometry)
    changed = deepcopy(geometry)
    lower, upper = changed['contact_height_bounds_m']
    changed['contact_height_bounds_m'] = [lower-added_band_half_height_m, upper+added_band_half_height_m]
    new = contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=GRIPPER_CONTRACT_TOLERANCE_M, **changed)
    assert len(new) == 4
    assert new[-1][1]['normal_translation_m'] > old[-1][1]['normal_translation_m']
    assert new[-1][1]['after']['translation_evidence_limit_m'] == limit


@pytest.mark.parametrize('reserve', [True, '0.0005', float('nan'), float('inf'), -.0005, 0.])
def test_invalid_explicit_reserve_cannot_propose(frozen_height, reserve):
    geometry, limit = frozen_height
    assert contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=reserve, **geometry) == ()


@pytest.mark.parametrize('invalid', ['normal', 'contact_interval', 'opening'])
def test_invalid_frozen_source_cannot_gain_a_height_proposal(frozen_height, invalid):
    geometry, limit = frozen_height
    if invalid == 'normal':
        geometry['support_normal_base'] = [float('nan'), 0., 1.]
    elif invalid == 'contact_interval':
        geometry['contact_height_bounds_m'] = [.01, -.01]
    else:
        geometry['opening_width_m'] = .06
    assert contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=GRIPPER_CONTRACT_TOLERANCE_M, **geometry) == ()


def test_small_remaining_window_keeps_the_original_three_only(frozen_height):
    geometry, limit = frozen_height
    old = contact_height_window_proposals(translation_evidence_limit_m=limit, **geometry)
    new = contact_height_window_proposals(translation_evidence_limit_m=limit,
        upper_interior_reserve_m=.01, **geometry)
    assert len(new) == 3
    for (p, a), (q, b) in zip(old, new):
        np.testing.assert_array_equal(p, q)
        assert a == b


def test_fourth_retry_rehashes_rechecks_and_binds_the_added_reserve(monkeypatch, frozen_height):
    from test_selected_contact_height_recipe import bound_fixture
    from test_contact_height_window_proposals import remote
    import test_contact_height_window_proposals as original_test
    monkeypatch.setattr(original_test, 'robust_geometry', lambda: deepcopy(frozen_height[0]))
    values, _ = bound_fixture()
    node, source, prepared, runtime, good, bad, calls, _, _ = values
    original = deepcopy(source)
    def strict(candidate, stages, received_prepared, received_runtime, operation, deadline_sec):
        calls.append(deepcopy(candidate))
        assert received_prepared is prepared and deadline_sec == 123.
        passed = len(calls) >= 5
        received_runtime['contact_sequence_precheck'] = dict(success=passed, failed_stage='grasp',
            reason=bad.reason, plan_id=candidate.plan_id)
        return (good if passed else bad), {}
    node._final_contact_sequence_evaluation = strict
    selected = deepcopy(source)
    node._strict_check_final_near_field_plan(selected, prepared, runtime)
    assert len(calls) == 5 and source == original and selected.plan_id != source.plan_id
    audit = runtime['contact_height_window_search']
    assert audit['maximum_proposals'] == 4 and audit['deadline_extended'] is False
    assert audit['evaluated_proposal_indices'] == [0, 1, 2, 3]
    assert node._registered_sequence_geometry_gate.call_count == 4
    recipe = runtime['_selected_contact_height_recipe']
    assert recipe['proposal_index'] == 3
    assert recipe['parameters']['height_window_upper_interior_reserve_m'] == GRIPPER_CONTRACT_TOLERANCE_M
    assert recipe['parameters']['height_window_maximum_proposals'] == 4
    assert node._contact_height_recipe_matches(recipe, source, prepared)
    for expected_count in (6, 7):
        publication = deepcopy(source)
        node._strict_check_final_near_field_plan(publication, prepared, runtime)
        assert publication == selected and len(calls) == expected_count
        assert runtime['contact_height_window_preferred_revalidation']['evaluated_proposal_indices'] == [3]
        assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False
    assert node._registered_sequence_geometry_gate.call_count == 6
    monkeypatch.setattr(remote, 'GRIPPER_CONTRACT_TOLERANCE_M', GRIPPER_CONTRACT_TOLERANCE_M+.0001)
    assert not node._contact_height_recipe_matches(recipe, source, prepared)


def test_fourth_retry_cannot_extend_the_original_deadline(monkeypatch, frozen_height):
    from test_selected_contact_height_recipe import bound_fixture
    from test_contact_height_window_proposals import remote
    import test_contact_height_window_proposals as original_test
    monkeypatch.setattr(original_test, 'robust_geometry', lambda: deepcopy(frozen_height[0]))
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, bad, calls, clock, _ = values
    original = deepcopy(source)
    def strict(candidate, stages, received_prepared, received_runtime, operation, deadline_sec):
        calls.append(deepcopy(candidate))
        received_runtime['contact_sequence_precheck'] = dict(success=False, failed_stage='grasp',
            reason=bad.reason, plan_id=candidate.plan_id)
        if len(calls) == 4:
            clock[0] = deadline_sec
        return bad, {}
    node._final_contact_sequence_evaluation = strict
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(deepcopy(source), prepared, runtime)
    assert len(calls) == 4 and source == original
    assert runtime['contact_height_window_search']['evaluated_proposal_indices'] == [0, 1, 2]
    assert runtime['contact_height_window_search']['deadline_extended'] is False


def test_superseded_request_stops_before_the_fourth_retry(monkeypatch, frozen_height):
    from test_selected_contact_height_recipe import bound_fixture
    from test_contact_height_window_proposals import remote
    import test_contact_height_window_proposals as original_test
    monkeypatch.setattr(original_test, 'robust_geometry', lambda: deepcopy(frozen_height[0]))
    values, _ = bound_fixture()
    node, source, prepared, runtime, _, bad, calls, _, _ = values
    generation = prepared.ticket.generation
    original = deepcopy(source)
    def require_current(ticket):
        if ticket.generation != generation:
            raise remote.StreamResultCancelled('new request superseded the frozen height search')
    node._require_stream_ticket_current = require_current
    def strict(candidate, stages, received_prepared, received_runtime, operation, deadline_sec):
        calls.append(deepcopy(candidate))
        received_runtime['contact_sequence_precheck'] = dict(success=False, failed_stage='grasp',
            reason=bad.reason, plan_id=candidate.plan_id)
        if len(calls) == 4:
            prepared.ticket.generation += 1
        return bad, {}
    node._final_contact_sequence_evaluation = strict
    with pytest.raises(remote.StreamResultCancelled):
        node._strict_check_final_near_field_plan(deepcopy(source), prepared, runtime)
    assert len(calls) == 4 and source == original
    assert '_selected_contact_height_recipe' not in runtime
