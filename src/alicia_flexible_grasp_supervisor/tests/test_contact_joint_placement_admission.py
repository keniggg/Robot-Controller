"""Joint placement changes candidate generation, never final admission."""
from copy import deepcopy
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
from test_contact_height_window_proposals import fixture, remote


def commit_fixture_header(plan):
    # The legacy height fixture updates the plan stamp without the geometry
    # stamp. Real bridge tests must reach the owner callback with a valid
    # committed scene, rather than being rejected during scene construction.
    plan.object_geometry.header = deepcopy(plan.header)
    plan.plan_id = remote.compute_plan_id(plan)
    scene = FrozenObservationScene.from_contact_plan(plan)
    assert scene.plan_id == plan.plan_id
    assert scene.source_ns == plan.header.stamp.to_nsec()


def bound_fixture(*, validated=True, strict_success=True, fallback_allowed=True):
    values = fixture()
    node, plan, prepared, runtime, good, bad, calls, clock, geometry = values

    def strict(candidate, stages, current, state, operation, deadline_sec):
        calls.append((deepcopy(candidate), operation, deadline_sec))
        passed = strict_success and operation == 'final_registered_height_window_strict_sequence'
        failure = dict(success=passed, failed_stage='grasp', reason=bad.reason,
                       plan_id=candidate.plan_id, contact_terminal_failure={'schema_version': 1})
        state['contact_sequence_precheck'] = failure
        state.pop('validated_contact_precheck_failure', None)
        state.pop('validated_contact_precheck_plan_id', None)
        if validated:
            state['validated_contact_precheck_failure'] = deepcopy(failure)
            state['validated_contact_precheck_plan_id'] = candidate.plan_id
        return (good if passed else bad), {}

    def joint(candidate, failure, frozen, parameters, deadline_check):
        assert deadline_check() is True
        normal = np.asarray(frozen['support_normal_base'])
        tangent = np.array([.0003, .0001, 0.])
        tangent -= normal * float(normal @ tangent)
        displacement = tangent + .0001 * normal
        proposed = frozen['planned_T_base_tool0'].copy()
        proposed[:3, 3] += displacement
        return proposed, dict(proposed=True, fallback_allowed=fallback_allowed,
            normal_translation_m=float(normal @ displacement),
            translation_base_m=displacement.tolist(),
            policy='explicit_joint_test_provider', motion_authorized=False)

    node._final_contact_sequence_evaluation = strict
    node._contact_joint_window_proposal = mock.Mock(side_effect=joint)
    node._contact_lateral_window_proposal = mock.Mock(
        side_effect=AssertionError('joint hint already occupies this slot'))
    return values


def test_joint_translation_rehashes_all_four_poses_and_reruns_original_admission():
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    before = deepcopy(plan)
    _, result, _ = node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert result.reachable and len(calls) == 2
    trial = runtime['contact_height_window_search']['proposals'][0]
    displacement = np.asarray(trial['placement']['translation_base_m'])
    assert trial['placement']['normal_translation_m'] != 0.
    assert trial['accepted'] and trial['geometry_gate']['ok']
    assert runtime['final_contact_height_margin']['ok']
    assert plan.object_geometry == before.object_geometry
    assert plan.plan_id == remote.compute_plan_id(plan) != before.plan_id
    for old, new in zip(before.poses, plan.poses):
        assert old.orientation == new.orientation
        np.testing.assert_allclose(
            [getattr(new.position, k) - getattr(old.position, k) for k in 'xyz'],
            displacement, atol=1e-15)
    assert plan.refinement_translation_m == pytest.approx(
        before.refinement_translation_m + np.linalg.norm(displacement))
    assert all(call[2] == 123. for call in calls)
    node._contact_lateral_window_proposal.assert_not_called()


@pytest.mark.parametrize('mismatch', ['missing', 'plan_id', 'evidence'])
def test_unvalidated_service_failure_cannot_reach_any_new_generator(mismatch):
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    before = deepcopy(plan)
    original = node._final_contact_sequence_evaluation

    def strict(*args, **kwargs):
        result = original(*args, **kwargs)
        state = args[3]
        if mismatch == 'missing':
            state.pop('validated_contact_precheck_failure')
        elif mismatch == 'plan_id':
            state['validated_contact_precheck_plan_id'] = 'different_plan'
        else:
            state['validated_contact_precheck_failure']['reason'] = 'different response'
        return result

    node._final_contact_sequence_evaluation = strict
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert plan == before and len(calls) == 1
    node._contact_joint_window_proposal.assert_not_called()
    node._contact_lateral_window_proposal.assert_not_called()
    node._registered_sequence_geometry_gate.assert_not_called()


def test_invalid_joint_source_does_not_fall_through_to_height_candidates():
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    before = deepcopy(plan)
    node._contact_joint_window_proposal.side_effect = None
    node._contact_joint_window_proposal.return_value = (
        None, dict(proposed=False, fallback_allowed=False, reason='raw following invalid'))
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert plan == before and len(calls) == 1
    node._registered_sequence_geometry_gate.assert_not_called()
    node._contact_lateral_window_proposal.assert_not_called()
    assert runtime['contact_height_window_search']['proposals'] == []


def test_valid_unavailable_optimizer_keeps_original_height_search():
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    node._contact_joint_window_proposal.side_effect = None
    node._contact_joint_window_proposal.return_value = (
        None, dict(proposed=False, fallback_allowed=True, reason='optimizer unavailable'))
    node._contact_lateral_window_proposal.side_effect = None
    node._contact_lateral_window_proposal.return_value = (None, {'proposed': False})
    _, result, _ = node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert result.reachable and len(calls) == 2
    node._contact_lateral_window_proposal.assert_called_once()
    trial = runtime['contact_height_window_search']['proposals'][0]
    assert trial['accepted'] and 'translation_base_m' not in trial['placement']


def test_selected_joint_recipe_is_rebuilt_and_completely_rechecked():
    node, source, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    prepared.ticket = NS(request_id=40, generation=9, target_epoch=1)
    first, second = deepcopy(source), deepcopy(source)
    node._strict_check_final_near_field_plan(first, prepared, runtime)
    node._strict_check_final_near_field_plan(second, prepared, runtime)
    assert first == second and first != source
    assert len(calls) == 3 and calls[1][0] == calls[2][0] == first
    recipe = runtime['_selected_contact_height_recipe']
    assert recipe['proposal_index'] == -1
    assert recipe['normal_translation_m'] != 0.
    np.testing.assert_array_equal(recipe['translation_base_m'],
        runtime['contact_height_window_preferred_revalidation']['joint_placement']['translation_base_m'])
    assert runtime['contact_height_window_preferred_revalidation']['strict_result_reused'] is False


def test_strict_failures_preserve_four_slots_and_original_target():
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture(strict_success=False)
    before = deepcopy(plan)
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
    trials = runtime['contact_height_window_search']['proposals']
    assert len(trials) == 4 and len(calls) == 5
    assert plan == before and not any(trial['accepted'] for trial in trials)
    assert all(call[2] == 123. for call in calls)


def test_phase_expiry_during_proposal_cannot_release_a_new_request():
    node, plan, prepared, runtime, _, _, calls, clock, _ = bound_fixture()
    before = deepcopy(plan)

    def expired(candidate, failure, geometry, parameters, deadline_check):
        clock[0] = 124.
        assert deadline_check() is False
        return None, dict(proposed=False, fallback_allowed=False, reason='phase expired')

    node._contact_joint_window_proposal.side_effect = expired
    with pytest.raises(remote.CandidateContractError):
        node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert plan == before and len(calls) == 1
    node._registered_sequence_geometry_gate.assert_not_called()


def test_nonterminal_failure_keeps_legacy_lateral_and_height_search():
    node, plan, prepared, runtime, _, _, calls, _, _ = fixture()
    _, result, _ = node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert result.reachable and len(calls) == 2
    assert runtime['contact_height_window_search']['joint_placement']['fallback_allowed'] is True


@pytest.mark.parametrize('kind', ['cancelled', 'deadline'])
def test_real_joint_helper_preserves_owner_callback_exception_through_bridge(kind):
    node, plan, _, _, _, _, _, _, _ = fixture()
    commit_fixture_header(plan)
    parameters = node._contact_height_recipe_parameters()
    frozen = remote.frozen_contact_geometry(plan, parameters['config'], parameters['opening'])
    error = (remote.StreamResultCancelled('test owner revoked') if kind == 'cancelled' else
             remote.CandidateContractError('MOVEIT_TIMEOUT', 'test original deadline expired'))

    def revoked():
        raise error

    # The original owner check runs before any optimizer/model access. A real
    # callback exception must not be reclassified as a new candidate failure.
    with mock.patch.object(remote.rospy, 'get_param', return_value='<robot name="unused"/>'):
        with pytest.raises(type(error)) as caught:
            node._contact_joint_window_proposal(plan,
                {'contact_terminal_failure': {'schema_version': 1}}, frozen, parameters, revoked)
    assert caught.value is error


@pytest.mark.parametrize('kind', ['cancelled', 'deadline'])
def test_real_joint_helper_owner_exception_survives_complete_retry_chain(kind):
    node, plan, prepared, runtime, _, _, calls, _, _ = bound_fixture()
    commit_fixture_header(plan)
    before = deepcopy(plan)
    node._contact_joint_window_proposal = remote.RemoteGrasp6DNode._contact_joint_window_proposal.__get__(node)
    error = (remote.StreamResultCancelled('test owner revoked during hint') if kind == 'cancelled' else
             remote.CandidateContractError('MOVEIT_TIMEOUT', 'test phase revoked during hint'))
    node._require_current_contact_precheck_phase = mock.Mock(side_effect=error)
    with mock.patch.object(remote.rospy, 'get_param', return_value='<robot name="unused"/>'):
        with pytest.raises(type(error)) as caught:
            node._strict_check_final_near_field_plan(plan, prepared, runtime)
    assert caught.value is error
    assert plan == before and len(calls) == 1
    node._registered_sequence_geometry_gate.assert_not_called()
    node._contact_lateral_window_proposal.assert_not_called()
