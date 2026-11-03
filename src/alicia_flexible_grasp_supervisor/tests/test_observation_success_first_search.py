"""Unknown observation discovery is exhaustive only within original authority.

The strict service is controlled here; no synthetic success claims actual IK,
collision or hardware validation. Existing generation/source checks run live.
"""
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest

from test_remote_grasp6d_streaming import (
    remote_node as remote, streaming_node, MutableClock, snapshot)
from alicia_flexible_grasp.grasp.grasp6d_pipeline import MoveItResult, _validated_moveit_result


@contextmanager
def search(count=37, *, passing=(), call_cost=.84, failure='OBSERVATION_PLANNING_TIMEOUT',
           tilt=True):
    source_clock, elapsed = MutableClock(50.), MutableClock(0.)
    node = streaming_node(clock=source_clock, start_worker=False)
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    node._unknown_observation_planning_active = lambda: True
    node._observation_tilt_clock = elapsed
    node._observation_comparison_clock = elapsed
    node._order_observation_checks = lambda rows: rows
    node.observation_following_guard_required = True
    node._observation_following_support_evaluation = lambda p, r, t: (r, {'ok': True})
    node.start_streaming()
    node.submit_stream_snapshot(snapshot(49.8))
    variants = []
    for index in range(count):
        pose = remote.PoseStamped()
        pose.pose.position.x = index
        variants.append(dict(sequence=NS(pregrasp=pose), camera_roll_offset_deg=0.,
            camera_tilt_offset_deg=10. if tilt else 0., preference_index=index))
    runtime = dict(prepared=NS(ticket=node._stream_worker_ticket), soft_evidence={},
        grasp_pose=variants[0]['sequence'].pregrasp, observation_sequence=variants[0]['sequence'],
        observation_variants=variants)
    node._stable_variant_runtime = {(3, 1): runtime}
    checked = []
    def strict(pose, observation=False):
        assert observation
        index = int(pose.pose.position.x)
        checked.append(index)
        elapsed.value += call_cost
        source_clock.value += call_cost
        if index in passing:
            result = MoveItResult(True, .2, .1,
                'observation_screened_duration_sec=%s' % (100-index), True, True, True, True)
        else:
            result = MoveItResult(False, .2, .1, failure + ': controlled rejection',
                failure_code=failure)
        return result, {}, ''
    node._strict_moveit_evaluation = strict
    try:
        yield NS(node=node, runtime=runtime, checked=checked, clock=source_clock, elapsed=elapsed,
            variants=variants, candidate=NS(track_id=3, variant_index=1))
    finally:
        node.shutdown_streaming_worker()


@pytest.mark.parametrize('winner', [4, 6, 36])
def test_discovery_reaches_fifth_seventh_and_last_original_view(winner):
    with search(passing=(winner,)) as c:
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert result.reachable
        assert c.checked[:winner+1] == list(range(winner+1))
        assert c.runtime['observation_sequence'] is c.variants[winner]['sequence']
        audit = c.runtime['observation_orientation_search']
        assert audit['success_first_search'] and audit['optimization_budget_starts_after_incumbent']
        assert c.node._observation_tilt_search_budget['discovery_checked_count'] == winner+1


def test_all_planning_timeouts_exhaust_original_set_but_do_not_prove_unreachable():
    with search() as c:
        result = c.node._check_moveit_stable_candidate(c.candidate)
        audit = c.runtime['observation_orientation_search']
        assert c.checked == list(range(37))
        assert result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert (audit['checked_variant_count'], audit['unchecked_variant_count'],
                audit['pending_variant_count']) == (37, 0, 37)
        assert not audit['search_complete']
        assert c.runtime['_observation_qualified_incumbent_ticket'] is None
        assert c.clock.value == pytest.approx(50.+37*.84)


def test_all_explicit_rejections_can_conclude_original_set_is_unreachable():
    with search(failure='MOVEIT_UNREACHABLE') as c:
        result = c.node._check_moveit_stable_candidate(c.candidate)
        audit = c.runtime['observation_orientation_search']
        assert c.checked == list(range(37))
        assert result.failure_code == 'MOVEIT_UNREACHABLE'
        assert audit['search_complete'] and audit['pending_variant_count'] == 0


def test_one_pending_among_explicit_rejections_is_not_hidden_by_last_rejection():
    with search(count=8, failure='MOVEIT_UNREACHABLE') as c:
        original = c.node._strict_moveit_evaluation
        def strict(*args, **kwargs):
            result, metrics, fallback = original(*args, **kwargs)
            if len(c.checked) == 2:
                result = replace(result, failure_code='OBSERVATION_PLANNING_TIMEOUT')
            return result, metrics, fallback
        c.node._strict_moveit_evaluation = strict
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert len(c.checked) == 8
        assert result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert c.runtime['observation_orientation_search']['pending_variant_count'] == 1


def test_service_transport_timeout_stops_after_one_call():
    with search(failure='MOVEIT_TIMEOUT') as c:
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [0]
        assert result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        audit = c.runtime['observation_orientation_search']
        assert audit['stop_reason'] == 'MOVEIT_TIMEOUT'
        assert (audit['checked_variant_count'], audit['unchecked_variant_count']) == (1, 36)


@pytest.mark.parametrize('tilt', [True, False])
def test_source_reserve_stops_new_tilt_and_original_roll_without_a_call(tilt):
    with search(tilt=tilt) as c:
        c.clock.value = 139.8
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == []
        assert result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert c.runtime['observation_orientation_search']['stop_reason'] == 'OBSERVATION_SOURCE_RESERVE_REACHED'


@pytest.mark.parametrize('tilt', [True, False])
def test_source_expiring_during_successful_call_cannot_become_incumbent(tilt):
    with search(passing=(0,), tilt=tilt) as c:
        c.clock.value = 139.3
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [0]
        assert not result.reachable and result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert c.runtime['_observation_qualified_incumbent_ticket'] is None
        assert c.runtime['observation_orientation_search']['variant_results'][0]['result_authority_expired']


def test_expiring_source_revokes_earlier_incumbent_after_comparison_call():
    with search(passing=(0,)) as c:
        c.clock.value = 138.5
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [0, 1]
        assert not result.reachable and result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert c.runtime['_observation_qualified_incumbent_ticket'] is None


def test_last_source_check_revokes_result_before_activating_selected_variant():
    with search(count=1, passing=(0,)) as c:
        original = c.node._observation_search_prepared_current
        calls = []
        def guard(prepared):
            calls.append(prepared)
            if len(calls) == 3:
                c.clock.value = 139.8
            return original(prepared)
        c.node._observation_search_prepared_current = guard
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert len(calls) == 3
        assert not result.reachable and result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'
        assert c.runtime['_observation_qualified_incumbent_ticket'] is None


@pytest.mark.parametrize('invalidate', ['generation', 'target_epoch'])
def test_ticket_revocation_during_call_cannot_become_incumbent(invalidate):
    with search(passing=(0,)) as c:
        original = c.node._strict_moveit_evaluation
        def strict(*args, **kwargs):
            result = original(*args, **kwargs)
            if invalidate == 'generation':
                c.node._stream_generation += 1
            else:
                c.node.target_instance_epoch += 1
            return result
        c.node._strict_moveit_evaluation = strict
        with pytest.raises(remote.StreamResultCancelled):
            c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [0] and c.runtime['_observation_qualified_incumbent_ticket'] is None


def test_following_failure_is_not_an_incumbent_and_does_not_end_discovery():
    with search(count=8, passing=range(8)) as c:
        c.node._observation_following_support_evaluation = lambda p, r, t: (
            replace(r, reachable=False, failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID',
                collision_free=None, within_joint_limits=None, ik_valid=None, planning_success=None),
            {'ok': False})
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == list(range(8)) and not result.reachable
        assert c.runtime['_observation_qualified_incumbent_ticket'] is None


def test_comparison_after_late_incumbent_keeps_six_check_cap_and_selects_best():
    with search(count=30, passing=range(6,30), call_cost=.1) as c:
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert result.reachable and c.checked == list(range(13))
        assert c.runtime['observation_sequence'] is c.variants[12]['sequence']
        budget = c.node._observation_tilt_search_budget
        assert budget['discovery_checked_count'] == 7
        assert budget['checked_count'] - budget['comparison_start_count'] == 6


def test_information_comparison_still_stops_at_eight_seconds_after_late_incumbent():
    with search(count=15, passing=range(6,15), call_cost=4.) as c:
        c.node._information_driven_observation_comparison = lambda: True
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert result.reachable and c.checked == list(range(9))
        assert c.runtime['observation_sequence'] is c.variants[8]['sequence']
        assert c.runtime['observation_orientation_search']['comparison_budget_reached']
        assert c.elapsed.value == 36.


def test_source_guard_crossing_reserve_is_pending_even_inside_prepared_validation():
    with search() as c:
        def expired(_):
            raise remote.CandidateContractError('OBSERVATION_SOURCE_RESERVE_REACHED', 'controlled expiry')
        c.node._require_unknown_observation_prepared = expired
        result = c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [] and result.failure_code == 'OBSERVATION_SEARCH_INCOMPLETE'


def test_changed_view_configuration_during_call_retains_existing_rejection():
    with search(passing=(0,)) as c:
        calls = []
        def unchanged(_):
            calls.append(1)
            if len(calls) == 2:
                raise ValueError('controlled camera configuration changed')
        c.node._require_diagnostic_observation_view_source = unchanged
        with pytest.raises(ValueError, match='camera configuration changed'):
            c.node._check_moveit_stable_candidate(c.candidate)
        assert c.checked == [0] and c.runtime['_observation_qualified_incumbent_ticket'] is None


@pytest.mark.parametrize('code', ['OBSERVATION_PLANNING_TIMEOUT', 'OBSERVATION_SEARCH_INCOMPLETE'])
def test_pending_codes_only_validate_as_negative_without_hard_evidence(code):
    result = MoveItResult(False, 0., 0., code, failure_code=code)
    assert _validated_moveit_result(result) == result
    assert _validated_moveit_result(replace(result, reachable=True)) is None
    assert _validated_moveit_result(replace(result, evidence_code='STRICT_SERVICE_SUCCESS')) is None
    assert _validated_moveit_result(replace(result, collision_free=True)) is None
    assert _validated_moveit_result(replace(result, collision_free=False,
        within_joint_limits=True, ik_valid=True, planning_success=True)) is None
    assert _validated_moveit_result(replace(result, reachable=True, collision_free=True,
        within_joint_limits=True, ik_valid=True, planning_success=True)) is None


@pytest.mark.parametrize('explicit', [False, True])
def test_strict_transport_preserves_exact_observation_planner_timeout(monkeypatch, explicit):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    response = NS(success=False, message='OBSERVATION_PLANNING_TIMEOUT: TIMED_OUT (-6)')
    if explicit:
        response.failure_code = 'OBSERVATION_PLANNING_TIMEOUT'
        response.message = 'controlled planner timeout'
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', lambda *a: lambda *args: response)
    result, _, _ = node._strict_moveit_evaluation(remote.PoseStamped(), observation=True)
    assert result.failure_code == 'OBSERVATION_PLANNING_TIMEOUT' and not result.reachable
    assert _validated_moveit_result(result) == result
    contact, _, _ = node._strict_moveit_evaluation(remote.PoseStamped(), observation=False)
    assert contact.failure_code == 'MOVEIT_UNREACHABLE'


def test_timeout_words_without_gateway_marker_do_not_change_generic_failure(monkeypatch):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', lambda *a: lambda *args:
        NS(success=False, message='other rejection containing OBSERVATION_PLANNING_TIMEOUT:'))
    result, _, _ = node._strict_moveit_evaluation(remote.PoseStamped(), observation=True)
    assert result.failure_code == 'MOVEIT_UNREACHABLE'


@pytest.mark.parametrize('success, code, expected', [
    (False, -6, 'OBSERVATION_PLANNING_TIMEOUT'),
    (True, -6, 'MOVEIT_CHECK_ERROR'),
    (False, '-6', 'MOVEIT_CHECK_ERROR'),
    (False, -1, 'MOVEIT_UNREACHABLE'),
])
def test_actual_planner_gateway_bool_reason_reaches_remote_without_false_unreachability(
        monkeypatch, success, code, expected):
    from test_observation_branch_hint import fixture
    from test_moveit_planner_pose_feedback import JointPlan, make_pose
    from test_motion_gateway_controller_start import MotionGateway, motion_gateway_node
    parameters = {'/robot_description': 'offline-test-description',
                  '/robot_description_semantic': 'offline-test-semantic'}
    monkeypatch.setattr(remote.rospy, 'get_param', lambda name, default=None:
        parameters.get(name, default))
    planner, manipulator, _, _ = fixture()
    def plan():
        manipulator.plan_calls += 1
        trajectory = JointPlan([[0.] * 6, list(manipulator.new_goal)], duration_sec=30.)
        return success, trajectory, .8, NS(val=code)
    manipulator.plan = plan
    planner.strict_execution_max_joint_velocity_rad_s = .08
    planner.strict_execution_velocity_scaling = .3
    planner.strict_execution_acceleration_scaling = .2
    gateway = MotionGateway.__new__(MotionGateway)
    gateway.planner = old_planner = object()
    gateway._observation_planner = planner
    gateway._configure_observation_timing = lambda p: ('offline-timing',)
    gateway._configure_controller_reference_guard = lambda p: None
    for name in ('loginfo', 'logwarn'):
        monkeypatch.setattr(motion_gateway_node.rospy, name, lambda *a, **k: None)
    pose = make_pose()
    planner.expected_pose = deepcopy(pose)
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **k: None)
    def service(name, _):
        assert name == '/supervisor/check_observation_pose_strict'
        return lambda target, execute: gateway.handle_observation_pose_strict(
            NS(target=target, execute=execute))
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', service)
    result, _, _ = node._strict_moveit_evaluation(pose, observation=True)
    assert not result.reachable and result.failure_code == expected
    assert _validated_moveit_result(result) == result
    assert gateway.planner is old_planner
    assert manipulator.plan_calls == 1 and not manipulator.executed_plans
    assert planner._last_pose_plan is None and not planner._observation_branch_hints


@pytest.mark.parametrize('message', ['OBSERVATION_PLANNING_RESULT_INVALID:',
    'unrelated: move_to_pose exception: OBSERVATION_PLANNING_RESULT_INVALID:',
    'move_to_pose exception: other message mentioning OBSERVATION_PLANNING_RESULT_INVALID:'])
def test_invalid_planner_words_outside_exact_gateway_contract_are_not_reclassified(monkeypatch, message):
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    monkeypatch.setattr(remote.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(remote.rospy, 'ServiceProxy', lambda *a: lambda *args:
        NS(success=False, message=message))
    result, _, _ = node._strict_moveit_evaluation(remote.PoseStamped(), observation=True)
    assert result.failure_code == 'MOVEIT_UNREACHABLE'
