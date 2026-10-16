"""Explicit supplemental-scene transactions with the real offline gateway methods."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
import rospy
from geometry_msgs.msg import PoseStamped

from test_observe_return_gateway import fixture, capture, begin_return, run_return
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot import observe_return_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import (
    ObservationPathError, committed_plan_digest)


def initial_session(node, plan, state):
    context = node._observe_return_origin
    return dict(version=2, phase='observing', selection=deepcopy(context['selection']),
        session_id=context['session_id'], stamp_ns=str(int(state['now'] * 1e9)),
        deadline_ns='190000000000', target_epoch=7, target_track_id=plan.target_track_id,
        origin_plan_id=plan.plan_id, origin_snapshot_stamp_ns=str(plan.header.stamp.to_nsec()),
        return_verified_stamp_ns='0', max_observations=3, view_index=1,
        view_plan_id=plan.plan_id, view_source_stamp_ns=str(plan.header.stamp.to_nsec()),
        view_request_id='initial:' + plan.plan_id, previous_gain_sha256='0' * 64)


def proposal(plan, session, state, index=2):
    candidate = deepcopy(plan)
    state['now'] = 10. + index
    candidate.header.stamp = rospy.Time.from_sec(9. + index)
    candidate.object_geometry.header.stamp = candidate.header.stamp
    candidate.poses[0].position.x += index * .001
    candidate.plan_id = compute_plan_id(candidate)
    update = deepcopy(session)
    update.update(stamp_ns=str(int(state['now'] * 1e9)), view_index=index,
        view_plan_id=candidate.plan_id, view_source_stamp_ns=str(candidate.header.stamp.to_nsec()),
        view_request_id=str(index) * 64, previous_gain_sha256='a' * 64)
    return candidate, update


def deliver(node, session):
    node._observe_return_view_session_cb(NS(data=json.dumps(session)))


def admitted_origin(monkeypatch):
    node, planner, plan, state, params, calls = fixture(monkeypatch)
    result = capture(node, plan)
    assert result.success, result.message
    session = initial_session(node, plan, state)
    deliver(node, session)
    assert node._observe_return_origin['view_session']['view_index'] == 1
    return node, planner, plan, state, params, calls, session


def admit(node, plan):
    node._pregrasp_task_stage = module.GraspStages.PLAN_PREGRASP
    node._compensation_task_label = module.ADMIT_VIEW_LABEL
    return node.handle_admit_observe_return_view(NS(plan=plan, execute=True))


@pytest.mark.parametrize('stage,label', [
    (module.GraspStages.MOVE_PREGRASP, module.ADMIT_VIEW_LABEL),
    (module.GraspStages.PLAN_PREGRASP, 'planning supplemental observation 2'),
    (module.GraspStages.FAILED, module.ADMIT_VIEW_LABEL),
])
def test_metadata_admission_requires_exact_planning_stage_before_measurement_ack(monkeypatch, stage, label):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    deliver(node, update)
    node._pregrasp_task_stage = stage
    node._compensation_task_label = label
    result = node.handle_admit_observe_return_view(NS(plan=candidate, execute=True))
    assert not result.success and 'REQUEST_OR_STAGE_UNAVAILABLE' in result.message
    assert 'active_view_scene' not in node._observe_return_origin
    assert state['moves'] == 0 and not calls


def test_topic_is_not_authority_but_explicit_admission_switches_scene_and_retains_origin(monkeypatch):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    origin = node._observe_return_origin
    old_scene = origin['scene']
    old_joints, old_goal, old_deadline = origin['origin_joints'], origin['goal_digest'], origin['deadline']
    candidate, update = proposal(plan, session, state)
    candidate.header.seq = 87  # ROS transport counters do not alter scene identity.
    deliver(node, update)
    assert node._observation_contexts.active_scene is old_scene
    node._observation_contexts.ingest(candidate)
    assert node._observation_contexts.active_scene is old_scene
    with pytest.raises(ObservationPathError, match='active frozen source'):
        node._observation_contexts.capture(PoseStamped(header=candidate.header, pose=candidate.poses[0]), state['now'], 20.)
    result = admit(node, candidate)
    proof = json.loads(result.message)
    assert result.success, proof
    assert proof['code'] == 'OBSERVE_RETURN_VIEW_ADMITTED'
    assert proof['physical_motion_submitted'] is False
    assert proof['plan_digest'] == committed_plan_digest(candidate)
    assert proof['origin_plan_digest'] == old_scene.digest
    assert proof['previous_view_plan_id'] == old_scene.plan_id
    assert proof['view_index'] == 2 and proof['epoch_ns'] == origin['epoch']
    assert proof['origin_goal_sha256'] == old_goal
    assert origin['scene'] is old_scene and origin['plan'].plan_id == plan.plan_id
    assert origin['origin_joints'] == old_joints and origin['deadline'] == old_deadline
    assert origin['active_view_scene'] is node._observation_contexts.active_scene
    assert origin['active_view_scene'].plan_id == candidate.plan_id
    scene, token = node._observation_contexts.capture(
        PoseStamped(header=candidate.header, pose=candidate.poses[0]), state['now'], 20.)
    node._observation_contexts.validate_capture(scene, token)
    with pytest.raises(ObservationPathError, match='revoked'):
        node._observation_contexts.validate_capture(old_scene, token)
    assert state['moves'] == 0 and not calls


def test_cannot_capture_new_origin_from_admitted_view(monkeypatch):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    original = node._observe_return_origin
    candidate, update = proposal(plan, session, state)
    deliver(node, update)
    assert admit(node, candidate).success
    node._compensation_task_label = module.CAPTURE_LABEL
    node._observation_task_state_stamp_ns = int(state['now'] * 1e9)
    result = capture(node, candidate)
    assert not result.success and 'ORIGIN_ALREADY_BOUND' in result.message
    assert node._observe_return_origin is original
    assert not calls and state['moves'] == 0


def test_two_supplements_return_to_original_measured_goal_and_revalidate_all_scenes(monkeypatch):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    deliver(node, update)
    assert admit(node, candidate).success
    third, final_session = proposal(candidate, update, state, index=3)
    deliver(node, final_session)
    assert admit(node, third).success
    begin_return(node, state)
    node._pregrasp_task_stage = module.GraspStages.MOVE_PREGRASP
    result = run_return(node, plan)
    assert result.success, result.message
    assert calls == [dict(goal=[0.] * 6, execute=True, start=[.15] * 6)]
    assert state['moves'] == 1
    for validator in (node._validate_frozen_observation_path,
                      node._revalidate_observation_contract_submission):
        assert [call.args[0][0].plan_id for call in validator.call_args_list] == [
            plan.plan_id, candidate.plan_id, third.plan_id]


@pytest.mark.parametrize('field,value', [
    ('view_index', 3), ('view_index', True), ('max_observations', 4),
    ('target_epoch', 8), ('target_track_id', 'other'), ('session_id', 'another'),
    ('origin_plan_id', 'different'), ('origin_snapshot_stamp_ns', '8000000000'),
    ('deadline_ns', '191000000000'), ('deadline_ns', '11000000000'),
    ('view_source_stamp_ns', '9000000000'), ('previous_gain_sha256', '0' * 64),
    ('view_request_id', 'bad request'), ('return_verified_stamp_ns', '1'),
    ('contact_deadline_ns', '1'), ('stamp_ns', '13000000000'),
])
def test_invalid_session_proposal_does_not_authorize_scene(monkeypatch, field, value):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    old = node._observation_contexts.active_scene
    candidate, update = proposal(plan, session, state)
    update[field] = value
    deliver(node, update)
    result = admit(node, candidate)
    assert not result.success, result.message
    assert node._observation_contexts.active_scene is old
    assert state['moves'] == 0 and not calls


@pytest.mark.parametrize('budget', [4, 5])
def test_gateway_configuration_cannot_expand_three_view_limit(monkeypatch, budget):
    node, _, plan, state, params, _ = fixture(monkeypatch)
    params['/grasp']['unknown_observe_return_max_observations'] = budget
    assert capture(node, plan).success
    deliver(node, initial_session(node, plan, state))
    assert 'view_session' not in node._observe_return_origin


def test_supplement_cannot_initialize_missing_original_session(monkeypatch):
    node, _, plan, state, _, _ = fixture(monkeypatch)
    assert capture(node, plan).success
    candidate, update = proposal(plan, initial_session(node, plan, state), state)
    deliver(node, update)
    assert 'view_session' not in node._observe_return_origin
    assert not admit(node, candidate).success


@pytest.mark.parametrize('change', [
    lambda n,s,p:setattr(n, '_manual_control_active', lambda: True),
    lambda n,s,p:setattr(n, '_control_reference_epoch_ns', 2_000_000_000),
    lambda n,s,p:n._observation_contexts.clear(),
    lambda n,s,p:p['/grasp_mode/selection'].update(generation=2),
    lambda n,s,p:setattr(n, '_fresh_actuation_status', lambda:'UNCONFIRMED'),
    lambda n,s,p:s.update(now=191.),
    lambda n,s,p:s.update(now=35.),  # scene source past configured validity
])
def test_changed_authority_or_stale_source_cannot_admit(monkeypatch, change):
    node, _, plan, state, params, calls, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    deliver(node, update)
    origin = node._observe_return_origin
    change(node, state, params)
    result = admit(node, candidate)
    assert not result.success
    assert 'active_view_scene' not in origin
    assert not calls and state['moves'] == 0


def test_late_topic_and_stage_delivery_are_bounded_and_still_fully_checked(monkeypatch):
    node, _, plan, state, _, _, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    def delayed(seconds):
        state['now'] += seconds
        deliver(node, update)
        node._pregrasp_task_stage = module.GraspStages.PLAN_PREGRASP
        node._compensation_task_label = module.ADMIT_VIEW_LABEL
    monkeypatch.setattr(module.time, 'sleep', delayed)
    result = node.handle_admit_observe_return_view(NS(plan=candidate, execute=True))
    assert result.success, result.message
    assert 12. < state['now'] < 13.


def test_revocation_during_session_wait_prevents_admission(monkeypatch):
    node, _, plan, state, _, _, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    def delayed(seconds):
        state['now'] += seconds
        deliver(node, update)
        node._pregrasp_task_stage = module.GraspStages.PLAN_PREGRASP
        node._compensation_task_label = module.ADMIT_VIEW_LABEL
        node._observation_contexts.clear()
    monkeypatch.setattr(module.time, 'sleep', delayed)
    result = node.handle_admit_observe_return_view(NS(plan=candidate, execute=True))
    assert not result.success and 'revoked' in result.message
    assert 'active_view_scene' not in node._observe_return_origin


def test_session_deadline_is_not_extended_by_return(monkeypatch):
    node, _, plan, state, _, calls, session = admitted_origin(monkeypatch)
    candidate, update = proposal(plan, session, state)
    deliver(node, update)
    assert admit(node, candidate).success
    begin_return(node, state)
    node._pregrasp_task_stage = module.GraspStages.MOVE_PREGRASP
    state['now'] = 189.
    result = run_return(node, plan)
    assert not result.success and 'PATH_TIME_BUDGET' in result.message
    assert state['moves'] == 0
    assert node._observe_return_origin['return_deadline'] == 190.


def path(contract):
    return dict(path_audit=dict(execution_tracking_contract=contract))


def contract(bound):
    return dict(policy='fixture', joint_names=['Joint%d' % i for i in range(1, 7)],
        path_position_tolerance_rad=[bound] * 6, joint_error_bounds_rad=[bound + .01] * 6,
        stop_trajectory_duration_sec=.2)


def test_combined_return_selects_strictest_proven_contract():
    paths = [path(contract(.09)), path(contract(.035)), path(contract(.06))]
    assert module.ObserveReturnGateway._observe_return_tightest_path(paths) is paths[1]['path_audit']


@pytest.mark.parametrize('change', [
    lambda c:c.update(stop_trajectory_duration_sec=.3),
    lambda c:c.update(path_position_tolerance_rad=[.09, .02] * 3),
    lambda c:c.update(joint_error_bounds_rad=[float('nan')] * 6),
])
def test_incompatible_scene_contracts_reject_instead_of_widening(change):
    first, second = contract(.05), contract(.035)
    change(second)
    with pytest.raises(ObservationPathError):
        module.ObserveReturnGateway._observe_return_tightest_path([path(first), path(second)])
