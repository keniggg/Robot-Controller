"""Offline integration of final compensation and same-source contact evidence."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_contact_execution import FIXTURE, plan_from_record
from test_endpoint_correction import sample
from test_pregrasp_gateway import make_gateway
from alicia_flexible_grasp.robot import pregrasp_gateway as module
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q


def gateway(monkeypatch, actual_contact=False):
    node, planner, _, state, calls, parameters = make_gateway(monkeypatch)
    plan, row = plan_from_record('gen18')
    ctx = node._pregrasp_context(plan)
    ctx.update(plan=plan, scene=NS(plan_id=plan.plan_id), pose_index=2,
               model=FIXTURE['robot_description'])
    ctx['config']['unknown_measured_endpoint_position_tolerance_m'] = .010
    node._pregrasp_context = lambda *_args: ctx
    fk = SerialUrdfFk(FIXTURE['robot_description'], ARM_NAMES)
    counts = [816, 1583, 2881, 2020, 1237, 2047] if actual_contact else row['measured_counts']
    state['actual'] = (np.asarray(counts)-2048)*Q
    state['sdk'] = state['actual'].copy() if actual_contact else (np.asarray(row['sdk_counts'])-2048)*Q
    def snapshot(*args, **kwargs):
        kwargs['context_validator']()
        result = sample(fk, state['sdk'], state['actual'], state['now'])
        result.update(accepted_gripper_opening_m=.04975,
                      opening_stamp_ns=result['accepted_stamp_ns'])
        return result
    node._endpoint_following_snapshot = snapshot
    return node, plan, ctx, state, calls


def test_final_compensation_does_not_accept_recorded_empty_close_without_step(monkeypatch):
    node, plan, _, _, calls = gateway(monkeypatch)
    response = node.handle_compensate_grasp_pose(NS(plan=plan, execute=False))
    assert response.success, response.message
    assert 'PREGRASP_PLANNED_ONLY' in response.message
    assert calls and calls[0][1] is False
    assert 'MEASURED_CONTACT_PATCH_MISS' in response.message


def test_hypothetical_valid_final_pose_needs_no_motion(monkeypatch):
    node, plan, _, _, calls = gateway(monkeypatch, actual_contact=True)
    response = node.handle_compensate_grasp_pose(NS(plan=plan, execute=True))
    assert response.success, response.message
    assert 'MEASURED_CONTACT_GEOMETRY_OK' in response.message
    assert not calls


def test_opening_must_be_from_the_same_accepted_packet(monkeypatch):
    node, plan, _, _, calls = gateway(monkeypatch)
    original = node._endpoint_following_snapshot
    def bad(*args, **kwargs):
        value = original(*args, **kwargs)
        value['opening_stamp_ns'] -= 1
        return value
    node._endpoint_following_snapshot = bad
    response = node.handle_compensate_grasp_pose(NS(plan=plan, execute=True))
    assert not response.success and 'OPENING_SOURCE_MISMATCH' in response.message
    assert not calls


def test_final_zero_step_revalidates_authority_after_evaluation(monkeypatch):
    node, plan, _, _, calls = gateway(monkeypatch, actual_contact=True)
    def validate(_):
        if node._validate_pregrasp_context.call_count >= 3:
            raise ValueError('EPOCH_CHANGED_DURING_EVALUATION')
    node._validate_pregrasp_context.side_effect = validate
    response = node.handle_compensate_grasp_pose(NS(plan=plan, execute=True))
    assert not response.success and 'EPOCH_CHANGED_DURING_EVALUATION' in response.message
    assert not calls


def test_final_zero_step_cannot_return_with_feedback_expired_during_evaluation(monkeypatch):
    node, plan, _, state, calls = gateway(monkeypatch, actual_contact=True)
    original = module.plan_contact_evaluator
    def slow_factory(*args):
        evaluate = original(*args)
        def slow(actual):
            result = evaluate(actual)
            state['now'] += .6
            return result
        return slow
    monkeypatch.setattr(module, 'plan_contact_evaluator', slow_factory)
    response = node.handle_compensate_grasp_pose(NS(plan=plan, execute=True))
    assert not response.success and 'FEEDBACK_EXPIRED_DURING_VALIDATION' in response.message
    assert not calls
