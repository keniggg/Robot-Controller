"""Real preference/approach callbacks on the labeled GEN11 numeric proxy.

Transport, context authority and the controller are inert test fixtures. This
proves callback arithmetic/hand-off only, not path collision or real hardware.
The nominal IK is reconstructed; it is not GEN11's unavailable MoveIt endpoint.
"""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.held_pregrasp_selection import select_held_pregrasp_endpoint
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q
from test_endpoint_correction import sample
from test_held_approach_gateway import APPROACH_CONFIG
from test_held_guide_joint_neighborhood import recorded
from test_pregrasp_feedforward import held_gateway_case


def counterexample_gateway(monkeypatch, response=True):
    data, fk = recorded()
    command, selected = select_held_pregrasp_endpoint(fk, data['source_sample'],
        *data['fixed_poses'], data['reconstructed_nominal_rad'],
        position_tolerance_m=.010, orientation_tolerance_rad=math.radians(5))
    values = held_gateway_case(monkeypatch, response)
    node, planner, plan, state, calls, params, context = values
    bias = np.asarray(data['source_sample']['sdk_positions_rad']) - data['source_sample']['accepted_positions_rad']
    state['sdk'] = command.copy()
    state['actual'] = command-bias
    for pose, row in zip(plan.poses[:2], data['fixed_poses']):
        pose.position.x, pose.position.y, pose.position.z = row[:3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = row[3:]
    plan.plan_id = compute_plan_id(plan)
    context['scene'].plan_id = plan.plan_id
    context['model'] = (Path(__file__).resolve().parents[3]/data['model_path']).read_text()
    context['config'].update(APPROACH_CONFIG, unknown_pregrasp_position_preference_m=.006)
    params['/grasp'] = deepcopy(context['config'])
    def capture_context(_plan, index=0):
        context['pose_index'] = index
        return context
    node._pregrasp_context = capture_context
    captured = []
    def snapshot(*args, **kwargs):
        kwargs['context_validator']()
        state['now'] += 1. if kwargs.get('after_ns') else .1
        current = sample(fk, state['sdk'], state['actual'], state['now'])
        captured.append(deepcopy(current))
        return current
    node._endpoint_following_snapshot = snapshot
    node._require_live_observation_task = mock.Mock()
    node._manual_control_active = lambda: False
    return values, selected, captured


def test_real_preference_noop_then_shared_approach_use_same_quantized_guide(monkeypatch):
    values, selected, captured = counterexample_gateway(monkeypatch)
    node, planner, plan, state, calls, params, context = values
    fixed = deepcopy(plan); begin_sdk = state['sdk'].copy(); begin_actual = state['actual'].copy()
    attempts = []
    original = PregraspFeedforward.propose_held_axes
    def observed(self):
        attempts.append(tuple(self.contract.tolerances))
        return original(self)
    monkeypatch.setattr(PregraspFeedforward, 'propose_held_axes', observed)
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    audit = json.loads(result.message)
    assert result.success, audit
    assert attempts == [(.010, math.radians(5))]
    assert audit['precision_preference_outcome'] == 'finite_guide_no_qualified_improvement'
    assert audit['no_motion_submitted'] and audit['steps_completed'] == 0
    assert audit['initial_accepted_stamp_ns'] < audit['accepted_stamp_ns']
    assert audit['initial_sdk_stamp_ns'] < audit['sdk_stamp_ns']
    assert len(captured) >= 2
    assert not calls and state['moves'] == 0
    assert not any(k.startswith('/supervisor/pregrasp_compensation_episodes/') for k in params)
    np.testing.assert_array_equal(state['sdk'], begin_sdk)
    np.testing.assert_array_equal(state['actual'], begin_actual)
    node._ensure_trajectory_controllers_started.assert_not_called()
    node._pregrasp_task_stage = GraspStages.APPROACH_TARGET
    node._compensation_task_label = 'planning bounded 6D approach'
    result = node.handle_compensate_approach(NS(plan=plan, execute=True))
    audit = json.loads(result.message)
    assert result.success, audit
    assert audit['execution_policy'] == 'held_axes_approach'
    assert audit['proposed_delta_counts'] == selected['predicted_approach']['proposed_delta_counts']
    assert audit['position_error_m'] <= .006
    assert audit['orientation_error_rad'] <= math.radians(5)
    assert audit['proposed_delta_counts'][1] == audit['proposed_delta_counts'][4] == 0
    assert max(abs(np.asarray(audit['proposed_delta_counts']))) <= 32
    assert state['moves'] == len(calls) == 1
    assert audit['accepted_stamp_ns'] > captured[1]['accepted_stamp_ns']
    assert not audit['real_grasp_success'] and plan == fixed
    planner._plan_cartesian_from_start_state.assert_not_called()
    assert not node.handle_compensate_approach(NS(plan=plan, execute=True)).success
    assert state['moves'] == len(calls) == 1


@pytest.mark.parametrize('fault', ['stale', 'epoch', 'moving', 'owner'])
def test_counterexample_preference_noop_requires_fresh_still_owned_feedback(monkeypatch, fault):
    values, _, _ = counterexample_gateway(monkeypatch)
    node, planner, plan, state, calls, params, context = values
    real_snapshot = node._endpoint_following_snapshot
    count = [0]
    def invalidated(*args, **kwargs):
        current = real_snapshot(*args, **kwargs); count[0] += 1
        if count[0] == 2:
            if fault == 'stale': current['accepted_stamp_ns'] -= 600_000_000
            elif fault == 'epoch': current['epoch_ns'] += 1
            elif fault == 'moving': current['accepted_positions_rad'][0] += 2*Q
            elif fault == 'owner': raise ValueError('test owner changed before preference no-op')
        return current
    node._endpoint_following_snapshot = invalidated
    result = node.handle_compensate_pregrasp(NS(plan=plan, execute=True))
    assert not result.success, result.message
    assert not calls and state['moves'] == 0
    assert not any(k.startswith('/supervisor/pregrasp_compensation_episodes/') for k in params)
    planner._plan_cartesian_from_start_state.assert_not_called()
