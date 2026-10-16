"""Bound repeated arithmetic while retaining actual gateway admission gates.

No ROS master/hardware. Numerical callbacks and planner transport are fixtures;
all ownership/context checks use the real gateway implementation.
"""
from copy import deepcopy
import json
from types import MethodType, SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_contact_pregrasp_gateway import case, captured_session
from alicia_flexible_grasp.robot import contact_pregrasp_gateway as module
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages


def guarded(monkeypatch):
    clock = [100.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    c = case(monkeypatch)
    origin = captured_session(c, monkeypatch)
    c.node._gui_direct_mode = False
    c.node._observation_task_state_stamp_ns = 0
    c.node._observation_task_sub = NS(get_num_connections=lambda: int(c.state.get('connected', True)))
    c.node._require_live_observation_task = MethodType(type(c.node)._require_live_observation_task, c.node)
    c.node._manual_control_active = lambda: (c.node._gui_direct_mode
        or c.params.get('/gui/joint_direct_mode', False) is True)
    context = c.node._pregrasp_context(deepcopy(c.plan), 0)
    context['geometry'] = deepcopy(c.params['/grasp_6d/remote/gripper_geometry'])
    context['gripper_config'] = deepcopy(c.params['/gripper'])
    c.node._bind_contact_pregrasp_session(context)
    audit = {}
    full = Mock(wraps=c.node._validate_contact_pregrasp_context)
    c.node._validate_contact_pregrasp_context = full
    guard = c.node._contact_pregrasp_compute_guard(context, audit)
    return NS(c=c, clock=clock, context=context, origin=origin, audit=audit, full=full, guard=guard)


def test_many_derivative_checks_do_not_repeat_full_parameter_validation(monkeypatch):
    g = guarded(monkeypatch)
    original_deadline = g.context['deadline']
    for _ in range(2000):
        assert g.guard() > 0
    assert g.full.call_count == 1
    assert g.audit['compute_guard']['local_checks'] == 2001
    g.clock[0] += .10001
    g.guard()
    assert g.full.call_count == 2
    assert g.context['deadline'] == original_deadline


@pytest.mark.parametrize('fault', ['manual_topic', 'epoch', 'revocation', 'inactive', 'blocked',
    'latch', 'disconnected', 'stage', 'actuation', 'origin', 'session', 'identity',
    'deadline', 'session_deadline', 'shutdown'])
def test_local_revocation_interrupts_every_compute_step_before_next_full_poll(monkeypatch, fault):
    g = guarded(monkeypatch); g.guard()
    c = g.c
    if fault == 'manual_topic': c.node._gui_direct_mode = True
    elif fault == 'epoch': c.node._control_reference_epoch_ns += 1
    elif fault == 'revocation': c.node._observation_contexts.clear()
    elif fault == 'inactive': c.node._observation_contexts.set_active(False)
    elif fault == 'blocked': c.node._observation_contexts.blocked = True
    elif fault == 'latch': c.node._observation_task_requires_inactive = True
    elif fault == 'disconnected':
        c.state['connected'] = False
        c.node._contact_frozen_notifications = {'old': {'notice': True}}
        c.node._contact_frozen_admissions = {'old': {'admitted': True}}
    elif fault == 'stage': c.node._pregrasp_task_stage = GraspStages.APPROACH_TARGET
    elif fault == 'actuation': c.node._fresh_actuation_status = lambda: ''
    elif fault == 'origin': c.node._observe_return_origin = deepcopy(g.origin)
    elif fault == 'session': g.origin['view_session']['view_index'] += 1
    elif fault == 'identity': g.origin['identity'] = 'changed'
    elif fault == 'deadline': g.clock[0] = g.context['deadline']
    elif fault == 'session_deadline': c.state['now'] = g.origin['view_session']['deadline_ns'] * 1e-9
    else: monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: True)
    with pytest.raises(ObservationPathError): g.guard()
    assert g.full.call_count == 1
    assert not c.calls
    if fault == 'disconnected':
        assert c.node._observation_task_requires_inactive is True
        assert c.node._contact_pregrasp_receipts == {}
        assert c.node._contact_frozen_notifications == {}
        assert c.node._contact_frozen_admissions == {}


@pytest.mark.parametrize('fault', ['manual_param', 'config', 'model', 'selection', 'geometry', 'plan'])
def test_parameter_or_content_changes_are_checked_within_original_computation_poll(monkeypatch, fault):
    g = guarded(monkeypatch); g.guard()
    c = g.c
    if fault == 'manual_param': c.params['/gui/joint_direct_mode'] = True
    elif fault == 'config': c.params['/grasp']['measured_endpoint_check_enabled'] = False
    elif fault == 'model': c.params['/robot_description'] += ' '
    elif fault == 'selection': c.params['/grasp_mode/selection']['generation'] += 1
    elif fault == 'geometry': c.params['/grasp_6d/remote/gripper_geometry']['support_clearance_m'] = .01
    else: g.context['plan'].poses[0].position.x += .01
    # Pure arithmetic can finish a sub-100-ms chunk, never submit a motion.
    g.clock[0] += .05
    assert g.guard() > 0
    g.clock[0] += .05001
    with pytest.raises(ObservationPathError): g.guard()
    assert g.full.call_count == 2 and not c.calls


def test_slow_full_validation_cannot_return_a_cached_positive_budget(monkeypatch):
    g = guarded(monkeypatch)
    real = g.full
    def slow(context):
        real(context)
        g.clock[0] = context['deadline']
    g.c.node._validate_contact_pregrasp_context = slow
    with pytest.raises(ObservationPathError, match='DEADLINE_EXPIRED'): g.guard()


def test_poll_interval_counts_from_start_of_slow_full_validation(monkeypatch):
    g = guarded(monkeypatch)
    real = g.full
    def slow(context):
        real(context)
        g.clock[0] += .11
    g.c.node._validate_contact_pregrasp_context = slow
    g.guard(); g.guard()
    assert real.call_count == 2


@pytest.mark.parametrize('phase', ['selection', 'path'])
@pytest.mark.parametrize('execute', [False, True])
def test_actual_service_keeps_full_gate_after_pure_computation(monkeypatch, phase, execute):
    c = case(monkeypatch)
    c.node._gui_direct_mode = False
    original_select = c.select.side_effect
    if phase == 'selection':
        def selection(*args, **kwargs):
            for _ in range(500): kwargs['deadline_check']()
            result = original_select(*args, **kwargs)
            c.params['/grasp']['measured_endpoint_check_enabled'] = False
            return result
        c.select.side_effect = selection
    else:
        def path(*args, **kwargs):
            remaining = args[9]
            for _ in range(500): remaining()
            c.params['/grasp']['measured_endpoint_check_enabled'] = False
            return {'command': {}, 'predicted_actual': {}}
        c.curve.side_effect = path
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=execute))
    assert not result.success, result.message
    assert 'CONFIGURATION' in json.loads(result.message)['reason']
    assert not c.calls and c.state['moves'] == 0


@pytest.mark.parametrize('execute', [False, True])
def test_actual_service_uses_bounded_callback_but_keeps_each_curve_and_fresh_sample(monkeypatch, execute):
    c = case(monkeypatch)
    original_select = c.select.side_effect
    def select(*args, **kwargs):
        for _ in range(200): kwargs['deadline_check']()
        return original_select(*args, **kwargs)
    c.select.side_effect = select
    def curve(*args, **kwargs):
        for _ in range(200): args[9]()
        return {'command': {}, 'predicted_actual': {}}
    c.curve.side_effect = curve
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=execute))
    assert result.success, result.message
    audit = json.loads(result.message)
    assert audit['compute_guard']['local_checks'] >= 400
    assert audit['compute_guard']['full_checks'] < audit['compute_guard']['local_checks']/10
    assert c.curve.call_count == 1
    assert c.node._validate_frozen_observation_path.call_count == int(execute)
    stages = [row['stage'] for row in audit['stage_timings']]
    assert 'moveit_seed' in stages and 'held_endpoint_selection' in stages
    assert 'selected_path_validation' in stages
    assert all(row['completed'] for row in audit['stage_timings'])


def test_failed_curve_stage_retains_elapsed_diagnostic_and_never_submits(monkeypatch):
    c = case(monkeypatch)
    def fail(*args, **kwargs):
        args[9]()
        raise ObservationPathError('recorded test collision')
    c.curve.side_effect = fail
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=False))
    assert not result.success
    audit = json.loads(result.message)
    row = audit['stage_timings'][-1]
    assert row['stage'] == 'selected_path_validation' and row['completed'] is False
    assert row['elapsed_sec'] >= 0. and not c.calls


@pytest.mark.parametrize('execute', [False, True])
def test_fresh_prediction_must_finish_before_the_final_full_context_gate(monkeypatch, execute):
    c = case(monkeypatch)
    def stale_prediction(*args, **kwargs):
        args[-1]['success'] = True
        c.params['/grasp']['measured_endpoint_check_enabled'] = False
    monkeypatch.setattr(module, 'predict_held_approach', stale_prediction)
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=execute))
    assert not result.success and 'CONFIGURATION' in json.loads(result.message)['reason']
    assert not c.calls and c.state['moves'] == 0
