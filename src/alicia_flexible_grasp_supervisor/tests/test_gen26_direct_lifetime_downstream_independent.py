"""Whole direct-task lifetime at real start, local motion and close boundaries.

Only clocks, ROS parameters and physical services are doubles. No live ROS.
"""
from copy import deepcopy
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from test_gen26_direct_task_authority_independent import direct_scenario
from test_gen26_contact_provenance_independent import inferred_plan
from test_grasp_task_sequence import grasp_task_node as task
from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('expired_heartbeat', [False, True])
def test_later_local_segment_cannot_restart_original_direct_lifetime(monkeypatch, index, expired_heartbeat):
    c = direct_scenario(monkeypatch)
    c.advance(600.)
    if expired_heartbeat:
        c.task_callback(True, c.state['now'])
    c.node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP if index == 0 else GraspStages.APPROACH_TARGET
    c.node._compensation_task_label = ('planning 6D pregrasp', 'planning bounded 6D approach',
                                     'planning linear 6D grasp pose')[index]
    with pytest.raises(ObservationPathError, match='DIRECT_CONTACT'):
        c.node._pregrasp_context(deepcopy(c.plan), index)
    assert c.state['moves'] == 0


@pytest.mark.parametrize('index', [0, 1, 2])
def test_later_local_context_keeps_original_remaining_budget_after_heartbeat(monkeypatch, index):
    c = direct_scenario(monkeypatch)
    original = deepcopy(c.node._direct_contact_authority._lease)
    c.advance(570.)
    c.task_callback(True, c.state['now'])
    c.node._pregrasp_task_stage = GraspStages.MOVE_PREGRASP if index == 0 else GraspStages.APPROACH_TARGET
    c.node._compensation_task_label = ('planning 6D pregrasp', 'planning bounded 6D approach',
                                     'planning linear 6D grasp pose')[index]
    context = c.node._pregrasp_context(deepcopy(c.plan), index)
    assert context['deadline'] == original['monotonic_deadline']
    assert 0. < context['deadline']-c.clock['mono'] < 60.
    assert c.node._direct_contact_authority._lease == original
    c.advance(30.)
    with pytest.raises(ObservationPathError, match='DIRECT_CONTACT'):
        c.node._validate_pregrasp_context(context)


@pytest.mark.parametrize('same_generation', [True, False])
def test_observed_origin_scope_does_not_change_legacy_budget_or_hide_other_task_expiry(monkeypatch, same_generation):
    c = direct_scenario(monkeypatch)
    # This isolates local-context scope only. Complete outer-session binding
    # and callback provenance remain covered by contact_pregrasp_gateway tests.
    selected = deepcopy(c.params['/grasp_mode/selection'])
    if not same_generation:
        selected['generation'] += 1
    c.node._observe_return_origin = {'selection': selected}
    c.advance(600.)
    if same_generation:
        context = c.node._pregrasp_context(deepcopy(c.plan), 0)
        assert 'local_direct_contact_task' not in context
        assert context['deadline']-c.clock['mono'] == pytest.approx(60.)
    else:
        with pytest.raises(ObservationPathError, match='DIRECT_CONTACT'):
            c.node._pregrasp_context(deepcopy(c.plan), 0)


@pytest.mark.parametrize('strategy', ['direct', 'observe_return'])
@pytest.mark.parametrize('fault', ['none', 'manual', 'manual_return', 'epoch', 'config', 'ros_expiry', 'mono_expiry',
                                 'ros_rollback', 'mono_rollback'])
@pytest.mark.parametrize('action_kind', ['motion', 'close'])
def test_real_task_start_keeps_fixed_lifetime_through_later_action(monkeypatch, fault, action_kind, strategy):
    plan, _, _, _ = inferred_plan()
    start = plan.header.stamp.to_sec() + 1.
    start_ns = task.rospy.Time.from_sec(start).to_nsec()
    clock = {'ros': start, 'mono': 1000., 'exact_ros_ns': None}
    cfg = {'use_grasp6d_plan': True, 'unknown_require_measured_bilateral_contact': False,
           'unknown_observe_return_session_timeout_sec': 600.}
    selection = dict(mode='unknown', strategy=strategy, generation=26,
                     stamp_ns=plan.header.stamp.to_nsec()-1)
    params = {'/grasp': cfg, '/grasp_mode/selection': selection,
              '/grasp/unknown_require_measured_bilateral_contact': False,
              '/robot_description': '<robot/>', '/gui/joint_direct_mode': False}
    monkeypatch.setattr(task.rospy, 'get_param', lambda key, default=None: params.get(key, default))
    def now():
        stamp = clock['exact_ros_ns']
        return (task.rospy.Time(stamp//10**9, stamp%10**9) if stamp is not None
                else task.rospy.Time.from_sec(clock['ros']))
    monkeypatch.setattr(task.rospy.Time, 'now', staticmethod(now))
    monkeypatch.setattr(task.rospy, 'get_time', lambda: clock['ros'])
    monkeypatch.setattr(task.time, 'monotonic', lambda: clock['mono'])
    node = task.GraspTaskNode.__new__(task.GraspTaskNode)
    node.active = False
    node._following_epoch_ns = plan.header.stamp.to_nsec()-2
    node._mode_bound_selection = deepcopy(selection)
    node._start_lock = threading.RLock()
    node._grasp6d_plan_lock = threading.RLock()
    node._automatic_actuation_gate = Mock(return_value=(True, ''))
    node._copy_requested_grasp6d_plan = Mock(return_value=(task.PlanValidationResult(True), plan))
    node._bound_target_drift_result = Mock(return_value=task.PlanValidationResult(True))
    node._set_near_field_active = Mock()
    node.pub = Mock()
    physical = Mock(return_value=NS(success=True, message='synthetic physical action'))
    checks = []
    def execute(grasp6d_plan):
        assert node._validate_bound_plan(grasp6d_plan, cfg).ok
        clock['ros'] += 10.; clock['mono'] += 10.
        node.set_state(GraspStages.MOVE_PREGRASP, 'first active publication after preflight')
        first = deepcopy(node.pub.publish.call_args.args[0])
        assert first.active is True and first.header.stamp.to_nsec() == start_ns
        deadline = node._direct_contact_started_budget.ros_deadline_ns
        clock['ros'] += 10.; clock['mono'] += 10.
        node.set_state(GraspStages.APPROACH_TARGET, 'later active heartbeat')
        heartbeat = node.pub.publish.call_args.args[0]
        assert heartbeat.header.stamp == now() and heartbeat.header.stamp > first.header.stamp
        assert node._direct_contact_started_budget.ros_deadline_ns == deadline == start_ns+600_000_000_000
        assert node._validate_bound_plan(grasp6d_plan, cfg).ok
        if fault == 'manual': params['/gui/joint_direct_mode'] = True
        elif fault == 'manual_return':
            node._direct_contact_gui_mode_cb(NS(data=True))
            node._direct_contact_gui_mode_cb(NS(data=False))
        elif fault == 'epoch': node._following_epoch_ns += 1
        elif fault == 'config': params['/grasp'] = dict(cfg, unknown_observe_return_session_timeout_sec=601.)
        elif fault == 'ros_expiry': clock['exact_ros_ns'] = start_ns+600_000_000_000
        elif fault == 'mono_expiry': clock['mono'] = 1600.
        elif fault == 'ros_rollback': clock['ros'] -= 1.
        elif fault == 'mono_rollback': clock['mono'] -= 1.
        if action_kind == 'close':
            ok, message = node._close_gripper({'use_compliant_close': True}, None, physical,
                                             execution_plan=grasp6d_plan, gcfg=cfg)
            checks.append((ok, message))
        else:
            result, _ = node._invoke_plan_bound_action(grasp6d_plan, cfg, 'approach', physical)
            checks.append((result.ok, result.reason))
        return checks[-1][0]
    node.execute = execute
    node.start_cb(NS(execute=True, plan_id=plan.plan_id))
    assert node.pub.publish.call_args.args[0].active is False
    assert node.pub.publish.call_args.args[0].header.stamp == now()
    if fault == 'none':
        assert checks and checks[0][0], checks
        physical.assert_called_once()
        return
    assert checks and not checks[0][0], checks
    assert 'DIRECT_CONTACT' in checks[0][1], checks
    physical.assert_not_called()
