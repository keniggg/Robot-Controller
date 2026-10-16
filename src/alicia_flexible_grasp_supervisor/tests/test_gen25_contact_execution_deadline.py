"""Nominal contact motion keeps its authenticated session, not a trim budget.

Use the actual gateway and action executor with synthetic transport/feedback;
the long duration comes from the failed GEN24 controller-reference audit.
"""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import rospy
from std_msgs.msg import String

from test_contact_pregrasp_gateway import case, captured_session, path
from alicia_flexible_grasp.robot import contact_pregrasp_gateway as gateway
from alicia_flexible_grasp.robot.contracted_trajectory import ContractedTrajectoryExecutor


RECORDED = json.loads((Path(__file__).parent / 'fixtures/gen25_contact_execution_duration.json').read_text())


def scenario(monkeypatch, *, outer_seconds=520., bound=True):
    mono = [100.]
    monkeypatch.setattr(gateway.time, 'monotonic', lambda: mono[0])
    c = case(monkeypatch)
    origin = None
    if bound:
        origin = captured_session(c, monkeypatch)
        # Construct a new synthetic captured fixture before any nominal call.
        # The production task-session callback validates its long lease.
        raw = deepcopy(origin.pop('view_session'))
        raw['deadline_ns'] = int((c.state['now'] + outer_seconds) * 1e9)
        origin['deadline'] = mono[0] + outer_seconds
        c.node._observe_return_view_session_cb(String(data=json.dumps(raw)))
        assert origin['view_session']['deadline_ns'] == raw['deadline_ns']
    def advance(seconds):
        mono[0] += seconds
        c.state['now'] += seconds
    c.node._validate_frozen_observation_path.return_value = {
        'controller_constraints_snapshot': {'goal_time': RECORDED['controller_goal_time_sec']}}
    c.node._revalidate_observation_contract_submission = Mock()
    before = []
    def execute(target, execute, **kwargs):
        trajectory = path(kwargs['start_joint_positions'], target)
        trajectory.joint_trajectory.points[-1].time_from_start = rospy.Duration.from_sec(
            RECORDED['final_duration_sec'])
        advance(6.)  # Real recorded computation was under the unchanged 60 s cap.
        report = c.planner.observation_path_validator(trajectory)
        kwargs['before_execute']()
        client = NS(send_goal=lambda _goal: c.state.update(moves=c.state['moves']+1),
            wait_for_result=lambda _timeout: (advance(RECORDED['final_duration_sec']) or True),
            get_result=lambda: NS(error_code=0, SUCCESSFUL=0), get_state=lambda: 3,
            cancel_goal=Mock())
        transport = ContractedTrajectoryExecutor(client, NS(get_num_connections=lambda:1,
            publish=Mock()), clock=lambda:mono[0])
        goal = NS(trajectory=trajectory.joint_trajectory,
            goal_time_tolerance=rospy.Duration.from_sec(RECORDED['controller_goal_time_sec']))
        def before_send():
            before.append(True)
            c.planner.observation_submission_revalidate(trajectory, report)
        return transport.execute(goal, before_send=before_send,
                                 still_authorized=c.planner.observation_execution_authorized)
    c.planner.plan_and_execute_joint_probe = execute
    return NS(c=c, mono=mono, origin=origin, advance=advance, before=before)


def test_recorded_long_nominal_trajectory_completes_inside_original_outer_session(monkeypatch):
    s = scenario(monkeypatch)
    original_origin = s.origin['deadline']
    result = s.c.node.handle_contact_pregrasp(NS(plan=s.c.plan, execute=True))
    assert result.success, result.message
    report = json.loads(result.message)
    assert s.before == [True] and s.c.state['moves'] == 1
    assert s.mono[0] > 160. and s.mono[0] < original_origin
    assert s.origin['deadline'] == original_origin
    assert report['code'] == 'CONTACT_PREGRASP_EXECUTED'
    assert not report['hardware_response_verified'] and not report['real_grasp_success']


@pytest.mark.parametrize('bound,outer', [(False, 520.), (True, 140.)])
def test_missing_or_insufficient_outer_budget_never_submits_long_path(monkeypatch, bound, outer):
    s = scenario(monkeypatch, outer_seconds=outer, bound=bound)
    result = s.c.node.handle_contact_pregrasp(NS(plan=s.c.plan, execute=True))
    assert not result.success
    assert 'INSUFFICIENT_EXECUTION_TIME' in result.message
    assert not s.before and s.c.state['moves'] == 0


@pytest.mark.parametrize('change', ['compute_overrun', 'ros_rollback', 'mono_rollback',
                                  'outer_mutation', 'manual', 'epoch', 'revocation'])
def test_final_transport_boundary_rechecks_compute_clocks_and_original_owner(monkeypatch, change):
    s = scenario(monkeypatch)
    c = s.c
    def boundary(*args, **kwargs):
        if change == 'compute_overrun': s.advance(55.)
        elif change == 'ros_rollback': c.state['now'] -= .01
        elif change == 'mono_rollback': s.mono[0] -= .01
        elif change == 'outer_mutation': s.origin['deadline'] += 600.
        elif change == 'manual': c.state['manual'] = True
        elif change == 'epoch': c.node._control_reference_epoch_ns += 1
        else: c.node._observation_contexts.clear()
    c.node._revalidate_observation_contract_submission.side_effect = boundary
    result = c.node.handle_contact_pregrasp(NS(plan=c.plan, execute=True))
    assert not result.success, result.message
    assert c.state['moves'] == 0

