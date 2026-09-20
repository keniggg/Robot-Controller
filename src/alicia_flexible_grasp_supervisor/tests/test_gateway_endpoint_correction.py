import json
from types import SimpleNamespace as NS
from unittest import mock

import numpy as np
import pytest

from test_gateway_tracking_probe import fixture as gateway_fixture
from test_gateway_tracking_probe import path_fixture
from test_endpoint_correction import fixture, sample
from test_motion_gateway_controller_start import motion_gateway_node as gateway
from alicia_flexible_grasp.robot.stationary_following import SDK_QUANTUM_RAD as Q


def make_gateway(monkeypatch, *, enabled=True, response=True, controller_ok=True):
    node, _ = gateway_fixture()
    fk, goal, bias, _ = fixture()
    state = {'now':10., 'sdk':goal.copy(), 'actual':goal+bias}
    xml = __import__('test_stationary_following').XML.replace('0.05 0 0','0.10 0 0')
    parameters = {}
    monkeypatch.setattr(gateway.rospy, 'set_param', lambda key,value: parameters.update({key:value}))
    monkeypatch.setattr(gateway.rospy, 'get_param', lambda key, default=None:
        xml if key == '/robot_description' else enabled
        if key == '~enable_endpoint_correction_experiments' else parameters.get(key,default))
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: state['now'])
    node._tracking_probe_context = mock.Mock(return_value=(NS(plan_id='scene'), 1_000_000_000, 3))
    node._command_reference_snapshot = lambda: NS(positions=state['sdk'].tolist())
    def snapshot(*args, **kwargs):
        if kwargs.get('after_ns'):
            state['now'] += 1.
        return sample(fk,state['sdk'],state['actual'],state['now'])
    node._endpoint_following_snapshot = snapshot
    calls = []
    def probe(req):
        calls.append(req)
        if req.execute:
            state['now'] += 1.
            delta = np.array(req.positions)-state['sdk']
            state['sdk'] = np.array(req.positions)
            if response:
                state['actual'] += delta
        return NS(success=controller_ok, message='controller result only')
    node.handle_tracking_probe = probe
    return node,state,calls


def test_experiment_disabled_by_default_never_plans_or_moves(monkeypatch):
    node,_,calls=make_gateway(monkeypatch,enabled=False)
    result=node.handle_endpoint_feedback_correction(NS(trigger=True))
    assert not result.success and 'NOT_ENABLED' in result.message
    assert not calls
    node._tracking_probe_context.assert_not_called()


def test_dry_run_only_plans_one_step_and_does_not_consume_epoch(monkeypatch):
    node,state,calls=make_gateway(monkeypatch,enabled=False)
    before=state['sdk'].copy()
    result=node.handle_endpoint_feedback_correction(NS(trigger=False))
    assert result.success
    assert len(calls)==1 and calls[0].execute is False
    assert np.array_equal(state['sdk'],before)
    assert not getattr(node,'_endpoint_correction_consumed_epoch',None)
    report=json.loads(result.message)
    assert report['physical_path_authorized'] is False


def test_compensation_requires_new_measured_arrival_not_controller_success(monkeypatch):
    node,state,calls=make_gateway(monkeypatch)
    result=node.handle_endpoint_feedback_correction(NS(trigger=True))
    report=json.loads(result.message)
    assert result.success and report['code']=='MEASURED_ENDPOINT_CONVERGED'
    assert report['fixed_goal_position_error_m']<=.006
    assert report['sdk_following_error_m']>.006
    assert report['fixed_goal_counts']==[2048]*6
    assert len(calls)==1
    node._endpoint_correction_consumed_epoch = None  # simulate gateway reload
    repeated=node.handle_endpoint_feedback_correction(NS(trigger=True))
    assert not repeated.success and 'ALREADY_CONSUMED' in repeated.message
    assert len(calls)==1
    assert all(c.execute is True for c in calls)
    repeated=node.handle_endpoint_feedback_correction(NS(trigger=True))
    assert not repeated.success and 'ALREADY_CONSUMED' in repeated.message
    assert len(calls)==1


@pytest.mark.parametrize('response,controller_ok',[(False,True),(True,False)])
def test_no_response_or_controller_failure_cannot_issue_next_step(monkeypatch,response,controller_ok):
    node,_,calls=make_gateway(monkeypatch,response=response,controller_ok=controller_ok)
    result=node.handle_endpoint_feedback_correction(NS(trigger=True))
    assert not result.success and len(calls)==1
    assert ('NO_BOUNDED_DIRECTIONAL_RESPONSE' if controller_ok else 'STEP_REJECTED') in result.message
    report = json.loads(result.message)
    assert report['no_implicit_rollback_or_torque_disable'] is True
    if controller_ok:
        assert report['steps_completed'] == 1
        assert report['last_measured_report']['stamp_sec'] > 10.
        assert report['last_measured_report']['position_error_m'] > .006
        assert report['current_sdk_counts'] != report['fixed_goal_counts']
        assert report['measurement_stamp_sec'] == report['last_measured_report']['stamp_sec']
        assert report['sdk_following_error_m'] == report['last_measured_report']['position_error_m']
        assert report['last_step_response']['measured_delta_counts'] == [0]*6
        assert report['last_step_response']['bounded_directional_response'] is False


def test_changed_sdk_reference_cannot_be_overwritten(monkeypatch):
    node,_,calls=make_gateway(monkeypatch)
    node._command_reference_snapshot = mock.Mock(side_effect=[NS(positions=[0.]*6),NS(positions=[Q]*6)])
    result=node.handle_endpoint_feedback_correction(NS(trigger=True))
    assert not result.success and 'REFERENCE_CHANGED' in result.message
    assert not calls


@pytest.mark.parametrize('deadline', [(15., 1000.), (1000., 15.)])
def test_final_timed_path_must_fit_both_episode_clocks(monkeypatch, deadline):
    node, planner, scene, plan = path_fixture(monkeypatch)
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: 10.)
    monkeypatch.setattr(gateway.time, 'monotonic', lambda: 10.)
    node._endpoint_correction_deadline = deadline
    with pytest.raises(ValueError, match='INSUFFICIENT_STEP_BUDGET'):
        node._validate_tracking_probe_path((scene,1,2), [0.]*6, [.005]*6, plan, planner)


def test_deadline_guard_allows_verified_path_with_feedback_budget(monkeypatch):
    node, planner, scene, plan = path_fixture(monkeypatch)
    monkeypatch.setattr(gateway.rospy, 'get_time', lambda: 10.)
    monkeypatch.setattr(gateway.time, 'monotonic', lambda: 10.)
    node._endpoint_correction_deadline = (16., 16.)
    assert node._validate_tracking_probe_path(
        (scene,1,2), [0.]*6, [.005]*6, plan, planner)['diagnostic_only']
