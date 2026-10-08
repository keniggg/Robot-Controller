"""Pure cap and mocked gateway contracts; no ROS node or hardware commands."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from alicia_flexible_grasp.robot.feedforward_timing import resolve_unknown_feedforward_timing
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from test_pregrasp_feedforward import held_gateway_case
from test_held_approach_gateway import approach_case


def resolve(config, planner=.12, per_joint=None, index=0, model='unknown_tabletop'):
    return resolve_unknown_feedforward_timing(config, model, index, planner, per_joint)


@pytest.mark.parametrize('config', [{}, {'unknown_feedforward_timing_enabled':False},
    {'unknown_feedforward_timing_enabled':False, 'unknown_feedforward_max_joint_velocity_rad_s':.008}])
def test_absent_or_disabled_retains_existing_two_milliradian_cap(config):
    cap, audit = resolve(config)
    assert cap == .002
    assert set(audit['effective_joint_velocity_limits_rad_s'].values()) == {.002}
    assert resolve(config, planner=.001)[0] == .001


def test_enabled_cap_preserves_lower_planner_and_per_joint_limits_without_mutating_inputs():
    config = dict(unknown_feedforward_timing_enabled=True, unknown_feedforward_max_joint_velocity_rad_s=.008)
    per_joint = {'Joint2':.003, 'Joint6':.001}
    original = deepcopy((config,per_joint))
    cap, audit = resolve(config, planner=.006, per_joint=per_joint)
    assert cap == .006
    assert audit['effective_joint_velocity_limits_rad_s']['Joint2'] == .003
    assert audit['effective_joint_velocity_limits_rad_s']['Joint6'] == .001
    assert (config,per_joint) == original
    planner = NS(strict_execution_max_joint_velocity_rad_s=cap,
                 strict_execution_joint_velocity_limits_rad_s=per_joint)
    assert MoveItPlanner._controller_reference_velocity_limits(planner,ARM_NAMES) == audit['effective_joint_velocity_limits_rad_s']


@pytest.mark.parametrize('velocity', [True,False,None,'0.008',0.,-1.,float('nan'),float('inf'),.0200000001])
@pytest.mark.parametrize('enabled', [False,True])
def test_invalid_velocity_never_silently_falls_back(velocity,enabled):
    with pytest.raises(ValueError,match='TIMING_INVALID'):
        resolve(dict(unknown_feedforward_timing_enabled=enabled,
                     unknown_feedforward_max_joint_velocity_rad_s=velocity))


@pytest.mark.parametrize('enabled', ['true',1,None,{}])
def test_enable_switch_is_strictly_boolean(enabled):
    with pytest.raises(ValueError,match='boolean'):
        resolve(dict(unknown_feedforward_timing_enabled=enabled))


@pytest.mark.parametrize('index,model', [(2,'unknown_tabletop'),(True,'unknown_tabletop'),(-1,'unknown_tabletop'),(0,'known')])
def test_configuration_cannot_authorize_another_stage_or_model(index,model):
    with pytest.raises(ValueError,match='SCOPE_INVALID'):
        resolve({'unknown_feedforward_timing_enabled':True},index=index,model=model)


@pytest.mark.parametrize('value', [0.,float('nan'),True,'0.002'])
def test_malformed_existing_limits_fail_closed(value):
    with pytest.raises(ValueError,match='TIMING_INVALID'):
        resolve({},planner=value)
    with pytest.raises(ValueError,match='TIMING_INVALID'):
        resolve({},per_joint={'Joint2':value})


@pytest.mark.parametrize('index', [0,1])
@pytest.mark.parametrize('succeed', [False,True])
def test_enabled_gateway_changes_only_timing_and_restores_every_temporary_setting(monkeypatch,index,succeed):
    case = approach_case if index else held_gateway_case
    node,planner,plan,state,calls,params,context = case(monkeypatch)
    context['config'].update(unknown_feedforward_timing_enabled=True,
        unknown_feedforward_max_joint_velocity_rad_s=.008)
    planner.strict_execution_joint_velocity_limits_rad_s = {'Joint3':.004}
    limits = deepcopy(planner.strict_execution_joint_velocity_limits_rad_s)
    digest = committed_plan_digest(plan)
    original_execute = planner.plan_and_execute_joint_probe
    observed = []
    def execute(*args,**kwargs):
        observed.append(planner.strict_execution_max_joint_velocity_rad_s)
        assert planner.strict_collinear_timing_enabled is True
        assert planner.observation_path_guard_required and planner.observation_tracking_contract_required
        assert callable(planner.observation_path_validator)
        assert callable(planner.observation_submission_revalidate)
        assert planner.strict_execution_joint_velocity_limits_rad_s == limits
        if not succeed:
            return False,'existing continuous stop reserve proof rejected'
        return original_execute(*args,**kwargs)
    planner.plan_and_execute_joint_probe = execute
    entry = node.handle_compensate_approach if index else node.handle_compensate_pregrasp
    result = entry(NS(plan=plan,execute=True))
    evidence = json.loads(result.message)
    assert observed == [.008]
    assert result.success is succeed
    assert evidence['feedforward_timing']['effective_global_velocity_rad_s'] == .008
    assert evidence['feedforward_timing']['effective_joint_velocity_limits_rad_s']['Joint3'] == .004
    expected = [-4,0,-18,0,0,32] if index else [-4,0,28,-2,0,-32]
    assert evidence['proposed_delta_counts'] == expected
    assert committed_plan_digest(plan) == digest
    assert planner.strict_execution_max_joint_velocity_rad_s == .12
    assert planner.strict_execution_joint_velocity_limits_rad_s == limits
    assert not hasattr(planner,'strict_collinear_timing_enabled')
    assert planner.observation_path_guard_required is False
    assert planner.observation_tracking_contract_required is False
    assert state['moves'] == int(succeed)
    assert len(calls) == len(params) == int(succeed)


@pytest.mark.parametrize('index', [0,1])
def test_invalid_gateway_configuration_fails_before_path_or_episode(monkeypatch,index):
    node,planner,plan,state,calls,params,context = (approach_case if index else held_gateway_case)(monkeypatch)
    context['config'].update(unknown_feedforward_timing_enabled=True,
        unknown_feedforward_max_joint_velocity_rad_s=.021)
    entry = node.handle_compensate_approach if index else node.handle_compensate_pregrasp
    result = entry(NS(plan=plan,execute=True))
    assert not result.success and 'TIMING_INVALID' in result.message
    assert not calls and not params and state['moves'] == 0
    node._ensure_trajectory_controllers_started.assert_not_called()
    assert planner.strict_execution_max_joint_velocity_rad_s == .12
    assert not hasattr(planner,'strict_collinear_timing_enabled')
