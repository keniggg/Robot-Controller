from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_observation_path_guard import point, trajectory
from test_contracted_trajectory import example
from alicia_flexible_grasp.robot.contracted_trajectory import bound_action_goal
from alicia_flexible_grasp.robot.observation_path_guard import segment_coefficients, _polynomial
from alicia_flexible_grasp.robot.observation_tracking_contract import (
    commanded_stop_reserve, tracking_contract_candidates, validate_command_derivatives,
    TRAJECTORY_STOP_POLICY,
)


@pytest.mark.parametrize('velocity',[-.004,0.,.004])
@pytest.mark.parametrize('acceleration',[-.004,0.,.004])
def test_small_motion_stop_reserve_encloses_noetic_stop_curve(velocity,acceleration):
    t=.5
    c=segment_coefficients(point([1.],0.,[velocity],[acceleration]),
                           point([1.],2*t,[-velocity],[0.]),2*t,1)
    reserve=commanded_stop_reserve(t,velocity=abs(velocity),acceleration=abs(acceleration))
    assert reserve<commanded_stop_reserve(t)
    for fraction in np.linspace(0.,.5,101):
        assert abs(_polynomial(c,fraction)[0]-1.)<=reserve


def tiny_action():
    plan,audit,cfg=example()
    hold=NS(positions=[0.]*6,velocities=[0.]*6,accelerations=[0.]*6)
    plan.joint_trajectory.points[-1].positions=[.006]*6
    from alicia_flexible_grasp.robot.observation_path_guard import wire_digest
    audit['trajectory_sha256']=wire_digest(plan)
    bounds=validate_command_derivatives(plan,hold)
    contract=tracking_contract_candidates(cfg,plan.joint_trajectory.joint_names,.5,command_bounds=bounds)[-1]
    audit.update(execution_tracking_contract=contract,command_derivative_bounds=bounds)
    audit['following_support']['joint_error_bounds_rad']=contract['joint_error_bounds_rad']
    return plan,audit,cfg,hold


def test_action_recomputes_stop_reserve_from_same_final_curve_and_bound_hold():
    plan,audit,cfg,hold=tiny_action()
    goal=bound_action_goal(plan,audit,cfg,.5,reference=hold)
    contract=audit['execution_tracking_contract']
    assert contract['policy']==TRAJECTORY_STOP_POLICY
    assert contract['commanded_stop_reserve_rad']<.002
    assert [t.position for t in goal.path_tolerance]==[.035]*6
    assert [t.position for t in goal.goal_tolerance]==[.035]*6
    assert goal.goal_time_tolerance.to_sec()==8.


@pytest.mark.parametrize('mutation',['missing_reference','reference','derivative','reserve','path'])
def test_changed_dynamic_stopping_proof_is_not_an_executable_action(mutation):
    plan,audit,cfg,hold=tiny_action()
    if mutation=='missing_reference':hold=None
    if mutation=='reference':hold.positions[0]+=.001
    if mutation=='derivative':audit['command_derivative_bounds']['maximum_velocity_rad_s'][0]=0.
    if mutation=='reserve':audit['execution_tracking_contract']['commanded_stop_reserve_rad']=0.
    if mutation=='path':plan.joint_trajectory.points[-1].positions[0]+=.001
    with pytest.raises(ValueError):bound_action_goal(plan,audit,cfg,.5,reference=hold)


@pytest.mark.parametrize('value',[-1.,.081,float('nan'),float('inf'),True])
def test_arbitrary_velocity_cannot_shrink_or_enlarge_stopping_contract(value):
    with pytest.raises(ValueError):commanded_stop_reserve(.5,velocity=value)


def test_controller_bridge_is_included_in_computed_stop_bounds():
    plan,audit,cfg,hold=tiny_action()
    hold.positions=[-.01]*6
    actual=validate_command_derivatives(plan,hold)
    assert actual['maximum_velocity_rad_s'][0]>audit['command_derivative_bounds']['maximum_velocity_rad_s'][0]
    with pytest.raises(ValueError):bound_action_goal(plan,audit,cfg,.5,reference=hold)
