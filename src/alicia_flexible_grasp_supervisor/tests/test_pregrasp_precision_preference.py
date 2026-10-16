"""Soft pose0 preference; synthetic goals on recorded FK/following inputs.

The 9.333 mm boundary reproduces the numeric question, not a recorded contact
plan. Controller responses below are explicit conditional plants, not hardware.
"""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from unittest import mock
import numpy as np
import pytest
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q
from test_pregrasp_feedforward import held_gateway_case


def case(monkeypatch, distance=.009333, response=True):
    values = held_gateway_case(monkeypatch, response)
    node, planner, plan, state, calls, params, context = values
    fk = SerialUrdfFk(context['model'], ARM_NAMES)
    actual = state['actual'].copy()
    origin = fk(actual)
    low, high = 0., .1
    for _ in range(50):
        middle = (low+high)/2
        q=actual.copy();q[2]+=middle
        if np.linalg.norm(fk(q)[:3,3]-origin[:3,3]) < distance: low=middle
        else: high=middle
    q=actual.copy();q[2]+=(low+high)/2
    goal=fk(q)
    p=plan.poses[0]
    p.position.x,p.position.y,p.position.z=goal[:3,3]
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quaternion_from_matrix(goal)
    plan.plan_id=compute_plan_id(plan);context['scene'].plan_id=plan.plan_id
    context['config']['unknown_pregrasp_position_preference_m']=.006
    return values


def run(values, execute=True):
    node,planner,plan,state,calls,params,context=values
    result=node.handle_compensate_pregrasp(NS(plan=plan,execute=execute))
    return result,json.loads(result.message)


def test_under_ten_mm_actually_attempts_original_bounded_held_guide(monkeypatch):
    values=case(monkeypatch)
    original=PregraspFeedforward.propose_held_axes
    attempts=[]
    def checked(self):
        p,_=self.contract.residual(self.initial['accepted_positions_rad'])
        attempts.append(p)
        assert self.contract.tolerances[0]==.010
        return original(self)
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',checked)
    result,audit=run(values)
    assert result.success,audit
    assert attempts==pytest.approx([.009333])
    assert values[3]['moves']==1 and len(values[4])==1
    assert audit['position_error_m']<.006
    assert audit['position_preference_m']==.006 and audit['position_acceptance_m']==.010
    assert audit['precision_preference_is_hard_gate'] is False
    assert audit['hardware_response_verified'] is True
    assert audit['proposed_delta_counts'][1]==audit['proposed_delta_counts'][4]==0
    assert max(abs(np.array(audit['proposed_delta_counts'])))<=32


def test_already_five_mm_keeps_no_motion(monkeypatch):
    values=case(monkeypatch,.005)
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',mock.Mock(side_effect=AssertionError('unneeded guide')))
    result,audit=run(values)
    assert result.success and audit['position_error_m']==pytest.approx(.005)
    assert audit['steps_completed']==0 and not values[4] and not values[5]


def test_seven_mm_conditional_improvement_can_execute_and_pass_hard_gate(monkeypatch):
    values=case(monkeypatch)
    # A finite legal guide that improves but cannot achieve the soft target.
    monkeypatch.setattr(PregraspCompensation, '_remaining_budget_guide',
                        lambda *_:(np.array([0,0,4,0,0,0]),.0073))
    result,audit=run(values)
    assert result.success,audit
    assert .006<audit['position_error_m']<.008
    assert audit['hardware_response_verified'] and audit['steps_completed']==1
    assert audit['position_acceptance_m']==.010


def zero_guide(monkeypatch):
    calls=[]
    def guide(self,actual):
        calls.append(actual.copy())
        return np.zeros(6,dtype=int),self.residual(actual)[0]
    monkeypatch.setattr(PregraspCompensation,'_remaining_budget_guide',guide)
    return calls


def test_finite_zero_guide_noop_requires_fresh_original_hard_acceptance(monkeypatch):
    values=case(monkeypatch);calls=zero_guide(monkeypatch)
    node=values[0];snapshot=node._endpoint_following_snapshot
    captures=[]
    def fresh(*args,**kwargs):
        if captures:values[3]['now']+=.1
        value=snapshot(*args,**kwargs);captures.append(value);return value
    node._endpoint_following_snapshot=fresh
    result,audit=run(values)
    assert result.success,audit
    assert len(calls)==1 and len(captures)==2
    for key in ('sdk_stamp_ns','accepted_stamp_ns'):
        assert audit[key]==captures[1][key]>audit['initial_'+key]==captures[0][key]
    assert audit['position_error_m']==pytest.approx(.009333)
    assert audit['precision_preference_outcome']=='finite_guide_no_qualified_improvement'
    assert audit['no_motion_submitted'] and audit['steps_completed']==0
    assert not values[4] and not values[5]


@pytest.mark.parametrize('fault',['stale','owner','epoch','sdk','moving','expired','hard_miss'])
def test_noop_never_swallows_invalid_fresh_context(monkeypatch,fault):
    values=case(monkeypatch,.00999 if fault=='hard_miss' else .009333);zero_guide(monkeypatch)
    node=values[0];snapshot=node._endpoint_following_snapshot;count=[0]
    def fresh(*args,**kwargs):
        row=snapshot(*args,**kwargs);count[0]+=1
        if count[0]==2:
            if fault=='owner':raise ValueError('OWNER_CHANGED')
            if fault=='stale':row['accepted_stamp_ns']-=600000000
            if fault=='epoch':row['epoch_ns']+=1
            if fault=='sdk':row['sdk_positions_rad'][0]+=Q
            if fault=='moving':row['accepted_positions_rad'][0]+=2*Q
            if fault=='expired':row['stamp_sec']+=61
            if fault=='hard_miss':row['accepted_positions_rad'][2]-=Q
        return row
    node._endpoint_following_snapshot=fresh
    result,audit=run(values)
    assert not result.success,audit
    assert not values[4] and not values[5]


@pytest.mark.parametrize('fault',['no_response','reverse','path_collision','execution_false','unsafe_guide','malformed_guide'])
def test_optional_improvement_does_not_hide_failure(monkeypatch,fault):
    response=False if fault=='no_response' else (lambda d:-d) if fault=='reverse' else True
    values=case(monkeypatch,response=response)
    node,planner,plan,state,calls,params,context=values
    if fault in ('path_collision','execution_false'):
        def failed(*args,**kwargs):
            if fault=='execution_false':kwargs['before_execute']()
            return False,'COLLISION' if fault=='path_collision' else 'controller failed'
        planner.plan_and_execute_joint_probe=failed
    elif fault=='unsafe_guide':
        monkeypatch.setattr(PregraspCompensation,'_remaining_budget_guide',lambda *_:(np.array([0,2,0,0,0,0]),0.))
    elif fault=='malformed_guide':
        monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',mock.Mock(side_effect=ValueError('invalid geometry')))
    result,audit=run(values)
    assert not result.success,audit
    assert audit['code']!='PREGRASP_MEASURED_CONVERGED'


@pytest.mark.parametrize('value',[True,0.,-.001,.007,float('nan'),'0.006'])
def test_invalid_preference_is_not_an_acceptance_override(monkeypatch,value):
    values=case(monkeypatch);values[-1]['config']['unknown_pregrasp_position_preference_m']=value
    result,audit=run(values)
    assert not result.success and audit['code']=='PREGRASP_PRECISION_PREFERENCE_INVALID'
    assert not values[4]
