"""Actual near-field deadline/ticket functions with read-only service stubs."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
from unittest import mock

import pytest
from test_contact_linear_compensation import recorded
from test_remote_grasp6d_streaming import remote_node as remote, streaming_node, MutableClock


def context(monkeypatch):
    clock=MutableClock(100.)
    node=streaming_node(clock=clock,start_worker=False)
    node.start_streaming()
    node.near_field_strategy='single_snapshot_direct'
    node.near_field_planning_active=True
    node._near_field_phase_id=2
    node._near_field_phase_deadline_sec=190.
    node._execution_plan_validity_now_sec=lambda:clock.value
    node.contact_sequence_precheck_enabled=True
    _,_,plan,_,_=recorded(2)
    ticket=NS(request_id=25,generation=node._stream_generation,target_epoch=node.target_instance_epoch)
    prepared=NS(near_field=True,near_field_phase_id=2,near_field_phase_deadline_sec=190.,ticket=ticket)
    evidence=dict(scope='read_only_candidate_constant_bias_prediction',plan_id=plan.plan_id,
        execution_authorized=False,success=True,checked_stage_names=['pregrasp','approach','grasp'],
        contact_prediction=dict(scope='constant_current_bias_prediction',future_following_verified=False,
            execution_authorized=False,nominal_geometry={'ok':True},predicted_command_geometry={'ok':True}))
    response=NS(success=True,evidence_json=json.dumps(evidence),joint_path_cost=.5,joint_max_delta=.2,
        failure_code='',message='execution_duration_lower_bound_sec=4 hardware_limiting_joint=Joint2')
    calls=[]
    def service(req):
        calls.append(deepcopy(req));return response
    wait=mock.Mock()
    proxy=mock.Mock(return_value=service)
    monkeypatch.setattr(remote.rospy,'wait_for_service',wait)
    monkeypatch.setattr(remote.rospy,'ServiceProxy',proxy)
    monkeypatch.setattr(remote.rospy,'loginfo',mock.Mock())
    return node,prepared,plan,clock,calls,wait,proxy,response


def check(node,prepared,plan,deadline,runtime=None):
    return node._final_contact_sequence_evaluation(plan,(),prepared,
        {} if runtime is None else runtime,'final_registered_strict_sequence',deadline_sec=deadline)


def test_live_original_ninety_second_phase_reaches_service_unchanged(monkeypatch):
    node,p,plan,clock,calls,*_=context(monkeypatch)
    deadline=node._direct_near_field_candidate_deadline_sec(p,{})
    assert deadline==190. and node._direct_near_field_active(p)
    runtime={};result,_=check(node,p,plan,deadline,runtime)
    assert result.reachable and len(calls)==1
    assert calls[0].deadline.to_sec()==190. and calls[0].plan==plan
    assert runtime['contact_sequence_precheck']['success'] is True


@pytest.mark.parametrize('fault',['inactive','replaced_phase','missing_live_deadline','stopped_ticket','target_epoch'])
def test_revoked_or_replaced_phase_never_submits_precheck(monkeypatch,fault):
    node,p,plan,clock,calls,wait,proxy,_=context(monkeypatch)
    if fault=='inactive':node.near_field_planning_active=False;node._near_field_phase_deadline_sec=0.
    if fault=='replaced_phase':node._near_field_phase_id=3;node._near_field_phase_deadline_sec=200.
    if fault=='missing_live_deadline':node._near_field_phase_deadline_sec=0.
    if fault=='stopped_ticket':node.stop_streaming()
    if fault=='target_epoch':node._advance_target_instance_epoch('TARGET_LOST')
    deadline=node._direct_near_field_candidate_deadline_sec(p,{})
    with pytest.raises(remote.StreamResultCancelled):check(node,p,plan,deadline)
    assert not calls;wait.assert_not_called();proxy.assert_not_called()


@pytest.mark.parametrize('deadline',[0.,-1.,float('nan'),191.,100.])
def test_missing_expired_or_extended_requested_deadline_is_not_unlimited(monkeypatch,deadline):
    node,p,plan,clock,calls,wait,proxy,_=context(monkeypatch)
    result,_=check(node,p,plan,deadline)
    assert not result.reachable and result.failure_code=='MOVEIT_TIMEOUT'
    assert not calls;wait.assert_not_called();proxy.assert_not_called()


@pytest.mark.parametrize('boundary',['wait','service'])
def test_termination_while_waiting_or_in_service_does_not_commit_response(monkeypatch,boundary):
    node,p,plan,clock,calls,wait,proxy,response=context(monkeypatch)
    if boundary=='wait':wait.side_effect=lambda *a,**k:node.stop_streaming()
    else:
        def service(req):
            calls.append(deepcopy(req));node.stop_streaming();return response
        proxy.return_value=service
    runtime={}
    with pytest.raises(remote.StreamResultCancelled):check(node,p,plan,190.,runtime)
    assert len(calls)==(1 if boundary=='service' else 0)
    assert 'contact_sequence_precheck' not in runtime and 'moveit_execution_stage_policy' not in runtime
    assert p.near_field_phase_deadline_sec==190.  # no fresh budget is manufactured


def test_expired_during_wait_never_submits_and_shortened_budget_is_respected(monkeypatch):
    node,p,plan,clock,calls,wait,proxy,_=context(monkeypatch)
    deadline=node._direct_near_field_candidate_deadline_sec(p,{'selection_deadline_sec':105.})
    assert deadline==105.
    wait.side_effect=lambda *a,**k:setattr(clock,'value',105.)
    result,_=check(node,p,plan,deadline)
    assert not result.reachable and result.failure_code=='MOVEIT_TIMEOUT'
    assert not calls;proxy.assert_not_called()


def test_non_near_field_zero_deadline_preserves_original_readonly_usage(monkeypatch):
    node,p,plan,clock,calls,wait,proxy,_=context(monkeypatch)
    node.stop_streaming();p=NS(near_field=False)
    result,_=check(node,p,plan,0.)
    assert result.reachable and len(calls)==1 and calls[0].deadline.to_sec()==0.


def test_live_legacy_zero_preserves_original_service_sentinel(monkeypatch):
    node,p,plan,clock,calls,*_=context(monkeypatch)
    node.near_field_strategy='legacy_gated'
    assert not node._direct_near_field_active(p)
    result,_=check(node,p,plan,0.)
    assert result.reachable and len(calls)==1 and calls[0].deadline.to_sec()==0.
    assert node._near_field_phase_deadline_sec==p.near_field_phase_deadline_sec==190.


@pytest.mark.parametrize('fault',['expired','revoked','replaced','in_service'])
def test_legacy_zero_does_not_bypass_original_phase_lifetime(monkeypatch,fault):
    node,p,plan,clock,calls,wait,proxy,response=context(monkeypatch)
    node.near_field_strategy='legacy_gated'
    if fault=='expired':clock.value=190.
    elif fault=='revoked':node.stop_streaming()
    elif fault=='replaced':node._near_field_phase_id+=1
    else:
        def service(req):
            calls.append(deepcopy(req));node.near_field_planning_active=False;return response
        proxy.return_value=service
    runtime={}
    if fault=='expired':
        result,_=check(node,p,plan,0.,runtime)
        assert not result.reachable and result.failure_code=='MOVEIT_TIMEOUT'
    else:
        with pytest.raises(remote.StreamResultCancelled):check(node,p,plan,0.,runtime)
    assert len(calls)==(1 if fault=='in_service' else 0)
    assert 'contact_sequence_precheck' not in runtime
