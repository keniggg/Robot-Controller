"""Offline scheduling tests use the production strict loop and source checks.

The ROS strict response and hardware following proof are fixture responses;
these tests establish dispatch order and retained gates, not robot feasibility.
"""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS, MethodType

import numpy as np
import pytest

from test_remote_modes import configured, module, base
from test_unknown_observation_view_source import generic_context
from test_silhouette_observation_view_source import encoded_result


def loop_context(configured, monkeypatch, tmp_path, *, count=3, first_failure=None):
    node, prepared, _, _, fk = generic_context(configured, monkeypatch, tmp_path)
    # Restore the actual mode wrappers replaced by the shared transport fixture.
    monkeypatch.setattr(node, '_strict_moveit_evaluation', MethodType(
        module.ModeAwareRemoteGrasp6DNode._strict_moveit_evaluation, node))
    monkeypatch.setattr(node, '_commit_strict_moveit_evaluation', MethodType(
        base.RemoteGrasp6DNode._commit_strict_moveit_evaluation, node))
    monkeypatch.setattr(node, '_joint_space_observation_active', lambda: True)
    rows=[]
    for index in range(count):
        T=fk([.02,0,0,0,0,0])
        pose=base.make_pose_stamped('base_link',T[:3,3],base.quaternion_from_matrix(T),stamp=prepared.stamp)
        audit=dict(information_utility=dict(utility=.5),orientation_source='measured_missing_side_geodesic')
        sequence=NS(pregrasp=pose,observation_view_audit=deepcopy(audit))
        rows.append(dict(sequence=sequence,observation_view=deepcopy(audit),
            search_family='joint_space_fk' if index in (1,2) else 'camera_orientation_lattice',
            observation_envelope=base.ObservationEnvelopeResult(True,'','',.02),
            observation_side_evidence={},camera_roll_offset_deg=0.,camera_tilt_offset_deg=45.,
            camera_target_distance_m=.26 + .01 * index,
            preference_index=index))
    runtime=dict(prepared=prepared,grasp_pose=rows[0]['sequence'].pregrasp,
        observation_sequence=rows[0]['sequence'],observation_variants=tuple(rows),soft_evidence={})
    node._stable_variant_runtime={(31,1):runtime}
    candidate=NS(track_id=31,variant_index=1)
    events=[];clock=[0.];node._observation_first_strict_clock=lambda:clock[0]
    node._observation_tilt_clock=lambda:clock[0]
    node._observation_comparison_clock=lambda:clock[0]
    hints=[]
    def hint(variants, current):
        assert current is prepared
        events.append(('hints',tuple(v['preference_index'] for v in variants)))
        hints.append(tuple(variants));clock[0]+=2.5
        node._unknown_observation_endpoint_hint_cache={'used_sec':2.5,'checked_count':len(variants)}
        return {},dict(status='complete',used_sec=2.5,checked_count=len(variants),execution_authority=False)
    monkeypatch.setattr(node,'_observation_endpoint_search_hints',hint)
    def strict(_self,target,observation=False):
        assert observation
        index=next(i for i,v in enumerate(rows) if v['sequence'].pregrasp is target)
        events.append(('strict',index));clock[0]+=.1
        result,_=encoded_result(prepared,fk,target=target)
        result=replace(result,reason=result.reason+' observation_screened_duration_sec=2.0')
        if index==1 and first_failure=='strict':
            result=replace(result,reachable=False,failure_code='MOVEIT_COLLISION')
        if index==1 and first_failure=='view':
            result=replace(result,reason='missing original full-path proof')
        return result,{},''
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_strict_moveit_evaluation',strict)
    def following(_self,_prepared,result,target=None):
        index=next(i for i,v in enumerate(rows) if v['sequence'].pregrasp is target)
        events.append(('following',index))
        if index==1 and first_failure=='following':
            return replace(result,reachable=False,failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'),{'ok':False}
        return result,{'ok':True}
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',following)
    checker,_=node._far_field_observation_search()
    return node,prepared,runtime,rows,candidate,checker,events,hints,clock


def test_first_fk_enters_original_strict_and_complete_view_gate_before_any_ik_hints(
        configured,monkeypatch,tmp_path):
    node,prepared,runtime,rows,candidate,checker,events,hints,clock=loop_context(configured,monkeypatch,tmp_path)
    before=[base.pose_matrix(v['sequence'].pregrasp).copy() for v in rows]
    result=checker(candidate)
    assert result.reachable and events==[('strict',1),('following',1)] and not hints
    report=runtime['observation_orientation_search']
    assert report['attempted_variant_count']==1 and report['configured_variant_count']==3
    assert report['search_complete'] is False
    assert report['variant_results'][0]['following_support']['unknown_observation_view']['ok']
    timing=report['scheduling']['first_strict_dispatch']
    assert timing['first_strict_delay_sec']==0. and timing['hint_elapsed_sec']==0.
    assert timing['first_full_path_qualified'] and not timing['execution_authority']
    assert node._observation_tilt_search_budget['checked_count']==1
    assert node._observation_information_comparison['ticket'] is prepared.ticket
    assert node._observation_information_comparison['deadline'] is not None
    for v,old in zip(rows,before):np.testing.assert_array_equal(base.pose_matrix(v['sequence'].pregrasp),old)


@pytest.mark.parametrize('failure',['strict','view','following'])
def test_failed_first_full_path_defers_to_original_hints_and_family_order_once(
        configured,monkeypatch,tmp_path,failure):
    node,_,runtime,rows,candidate,checker,events,hints,_=loop_context(
        configured,monkeypatch,tmp_path,first_failure=failure)
    result=checker(candidate)
    assert result.reachable
    assert events[0]==('strict',1)
    assert len(hints)==1 and all(v is not rows[1] for v in hints[0])
    assert [v for kind,v in events if kind=='strict']==[1,2,0]
    assert events.index(next(e for e in events if e[0]=='hints'))<events.index(('strict',2))
    timing=runtime['observation_orientation_search']['scheduling']['first_strict_dispatch']
    assert not timing['first_full_path_qualified'] and timing['hint_elapsed_sec']==2.5
    assert node._observation_tilt_search_budget['checked_count']==3
    assert runtime['unknown_observation_scheduling']['candidate_count']==len(rows)


@pytest.mark.parametrize('scope',['foreign_ticket','prior_incumbent','no_state'])
def test_old_or_foreign_comparison_never_becomes_first_fk_success(
        configured,monkeypatch,tmp_path,scope):
    node,prepared,runtime,rows,_,_,events,hints,_=loop_context(configured,monkeypatch,tmp_path)
    node._active_unknown_observation_check_runtime=runtime
    node._observation_information_comparison={'ticket':prepared.ticket,'deadline':None}
    if scope=='foreign_ticket':node._observation_information_comparison['ticket']=replace(prepared.ticket,request_id=999)
    elif scope=='prior_incumbent':node._observation_information_comparison['deadline']=123.
    else:node._observation_information_comparison=None
    result=tuple(node._order_observation_checks(tuple(rows)))
    assert len(hints)==1 and len(result)==3
    assert 'observation_first_strict' not in runtime and events[0][0]=='hints'


def test_weaker_fk_cannot_precede_stronger_support_lattice(configured,monkeypatch,tmp_path):
    node,prepared,runtime,rows,_,_,events,hints,_=loop_context(configured,monkeypatch,tmp_path)
    node._active_unknown_observation_check_runtime=runtime
    node._observation_information_comparison={'ticket':prepared.ticket,'deadline':None}
    support=node._observation_support_schedule(prepared)
    normal=support.evaluate; lattice_pose=base.pose_matrix(rows[0]['sequence'].pregrasp)
    # Distinct FK transforms allow source-bound scheduling to distinguish rows.
    for i in (1,2):rows[i]['sequence'].pregrasp.pose.position.y+=i*.001
    def evaluate(pose):
        result=normal(pose)
        return dict(result,status='sufficient' if np.array_equal(pose,lattice_pose) else 'insufficient')
    monkeypatch.setattr(support,'evaluate',evaluate)
    result=tuple(node._order_observation_checks(tuple(rows)))
    assert result[0] is rows[0] and len(hints)==1
    assert 'observation_first_strict' not in runtime


def test_current_ticket_revocation_after_strict_cannot_turn_into_incumbent(
        configured,monkeypatch,tmp_path):
    node,prepared,runtime,_,candidate,checker,events,hints,_=loop_context(configured,monkeypatch,tmp_path)
    original=node._diagnostic_observation_view_evaluation
    def invalidated(*args):
        result=original(*args)
        node._advance_target_instance_epoch('TEST_SOURCE_REVOKED',preserve_identity=False)
        return result
    monkeypatch.setattr(node,'_diagnostic_observation_view_evaluation',invalidated)
    with pytest.raises(base.StreamResultCancelled):checker(candidate)
    assert events[0]==('strict',1) and not hints
    assert node._observation_information_comparison['deadline'] is None
    assert runtime['observation_first_strict']['first_full_path_qualified'] is False


def test_failed_following_keeps_finite_discovery_active_without_an_incumbent(
        configured,monkeypatch,tmp_path):
    node,_,runtime,rows,candidate,checker,events,hints,clock=loop_context(
        configured,monkeypatch,tmp_path,count=10,first_failure='following')
    def rejected(_self,_prepared,result,target=None):
        clock[0]+=.91
        return replace(result,reachable=False,failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'),{'ok':False}
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',rejected)
    result=checker(candidate)
    assert not result.reachable
    strict=[value for kind,value in events if kind=='strict']
    assert strict[:3]==[1,2,0] and sorted(strict)==list(range(10))
    assert len(hints)==1
    assert node._observation_tilt_search_budget['checked_count']==10
    assert node._observation_tilt_search_budget['discovery_checked_count']==10
    assert node._observation_tilt_search_budget['used_sec']==pytest.approx(10.1)
    assert runtime['_observation_qualified_incumbent_ticket'] is None
    report=runtime['observation_orientation_search']
    assert report['checked_variant_count']==10 and report['unchecked_variant_count']==0
    assert report['search_complete'] is True
    assert not any(message.valid for message in node.rich_plan_pub.messages)
    assert base.OBSERVATION_TILT_STRICT_BUDGET_SEC==3.
    assert base.OBSERVATION_TILT_MAX_STRICT_CHECKS==6
    assert base.OBSERVATION_COMPARISON_AFTER_INCUMBENT_SEC==8.


@pytest.mark.parametrize('call_cost,comparison_checks', [(.1,6), (4.,2)])
def test_late_full_qualification_starts_original_comparison_limits_in_real_wrapper(
        configured,monkeypatch,tmp_path,call_cost,comparison_checks):
    node,prepared,runtime,rows,candidate,checker,events,hints,clock=loop_context(
        configured,monkeypatch,tmp_path,count=20,first_failure='following')
    def following(_self,_prepared,result,target=None):
        strict=[value for kind,value in events if kind=='strict']
        clock[0]+=call_cost-.1
        if len(strict)<7:
            return replace(result,reachable=False,
                failure_code='OBSERVATION_FOLLOWING_SUPPORT_INVALID'),{'ok':False}
        return result,{'ok':True}
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_observation_following_support_evaluation',following)
    result=checker(candidate)
    strict=[value for kind,value in events if kind=='strict']
    assert result.reachable and len(hints)==1
    assert strict[0]==1 and len(strict)==len(set(strict))==7+comparison_checks
    assert runtime['_observation_qualified_incumbent_ticket'] is prepared.ticket
    budget=node._observation_tilt_search_budget
    assert budget['discovery_checked_count']==7
    assert budget['comparison_start_count']==7
    assert budget['checked_count']-budget['comparison_start_count']==comparison_checks
    assert budget['comparison_start_used_sec']==pytest.approx(7*call_cost)
    report=runtime['observation_orientation_search']
    assert report['unchecked_variant_count']==20-len(strict)
    assert report['comparison_budget_reached'] is True
    assert budget['used_sec']-budget['comparison_start_used_sec']==pytest.approx(
        comparison_checks*call_cost)
    assert (clock[0]>=node._observation_information_comparison['deadline']) is (call_cost==4.)
    # Neither the late incumbent nor its comparison checks publish a plan.
    assert not any(message.valid for message in node.rich_plan_pub.messages)
