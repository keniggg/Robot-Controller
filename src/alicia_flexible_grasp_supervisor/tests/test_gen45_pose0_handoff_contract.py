"""Exact G44 source numbers, fresh no-motion handoff, unchanged response gates.

The plan and encoder wires are historical. The service transport and liveness
providers below are offline; replay clocks do not alter any source timestamp.
"""
import base64
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
import rospy
from tf.transformations import quaternion_from_matrix, rotation_matrix

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp.grasp.contact_execution import contact_stage_next_path_contract
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id, plan_id_matches_content
from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q
from test_pregrasp_feedforward import held_gateway_case

FLAGS = ('unknown_approach_held_axes_feedforward_enabled',
    'unknown_contact_cartesian_compensation_enabled', 'contact_endpoint_precision_enabled',
    'measured_endpoint_check_enabled', 'pregrasp_cartesian_compensation_enabled',
    'unknown_pregrasp_feedforward_enabled', 'unknown_pregrasp_held_axes_feedforward_enabled')
FIXTURE_SHA = '40349dcb9d3e245b13211c2a13880fc1cf9389048b5814db56495f296fb2fe7c'


@pytest.fixture(autouse=True)
def no_external_boundaries(monkeypatch):
    import socket
    class BoundaryError(BaseException): pass
    def reject(*a, **k): raise BoundaryError('GEN45 forbids external boundary '+repr(a))
    for name in ('connect', 'connect_ex', 'bind', 'listen', 'sendto'):
        monkeypatch.setattr(socket.socket, name, reject)
    monkeypatch.setattr(socket, 'create_connection', reject)
    for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy', 'wait_for_service'):
        monkeypatch.setattr(rospy, name, reject)


def original():
    p=Path(__file__).parent/'fixtures/gen45_g44_pregrasp.json'
    assert hashlib.sha256(p.read_bytes()).hexdigest()==FIXTURE_SHA
    d=json.loads(p.read_text()); raw=base64.b64decode(d['plan_wire']['wire_base64'])
    assert hashlib.sha256(raw).hexdigest()==d['plan_wire']['wire_sha256']
    plan=Grasp6DPlan();plan.deserialize(raw)
    assert plan_id_matches_content(plan) and plan.plan_id==d['plan_id']
    work=Path(__file__).resolve().parents[3];xml=(work/d['robot_model_path']).read_text()
    assert hashlib.sha256(xml.encode()).hexdigest()==d['model_sha256']
    return NS(data=d,plan=plan,xml=xml,fk=SerialUrdfFk(xml,ARM_NAMES))


def g44_gateway(monkeypatch):
    c=original();node,planner,_,state,calls,params,_=held_gateway_case(monkeypatch)
    first,second,after=deepcopy(c.data['recomputed_original_snapshots'])
    state.update(sdk=np.asarray(first['sdk_positions_rad']),actual=np.asarray(first['accepted_positions_rad']),now=first['stamp_sec'])
    cfg=deepcopy(c.data['config'])
    params.update({'/grasp':cfg,'/robot_description':c.xml,
        '/robot/observation_tracking_contract_enabled':True,
        '/alicia_d_driver_node/endpoint_feedback_trim_enabled':False,
        '/grasp_6d/remote/gripper_geometry':{}})
    context=dict(plan=deepcopy(c.plan),scene=FrozenObservationScene.from_contact_plan(c.plan),
        epoch=first['epoch_ns'],revocation=3,config=cfg,model=c.xml,pose_index=0,
        deadline=time.monotonic()+60.)
    node._observation_context_lock=threading.RLock()
    del node._validate_pregrasp_context
    authority=dict(manual=False)
    node._manual_control_active=lambda:authority['manual']
    node._require_live_observation_task=Mock()
    node._fresh_actuation_status=lambda:'CONFIRMED: explicit replay provider'
    node._control_reference_epoch_ns=first['epoch_ns']
    node._observation_contexts=NS(revocation=3)
    from alicia_flexible_grasp.grasp.grasp_state_machine import GraspStages
    node._pregrasp_task_stage=GraspStages.MOVE_PREGRASP
    node._pregrasp_context=lambda plan,index=0:context
    captures=[]
    def snapshot(*a,**kw):
        kw['context_validator']()
        row=deepcopy(first if not captures else second)
        state['now']=row['stamp_sec'];captures.append(row)
        return row
    node._endpoint_following_snapshot=snapshot
    return NS(c=c,node=node,planner=planner,state=state,calls=calls,params=params,
        context=context,authority=authority,first=first,second=second,after=after,captures=captures)


@pytest.mark.parametrize('execute',[False,True])
def test_real_g44_hard_met_start_hands_off_fresh_without_guide_or_controller(monkeypatch,execute):
    r=g44_gateway(monkeypatch)
    guide=Mock(side_effect=AssertionError('unnecessary guide'))
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',guide)
    r.planner.plan_and_execute_joint_probe=Mock(side_effect=AssertionError('unnecessary motion'))
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=execute))
    d=json.loads(response.message)
    assert response.success,d
    assert d['code']=='PREGRASP_MEASURED_CONVERGED'
    assert d['position_error_m']==pytest.approx(.009284625450334698,abs=1e-14)
    assert d['precision_preference_outcome']=='already_hard_converged_next_fresh_path_required'
    assert d['required_next_path_contract']=='fresh_held_axes_approach'
    assert d['steps_completed']==0 and d['no_motion_submitted'] is True
    assert not d['future_path_checked'] and not d['execution_authorized']
    assert len(r.captures)==2
    for k in ('sdk_stamp_ns','accepted_stamp_ns'):
        assert d[k]==r.second[k]>d['initial_'+k]==r.first[k]
    guide.assert_not_called();r.planner.plan_and_execute_joint_probe.assert_not_called()
    r.node._ensure_trajectory_controllers_started.assert_not_called()
    r.node._synchronize_trajectory_controller_to_feedback.assert_not_called()
    assert not r.calls and r.state['moves']==0
    assert not any(k.startswith('/supervisor/pregrasp_compensation_episodes/')for k in r.params)


@pytest.mark.parametrize('flag',FLAGS)
@pytest.mark.parametrize('value',[False,'true',None])
def test_shared_task_and_gateway_route_requires_each_exact_true_flag(flag,value):
    from test_contact_execution import module as task
    cfg=deepcopy(original().data['config'])
    if value is None:cfg.pop(flag)
    else:cfg[flag]=value
    for index in (0,1):
        assert contact_stage_next_path_contract(cfg,index) is None
        assert task.GraspTaskNode._contact_stage_next_path_contract(cfg,index) is None
    assert contact_stage_next_path_contract(original().data['config'],2) is None


@pytest.mark.parametrize('fault',['stale','owner','epoch','model','reference','moving'])
def test_no_motion_handoff_still_rejects_changed_fresh_contract(monkeypatch,fault):
    r=g44_gateway(monkeypatch);original_snapshot=r.node._endpoint_following_snapshot
    def changed(*a,**kw):
        row=original_snapshot(*a,**kw)
        if len(r.captures)==2:
            if fault=='stale':row['accepted_stamp_ns']-=600000000
            elif fault=='owner':r.authority['manual']=True
            elif fault=='epoch':row['epoch_ns']+=1
            elif fault=='model':r.params['/robot_description']+='\n'
            elif fault=='reference':r.state['sdk']=r.state['sdk'].copy();r.state['sdk'][0]+=Q
            elif fault=='moving':row['accepted_positions_rad'][0]+=2*Q
        return row
    r.node._endpoint_following_snapshot=changed
    r.planner.plan_and_execute_joint_probe=Mock(side_effect=AssertionError('failed handoff cannot move'))
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=True));d=json.loads(response.message)
    assert not response.success,d
    assert d['code']!='PREGRASP_MEASURED_CONVERGED'
    r.planner.plan_and_execute_joint_probe.assert_not_called()
    r.node._ensure_trajectory_controllers_started.assert_not_called()
    assert not any(k.startswith('/supervisor/pregrasp_compensation_episodes/')for k in r.params)


def test_above_hard_pose0_range_still_requires_original_guide(monkeypatch):
    r=g44_gateway(monkeypatch);pose=r.c.plan.poses[0];T=r.c.fk(r.first['accepted_positions_rad'])
    target=np.asarray([pose.position.x,pose.position.y,pose.position.z]);direction=(target-T[:3,3])/np.linalg.norm(target-T[:3,3])
    target+=.001*direction;pose.position.x,pose.position.y,pose.position.z=target
    r.c.plan.plan_id=compute_plan_id(r.c.plan);r.context['plan']=deepcopy(r.c.plan)
    r.context['scene']=FrozenObservationScene.from_contact_plan(r.c.plan)
    guide=Mock(side_effect=ValueError('GEN45_ORIGINAL_GUIDE_REQUIRED'))
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',guide)
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=True))
    assert not response.success and 'GEN45_ORIGINAL_GUIDE_REQUIRED' in response.message
    guide.assert_called_once()
    assert not r.calls


def test_excess_orientation_remains_outside_local_capture(monkeypatch):
    r=g44_gateway(monkeypatch);T=r.c.fk(r.first['accepted_positions_rad'])
    goal=T@rotation_matrix(math.radians(6),[1,0,0]);q=quaternion_from_matrix(goal)
    p=r.c.plan.poses[0];p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
    r.c.plan.plan_id=compute_plan_id(r.c.plan);r.context['plan']=deepcopy(r.c.plan)
    r.context['scene']=FrozenObservationScene.from_contact_plan(r.c.plan)
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=True))
    assert not response.success and 'OUTSIDE_LOCAL_CAPTURE_RANGE' in response.message
    assert not r.calls


def test_incomplete_route_keeps_original_preference_attempt(monkeypatch):
    r=g44_gateway(monkeypatch);r.context['config'].pop('unknown_approach_held_axes_feedforward_enabled')
    guide=Mock(side_effect=ValueError('GEN45_LEGACY_OPTIONAL_GUIDE'))
    monkeypatch.setattr(PregraspFeedforward,'propose_held_axes',guide)
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=True))
    assert not response.success and 'GEN45_LEGACY_OPTIONAL_GUIDE' in response.message
    guide.assert_called_once();assert not r.calls


def test_consumed_historical_episode_cannot_be_reclassified_as_new_noop(monkeypatch):
    r=g44_gateway(monkeypatch)
    identity='%s:%s'%(r.context['epoch'],r.c.plan.plan_id)
    key='/supervisor/pregrasp_compensation_episodes/'+hashlib.sha256(identity.encode()).hexdigest()
    r.params[key]=dict(state='consumed_before_submission',steps_submitted=1)
    response=r.node.handle_compensate_pregrasp(NS(plan=r.c.plan,execute=True))
    assert not response.success and 'EPISODE_ALREADY_CONSUMED' in response.message
    assert not r.captures and not r.calls


def test_actual_submitted_no_progress_remains_failure_in_original_evaluator():
    c=original();first,_,after=deepcopy(c.data['recomputed_original_snapshots'])
    recovery=PregraspFeedforward(c.fk,first,c.plan.poses[0],position_tolerance_m=.010)
    step,report=recovery.propose_held_axes()
    assert report['proposed_delta_counts']==c.data['original_proposal']['proposed_delta_counts']
    # Only the logical completion boundary is supplied by the replay provider;
    # all actual encoder positions, source stamps and epochs remain unaltered.
    recovery.committed(step,after['stationary_window_start_ns']*1e-9-.01)
    ok,result=recovery.evaluate(after)
    assert not ok and result['code']=='PREGRASP_NO_CARTESIAN_PROGRESS'
    assert result['hardware_response_verified'] and result['steps_completed']==1
    assert result['position_error_m']==pytest.approx(.009541368427238467,abs=1e-14)
    assert not result['real_grasp_success']
