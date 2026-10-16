"""One real producer/remote/task message chain. All I/O is local and inert."""
from copy import deepcopy
from collections import deque
import json
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.unknown_pending_identity import reference_binding
from test_observation_measurement_remote import context, at
from test_observation_measurement_unknown import send, ack, arrival
from test_unknown_pending_identity import publication, deliver
from test_locked_rgb_hold import custom_frame, module
from test_remote_modes import Publisher, direct_observation, base
from test_task_modes import configured
from test_station_compute_wait_task import recorded
from test_station_compute_wait_task import deliver as task_deliver


def owners(monkeypatch):
    c=context.__wrapped__(monkeypatch); p=c.producer; r=c.node
    _,_,plan,_,_=recorded(2)
    plan.plan_id=c.request['view_plan_id'];plan.header.stamp=at(c.request['view_source_stamp_ns'])
    plan.header.frame_id='base_link';plan.diagnostic='FAR_FIELD_OBSERVATION_PLAN'
    plan.model_choice='unknown_tabletop';plan.target_track_id=c.identity.track_id
    plan.object_geometry.target_track_id=c.identity.track_id
    plan.object_geometry.pose_base=deepcopy(c.ready['object'].pose_base.pose)
    r._observe_return_prefix=deepcopy(plan);r._observe_return_origin_prefix=deepcopy(plan)
    c.request['plan_sha256']=committed_plan_digest(plan)
    p._observation_measurement_ack_pub=Mock()
    send(p,c.request);assert ack(p)['accepted']
    r._measurement_remote_request_cb(String(data=json.dumps(c.request)))
    assert json.loads(r._measurement_remote_ack_pub.messages[-1].data)['accepted']
    req=arrival(p,c.clock,c.joint,c.request)
    r._measurement_remote_request_cb(String(data=json.dumps(req)))
    for _ in range(3):
        c.clock[0]+=.2;custom_frame(p,c.clock,c.joint,target_depth=.35)
    row=publication(p);deliver(r,row)
    r._measurement_remote_ack_cb(String(data=json.dumps(ack(p))))
    assert json.loads(r._measurement_remote_ack_pub.messages[-1].data)['qualified']
    c.clock[0]+=.01
    req=dict(req,state='qualified',request_id='initial-qualified',stamp_ns=module.rospy.Time.now().to_nsec(),source_stamps_ns=ack(p)['source_stamps_ns'])
    send(p,req);r._measurement_remote_request_cb(String(data=json.dumps(req)))
    assert ack(p)['accepted']
    phase=NearFieldPlanningPhase(active=True,phase_id=7,planning_stage='contact_replan',reference_center_valid=True,reference_target_track_id=c.identity.track_id)
    phase.header.stamp=module.rospy.Time.now();phase.header.frame_id='base_link'
    phase.reference_source_stamp=at(req['source_stamps_ns'][-1]);phase.deadline=module.rospy.Time.from_sec(c.clock[0]+30.)
    p._station_compute_phase_cb(phase);p._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
    r._observe_return_phase='observing_contact';r._observe_return_paused=False
    r._station_compute_task_state='PLAN_PREGRASP';r.robot_execution_active=True
    r._station_compute_phase=deepcopy(phase);r.near_field_planning_active=True
    r.latest_joint_state=JointState(name=['joint%d'%i for i in range(1,7)],position=[0.]*6)
    r._measurement_remote_reference['binding']=reference_binding(row['status'],r._mode_selection)
    with monkeypatch.context() as init:
        t,_=configured.__wrapped__(init)
    t._mode_selection=deepcopy(r._mode_selection);t._mode_bound_selection=deepcopy(r._mode_selection)
    t.active=True;t.stage=2;t._execution_authority_revoked=False
    t._bound_execution_plan=deepcopy(plan);t._bound_execution_plan_id=plan.plan_id
    t._bound_execution_plan_digest=t._execution_plan_digest(plan);t.latest_grasp6d_plan=deepcopy(plan)
    t._observe_return_session=deepcopy(r._observe_return_session);t._observation_measurement_request=deepcopy(req)
    t._published_near_field_phase=deepcopy(phase);t._near_field_phase_id=phase.phase_id
    t._near_field_active=True;t._near_field_phase_deadline_sec=phase.deadline.to_sec()
    t._following_evidence_lock=threading.RLock();t._following_epoch_ns=req['control_epoch_ns']
    t._following_accepted_history=deque(maxlen=128);t._following_sdk_history=deque(maxlen=128)
    oldjoint=c.joint
    def joint(stamp,q=None):
        values=[0.]*6 if q is None else list(q)
        oldjoint(stamp,values)
        msg=JointState();msg.header.stamp=module.rospy.Time.from_sec(stamp);msg.header.frame_id='sdk_measured'
        msg.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];msg.position=values+[.05]
        r._station_compute_accepted_cb(msg);t.accepted_joint_cb(msg)
    c.joint=joint;c.task=t;c.plan=plan;c.phase=phase
    for ns, values in tuple(p._rgb_interest_feedback):
        msg=JointState();msg.header.stamp=at(ns);msg.header.frame_id='sdk_measured'
        msg.name=['Joint%d'%i for i in range(1,7)]+['right_finger'];msg.position=list(values)+[.05]
        r._station_compute_accepted_cb(msg);t.accepted_joint_cb(msg)
    c.clock[0]+=.2;custom_frame(p,c.clock,c.joint,target_depth=.35)
    row=publication(p);deliver(r,row);task_deliver(NS(node=t),row)
    assert p._station_compute_registration is not None,row['status']
    assert r._station_compute_reference is not None,r.geometry_pub.messages[-1]
    assert t._station_task_record is not None,t._unknown_target_loss
    c.binding=deepcopy(row['status']['station_compute_wait']['binding'])
    c.ready=row
    return c


def test_three_owners_real_drift_measurement_and_new_registered_source(monkeypatch):
    c=owners(monkeypatch);p=c.producer;r=c.node;t=c.task
    q=[0.,0.,0.,0.,0.,-.006135923151542567]
    c.clock[0]+=.1;c.joint(c.clock[0],q)
    p.process(None)
    row=publication(p)
    r._silhouette_source_status_cb(String(data=json.dumps(row['status'])))
    t._unknown_support_status_cb(String(data=json.dumps(row['status'])))
    assert p._station_requalification_hold and r._station_requalification_hold and t._station_requalification_task
    assert p.first_loss is None and t._unknown_target_loss is None
    # Remaining chain is wired below without replacing any measurement gate.
    from test_station_requalification_reached_remote import reached_phase
    reached_phase(c, t._observation_measurement_request)
    r._observe_return_step_gain=dict(information_gain_valid=True,newly_covered_cells=4)
    # Real registration consumes a deterministic measured cloud. RGB-D
    # reconstruction is isolated; all identity, registration and join gates run.
    monkeypatch.setattr(r,'_resolve_reached_view_observation',
        lambda phase: direct_observation(r,phase.reference_source_stamp.to_nsec()))
    monkeypatch.setattr(module.rospy,'is_shutdown',lambda:False)
    def clock_step(seconds):
        c.clock[0]+=seconds;c.joint(c.clock[0],q)
    monkeypatch.setattr(module.rospy,'sleep',clock_step)
    requests=[];errors=[];phases=[];callback_threads=[]
    def producer_ack(message):
        r._measurement_remote_ack_cb(deepcopy(message))
        t._observation_measurement_ack_cb(deepcopy(message))
    p._observation_measurement_ack_pub.publish.side_effect=producer_ack
    prior_remote=r._measurement_remote_ack_pub
    def remote_ack(message):
        prior_remote.messages.append(deepcopy(message))
        t._observation_measurement_ack_cb(deepcopy(message))
    r._measurement_remote_ack_pub=NS(publish=remote_ack)
    def wire(message):
        req=json.loads(message.data);requests.append(req)
        def callbacks():
            try:
                # Both are the actual public measurement request callbacks.
                p._observation_measurement_request_cb(deepcopy(message))
                r._measurement_remote_request_cb(deepcopy(message))
                if req['state']=='requalifying':
                    count=p.pubs['object'].publish.call_count
                    for i in range(3):
                        c.clock[0]+=.2
                        custom_frame(p,c.clock,lambda stamp,values=None:c.joint(stamp,q),target_depth=.35)
                        assert p.pubs['object'].publish.call_count==count+(i==2), publication(p)['status']
                        if i==2:
                            current=publication(p)
                            deliver(r,current)
                            task_deliver(NS(node=t),current)
                            r._observation_measurement_poll()
            except BaseException as exc:
                errors.append(exc)
        worker=threading.Thread(target=callbacks);callback_threads.append(worker)
        worker.start();worker.join(2)
        assert not worker.is_alive(),'measurement callback blocked on a task-held plan/mode lock'
        assert not errors,errors
    t._observation_measurement_pub=NS(publish=wire)
    def phase_wire(phase):
        phases.append(deepcopy(phase))
        p._station_compute_phase_cb(deepcopy(phase))
        r.near_field_state_cb(deepcopy(phase))
    t.near_field_phase_pub=NS(publish=phase_wire)
    c.clock[0]+=.001;c.joint(c.clock[0],q)
    original_deadline=t._station_requalification_task['witness']['deadline_ns']
    # This is the real scheduling consumer, not a direct recovery-helper call.
    outcome, candidate=t._copy_near_field_preview_candidate(c.plan,0,{})
    assert not outcome.ok and candidate is None
    assert [v['state'] for v in requests]==['executing','requalifying','qualified'], (outcome,t._unknown_target_loss,ack(p),prior_remote.messages[-1])
    assert not errors and len(phases)==1
    assert r._latest_registration_evidence.result.ok
    assert not r._observe_return_paused
    assert t._station_requalification_task['finalizing_phase'] is not None
    assert p._station_requalification_hold['finalizing']
    assert r._station_requalification_hold is not None
    # All three retain the recovery deadline until the same real new source.
    c.clock[0]+=.2
    custom_frame(p,c.clock,lambda stamp,values=None:c.joint(stamp,q),target_depth=.35)
    current=publication(p)
    assert current['status']['station_compute_wait']['source_ns']<original_deadline
    deliver(r,current);task_deliver(NS(node=t),current)
    assert p._station_requalification_hold is None
    assert r._station_requalification_hold is None
    assert t._station_requalification_task is None
    contract=current['status']['station_compute_wait']
    assert p._station_compute_registration['contract']==contract
    assert r._station_compute_reference['contract']==contract
    assert t._station_task_record['contract']==contract
    assert t.latest_obj.detected and r.latest_object.detected
    assert not t._execution_authority_revoked and t._unknown_target_loss is None and p.first_loss is None
    assert contract['binding']['phase_id']==phases[0].phase_id
    assert contract['phase_deadline_ns']==c.phase.deadline.to_nsec()
    assert r._current_stream_target_identity()==c.identity
    assert all(not value['motion_authorized'] for value in requests)

    new_ns=contract['source_ns'];old_ns=c.ready['status']['station_compute_wait']['source_ns']
    old_ticket=NS(payload=(NS(stamp_ns=old_ns,sample_stamp_ns=(old_ns,)),))
    new_ticket=NS(payload=(NS(stamp_ns=new_ns,sample_stamp_ns=(new_ns,)),))
    assert r._station_compute_ticket_code(old_ticket)=='TARGET_PROCESSING_PENDING'
    assert r._station_compute_ticket_code(new_ticket) == ''
    old_candidate=deepcopy(c.plan);old_candidate.diagnostic='CONTACT_EXECUTION_PLAN';old_candidate.header.stamp=at(old_ns)
    new_candidate=deepcopy(old_candidate);new_candidate.header.stamp=at(new_ns)
    assert t._station_task_plan_failure(old_candidate).code=='TARGET_PROCESSING_PENDING'
    assert t._station_task_plan_failure(new_candidate) is None
