"""Offline real producer/remote/task protocol with original validators.

The geometry is translated from the sealed GEN11 plan into the synthetic RGBD
fixture scene. Source times/identity are this test's actual producer lineage.
This establishes protocol behavior, never a new collision/path or grasp result.
"""
from copy import deepcopy
from collections import deque
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import base64, gzip, hashlib, io, json, threading, itertools
import numpy as np
import pytest, roslib.message, rospy, yaml
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
from alicia_flexible_grasp_supervisor.msg import NearFieldPlanningPhase, GraspState
from test_task_modes import module as task_module, base as task_base
from test_locked_rgb_hold import locked, custom_frame, latest_status
from test_observation_measurement_unknown import send, ack, arrival, next_frame
from test_unknown_pending_identity import consumer_for, deliver, publication
from test_observation_measurement_remote import request as remote_request
from test_station_compute_work_handoff import configure, prepared_for
from test_remote_modes import Publisher, base as remote_base
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.runner_contract import observed_contact_audit_error, audit_matches_preview
from alicia_grasp_modes.station_contact_admission import KEY

FIXTURE=Path(__file__).parent/'fixtures/gen11_contact_transport.json.gz'
def at(ns):return rospy.Time(int(ns)//10**9,int(ns)%10**9)
def nsnow():return rospy.Time.now().to_nsec()
def xyz(p):return np.array([p.x,p.y,p.z])
def setxyz(p,a):p.x,p.y,p.z=map(float,a)

def original_geometry():
    with gzip.open(FIXTURE,'rt') as f:data=json.load(f)
    plans={}
    for kind,row in data['wire'].items():
        raw=base64.b64decode(row['serialized_base64'])
        assert hashlib.sha256(raw).hexdigest()==row['serialized_sha256']
        plans[kind]=roslib.message.get_message_class(row['msg_type'])().deserialize(raw)
    return plans,data['report']

def adapted_plan(original,source,track,delta):
    plan=deepcopy(original);plan.header.stamp=at(source)
    plan.object_geometry.header=deepcopy(plan.header)
    plan.target_track_id=plan.object_geometry.target_track_id=track
    setxyz(plan.object_geometry.pose_base.position,xyz(plan.object_geometry.pose_base.position)+delta)
    normal=xyz(plan.object_geometry.support_normal_base)
    plan.object_geometry.support_offset_m-=float(normal@delta)
    for pose in plan.poses:setxyz(pose.position,xyz(pose.position)+delta)
    if plan.contact_geometry_evidence:
        evidence=json.loads(plan.contact_geometry_evidence)
        evidence['contact_center_base']=(np.array(evidence['contact_center_base'])+delta).tolist()
        evidence['grasp_tool_pose_base'][:3]=(np.array(evidence['grasp_tool_pose_base'][:3])+delta).tolist()
        plan.contact_geometry_evidence=json.dumps(evidence,sort_keys=True,separators=(',',':'))
    plan.plan_id=task_base.compute_plan_id(plan)
    return plan

def task_for(c,monkeypatch):
    n=task_module.ModeAwareGraspTaskNode.__new__(task_module.ModeAwareGraspTaskNode)
    n._initialize_mode_boundary();n.active=False;n.states=[];n.set_state=lambda *a,**k:n.states.append(a)
    assert n.reset_mode_cb(None).success
    n._mode_bound_selection=deepcopy(n._mode_selection);n.active=True;n.stage=task_base.GraspStages.PLAN_PREGRASP
    n._bound_execution_plan=deepcopy(c.observation);n._bound_execution_plan_id=c.observation.plan_id
    n._bound_execution_plan_digest=n._execution_plan_digest(c.observation)
    n.latest_grasp6d_plan=deepcopy(c.observation);n._execution_authority_revoked=False
    n._observe_return_session=deepcopy(c.session);n._observation_measurement_request=deepcopy(c.measurement)
    n._published_near_field_phase=deepcopy(c.phase);n._near_field_active=True;n._near_field_phase_id=c.phase.phase_id
    n._near_field_phase_deadline_sec=c.phase.deadline.to_sec();n._near_field_phase_started_sec=c.phase.header.stamp.to_sec()
    # This fixture assigns the phase manually; mirror the real setter caches.
    n._near_field_phase_started_ns=c.phase.header.stamp.to_nsec()
    n._near_field_phase_deadline_ns=c.phase.deadline.to_nsec()
    n._following_evidence_lock=threading.RLock();n._following_epoch_ns=c.measurement['control_epoch_ns']
    n._following_accepted_history=deque(maxlen=128);n._following_sdk_history=deque(maxlen=128)
    n._observe_return_origin_proof=dict(epoch_ns=c.measurement['control_epoch_ns'])
    n._contact_station_acks={};n._frozen_contact_pub=Publisher()
    transform=np.array([[0.,-1.,0.,.1],[0.,0.,1.,-.2],[-1.,0.,0.,.4],[0.,0.,0.,1.]])
    def lookup(parent,child,when,*a):
        msg=TransformStamped();msg.header.frame_id=parent;msg.header.stamp=when;msg.child_frame_id=child
        setxyz(msg.transform.translation,transform[:3,3]);q=quaternion_from_matrix(transform)
        msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q)
        return msg
    n.tf_buffer=NS(lookup_transform=lookup)
    return n

def feed_feedback(c):
    # Transport of measured fixture feedback, never a substituted gate.
    for ns,q in c.producer._rgb_interest_feedback:
        if getattr(c,'last_feedback',0)>=ns:continue
        m=JointState();m.header.stamp=at(ns);m.header.frame_id='sdk_measured'
        m.name=['Joint%d'%i for i in range(1,7)];m.position=list(q)
        c.node._station_compute_accepted_cb(m)
        if getattr(c,'task',None) is not None:c.task.accepted_joint_cb(m)
        c.last_feedback=ns
    c.node.latest_joint_state=deepcopy(m if 'm' in locals() else c.node.latest_joint_state)

def deliver_both(c,row,order=('object','mask','status')):
    for key in order:
        deliver(c.node,row,(key,))
        if key=='object':c.task.obj_cb(deepcopy(row[key]))
        elif key=='mask':c.task._station_compute_task_mask_cb(deepcopy(row[key]))
        else:c.task._unknown_support_status_cb(String(data=json.dumps(row[key])))

def measured_row(c,order=('object','mask','status'),step=.4):
    c.clock[0]+=step
    pair=custom_frame(c.producer,c.clock,c.joint,target_depth=.35)
    feed_feedback(c)
    row=publication(c.producer)
    assert row['status']['state']=='ready',row['status']
    c.node.frames.update_color(c.producer.bridge.imgmsg_to_cv2(pair[0],'bgr8'),pair[0].header.stamp,pair[0].header.frame_id)
    c.node.frames.update_depth(c.producer.bridge.imgmsg_to_cv2(pair[1],'32FC1')*1000.,pair[1].header.stamp,pair[1].header.frame_id)
    deliver_both(c,row,order)
    assert c.task._station_task_record is not None
    assert not c.task._station_task_record['pending']
    return row

def context(monkeypatch):
    producer,clock,feed,joint,params=locked(monkeypatch)
    remote=consumer_for(producer,clock,params);remote._ros_source_clock_ns=nsnow
    deliver(remote,publication(producer));identity=remote._current_stream_target_identity()
    plans,report=original_geometry()
    center=xyz(publication(producer)['object'].pose_base.pose.position)
    delta=center-xyz(plans['CONTACT_EXECUTION_PLAN'].object_geometry.pose_base.position)
    observation=adapted_plan(plans['FAR_FIELD_OBSERVATION_PLAN'],producer.last_good_source_stamp_ns,identity.track_id,delta)
    session=dict(version=2,session_id='independent-session',selection=deepcopy(remote._mode_selection),phase='observing',
        target_epoch=identity.epoch,target_track_id=identity.track_id,origin_plan_id=observation.plan_id,
        origin_snapshot_stamp_ns=observation.header.stamp.to_nsec(),view_plan_id=observation.plan_id,
        view_source_stamp_ns=observation.header.stamp.to_nsec(),view_index=2,deadline_ns=nsnow()+60_000_000_000,
        stamp_ns=nsnow(),view_request_id='independent-view-2',previous_gain_sha256='0'*64,max_observations=2)
    req=dict(session,version=1,state='executing',lease_id='independent-lease',request_id='independent-executing',
        plan_sha256=committed_plan_digest(observation),control_epoch_ns=producer._rgb_interest_epoch,
        motion_kind='observation',arrival_stamp_ns=0,capture_deadline_ns=0,metric_geometry_authority=False,motion_authorized=False)
    req.pop('phase')
    remote._observe_return_prefix=deepcopy(observation);remote._observe_return_origin_prefix=deepcopy(observation)
    remote._observe_return_phase='observing';remote._observe_return_paused=True;remote._observe_return_saved=object()
    remote._measurement_remote_ack_pub=Publisher();remote._measurement_remote_epoch_ns=producer._rgb_interest_epoch
    remote._observe_return_session=deepcopy(session)
    producer._observation_measurement_ack_pub=Mock()
    send(producer,req);assert ack(producer)['accepted']
    assert remote_request(remote,req)['accepted']
    req=arrival(producer,clock,joint,req);assert remote_request(remote,req)['accepted']
    for _ in range(3):next_frame(producer,clock,joint)
    final=publication(producer)
    deliver(remote,final)
    remote._measurement_remote_ack_cb(String(data=json.dumps(ack(producer))))
    sources=ack(producer)['source_stamps_ns'];clock[0]+=.01
    measurement=dict(req,state='qualified',request_id='independent-qualified',stamp_ns=nsnow(),source_stamps_ns=sources)
    send(producer,measurement);assert ack(producer)['accepted']
    assert remote_request(remote,measurement)['accepted']
    phase=NearFieldPlanningPhase();phase.active=True;phase.phase_id=2;phase.planning_stage='contact_replan'
    phase.header.stamp=at(nsnow()+1);phase.header.frame_id='base_link';phase.deadline=rospy.Time(int(clock[0])+30)
    phase.reference_source_stamp=at(sources[-1]);phase.reference_center_valid=True;phase.reference_target_track_id=identity.track_id
    phase.reference_label='unknown_small';setxyz(phase.reference_center_base,center)
    producer._station_compute_phase_cb(phase);producer._station_compute_state_cb(GraspState(state='PLAN_PREGRASP',active=True))
    remote._observe_return_phase='observing_contact';remote._observe_return_paused=False;remote._station_compute_task_state='PLAN_PREGRASP'
    remote.robot_execution_active=True;remote._station_compute_phase=deepcopy(phase);remote.near_field_planning_active=True
    remote._observe_return_diagnostic_observation_only=False
    config=yaml.safe_load((Path(task_base.__file__).parents[1]/'config/grasp_params.yaml').read_text())
    params['/grasp']=config['grasp']
    for k,v in config['grasp_6d'].items():params['/grasp_6d/'+k]=v
    params['/gui/joint_direct_mode']=False
    params.update({'/handeye/base_frame':'base_link','/handeye/camera_frame':'camera_link','/perception/projection_frame_convention':'ros_camera_link'})
    c=NS(node=remote,producer=producer,clock=clock,joint=joint,params=params,session=session,measurement=measurement,
        phase=phase,observation=observation,plans=plans,report=report,delta=delta,identity=identity,ready=final)
    c.task=task_for(c,monkeypatch);feed_feedback(c)
    # Configure the original direct-snapshot implementation, then obtain all
    # three source witnesses through actual producer and consumer callbacks.
    configure(c)
    remote._near_field_phase_started_sec=phase.header.stamp.to_sec()
    remote.last_submitted_stamp_ns=phase.header.stamp.to_nsec()
    for _ in range(3):
        c.ready=measured_row(c);remote._poll_direct_snapshot()
    c.ticket=remote._stream_worker_ticket
    assert c.ticket is not None,remote._direct_snapshot_last_rejection
    assert remote._station_compute_work_certificate(c.ticket)
    c.contact=adapted_plan(plans['CONTACT_EXECUTION_PLAN'],c.ticket.payload[0].stamp_ns,identity.track_id,delta)
    c.gcfg=c.task._strategy_config(params['/grasp'])
    assert task_base.validate_execution_plan(c.contact,clock[0],120.).ok
    return c

def test_real_three_source_actor_setup(monkeypatch):
    c=context(monkeypatch)
    assert len(c.ticket.payload[0].sample_stamp_ns)==3
    assert len(c.task._station_contact_sources)==3

def computed(c,order=('object','mask','status')):
    n=c.node;prepared=prepared_for(n,c.ticket)
    assert n._activate_prepared_geometry(prepared)
    legacy=remote_base.PoseArray();legacy.header=deepcopy(c.contact.header)
    n._publish_preview_plan(c.contact,legacy,ticket=c.ticket)
    assert n.preview_rich_plan_pub.messages[-1]==c.contact
    c.task.grasp6d_preview_plan_cb(deepcopy(n.preview_rich_plan_pub.messages[-1]))
    # No producer reports missing geometry. Only clocks progress beyond the
    # current source's unchanged 0.8 s validity while fresh feedback continues.
    c.clock[0]+=.82;c.joint(c.clock[0]);feed_feedback(c)
    n._station_compute_poll();c.task._station_compute_task_tick()
    c.task.target_geometry_cb(deepcopy(n.geometry_pub.messages[-1]))
    c.task.grasp6d_preview_plan_cb(deepcopy(n.preview_rich_plan_pub.messages[-1]))
    assert c.task._station_task_record['pending']
    c.ready=measured_row(c,step=.2,order=order)
    assert n._station_compute_work_certificate(c.ticket)
    receipt=n._station_contact_computation_receipt(c.contact,c.ticket)
    assert receipt is not None
    c.receipt=receipt
    return c

def adapted_report(c):
    report=deepcopy(c.report);source=c.contact.header.stamp.to_nsec()
    report.update(plan_id=c.contact.plan_id,snapshot_stamp_ns=source,
        grasp_mode_selection=deepcopy(c.session['selection']),target_epoch=c.identity.epoch)
    report[KEY]=deepcopy(c.receipt)
    metadata=report['observe_return']
    for key in ('version','session_id','origin_plan_id','target_epoch','target_track_id','view_index',
                'view_plan_id','view_request_id','previous_gain_sha256','origin_snapshot_stamp_ns','deadline_ns','view_source_stamp_ns'):
        metadata[key]=deepcopy(c.session[key])
    metadata['initial_surface_stamp_ns']=c.session['origin_snapshot_stamp_ns']
    metadata['measurement_receipt_sha256']=measurement_digest(c.measurement)
    metadata['qualified_source_stamps_ns']=deepcopy(c.measurement['source_stamps_ns'])
    stamps=[c.session['origin_snapshot_stamp_ns'],c.phase.reference_source_stamp.to_nsec(),source]
    metadata['fused_source_stamps_ns']=stamps
    metadata['reached_view']=dict(phase_id=c.phase.phase_id,start_stamp_ns=c.phase.header.stamp.to_nsec(),
        source_stamp_ns=c.phase.reference_source_stamp.to_nsec(),deadline_ns=c.phase.deadline.to_nsec(),registration_phase_id=c.phase.phase_id)
    surface=report['multiview_surface'];surface.update(target_epoch=c.identity.epoch,target_track_id=c.identity.track_id,
        phase_id=c.phase.phase_id,source_stamps_ns=stamps)
    surface['latest_registration']['source_stamp_ns']=source
    selected=report['selected'];selected['final_strict_plan_id']=c.contact.plan_id
    for stage,pose in zip(selected['execution_sequence']['stages'],c.contact.poses):
        stage['position_m']=xyz(pose.position).tolist()
        stage['quaternion_xyzw']=[getattr(pose.orientation,k) for k in ('x','y','z','w')]
    assert observed_contact_audit_error(report,session=c.session,reached_phase=c.phase,measurement=c.measurement,now_ns=nsnow())==''
    assert audit_matches_preview(report,c.contact,True)
    return report

def audit(c,tmp_path):
    report=adapted_report(c)
    c.node.gate_audit_output_path=str(tmp_path/'contact_audit.json')
    c.node.mujoco_audit_output_path=str(tmp_path/'mujoco_audit.json')
    # Only the geometric report input is a recorded/translated fixture. The
    # original finalizer, fsync/atomic replace, commit callback, proof binding
    # and serialized reference callback run unchanged.
    c.node._build_streaming_gate_audit_report=lambda *a,**kw:(deepcopy(report),{},None)
    c.node.gate_audit_pub=NS(publish=c.task._observe_return_audit_cb)
    _,reference=c.node._finalize_streaming_gate_audit(prepared_for(c.node,c.ticket),None,{},
        'PREVIEW_READY')
    assert reference['atomic_committed']
    assert hashlib.sha256(Path(reference['report_path']).read_bytes()).hexdigest()==reference['report_sha256']
    assert c.task._observe_return_reports[c.contact.plan_id]==report
    return report

def copy_candidate(c):return c.task._copy_near_field_preview_candidate(c.observation,c.phase.header.stamp.to_nsec(),c.gcfg)

def freeze(c):
    result,plan=copy_candidate(c)
    assert result.ok,(result.code,result.reason)
    assert plan==c.contact
    # The original loop owns copy, before-hook, atomic invoke/freeze, after-
    # hook and freeze notice. Only its stream-control transport is inert.
    streams=[]
    c.task._set_near_field_preview_stream=lambda cfg,enabled:streams.append(enabled) or True
    frozen=c.task._maybe_rebind_near_field_grasp6d_plan(c.gcfg,{},c.observation)
    assert frozen is not None,c.task.states
    assert frozen==c.contact
    assert len(c.task._frozen_contact_pub.messages)==1
    assert streams==[True]
    return frozen

def retirement(c,plan,tick_at='before_prepare',move_delay=0.):
    n=c.task;n.stage=task_base.GraspStages.MOVE_PREGRASP
    state=GraspState(state='MOVE_PREGRASP',active=True)
    c.producer._station_compute_state_cb(state);c.node.grasp_state_cb(state)
    if tick_at=='before_prepare':
        with c.node._stream_condition:c.node._station_compute_tick_locked()
    for _ in range(int(round(move_delay/.2))):
        c.clock[0]+=.2
        custom_frame(c.producer,c.clock,c.joint,target_depth=.35)
        feed_feedback(c)
        deliver_both(c,publication(c.producer))
    c.producer._contact_station_ack_pub=Publisher();c.node._contact_station_ack_pub=Publisher()
    c.handshakes=[]
    def publish(msg):
        request=json.loads(msg.data);c.handshakes.append(request)
        for actor in (c.producer,c.node):
            # Both original contexts, callbacks and reply validators run.
            actor._contact_station_request_cb(msg)
            response=actor._contact_station_ack_pub.messages[-1]
            assert json.loads(response.data)['accepted'],(type(actor).__name__,json.loads(response.data).get('reason'))
            n._contact_station_ack_cb(response)
            if tick_at=='between_acks' and actor is c.producer:
                with c.node._stream_condition:c.node._station_compute_tick_locked()
    n._contact_station_pub=NS(publish=publish)
    entered=[];c.motion_sentinel=entered
    executor=n._wrap_pose_submission_executor(task_base.GraspStages.MOVE_PREGRASP,plan,c.gcfg,
        lambda pose,execute:entered.append((pose,execute)))
    validation,response=n._invoke_plan_bound_action(plan,c.gcfg,'contact pregrasp strict submission',
        lambda:executor(plan.poses[0],True))
    assert validation.ok,(validation.code,validation.reason)
    assert entered==[(plan.poses[0],True)]
    assert [r['state'] for r in c.handshakes]==['prepare','retire']
    assert n._contact_station_retired_measurement==measurement_digest(c.measurement)
    return entered

@pytest.mark.parametrize('tick_at',['before_prepare','between_acks'])
@pytest.mark.parametrize('move_delay',[0.,1.2])
def test_three_actual_actors_computation_to_original_contact_action_entry(monkeypatch,tmp_path,tick_at,move_delay):
    c=computed(context(monkeypatch));audit(c,tmp_path)
    original=deepcopy(c.contact)
    plan=freeze(c);retirement(c,plan,tick_at=tick_at,move_delay=move_delay)
    assert c.contact==original
    assert c.task._bound_execution_plan==original

def test_late_audit_never_changes_source_or_authorizes_before_its_receipt(monkeypatch,tmp_path):
    c=computed(context(monkeypatch))
    before=deepcopy(c.task._bound_execution_plan)
    result,plan=copy_candidate(c)
    assert not result.ok and result.code=='TARGET_PROCESSING_PENDING' and plan is None
    assert c.task._bound_execution_plan==before
    audit(c,tmp_path)
    result,plan=copy_candidate(c)
    assert result.ok and plan==c.contact
    assert c.contact.header.stamp.to_nsec()<c.task._station_task_cutoff
    assert c.task._bound_execution_plan==before

@pytest.mark.parametrize('order',list(itertools.permutations(('object','mask','status'))))
def test_recovery_three_connections_permutation_reaches_original_action_once(monkeypatch,tmp_path,order):
    c=computed(context(monkeypatch),order=order);audit(c,tmp_path)
    retirement(c,freeze(c))

@pytest.mark.parametrize('fault',['plan_digest','source_object','source_mask','source_binding','current_witness',
    'future_issued','plan_source','ticket_epoch','authority_flag','missing_source'])
def test_provenance_changes_do_not_replace_original_gates(monkeypatch,tmp_path,fault):
    c=computed(context(monkeypatch));report=audit(c,tmp_path)
    value=deepcopy(report[KEY])
    if fault=='plan_digest':value['plan_sha256']='f'*64
    elif fault=='source_object':value['sources'][0]['object_sha256']='f'*64
    elif fault=='source_mask':value['sources'][0]['mask']['sha256']='f'*64
    elif fault=='source_binding':value['sources'][0]['contract']['binding']['lease_id']='other'
    elif fault=='current_witness':value['current']['object_sha256']='f'*64
    elif fault=='future_issued':value['issued_ns']=nsnow()+1
    elif fault=='plan_source':value['plan_source_ns']+=1
    elif fault=='ticket_epoch':value['ticket']['target_epoch']+=1
    elif fault=='authority_flag':value['motion_authorized']=True
    else:del c.task._station_contact_sources[value['sources'][0]['contract']['source_ns']]
    c.task._observe_return_reports[c.contact.plan_id][KEY]=value
    result,plan=copy_candidate(c)
    assert not result.ok and plan is None
    assert c.task._bound_execution_plan.plan_id==c.observation.plan_id

@pytest.mark.parametrize('fault',['ordinary_invalid','terminal_failure'])
def test_real_invalid_public_preview_cannot_be_hidden_by_private_candidate(monkeypatch,tmp_path,fault):
    c=computed(context(monkeypatch));audit(c,tmp_path)
    invalid=remote_base.Grasp6DPlan();invalid.header.frame_id='base_link';invalid.header.stamp=at(nsnow())
    invalid.valid=False
    if fault=='terminal_failure':
        invalid.candidate_source='near_field_terminal';invalid.diagnostic='NEAR_FIELD_NO_REACHABLE_CANDIDATE: original hard search exhausted'
    else:invalid.diagnostic='TARGET_LOST: actual invalid producer result'
    c.task.grasp6d_preview_plan_cb(invalid)
    assert c.task._station_contact_proposal is None
    result,plan=copy_candidate(c)
    assert not result.ok and plan is None
    assert c.task._bound_execution_plan.plan_id==c.observation.plan_id

def test_stage_label_alone_does_not_grant_old_observation_action(monkeypatch,tmp_path):
    c=computed(context(monkeypatch));audit(c,tmp_path)
    entered=[]
    check,result=c.task._invoke_plan_bound_action(c.observation,c.gcfg,'near-field contact rebind',lambda:entered.append(True))
    assert not check.ok and result is None and not entered

@pytest.mark.parametrize('fault',['replacement','invalid','stop','epoch','manual','phase_deadline'])
def test_before_hook_to_atomic_commit_rechecks_candidate_and_real_authority(monkeypatch,tmp_path,fault):
    c=computed(context(monkeypatch));audit(c,tmp_path)
    assert c.task._before_near_field_contact_freeze(c.contact,c.observation,c.gcfg) is None
    if fault=='replacement':
        replacement=deepcopy(c.contact);replacement.poses[0].position.x+=.001;replacement.plan_id=task_base.compute_plan_id(replacement)
        c.task.grasp6d_preview_plan_cb(replacement)
    elif fault=='invalid':
        invalid=remote_base.Grasp6DPlan();invalid.header.stamp=at(nsnow());invalid.diagnostic='TARGET_LOST: post-hook fault'
        c.task.grasp6d_preview_plan_cb(invalid)
    elif fault=='stop':c.task.active=False;c.task._clear_grasp6d_authority()
    elif fault=='epoch':c.task._following_epoch_ns+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    else:c.clock[0]=c.phase.deadline.to_sec()
    entered=[]
    check,result=c.task._invoke_plan_bound_action(c.observation,c.gcfg,'near-field contact rebind',
        lambda:entered.append(c.task._freeze_execution_plan(c.contact,allow_target_occlusion=True)))
    assert not check.ok and result is None and not entered
    assert c.task._bound_execution_plan is None or c.task._bound_execution_plan.plan_id!=c.contact.plan_id

@pytest.mark.parametrize('fault',['revocation','epoch','manual','target_changed'])
def test_admitted_contact_never_revives_after_actual_bound_guard_failure(monkeypatch,tmp_path,fault):
    c=computed(context(monkeypatch));audit(c,tmp_path);plan=freeze(c)
    if fault=='revocation':c.task._clear_grasp6d_authority()
    elif fault=='epoch':c.task._following_epoch_ns+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    else:
        geometry=deepcopy(plan.object_geometry);geometry.valid=False;geometry.failure_reason='SUPPORT_MEASUREMENT_INVALID: independent true fault'
        geometry.header.stamp=at(nsnow());c.task.target_geometry_cb(geometry)
    entered=[]
    check,result=c.task._invoke_plan_bound_action(plan,c.gcfg,'contact pregrasp strict submission',lambda:entered.append(True))
    assert not check.ok and result is None and not entered


def test_new_partial_in_metadata_wait_lock_gap_waits_without_duplicate_commit(monkeypatch,tmp_path):
    c=computed(context(monkeypatch));audit(c,tmp_path)
    assert c.task._before_near_field_contact_freeze(c.contact,c.observation,c.gcfg) is None
    original=c.task._grasp6d_plan_guard
    gap=threading.Event();resume=threading.Event();done=threading.Event()
    target=[];depth=threading.local();interposed=[];result=[];entered=[]
    @contextmanager
    def guard():
        worker=bool(target and threading.get_ident()==target[0])
        level=getattr(depth,'value',0);depth.value=level+1
        try:
            with original():yield
        finally:depth.value=level
        if worker and level==0 and not interposed:
            interposed.append(True);gap.set()
            assert resume.wait(2),'test callback injection stalled'
    monkeypatch.setattr(c.task,'_grasp6d_plan_guard',guard)
    def run():
        target.append(threading.get_ident())
        try:
            result.append(c.task._invoke_plan_bound_action(c.observation,c.gcfg,'near-field contact rebind',
                lambda:entered.append(c.task._freeze_execution_plan(c.contact,allow_target_occlusion=True))))
        except BaseException as exc:result.append(exc)
        finally:done.set()
    worker=threading.Thread(target=run,daemon=True);worker.start()
    assert gap.wait(2)
    c.clock[0]+=.82;c.joint(c.clock[0]);feed_feedback(c)
    c.task._station_compute_task_tick()
    c.clock[0]+=.2
    custom_frame(c.producer,c.clock,c.joint,target_depth=.35);feed_feedback(c)
    row=publication(c.producer);deliver_both(c,row,('object',))
    assert c.task._station_task_record['pending']
    resume.set()
    assert not done.wait(.04) and not entered
    deliver_both(c,row,('mask','status'))
    assert not c.task._station_task_record['pending'],(row['status'].get('state'),row['status'].get('detail'),
        row['object'].header.stamp.to_nsec(),c.task._station_task_record['contract']['source_ns'])
    assert c.task._station_task_plan_failure(c.contact) is None,c.task._station_task_plan_failure(c.contact)
    assert done.wait(2);worker.join()
    assert len(result)==1 and not isinstance(result[0],BaseException),result
    check,value=result[0]
    assert check.ok and value is None and entered==[c.contact]


@pytest.mark.parametrize('fault',['manual','epoch','deadline','remote_pending'])
def test_retirement_real_context_failure_never_reaches_action(monkeypatch,tmp_path,fault):
    c=computed(context(monkeypatch));audit(c,tmp_path);plan=freeze(c)
    if fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='epoch':c.producer._rgb_interest_epoch+=1
    elif fault=='deadline':
        c.clock[0]=c.phase.deadline.to_sec();c.joint(c.clock[0]);feed_feedback(c)
    else:c.node._station_compute_pending=True
    with pytest.raises((AssertionError,RuntimeError,ValueError)):
        retirement(c,plan)
    assert not getattr(c,'motion_sentinel',[])
