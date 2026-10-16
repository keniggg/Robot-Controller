"""Real RGB-D callbacks/ticket/preview/receipt; no ROS graph or robot action.

The fixed synthetic pixels and contact geometry test provenance, not physical
grasp quality. Source times are never replaced by the current measurement time.
"""
from copy import deepcopy
import itertools
import hashlib
import json
import threading
from types import SimpleNamespace as NS

import pytest
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station, clock, feedback, row_at
from test_station_compute_work_handoff import submitted, prepared_for, next_row, begin_partial
from test_unknown_pending_identity import deliver
from test_remote_modes import base, module
from alicia_flexible_grasp.grasp.contact_evidence import encode_contact_evidence
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_grasp_modes.station_contact_admission import KEY, validate_receipt
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_grasp_modes.contact_station_retirement import ack
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from test_remote_modes import Publisher


def contact_plan(n, ticket):
    plan = base.Grasp6DPlan()
    plan.valid = True
    plan.header.stamp = at(ticket.payload[0].stamp_ns)
    plan.header.frame_id = 'base_link'
    plan.object_geometry.header = deepcopy(plan.header)
    plan.diagnostic = 'CONTACT_EXECUTION_PLAN'
    plan.model_choice = 'unknown_tabletop'
    plan.refinement_status = 'VALID_3D'
    plan.refinement_inlier_count = 100
    plan.fused_view_count = 3
    plan.refinement_overlap_fraction = .95
    plan.refinement_rmse_m = .001
    plan.refinement_translation_m = .001
    plan.refinement_rotation_deg = .1
    plan.target_track_id = n._current_stream_target_identity().track_id
    plan.candidate_source = 'tabletop_geometry'
    plan.candidate_source_lineage = ['tabletop_geometry']
    plan.required_open_width_m = .044
    geometry = plan.object_geometry
    geometry.valid = True
    geometry.target_track_id = plan.target_track_id
    geometry.source_mode = 'instance_mask'
    geometry.pose_base.orientation.w = 1.
    geometry.pose_base.position.z = .01
    geometry.size_xyz_m.x = .04
    geometry.size_xyz_m.y = .03
    geometry.size_xyz_m.z = .02
    geometry.support_normal_base.z = 1.
    plan.poses = [Pose() for _ in range(4)]
    for pose in plan.poses:
        pose.orientation.w = 1.
    plan.contact_geometry_evidence = encode_contact_evidence(
        contact_center_base=[0., 0., .01], contact_height_axis_base=[0., 0., 1.],
        contact_height_bounds_m=[-.005, .005], minimum_contact_overlap_m=.001,
        grasp_pose=plan.poses[2])
    plan.plan_id = compute_plan_id(plan)
    return plan


def published_contact(c):
    n = c.node
    t, row, ticket = submitted(c)
    prepared = prepared_for(n, ticket)
    assert n._activate_prepared_geometry(prepared)
    plan = contact_plan(n, ticket)
    assert n._station_contact_preview_work(plan, ticket)
    legacy = base.PoseArray(); legacy.header = deepcopy(plan.header)
    n._publish_preview_plan(plan, legacy, ticket=ticket)
    assert n.preview_rich_plan_pub.messages[-1] == plan
    assert not n._station_compute_publication_started
    assert not n._station_compute_work_published
    return t, row, ticket, prepared, plan


@pytest.mark.parametrize('order', list(itertools.permutations(('object', 'mask', 'status'))))
def test_preview_clock_wait_then_fresh_receipt_keeps_original_three_sources(station, order):
    c = station; n = c.node
    t, row, ticket, prepared, plan = published_contact(c)
    original = deepcopy(plan)
    for i in range(3, 6):
        row = next_row(c, t+i*400000000, row['status']['station_compute_wait'], order)
    receipt = n._station_contact_computation_receipt(plan, ticket)
    assert validate_receipt(receipt, plan, now_ns=n._ros_source_clock_ns()) == c.binding
    assert [r['contract']['source_ns'] for r in receipt['sources']] == [t,t+400000000,t+800000000]
    assert receipt['current']['contract']['source_ns'] == t+2000000000
    assert receipt['plan_source_ns'] == t+800000000
    assert plan == original
    assert receipt['motion_authorized'] is False
    assert receipt['metric_geometry_authority'] is False
    assert not n._station_compute_publication_started
    assert n._station_compute_work_certificate(ticket)


def test_receipt_wait_releases_callback_locks_until_actual_complete_join(station, monkeypatch):
    c = station; n = c.node
    t,row,ticket,prepared,plan = published_contact(c)
    new = begin_partial(c,t,row)
    entered = threading.Event(); done = threading.Event(); result = []
    original_wait = n._stream_condition.wait
    def wait(*a, **kw):
        entered.set(); return original_wait(*a, **kw)
    monkeypatch.setattr(n._stream_condition, 'wait', wait)
    def run():
        try: result.append(n._station_contact_computation_receipt(plan,ticket))
        except Exception as exc: result.append(exc)
        finally: done.set()
    thread = threading.Thread(target=run,daemon=True);thread.start()
    assert entered.wait(2) and not done.is_set()
    deliver(n,new,('object','status'))
    assert done.wait(2);thread.join()
    assert len(result)==1 and isinstance(result[0],dict), result
    assert result[0]['current']['contract']['source_ns']==new['status']['source_stamp_ns']


@pytest.mark.parametrize('fault', ['false','explicit_pending','epoch','manual','moved','anchor'])
def test_private_contact_preview_never_survives_real_invalidity(station, fault):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    ns=t+1200000000
    feedback(c,ns-10000000);feedback(c,ns+10000000);clock(c,ns+420000000)
    new=row_at(c,ns,recovery=row['status']['station_compute_wait'])
    if fault=='false':new['object'].detected=False
    elif fault=='explicit_pending':
        new['status']['state']='processing_deadline_pending'
        new['status']['station_compute_wait']=row['status']['station_compute_wait']
    elif fault=='epoch':n._measurement_remote_epoch_ns+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='moved':feedback(c,n._ros_source_clock_ns(),q=[.02,0,0,0,0,0])
    else:new['status']['anchor_position_base'][0]+=.01
    deliver(n,new)
    assert not n._station_compute_work_certificate(ticket)
    assert n._station_contact_computation_receipt(plan,ticket) is None


@pytest.mark.parametrize('fault', ['object','mask','status'])
def test_original_witness_contradiction_after_history_eviction_revokes_work(station,fault):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    old_object=deepcopy(n._station_compute_objects[t])
    old_mask=deepcopy(n._station_compute_masks[t][2])
    old_status=deepcopy(n._station_compute_status[t])
    next_row(c,t+1200000000,row['status']['station_compute_wait'])
    n._station_compute_ready_history.pop(t)
    n._station_compute_objects.pop(t,None);n._station_compute_masks.pop(t,None)
    n._station_compute_status.pop(t,None)
    if fault=='object':old_object.detected=False;n.object_cb(old_object)
    elif fault=='mask':old_mask.data=bytes(len(old_mask.data));n.mask_cb(old_mask)
    else:
        old_status['state']='processing_deadline_pending'
        n._silhouette_source_status_cb(String(data=json.dumps(old_status)))
    assert not n._station_compute_work_certificate(ticket)
    assert n._station_contact_computation_receipt(plan,ticket) is None


@pytest.mark.parametrize('change', ['source','geometry_source','target','model','diagnostic','evidence'])
def test_only_exact_registered_contact_preview_has_private_lifetime(station,change):
    c=station;n=c.node;t,row,ticket=submitted(c)
    plan=contact_plan(n,ticket)
    if change=='source':plan.header.stamp=at(plan.header.stamp.to_nsec()+1)
    elif change=='geometry_source':plan.object_geometry.header.stamp=at(1)
    elif change=='target':plan.target_track_id+='other'
    elif change=='model':plan.model_choice='carton'
    elif change=='diagnostic':plan.diagnostic='CONTACT_PLAN'
    else:plan.contact_geometry_evidence=''
    assert not n._station_contact_preview_work(plan,ticket)
    assert n._station_contact_computation_receipt(plan,ticket) is None


def test_original_audit_builder_binds_receipt_to_selected_preview(station):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    n._latest_streaming_audit={'plan_id':plan.plan_id,'request_id':ticket.request_id}
    selection=NS(selected=NS(track_id=1,variant_index=0),checked=(),reachable=())
    report,_,_=n._build_streaming_gate_audit_report(prepared,selection,{},'PREVIEW_READY',
        base_report={'rows':[]})
    assert report['plan_id']==plan.plan_id
    assert report[KEY]['plan_sha256']
    assert report[KEY]['plan_source_ns']==plan.header.stamp.to_nsec()
    assert report['snapshot_stamp_ns']==plan.header.stamp.to_nsec()
    assert validate_receipt(report[KEY],plan,now_ns=n._ros_source_clock_ns())==c.binding
    n._latest_streaming_audit['plan_id']='different-plan'
    report,_,_=n._build_streaming_gate_audit_report(prepared,selection,{},'PREVIEW_READY',
        base_report={'rows':[]})
    assert KEY not in report


def test_clock_only_notice_has_exact_owner_but_terminal_clears_private_work(station):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    new=begin_partial(c,t,row)
    notice=n.latest_preview_rich_plan
    assert not notice.valid
    code,payload=notice.diagnostic.split(': ',1)
    payload=json.loads(payload)
    assert code=='TARGET_PROCESSING_PENDING' and payload['clock_only'] is True
    assert payload['station_compute_wait']==row['status']['station_compute_wait']
    assert notice.target_track_id==plan.target_track_id
    assert n._station_contact_retained_proposal(ticket)==plan
    terminal=base.Grasp6DPlan();terminal.diagnostic='REAL_TERMINAL_FAILURE'
    legacy=base.PoseArray()
    n._publish_preview_plan(terminal,legacy)
    assert n._station_contact_retained_proposal(ticket) is None
    assert not n._station_compute_work_certificate(ticket)


def test_actual_audit_wait_and_disk_commit_do_not_hold_input_locks(station,monkeypatch,tmp_path):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    n._latest_streaming_audit={'plan_id':plan.plan_id,'request_id':ticket.request_id}
    n.gate_audit_output_path=str(tmp_path/'discardable_fixture_audit.json')
    n.mujoco_audit_output_path=str(tmp_path/'discardable_fixture_simulation.json')
    selection=NS(selected=NS(track_id=1,variant_index=0),checked=(),reachable=())
    new=begin_partial(c,t,row)
    entered=threading.Event();done=threading.Event();result=[];lock_probes=[]
    wait=n._stream_condition.wait
    def tracked_wait(*a,**kw):entered.set();return wait(*a,**kw)
    monkeypatch.setattr(n._stream_condition,'wait',tracked_wait)
    original_fsync=base.os.fsync
    def fsync(fd):
        acquired=threading.Event()
        def acquire():
            with n._stream_condition,n._geometry_state_guard():acquired.set()
        worker=threading.Thread(target=acquire,daemon=True);worker.start()
        assert acquired.wait(2),'audit fsync retained an input lock'
        worker.join();lock_probes.append(True)
        return original_fsync(fd)
    monkeypatch.setattr(base.os,'fsync',fsync)
    def build():
        try:
            with n._mode_work_lock:
                result.append(n._finalize_streaming_gate_audit(prepared,selection,{},
                    'PREVIEW_READY',base_report={'rows':[]},publish_reference=False))
        except Exception as exc:result.append(exc)
        finally:done.set()
    worker=threading.Thread(target=build,daemon=True);worker.start()
    assert entered.wait(2) and not done.is_set()
    deliver(n,new,('object','status'))
    assert done.wait(3);worker.join()
    assert len(result)==1 and isinstance(result[0],tuple),result
    report,reference=result[0]
    assert report['plan_id']==plan.plan_id
    assert report[KEY]['plan_source_ns']==plan.header.stamp.to_nsec()
    assert report[KEY]['current']['contract']['source_ns']==new['status']['source_stamp_ns']
    payload=(tmp_path/'discardable_fixture_audit.json').read_bytes()
    assert reference['report_sha256']==hashlib.sha256(payload).hexdigest()
    assert json.loads(payload)[KEY]==report[KEY]
    assert lock_probes==[True]


@pytest.mark.parametrize('fault',['manual','epoch','feedback_stale','phase','clock_back','deadline'])
def test_preview_receipt_wait_keeps_original_owner_and_deadline_cancellation(station,monkeypatch,fault):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    begin_partial(c,t,row)
    entered=threading.Event();done=threading.Event();result=[]
    wait=n._stream_condition.wait
    def tracked(*a,**kw):entered.set();return wait(*a,**kw)
    monkeypatch.setattr(n._stream_condition,'wait',tracked)
    def run():
        try:result.append(n._station_contact_computation_receipt(plan,ticket))
        except Exception as exc:result.append(exc)
        finally:done.set()
    worker=threading.Thread(target=run,daemon=True);worker.start()
    assert entered.wait(2)
    if fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='epoch':n._measurement_remote_epoch_ns+=1
    elif fault=='feedback_stale':c.clock[0]+=.81
    elif fault=='phase':n._station_compute_task_state='MOVE_PREGRASP'
    elif fault=='clock_back':c.clock[0]-=.2
    else:c.clock[0]=n._station_compute_reference['contract']['identity_deadline_ns']/1e9+.001
    with n._stream_condition:n._stream_condition.notify_all()
    assert done.wait(2);worker.join()
    assert len(result)==1 and isinstance(result[0],base.StreamResultCancelled),result


def committed_contact(c,tmp_path):
    n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    n._latest_streaming_audit={'plan_id':plan.plan_id,'request_id':ticket.request_id}
    n.gate_audit_output_path=str(tmp_path/'discardable_contact_audit.json')
    n.mujoco_audit_output_path=str(tmp_path/'discardable_contact_simulation.json')
    n.gate_audit_pub=Publisher()
    new=begin_partial(c,t,row)
    deliver(n,new,('object','status'))
    assert not n.latest_preview_rich_plan.valid
    selection=NS(selected=NS(track_id=1,variant_index=0),checked=(),reachable=())
    report,reference=n._finalize_streaming_gate_audit(prepared,selection,{},
        'PREVIEW_READY',base_report={'rows':[]})
    proof=n._station_compute_work[ticket.request_id]
    assert proof['contact_published'] is True
    assert proof['contact_audit']['reference']==reference
    published=json.loads(n.gate_audit_pub.messages[-1].data)
    assert published['report_sha256']==reference['report_sha256']
    now=n._ros_source_clock_ns();m=n._measurement_remote_request
    req=dict(version=1,state='prepare',request_id='contact:retire',
        selection=deepcopy(n._mode_selection),session_id=m['session_id'],
        target_track_id=m['target_track_id'],target_epoch=m['target_epoch'],
        control_epoch_ns=m['control_epoch_ns'],measurement_sha256=measurement_digest(m),
        plan_id=plan.plan_id,plan_sha256=committed_plan_digest(plan),
        plan_source_ns=plan.header.stamp.to_nsec(),phase_id=c.phase.phase_id,
        phase_started_ns=c.phase.header.stamp.to_nsec(),stage='MOVE_PREGRASP',
        stamp_ns=now,deadline_ns=now+800000000,metric_geometry_authority=False,
        motion_authorized=False,controller_motion_confirmed=False)
    n._contact_station_ack_pub=Publisher()
    n.grasp_state_cb(NS(state='MOVE_PREGRASP',active=True))
    with n._stream_condition:n._station_compute_tick_locked()
    assert n._station_compute_reference is None
    return ticket,plan,proof,req


@pytest.mark.parametrize('fault',[None,'no_atomic_audit','not_published','digest','source',
    'epoch','phase','manual','proof_canceled',
    'audit_reference_changed','source_contradiction'])
def test_actual_remote_retire_after_move_timer_requires_published_and_atomic_bound_proof(station,tmp_path,fault):
    c=station;n=c.node;ticket,plan,proof,req=committed_contact(c,tmp_path)
    if fault=='no_atomic_audit':proof.pop('contact_audit')
    elif fault=='not_published':proof['contact_published']=False
    elif fault=='digest':req['plan_sha256']='d'*64
    elif fault=='source':req['plan_source_ns']-=1
    elif fault=='epoch':n._measurement_remote_epoch_ns+=1
    elif fault=='phase':n._station_compute_phase.phase_id+=1
    elif fault=='manual':c.params['/gui/joint_direct_mode']=True
    elif fault=='proof_canceled':n._station_compute_work.clear()
    elif fault=='audit_reference_changed':n._latest_gate_audit_reference['report_sha256']='e'*64
    elif fault=='source_contradiction':
        ns=proof['source_stamps_ns'][0]
        msg=deepcopy(n._station_compute_objects[ns]);msg.detected=False;n.object_cb(msg)
    n._contact_station_request_cb(String(data=json.dumps(req)))
    response=json.loads(n._contact_station_ack_pub.messages[-1].data)
    assert response['accepted'] is (fault is None),response
    if fault is None:
        # The remote validates the original two prepared ACK schema; the
        # independent three-actor test supplies the real producer ACK.
        retire=dict(req,state='retire',prepared_acks={'remote':response,'unknown':ack(req,'unknown')})
        n._contact_station_request_cb(String(data=json.dumps(retire)))
        assert json.loads(n._contact_station_ack_pub.messages[-1].data)['accepted']
        assert not n._station_compute_work
        assert n._contact_station_retired_measurement==req['measurement_sha256']


def test_move_pregrasp_planning_duration_does_not_reuse_expired_station_geometry(station,tmp_path):
    c=station;n=c.node;ticket,plan,proof,req=committed_contact(c,tmp_path)
    old_source=max(n._station_compute_ready_history)
    for _ in range(4):
        clock(c,n._ros_source_clock_ns()+400000000)
        msg=row_at(c,n._ros_source_clock_ns()-10000000)
        deliver(n,msg)
    assert n._ros_source_clock_ns()-old_source>800000000
    assert not n._station_compute_reference
    now=n._ros_source_clock_ns()
    req.update(stamp_ns=now,deadline_ns=now+800000000)
    n._contact_station_request_cb(String(data=json.dumps(req)))
    response=json.loads(n._contact_station_ack_pub.messages[-1].data)
    assert response['accepted'],response
    assert response['metric_geometry_authority'] is False
    assert response['motion_authorized'] is False


def test_failed_original_atomic_audit_never_creates_retirement_proof(station,monkeypatch,tmp_path):
    c=station;n=c.node;t,row,ticket,prepared,plan=published_contact(c)
    n._latest_streaming_audit={'plan_id':plan.plan_id,'request_id':ticket.request_id}
    n.gate_audit_output_path=str(tmp_path/'discardable_failed_contact_audit.json')
    n.mujoco_audit_output_path=str(tmp_path/'discardable_failed_contact_simulation.json')
    def failed(*args):raise OSError('injected atomic replace failure')
    monkeypatch.setattr(base.os,'replace',failed)
    selection=NS(selected=NS(track_id=1,variant_index=0),checked=(),reachable=())
    with pytest.raises(base.CandidateContractError):
        n._finalize_streaming_gate_audit(prepared,selection,{},'PREVIEW_READY',
            base_report={'rows':[]},publish_reference=False)
    assert 'contact_audit' not in n._station_compute_work[ticket.request_id]
