"""Real five-source fuser/submission/station admission; no ROS or hardware.

Synthetic fixed-scene pixels reuse recorded GEN3 source spacing. Private
recorded raw snapshot contents were not retained and are not fabricated.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import pytest
from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_observation_measurement_remote import context
from test_station_compute_work_handoff import configure, rgbd, next_row, prepared_for
from test_unknown_pending_identity import deliver
from test_remote_modes import base

RECORDED_SOURCES = (1791680821221001386,1791680821555781841,
    1791680821951685428,1791680822619601249,1791680823222527027)


def observation_station(c):
    n=c.node;t=configure(c)
    n.near_field_planning_active=False
    n._observe_return_phase='observation_prefix'
    n._observe_return_trigger='BILATERAL_SURFACE_EVIDENCE_MISSING'
    n._observe_return_pending_supplement=dict(session_id=c.binding['session_id'],
        target_epoch=c.binding['target_epoch'],target_track_id=c.binding['target_track_id'],
        previous_view_index=c.binding['view_index'],view_index=c.binding['view_index']+1,
        deadline_ns=c.session_deadline,contact_or_motion_authorized=False)
    assert n._independent_observation_planning_active()
    assert n._station_compute_scope()
    return t


def submitted_observation(c):
    n=c.node;t=observation_station(c);previous=None
    stamps=tuple(t+ns-RECORDED_SOURCES[0] for ns in RECORDED_SOURCES)
    for i,ns in enumerate(stamps):
        # Keep admission itself fresh; recorded private callback arrival times
        # are unavailable. The explicit later handoff exercises expiry.
        feedback(c,ns-10000000);feedback(c,ns+10000000);clock(c,ns+100000000)
        row=row_at(c,ns,recovery=previous);rgbd(c,row);deliver(n,row)
        assert not n._station_compute_pending
        previous=row['status']['station_compute_wait']
        admitted=base.RemoteGrasp6DNode._poll_stream_snapshot(n)
        assert admitted is (i==4)
    ticket=n._stream_worker_ticket
    assert ticket.payload[0].sample_stamp_ns==stamps
    assert ticket.payload[0].quality.fused_frames==5
    return t,row,ticket


def test_recorded_five_frame_observation_submission_has_station_work_proof(station):
    c=station;n=c.node;t,row,ticket=submitted_observation(c)
    assert n._station_compute_work_certificate(ticket)
    deadline=deepcopy(n._station_compute_reference['contract'])
    ns=row['status']['source_stamp_ns']+400000000
    feedback(c,ns-10000000);feedback(c,ns+10000000);clock(c,ns+420000000)
    new=row_at(c,ns,recovery=row['status']['station_compute_wait']);rgbd(c,new)
    deliver(n,new,('object',))
    assert n._station_compute_pending and n._station_compute_soft_wait
    assert not n.latest_object_geometry.valid
    assert not n.latest_preview_rich_plan.valid
    n._require_stream_ticket_current(ticket)
    assert n._station_compute_reference['contract']==deadline
    deliver(n,new,('mask','status'))
    assert not n._station_compute_pending
    n._require_stream_ticket_current(ticket)
    recovered=n._station_compute_reference['contract']
    assert recovered==new['status']['station_compute_wait']
    assert recovered['phase_deadline_ns']==deadline['phase_deadline_ns']
    assert recovered['session_deadline_ns']==deadline['session_deadline_ns']
    assert recovered['identity_deadline_ns']<=recovered['phase_deadline_ns']
    assert n._stream_worker_ticket is ticket
    assert n._pipeline_counters['submitted']==1
    # Actual geometry activation and preview publication recheck the fresh
    # join. This synthetic message proves lifecycle only, never reachability.
    assert n._activate_prepared_geometry(prepared_for(n,ticket))
    rich=base.Grasp6DPlan();rich.valid=True;rich.header.stamp=deepcopy(row['object'].header.stamp)
    rich.header.frame_id='base_link';rich.plan_id='synthetic-observation-lifecycle'
    rich.diagnostic='SYNTHETIC_OBSERVATION_LIFECYCLE'
    legacy=base.PoseArray();legacy.header=deepcopy(rich.header)
    n._publish_preview_plan(rich,legacy,ticket=ticket)
    assert n.preview_rich_plan_pub.messages[-1].valid
    assert n._station_compute_work_published
    clock(c,new['status']['station_compute_wait']['geometry_deadline_ns']+1)
    assert n._station_compute_poll()
    assert not n._station_compute_soft_wait
    assert n._station_compute_ticket_code(ticket)=='TARGET_PROCESSING_PENDING'


@pytest.mark.parametrize('bad',['few','many','duplicate','order','old_newest','unregistered',
    'mask_digest','wrong_raw','identity','phase','active_contact','config','manual','epoch',
    'mixed_binding','late_historical_admission','missing_supplement'])
def test_observation_proof_rejects_wrong_source_set_or_scope(station,bad):
    c=station;n=c.node;t,row,ticket=submitted_observation(c);snapshot=ticket.payload[0]
    if bad=='few':snapshot=replace(snapshot,sample_stamp_ns=snapshot.sample_stamp_ns[:-1])
    elif bad=='many':snapshot=replace(snapshot,sample_stamp_ns=snapshot.sample_stamp_ns+(snapshot.stamp_ns+1,))
    elif bad=='duplicate':snapshot=replace(snapshot,sample_stamp_ns=(snapshot.sample_stamp_ns[1],)+snapshot.sample_stamp_ns[1:])
    elif bad=='order':snapshot=replace(snapshot,sample_stamp_ns=tuple(reversed(snapshot.sample_stamp_ns)))
    elif bad=='old_newest':clock(c,snapshot.stamp_ns+800000001)
    elif bad=='unregistered':n._station_compute_ready_history.pop(snapshot.sample_stamp_ns[0])
    elif bad=='mask_digest':snapshot=replace(snapshot,sample_mask_sha256=('f'*64,)+snapshot.sample_mask_sha256[1:])
    elif bad=='wrong_raw':snapshot=replace(snapshot,original_sample=None)
    elif bad=='identity':snapshot=replace(snapshot,target_epoch=snapshot.target_epoch+1,
        target_identity=replace(snapshot.target_identity,epoch=snapshot.target_epoch+1))
    elif bad=='phase':n._observe_return_phase='observing'
    elif bad=='active_contact':n.near_field_planning_active=True
    elif bad=='config':n.planning_snapshot_frames=4
    elif bad=='manual':c.params['/gui/joint_direct_mode']=True
    elif bad=='epoch':n._measurement_remote_epoch_ns+=1
    elif bad=='mixed_binding':
        n._station_compute_ready_history[snapshot.sample_stamp_ns[0]]['binding']['view_index']+=1
    elif bad=='late_historical_admission':
        item=n._station_compute_ready_history[snapshot.sample_stamp_ns[0]]
        item['admitted_ns']=item['deadline_ns']
    elif bad=='missing_supplement':n._observe_return_pending_supplement=None
    assert n._station_compute_snapshot_work(snapshot) is None


@pytest.mark.parametrize('bad',['phase','config','frame_count','frame_set','generation','target_epoch','deadline','span_config'])
def test_certificate_cannot_cross_request_policy_or_deadline(station,bad):
    c=station;n=c.node;t,row,ticket=submitted_observation(c)
    assert n._station_compute_work_certificate(ticket)
    if bad=='phase':n._observe_return_phase='observing_contact';n.near_field_planning_active=True
    elif bad=='config':n.planning_snapshot_frames=4
    elif bad=='frame_count':n._station_compute_work[ticket.request_id]['snapshot_policy']['frame_count']=3
    elif bad=='frame_set':n._station_compute_work[ticket.request_id]['source_stamps_ns']=tuple(ticket.payload[0].sample_stamp_ns[:-1])
    elif bad=='generation':n._stream_generation+=1
    elif bad=='target_epoch':n.target_instance_epoch+=1
    elif bad=='span_config':n.planning_snapshot_max_span_sec+=1
    else:clock(c,c.phase.deadline.to_nsec()+1);n._station_compute_poll()
    assert not n._station_compute_work_certificate(ticket)
