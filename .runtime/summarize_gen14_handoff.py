#!/usr/bin/env python3
"""Sealed measurement and supplemental-view handoff evidence; no ROS calls."""
from pathlib import Path
import collections,json,hashlib
import rosbag

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T050350.036425Z_ce203c7b9d4c4ba29e4303eb51b69d62'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen14_handoff_analysis'
manifest=json.loads((RECORD/'manifest.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed'] is True
topics=['/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack',
        '/grasp_mode/observe_return_session','/grasp_mode/observe_return_ack','/grasp_6d/status',
        '/grasp_6d/gate_audit','/grasp_6d/preview_plan_enriched','/grasp/near_field_phase',
        '/grasp/state','/perception/unknown/detector_status']
records={t:[] for t in topics};audit_summaries=[]
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,msg,received in bag.read_messages(topics=topics):
        if hasattr(msg,'data'):
            try:value=json.loads(msg.data)
            except ValueError:value=msg.data
            if topic=='/grasp_6d/gate_audit':
                value={k:value.get(k) for k in ('request_id','generation','plan_id','snapshot_stamp_ns','observe_return','outcome','promotion')}
            if topic=='/perception/unknown/detector_status':
                value={k:value.get(k) for k in ('generation','source_stamp_ns','state','detail','target_lost','anchor_source_stamp_ns','observation_measurement','geometry_quality','geometry_metrics') if k in value}
        elif topic=='/grasp_6d/preview_plan_enriched':
            value={k:getattr(msg,k) for k in ('valid','plan_id','diagnostic','candidate_source','target_track_id','fused_view_count')}
            value.update(source_stamp_ns=msg.header.stamp.to_nsec(),pose_count=len(msg.poses),frame_id=msg.header.frame_id)
        elif topic=='/grasp/near_field_phase':
            value={}
            for k in msg.__slots__:
                v=getattr(msg,k)
                if isinstance(v,(str,int,float,bool)):value[k]=v
                elif hasattr(v,'to_nsec'):value[k+'_ns']=v.to_nsec()
                elif k=='header':value.update(header_stamp_ns=v.stamp.to_nsec(),frame_id=v.frame_id)
        else:
            value={k:getattr(msg,k) for k in msg.__slots__ if isinstance(getattr(msg,k),(str,int,float,bool))}
        records[topic].append(dict(received_ns=received.to_nsec(),message=value))
requests=records['/grasp_mode/observe_return_measurement_request'];acks=records['/grasp_mode/observe_return_measurement_ack']
arrival=next(r['message'] for r in requests if r['message']['state']=='requalifying')
qualified=[r for r in acks if r['message']['state']=='requalifying' and r['message'].get('qualified') is True]
assert {r['message']['participant'] for r in qualified}=={'unknown','remote'}
stamps=qualified[0]['message']['source_stamps_ns'];assert all(r['message']['source_stamps_ns']==stamps for r in qualified)
ack_ns=max(r['received_ns'] for r in qualified)
arrival_status=[r for r in records['/perception/unknown/detector_status'] if arrival['arrival_stamp_ns']<=r['received_ns']<=arrival['capture_deadline_ns']]
supplement=[r for r in records['/grasp_6d/status'] if isinstance(r['message'],str) and r['message'].startswith('OBSERVE_RETURN_SUPPLEMENTAL_VIEW_REQUIRED')]
assert len(supplement)==1
after=[r for r in records['/grasp_6d/preview_plan_enriched'] if r['received_ns']>supplement[0]['received_ns']]
summary=dict(record=str(RECORD),sealed_manifest=dict(state=manifest['state'],recorder_closed=manifest['recorder_closed'],result=manifest['result'],failure_reason=manifest['failure_reason'],completed_at=manifest['completed_at']),
    manifest_sha256=hashlib.sha256((RECORD/'manifest.json').read_bytes()).hexdigest(),selection=arrival['selection'],
    arrival_passed=True,arrival_request_ns=arrival['arrival_stamp_ns'],capture_deadline_ns=arrival['capture_deadline_ns'],
    both_qualified_ack_received_ns=ack_ns,arrival_to_both_qualified_ack_sec=(ack_ns-arrival['arrival_stamp_ns'])*1e-9,
    qualified_source_stamps_ns=stamps,current_source_age_at_latest_ack_receipt_sec=(ack_ns-stamps[-1])*1e-9,
    initial_history_age_at_latest_ack_receipt_sec=(ack_ns-stamps[0])*1e-9,
    settle=arrival['settle'],arrival_statuses=arrival_status,measurement_requests=requests,measurement_acks=acks,
    sessions=records['/grasp_mode/observe_return_session'],session_acks=records['/grasp_mode/observe_return_ack'],
    planner_status=records['/grasp_6d/status'],preview_plans=records['/grasp_6d/preview_plan_enriched'],
    gate_audit_count=len(records['/grasp_6d/gate_audit']),gate_audit_summaries=records['/grasp_6d/gate_audit'],
    near_field_phases=records['/grasp/near_field_phase'],task_states=records['/grasp/state'],
    supplement_required_event=supplement[0],supplement_preview_count=len(after),
    observations=['Real first observation arrival and original RGB/three-frame requalification passed, both owners confirmed the same three source stamps.',
        'Contact request20 registration was accepted. The later supplemental-view search failure is separate from registration and arrival qualification.',
        'The supplemental-required status proves remote installed a pending view request.',
        'No second observing session or motion measurement lease was recorded.',
        'Runner recorded UNKNOWN_OBSERVATION_NO_SAFE_VIEW: OBSERVATION_SEARCH_INCOMPLETE checked4/210, then task returned NEAR_FIELD_PLAN_PHASE_INVALID.',
        'This run stops at supplemental observation search, not RGB arrival. Candidate-specific rejection diagnosis is independently owned by the code agent.'],
    private_state_limitations=['The full numeric _observe_return_step_gain and _observe_return_pending_supplement dictionaries are not directly published on a dedicated recorded topic.',
        'The supplemental-required status contains the view_request_id; this demonstrates that branch completion, but does not recover every private numeric gain field.',
        'Recorded source-age calculations use receipt time and therefore include transport latency.'],
    contact_or_close_started=False,no_ros_actions=True,production_modified=False,physical_grasp_verified=False)
registration_audit=RECORD/'keyframes_audits/55144b5a6701e7f4f7043bc4ac3c68a883d486933c7e841834136afc73587021.json'
rd=json.loads(registration_audit.read_text())
summary['accepted_contact_request_registration']=dict(path=str(registration_audit),request_id=rd['request_id'],snapshot_stamp_ns=rd['snapshot_stamp_ns'],registration=rd['multiview_surface']['latest_registration'])
summary['runner_terminal_lines']=[l for l in (RECORD/'runner.log').read_text().splitlines() if any(k in l for k in ('OBSERVE_RETURN_PLANNING_TERMINAL','GRASP_RESULT'))]
summary['detector_lost_after_arrival']=[r for r in records['/perception/unknown/detector_status'] if r['received_ns']>=arrival['arrival_stamp_ns'] and r['message'].get('target_lost') is True]
OUT.mkdir(parents=True,exist_ok=True);p=OUT/'arrival_and_terminal.json' 
with p.open('x') as f:json.dump(summary,f,indent=2,ensure_ascii=False,allow_nan=False)
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),arrival_to_qualified_sec=summary['arrival_to_both_qualified_ack_sec'],latest_age_sec=summary['current_source_age_at_latest_ack_receipt_sec'],supplement_preview_count=len(after)),ensure_ascii=False))
