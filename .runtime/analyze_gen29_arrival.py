#!/usr/bin/env python3
"""Read only sealed gen29 metadata; preserve arrival rejection chronology."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import rosbag

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261009T124424.224503Z_ee2856eb8dab4529a6d029babf69f7a5'
OUT = EXT/'ros_log/full_latest_ros_20261009T003407Z/gen29_handoff_analysis'
manifest = json.loads((REC/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
statuses, requests, acks, tasks, joints = [], [], [], [], []
topics = ['/perception/unknown/detector_status', '/grasp_mode/observe_return_measurement_request',
          '/grasp_mode/observe_return_measurement_ack', '/grasp/state', '/alicia_d/accepted_joint_states']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic, message, stamp in bag.read_messages(topics=topics):
        row = dict(received_ns=stamp.to_nsec())
        if topic.endswith('accepted_joint_states'):
            if message.header.frame_id == 'sdk_measured':
                joints.append(dict(row, source_stamp_ns=message.header.stamp.to_nsec(),
                    names=list(message.name), positions=list(message.position)))
        elif topic == '/grasp/state':
            tasks.append(dict(row, source_stamp_ns=message.header.stamp.to_nsec(),
                state=message.state, message=message.message, active=message.active,
                success=message.success))
        else:
            payload = json.loads(message.data)
            if topic.endswith('detector_status'):
                if payload.get('generation') == 29:
                    statuses.append(dict(row, message=payload))
            else:
                (requests if topic.endswith('request') else acks).append(dict(row, message=payload))
views=[]
for rr in requests:
    request=rr['message']
    if request['state'] != 'requalifying':continue
    start,deadline=request['arrival_stamp_ns'],request['capture_deadline_ns']
    selected=[r for r in statuses if start<=r['received_ns']<=deadline and r['message'].get('observation_measurement',{}).get('request',{}).get('lease_id')==request['lease_id']]
    rows=[]
    for r in selected:
        msg=r['message'];e=msg['observation_measurement'];source=msg.get('source_stamp_ns')
        rows.append(dict(received_ns=r['received_ns'],source_stamp_ns=source,status_state=msg['state'],detail=msg.get('detail'),reason=e.get('reason'),qualified=e.get('qualified'),measurement_state=e.get('state'),source_stamps_ns=e.get('source_stamps_ns'),source_to_status_bag_receipt_sec=(r['received_ns']-source)*1e-9 if source else None,measurement_evaluated=e.get('measurement_evaluated'),exact_tf_consumed=e.get('exact_tf_consumed')))
    feedback=[r for r in joints if start<=r['source_stamp_ns']<=deadline]
    names=feedback[0]['names'] if feedback else []
    span=[max(r['positions'][i] for r in feedback)-min(r['positions'][i] for r in feedback) for i in range(len(names))]
    views.append(dict(view_index=request['view_index'],arrival_request=request,arrival_window=dict(start_ns=start,deadline_ns=deadline,duration_sec=(deadline-start)*1e-9),statuses=rows,reason_counts=dict(Counter(r['reason'] for r in rows)),qualified_count=sum(r['qualified'] is True for r in rows),protocol_acks=[r for r in acks if r['message']['lease_id']==request['lease_id']],arrival_stationary_feedback=dict(count=len(feedback),names=names,joint_peak_to_peak=span,first=feedback[0] if feedback else None,last=feedback[-1] if feedback else None)))
first_loss=next((r for r in statuses if r['message'].get('target_lost') is True),None)
summary=dict(record=str(REC),generation=29,manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),sealed_manifest={k:manifest[k] for k in ('state','result','failure_reason','recorder_closed','completed_at')},views=views,first_loss=first_loss,task_states=tasks,requests=requests,limitations=['Read only sealed non-temperature metadata.','Bag receipt ages are not exact computation completion timestamps.','No pixel replay or inferred low-level cause is claimed by this chronology.'])
OUT.mkdir(exist_ok=True)
(OUT/'arrival_and_terminal.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
(OUT/'detector_status.json').write_text(json.dumps(statuses,ensure_ascii=False,indent=2)+'\n')

from statistics import median

def stats(values):
    values=[v for v in values if isinstance(v,(int,float))]
    return dict(count=len(values),minimum=min(values),median=median(values),maximum=max(values)) if values else dict(count=0)

view_summaries=[]
for view in views:
    request=view['arrival_request'];lease=request['lease_id']
    qualified=[r for r in acks if r['message'].get('lease_id')==lease and r['message'].get('state')=='requalifying' and r['message'].get('accepted') is True and r['message'].get('qualified') is True]
    participants={r['message']['participant']:r for r in qualified}
    stamps=[r['message'].get('source_stamps_ns') for r in participants.values()]
    remote=participants.get('remote')
    view_summaries.append(dict(view_index=request['view_index'],arrival_stamp_ns=request['arrival_stamp_ns'],qualified_participants=list(participants),same_three_sources=len(participants)==2 and len(stamps[0])==3 and stamps[0]==stamps[1],source_stamps_ns=stamps[0] if stamps else [],arrival_to_remote_ack_bag_receipt_sec=(remote['received_ns']-request['arrival_stamp_ns'])*1e-9 if remote else None,arrival_reason_counts=view['reason_counts']))
first_arrival=min((v['arrival_stamp_ns'] for v in view_summaries),default=2**63)
normal=[];all_ready=[]
for r in statuses:
    m=r['message'];t=m.get('frame_timing',{});g=m.get('geometry_metrics',{});f=g.get('observation_fragment_group',{})
    if m.get('state')!='ready':continue
    row=dict(source_stamp_ns=m.get('source_stamp_ns'),received_ns=r['received_ns'],detail=m.get('detail'),view_index=m.get('observation_measurement',{}).get('request',{}).get('view_index'),support_source=g.get('support_source'),fragment_original_metric_matches=f.get('original_metric_matches'),fragment_original_candidate_count=f.get('original_candidate_count'),fragment_final_measured_points=f.get('final_measured_points'),processing_wall_sec=t.get('processing_wall_sec'),processing_thread_cpu_sec=t.get('processing_thread_cpu_sec'),tracking_wall_sec=t.get('stage_wall_sec',{}).get('tracking'),tracking_thread_cpu_sec=t.get('stage_thread_cpu_sec',{}).get('tracking'),processing_entry_source_age_sec=t.get('processing_start_source_age_sec'),status_source_age_sec=t.get('status_source_age_sec'),initial_completion=t.get('station_frame_completion'),arrival_completion=t.get('arrival_frame_completion'),hold_total=t.get('rgb_components',{}).get('hold.total',[]))
    all_ready.append(row)
    if m.get('detail')=='tracked' and m.get('source_stamp_ns',0)>first_arrival:normal.append(row)
fragment=[r for r in normal if r['fragment_original_metric_matches'] is not None]
metrics=('processing_wall_sec','processing_thread_cpu_sec','tracking_wall_sec','tracking_thread_cpu_sec','processing_entry_source_age_sec','status_source_age_sec')
timing=dict(record=str(REC),generation=29,sealed_manifest=summary['sealed_manifest'],record_manifest_sha256=summary['manifest_sha256'],status_count=len(statuses),status_state_counts=dict(Counter(r['message'].get('state') for r in statuses)),arrival_views=view_summaries,all_arrivals_three_source_qualified=bool(view_summaries) and all(v['same_three_sources'] for v in view_summaries),first_loss=first_loss,first_loss_absent=first_loss is None,all_ready_count=len(all_ready),normal_ready_after_first_arrival_count=len(normal),normal_fragment_group_count=len(fragment),successful_completion_past_old_reference_deadline={key:sum(bool((r.get(field) or {}).get('used_past_geometry_deadline')) for r in all_ready) for key,field in [('initial','initial_completion'),('arrival','arrival_completion')]},normal_timings={k:stats([r.get(k) for r in normal]) for k in metrics},fragment_timings={k:stats([r.get(k) for r in fragment]) for k in metrics},by_view={str(i):dict(normal_ready_count=sum(r['view_index']==i for r in normal),fragment_ready_count=sum(r['view_index']==i for r in fragment),timings={k:stats([r.get(k) for r in normal if r['view_index']==i]) for k in metrics}) for i in (1,2,3)},normal_frame_summary=normal,all_ready_summary=all_ready,limitations=['Sealed non-temperature metadata only; no RGB/depth decoding or pixel replay.','Status and ACK bag receipt is not internal completion time.','tracking includes complete fragment-group work plus qualification/hold overhead, not an isolated optimized helper benchmark.','Normal rows require original state ready/detail tracked and source after first arrival; view index uses original observation measurement request retained on status.','Different view geometry and load prevent causal performance claims from cross-generation medians alone.'])
task_failure=next((r for r in tasks if r.get('state')=='FAILED'),None)
failed_at=task_failure['source_stamp_ns'] if task_failure else None
before_failure=[r for r in normal if failed_at is None or r['received_ns']<failed_at]
timing.update(normal_unique_source_count=len({r['source_stamp_ns'] for r in normal}),all_ready_unique_source_count=len({r['source_stamp_ns'] for r in all_ready}),unavailable_rows=[dict(received_ns=r['received_ns'],source_stamp_ns=r['message'].get('source_stamp_ns'),detail=r['message'].get('detail'),target_lost=r['message'].get('target_lost')) for r in statuses if r['message'].get('state')=='unavailable'],pending_detail_counts=dict(Counter(r['message'].get('detail') for r in statuses if r['message'].get('state')=='pending')),first_task_failure=task_failure,normal_ready_before_task_failure_count=len(before_failure),normal_before_task_failure_by_view={str(i):sum(r['view_index']==i for r in before_failure) for i in (1,2,3)})
timing['limitations'].append('All-record normal counts include continued idle perception after task FAILED; before-task-failure count is separately bounded by actual task state source time versus recorded status receipt.')
(OUT/'timing_summary.json').write_text(json.dumps(timing,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in timing.items() if k not in ('normal_frame_summary','all_ready_summary','first_loss')},ensure_ascii=False,indent=2))
print('OUTPUT_SHA256',json.dumps({n:hashlib.sha256((OUT/n).read_bytes()).hexdigest() for n in ['arrival_and_terminal.json','detector_status.json','timing_summary.json']},sort_keys=True))
