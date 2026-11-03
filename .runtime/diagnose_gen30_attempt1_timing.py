"""Read sealed non-temperature topics only; no ROS connections or hardware I/O."""
from pathlib import Path
import json,hashlib,re,collections,statistics,os
import rosbag
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'grasp_records/20261016T021226.502516Z_0494c8bc84a34a74933e8eae122dd1f4'
C=E/'ros_log/full_latest_gen29_restart_20261016T000728Z'
out=C/'verification/gen30_attempt1_timing_diagnosis';out.mkdir(exist_ok=False)
manifest=json.loads((R/'manifest.json').read_text());outcome=json.loads((R/'recorder_outcome.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed'] and outcome['bag_closed']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
bag=R/'keyframes.bag';ident=(bag.stat().st_size,bag.stat().st_mtime_ns)
expected=next(x['sha256'] for x in manifest['files'] if x['path']=='keyframes.bag')
h=hashlib.sha256()
with bag.open('rb') as f:
 for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
assert h.hexdigest()==expected
TOPICS=['/perception/unknown/detector_status','/grasp/near_field_phase','/grasp/state','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/grasp_6d/status','/rosout_agg']
def conv(m):
 if hasattr(m,'__slots__'):return {s:conv(getattr(m,s)) for s in m.__slots__}
 if isinstance(m,(list,tuple)):return [conv(x) for x in m]
 return m
status=[];events=[];planning_logs=[];metrics=[];selected_full=[]
with rosbag.Bag(str(bag),'r') as b:
 for topic,m,ts in b.read_messages(topics=TOPICS):
  ns=ts.to_nsec()
  if topic=='/rosout_agg':
   name=m.name
   if name in ('/move_group','/motion_gateway') or name.startswith('/move_group_commander_wrappers_'):
    planning_logs.append(dict(bag_ns=ns,log_ns=m.header.stamp.to_nsec(),logger=name,message=m.msg,file=m.file,line=m.line))
   elif name=='/remote_grasp6d_node' and 'remote 6D pipeline metrics: ' in m.msg:
    d=json.loads(m.msg.split('pipeline metrics: ',1)[1]);metrics.append(dict(bag_ns=ns,log_ns=m.header.stamp.to_nsec(),**d))
   continue
  if topic=='/perception/unknown/detector_status':
   d=json.loads(m.data);ft=d.get('frame_timing')or{};contract=d.get('station_compute_wait')or{};binding=contract.get('binding')or{}
   matches=ft.get('observation_appearance_identity')or[]
   row={k:d.get(k) for k in ['state','detail','source_stamp_ns','generation','selection_stamp_ns','target_locked','target_lost','first_loss','processing_seconds']}
   row.update(bag_ns=ns,message_sha256=hashlib.sha256(m.data.encode()).hexdigest(),frame_timing={k:ft[k] for k in ['processing_start_ros_ns','processing_start_source_age_sec','processing_wall_sec','processing_thread_cpu_sec','status_build_ros_ns','status_source_age_sec','failure_ros_ns','failure_source_age_sec','failed_stage','stage_wall_sec','stage_thread_cpu_sec','arrival_station_invalidated_reason','arrival_frame_completion'] if k in ft},station_contract={k:v for k,v in contract.items() if k!='binding'},station_binding={k:v for k,v in binding.items() if k not in ['reference_binding','station_q_rad']},station_binding_sha256=hashlib.sha256(json.dumps(binding,sort_keys=True).encode()).hexdigest() if binding else None,appearance_codes=sorted(set(x.get('code','') for x in matches)),minimum_current_template_iou=min([x['same_station_mask_iou'] for x in matches if 'same_station_mask_iou'in x],default=None))
   status.append(row)
   if d.get('target_lost') and ns>1792116802000000000 and not selected_full:selected_full.append(dict(bag_ns=ns,status=d))
   continue
  if hasattr(m,'data'):
   try:d=json.loads(m.data)
   except (TypeError,ValueError):d=m.data
  else:d=conv(m)
  if isinstance(d,dict):d.pop('reference_contact_plan',None)
  events.append(dict(topic=topic,bag_ns=ns,message=d))
for name,rows in [('status_timeline.jsonl',status),('state_phase_measurement.jsonl',events),('planning_logs.jsonl',planning_logs),('pipeline_metrics.jsonl',metrics)]:
 (out/name).write_text(''.join(json.dumps(r,ensure_ascii=False,separators=(',',':'))+'\n' for r in rows))
(out/'first_lost_status.json').write_text(json.dumps(selected_full,indent=2)+'\n')
phase=next(r for r in events if r['topic']=='/grasp/near_field_phase' and r['message']['active'])
phase_start=phase['message']['header']['stamp']['secs']*10**9+phase['message']['header']['stamp']['nsecs'];phase_deadline=phase['message']['deadline']['secs']*10**9+phase['message']['deadline']['nsecs']
end=1792116893000000000
completed=[m for m in metrics if m['event']=='request_completed']
intervals=[]
for m in completed:
 if m['request_id']in [17,18,87,88]:
  intervals.append(dict(request_id=m['request_id'],generation=m['generation'],end_log_ns=m['log_ns'],inferred_submit_ns=m['log_ns']-round(m['end_to_end_ms']*1e6),end_to_end_ms=m['end_to_end_ms'],ros_prepare_ms=m.get('ros_prepare_ms'),status=m['status'],snapshot_stamp_ns=m['snapshot_evidence']['snapshot_stamp_ns'],source_stamps_ns=m['snapshot_evidence']['source_stamp_ns'],inference_ms=m.get('wsl_total_ms'),transport_ms=m.get('transport_ms')))
requests=[];pending=None
for row in planning_logs:
 if row['logger']!='/motion_gateway' or not phase_start<row['log_ns']<end:continue
 msg=row['message']
 if msg.startswith('check_pose_strict request '):
  assert pending is None
  pending=dict(start_log_ns=row['log_ns'],request_message=msg,request_file=row['file'],request_line=row['line'])
 elif msg.startswith('check_pose_strict result '):
  assert pending is not None
  pending.update(end_log_ns=row['log_ns'],result_message=msg,wall_sec=(row['log_ns']-pending['start_log_ns'])*1e-9)
  calls=[x for x in planning_logs if x['logger']=='/move_group' and pending['start_log_ns']<=x['log_ns']<=row['log_ns']]
  durations=[(x,float(re.search(r'after ([0-9.]+) seconds',x['message']).group(1))) for x in calls if re.search(r'No solution found after ([0-9.]+) seconds',x['message'])]
  pending['goal_sampling_failed']=any('Unable to sample any valid states for goal tree'in x['message'] for x in calls)
  if durations:
   pending['reported_planner_sec']=durations[-1][1]
   pending['planner_no_solution_log_ns']=durations[-1][0]['log_ns']
   pending['after_no_solution_until_gateway_return_sec']=(row['log_ns']-durations[-1][0]['log_ns'])*1e-9
  pending['interval_request_id']=next((x['request_id'] for x in intervals if x['request_id']in (18,87) and x['inferred_submit_ns']<=pending['start_log_ns']<=x['end_log_ns']),None)
  requests.append(pending);pending=None
(out/'paired_strict_view_checks.json').write_text(json.dumps(requests,indent=2)+'\n')
within=[s for s in status if phase_start<=s['bag_ns']<=phase_deadline and s['generation']==2]
ready=[s for s in within if s['state']=='ready'];lost=[s for s in status if s['bag_ns']>phase_start and s.get('target_lost')]
firstlost=lost[0] if lost else None
lastready=max(ready,key=lambda r:r['bag_ns'])
changes=[];last=None
for s in within:
 key=(s['state'],s['detail'])
 if key!=last:changes.append({k:s[k] for k in ['bag_ns','source_stamp_ns','state','detail']});last=key
def quantile(xs):
 xs=sorted(x for x in xs if isinstance(x,(int,float)))
 return dict(count=len(xs),min=xs[0],median=statistics.median(xs),max=xs[-1]) if xs else None
segments=[]
for i in intervals:
 if i['request_id'] not in (18,87):continue
 checks=[x for x in requests if x['interval_request_id']==i['request_id']]
 segments.append(dict(**i,strict_check_count=len(checks),strict_check_wall_sec=sum(x['wall_sec'] for x in checks),all_checks_timeout=all('OBSERVATION_PLANNING_TIMEOUT' in x['result_message'] for x in checks),all_failed_goal_sampling=all(x['goal_sampling_failed'] for x in checks),first_check_log_ns=min((x['start_log_ns'] for x in checks),default=None),last_check_result_ns=max((x['end_log_ns'] for x in checks),default=None),pre_first_check_wall_sec=(checks[0]['start_log_ns']-i['inferred_submit_ns'])*1e-9 if checks else None))
summary=dict(record_dir=str(R),bag_sha256=expected,bag_bytes=ident[0],sealed_inputs_checked=True,no_ros_network_calls=True,no_hardware_access=True,no_temperature_topics_or_driver_diagnostics_read=True,selection_generation=2,selection_stamp_ns=1792116748543925762,phase_start_ns=phase_start,phase_deadline_ns=phase_deadline,phase_duration_sec=(phase_deadline-phase_start)*1e-9,ready_status_count=len(ready),status_detail_counts=dict(collections.Counter(s['detail'] for s in within)),status_changes=changes,ready_processing_wall_sec=quantile([s['frame_timing'].get('processing_wall_sec') for s in ready]),ready_source_age_sec=quantile([s['frame_timing'].get('status_source_age_sec') for s in ready]),ready_template_iou=quantile([s['minimum_current_template_iou'] for s in ready]),ready_max_interpublication_gap_sec=max((b['bag_ns']-a['bag_ns'])*1e-9 for a,b in zip(ready,ready[1:])),last_ready=lastready,first_lost=firstlost,first_loss_after_phase_deadline_sec=(firstlost['bag_ns']-phase_deadline)*1e-9 if firstlost else None,station_binding_sha256_unique=sorted(set(s['station_binding_sha256'] for s in ready if s['station_binding_sha256'])),completed_request_intervals=intervals,supplement_search_segments=segments,strict_check_count=len(requests),strict_check_wall_sec=sum(x['wall_sec'] for x in requests),strict_check_wall_statistics=quantile([x['wall_sec'] for x in requests]),planner_reported_statistics=quantile([x.get('reported_planner_sec') for x in requests]),post_planner_return_statistics=quantile([x.get('after_no_solution_until_gateway_return_sec') for x in requests]),pending_gateway_request_at_end=pending,metrics_counts=dict(collections.Counter(m['event']+':'+m['status'] for m in metrics)),limited_attribution=['Request attribution of gateway calls uses nonoverlapping logged request intervals; gateway messages do not carry remote request_id.','End-to-end includes queue/wait/setup/search; only explicit gateway and planner spans are attributed. Remaining pre-first-check time is not assumed to be CPU or a specific subroutine.','No candidate is declared unreachable solely from planning timeout; no motion was authorized by these execute=False checks.'])
(out/'observed_timing.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
assert ident==(bag.stat().st_size,bag.stat().st_mtime_ns)
receipt=dict(input_sha256={str(R/p):sha(R/p) for p in ('manifest.json','recorder_outcome.json','perception_execution_configuration.json')},bag_sha256=expected,script_sha256=sha(Path(__file__)),outputs={p.name:sha(p) for p in out.iterdir() if p.is_file()},source_unchanged=True,production_modified=False)
(out/'extraction_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({k:v for k,v in summary.items() if k not in ('last_ready','first_lost','status_changes','station_binding_sha256_unique')},ensure_ascii=False,indent=2))
print('out',out)
