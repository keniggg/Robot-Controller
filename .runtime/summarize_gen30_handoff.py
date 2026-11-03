"""Summarize already exported sealed metadata and original text audit only."""
from pathlib import Path
import json,hashlib,re
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
D=B/'gen30_handoff_analysis';A=json.loads((D/'arrival_and_terminal.json').read_text());S=json.loads((D/'detector_status.json').read_text());R=Path(A['record'])
m=A['first_loss']['message'];t=m['frame_timing'];loss=m['first_loss'];s=loss['source_stamp_ns'];r=loss['last_good_source_stamp_ns'];observed=loss['observed_stamp_ns']
last=next(x for x in S if x['message'].get('state')=='ready' and x['message'].get('source_stamp_ns')==r)
token=t.get('arrival_frame_completion',{})
summary=dict(record=str(R),generation=30,sealed_manifest=A['sealed_manifest'],first_loss=dict(source_stamp_ns=s,last_good_source_stamp_ns=r,observed_stamp_ns=observed,bag_received_ns=A['first_loss']['received_ns'],reason=loss['reason'],failed_stage=t['failed_stage'],underlying_reason=t.get('arrival_station_invalidated_reason'),capture_gap_sec=(s-r)*1e-9,current_age_at_first_loss_sec=(observed-s)*1e-9,old_reference_age_at_first_loss_sec=(observed-r)*1e-9,current_deadline_ns=s+800000000,current_deadline_exceeded_ns=observed-(s+800000000),frame_completion=token,frame_timing=t),last_good=last,findings=['Current source was timely at exact ingress and processing entry; the normal station completion token was present and bound to this same source/reference.','Segmentation took1.634106307 seconds wall and1.029160025 seconds current-thread CPU; post_segment current source was already1.741712808 seconds old. No tracking/fragment/refine timer was entered.','The underlying marker reason says completion binding changed or expired; elapsed current age independently proves the fixed completion deadline was exceeded. It is not solely an old-reference-age failure.','Latest pending exact source remained about50ms old; no camera input loss is proved.','No preceding typed support/metric pending is recorded; pending rows are observation lifecycle waiting.','Segment wall/thread discrepancy does not distinguish scheduling, native-library contention or internal subroutine cost without more evidence.'],limitations=['No pixel replay or isolated segmentation profiling performed.','Segmentation is a composite stage; its duration is not attributed to RANSAC/connected-components without additional instrumentation.','This metadata subset alone does not prove absence of every possible independent q/configuration change; current-source deadline expiry already suffices to forbid publication.'],input_sha256={n:hashlib.sha256((D/n).read_bytes()).hexdigest() for n in ['arrival_and_terminal.json','detector_status.json','timing_summary.json']})
log=B/'observation_exposure_tiers_deployment/consumers.log';rejections=[]
for l in log.read_text(errors='replace').splitlines():
 if 'remote 6D rejected contact evidence: ' in l:
  textstamp=re.search(r'\[([0-9]+\.[0-9]+)\]',l).group(1)
  rejections.append(dict(log_stamp_sec_text=textstamp,message=json.loads(l.split('remote 6D rejected contact evidence: ',1)[1])))
selected=[];seen=set()
for f in (R/'keyframes_audits').glob('*.json'):
 d=json.loads(f.read_text());sel=d.get('selected')
 if not isinstance(sel,dict) or not sel.get('observation_view') or d['request_id'] in seen:continue
 seen.add(d['request_id']);v=sel['observation_view']
 selected.append(dict(request_id=d['request_id'],snapshot_stamp_ns=d['snapshot_stamp_ns'],plan_id=d['plan_id'],audit_path=str(f),audit_sha256=hashlib.sha256(f.read_bytes()).hexdigest(),proposal=v.get('proposal'),proposal_search=v.get('proposal_search'),missing_side_gain=v.get('observe_return_missing_side_gain'),moveit=sel.get('moveit'),observation_orientation_search=sel.get('observation_orientation_search')))
for name,data in [('failure_timing.json',summary),('contact_rejections.json',dict(record=str(R),log_path=str(log),rows=rejections,scope='original_rejected_contact_diagnostics_no_motion_or_future_response_claim')),('selected_views.json',dict(record=str(R),selected=selected))]:
 p=D/name;p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');print(name,hashlib.sha256(p.read_bytes()).hexdigest())
print('firstloss',summary['first_loss']['current_age_at_first_loss_sec'],summary['first_loss']['old_reference_age_at_first_loss_sec'],'contact_rows',len(rejections))
