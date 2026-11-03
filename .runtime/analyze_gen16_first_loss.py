#!/usr/bin/env python3
"""Sealed source-age/support-gap facts, accepted joints and camera filter metadata."""
from pathlib import Path
from collections import Counter
import json,hashlib,numpy as np,rosbag
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');NEW=ROOT/'ros_log/full_latest_ros_20261009T003407Z';REC=ROOT/'grasp_records/20261009T054827.257779Z_e15d068b793c43ada6713a78f421f2bb';OUT=NEW/'gen16_handoff_analysis';OUT.mkdir(exist_ok=True)
manifest=json.loads((REC/'manifest.json').read_text());assert manifest['recorder_closed'] is True and manifest['state']=='complete'
statuses=[];joints=[];camera=[];tasks=[];requests=[];acks=[]
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/perception/unknown/detector_status','/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata','/grasp/state','/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack']):
  ns=t.to_nsec()
  if topic=='/perception/unknown/detector_status':
   a=json.loads(m.data)
   if a.get('generation')==16:statuses.append(dict(received_ns=ns,message=a))
  elif topic=='/alicia_d/accepted_joint_states':
   if m.header.frame_id=='sdk_measured':joints.append(dict(received_ns=ns,stamp_ns=m.header.stamp.to_nsec(),joint_names=list(m.name),positions_rad=list(m.position)))
  elif topic=='/grasp_record/camera_runtime_metadata':
   a=json.loads(m.data);p=a.get('parameters',{});camera.append(dict(received_ns=ns,parameter_read_interval={k:v for k,v in a.items() if k!='parameters'},stationary_temporal=p.get('runtime_profile',{}).get('stationary_temporal'),runtime_profile={k:v for k,v in p.get('runtime_profile',{}).items() if k!='stationary_temporal'}))
  elif topic=='/grasp/state':tasks.append(dict(received_ns=ns,state=m.state,message=m.message,active=m.active,success=m.success,stamp_ns=m.header.stamp.to_nsec()))
  else:(requests if topic.endswith('request') else acks).append(dict(received_ns=ns,message=json.loads(m.data)))
loss=next(r for r in statuses if r['message'].get('target_lost') is True);lm=loss['message'];observed=lm['first_loss']['observed_stamp_ns'];lastgood=lm['first_loss']['last_good_source_stamp_ns'];source=lm['source_stamp_ns'];generation_start=lm['selection_stamp_ns'];before=[r for r in statuses if r['received_ns']<=loss['received_ns']];previous_ready=[r for r in before if r['message'].get('state')=='ready'];lastready=previous_ready[-1] if previous_ready else None;gapwindow=[r for r in before if r['message'].get('source_stamp_ns',0)>lastgood]
def short(r):
 m=r['message'];t=m.get('frame_timing',{});q=m.get('source_qualification',{})
 return dict(received_ns=r['received_ns'],source_stamp_ns=m.get('source_stamp_ns'),state=m.get('state'),detail=m.get('detail'),support_source=m.get('geometry_metrics',{}).get('support_source'),source_qualification=q,failed_stage=t.get('failed_stage'),status_source_age_sec=t.get('status_source_age_sec'),processing_start_source_age_sec=t.get('processing_start_source_age_sec'),processing_wall_sec=t.get('processing_wall_sec'),processing_thread_cpu_sec=t.get('processing_thread_cpu_sec'),ingress_to_processing_sec=t.get('ingress_to_processing_sec'),input_lock_wait_sec=t.get('input_lock_wait_sec'),stages=t.get('stages'),decode_components=t.get('decode_components'))
runj=[r for r in joints if r['stamp_ns']>=generation_start];assert runj;positions=np.array([r['positions_rad'] for r in runj]);names=runj[-1]['joint_names'];assert all(r['joint_names']==names for r in runj)
timing=[r['message'].get('frame_timing',{}) for r in before];tm={}
for field in ['status_source_age_sec','processing_start_source_age_sec','processing_wall_sec','processing_thread_cpu_sec','ingress_to_processing_sec','input_lock_wait_sec']:
 vals=[(r.get(field),i) for i,r in enumerate(timing) if isinstance(r.get(field),(int,float))]
 if vals:
  mx,i=max(vals);tm[field]=dict(maximum=mx,at_source_ns=before[i]['message'].get('source_stamp_ns'),median=float(np.median([v for v,_ in vals])))
camera_window=[r for r in camera if lastgood-3_000_000_000<=r['received_ns']<=loss['received_ns']+1_000_000_000]
result=dict(record=str(REC),sealed_manifest={k:manifest.get(k) for k in ['state','result','failure_reason','recorder_closed','completed_at']},manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),selection_generation=16,selection_stamp_ns=generation_start,first_loss=loss,last_ready=lastready,age_at_terminal_observation=dict(current_source_sec=(observed-source)*1e-9,last_good_reference_sec=(observed-lastgood)*1e-9,current_source_stamp_ns=source,last_good_reference_source_stamp_ns=lastgood,observed_stamp_ns=observed,existing_limit_sec=.8),counts_before_first_loss=dict(Counter(r['message'].get('detail','') for r in before)),pending_after_last_good=[short(r) for r in gapwindow],all_gen16_status_before_first_loss=before,frame_timing_summary=tm,camera_runtime_samples_near_loss=camera_window,camera_metadata_limit='Parameter reads are low-rate snapshots; not exact image-bound filter state. No inference of the exact first-loss filter state from nearest sample alone.',accepted_joint_feedback=dict(sample_count=len(runj),joint_names=names,first=runj[0],last=runj[-1],peak_to_peak_rad=np.ptp(positions,axis=0).tolist(),all_recorded_joint_positions_unchanged=bool(np.all(positions==positions[0]))),grasp_task_states=tasks,measurement_requests=requests,measurement_acks=acks,grasp_start_seen=any(r['active'] for r in tasks),production_modified=False,ros_calls=False,temperature_read=False)
p=OUT/'first_loss_analysis.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),age=result['age_at_terminal_observation'],loss=short(loss),pending=result['pending_after_last_good'],timing=tm,q_range=result['accepted_joint_feedback']['peak_to_peak_rad'],last_q=runj[-1],request_count=len(requests),taskstates=tasks),ensure_ascii=False,indent=2))
