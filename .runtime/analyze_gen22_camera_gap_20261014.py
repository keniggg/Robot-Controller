#!/usr/bin/env python3
"""Read sealed GEN21 evidence only; no ROS graph, device, or parameter access."""
import json
import hashlib
import re
from pathlib import Path
import rosbag
import genpy

E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen18_restart_20261014T012216Z'
D=E/'grasp_records/20261014T092930.740068Z_ded6108f8ba34a848a203c77f2c88628'
O=N/'verification/gen22_camera_gap_evidence'
O.mkdir(parents=True,exist_ok=True)
START,END=1791970214.,1791970223.
selected_topics=['/experiment/observation_rgbd/metadata','/perception/unknown/detector_status',
 '/grasp_mode/observe_return_measurement_ack','/grasp_mode/observe_return_measurement_request',
 '/grasp/near_field_phase','/grasp/state','/perception/object', '/perception/object_mask',
 '/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw',
 '/alicia_d/accepted_joint_states','/alicia_controller/state']
status_rows=[];image_rows=[];objects=[];acks=[];states=[];joints=[];phases=[]
with rosbag.Bag(str(D/'keyframes.bag'),'r') as bag:
 for topic,msg,t in bag.read_messages(topics=selected_topics,
         start_time=genpy.Time.from_sec(START),end_time=genpy.Time.from_sec(END)):
  receipt=t.to_nsec()
  if topic=='/perception/unknown/detector_status':
   status_rows.append(dict(bag_receipt_ns=receipt,payload=json.loads(msg.data)))
  elif topic=='/experiment/observation_rgbd/metadata':
   x=json.loads(msg.data);image_rows.append(dict(topic=topic,bag_receipt_ns=receipt,source_stamp_ns=x['source_stamp_ns'],sample_rate_limit_hz=x['sample_rate_limit_hz']))
  elif topic.startswith('/grasp_mode/observe_return_measurement_'):
   acks.append(dict(topic=topic,bag_receipt_ns=receipt,payload=json.loads(msg.data)))
  elif topic=='/grasp/near_field_phase':
   phases.append(dict(bag_receipt_ns=receipt,stamp_ns=msg.header.stamp.to_nsec(),reference_source_stamp_ns=msg.reference_source_stamp.to_nsec(),text=str(msg)))
  elif topic=='/grasp/state':
   states.append(dict(bag_receipt_ns=receipt,text=str(msg)))
  elif topic=='/perception/object':
   objects.append(dict(bag_receipt_ns=receipt,source_stamp_ns=msg.header.stamp.to_nsec(),detected=msg.detected,confidence=msg.confidence))
  elif topic in ('/alicia_d/accepted_joint_states','/alicia_controller/state'):
   if topic.endswith('accepted_joint_states'):positions=list(msg.position[:6])
   else:positions=list(msg.actual.positions[:6])
   joints.append(dict(topic=topic,bag_receipt_ns=receipt,source_stamp_ns=msg.header.stamp.to_nsec(),positions=positions))
  else:image_rows.append(dict(topic=topic,bag_receipt_ns=receipt,source_stamp_ns=msg.header.stamp.to_nsec(),seq=msg.header.seq))

camera=[];logs=[]
for path in [N/'gen21_attempt_1_watch.log',N/'gen21_deployment/consumers.log',N/'gen20_full_storage_recovery/core.log']:
 for lineno,line in enumerate(path.read_text(errors='replace').splitlines(),1):
  m=re.search(r'(17919702\d\d\.\d+)',line)
  if not m or not START<=float(m.group(1))<=END:continue
  if any(x in line for x in ['temperature','Temperature','temperatures']):continue
  if any(x in line for x in ['Camera','Unknown target first loss','Reached-view','qualified','station_task_owner','TARGET_LOST','source_frame_stale','waiting for near-field']):
   row=dict(path=str(path),line=lineno,time_sec=float(m.group(1)),text=line)
   logs.append(row)
   if 'Camera backend read stages: ' in line:
    try:detail=json.loads(line.split('Camera backend read stages: ',1)[1])
    except ValueError:continue
    camera.append(dict(path=str(path),line=lineno,time_sec=float(m.group(1)),detail=detail))

def slim_frames(x):
 return {k:{f:v.get(f) for f in ('frame_number','device_timestamp_ms','metadata_raw','present')}
   for k,v in (x or {}).items() if isinstance(v,dict)}
camera_summary=[]
for row in camera:
 d=row['detail'];h=d.get('host_wait',{})
 camera_summary.append(dict(path=row['path'],line=row['line'],time_sec=row['time_sec'],
   host_wait_sec=h.get('elapsed_sec'),host_enter=h.get('enter',{}).get('wall_time_sec'),host_exit=h.get('exit',{}).get('wall_time_sec'),
   thread_cpu_sec=h.get('thread_cpu_elapsed_sec'),pair_timing=d.get('pair_timing'),
   raw_frames=slim_frames(d.get('raw_frames')),keys=list(d),
   previous_raw_frames=slim_frames(d.get('previous_completed_raw_frames',{}).get('raw_frames'))))

status_summary=[]
for row in status_rows:
 d=row['payload']
 out={k:d[k] for k in ('source_stamp_ns','state','detail','target_lost','first_loss','color_seq','depth_seq','processing_timing','observation_measurement','observation_arrival_retry','initial_station_current_support') if k in d}
 out['bag_receipt_ns']=row['bag_receipt_ns']
 out['timing_keys']=[k for k in d if any(s in k for s in ('time','timing','loss','station'))]
 status_summary.append(out)

for name,data in [('statuses_full.json',status_rows),('status_summary.json',status_summary),('camera_stages_full.json',camera),('camera_stages_summary.json',camera_summary),('images.json',image_rows),('objects.json',objects),('measurement_messages.json',acks),('states.json',states),('phases.json',phases),('joints.json',joints),('logs.json',logs)]:
 (O/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
identity=[]
for p in [D/'manifest.json',D/'recorder_outcome.json',D/'keyframes.bag']:
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 identity.append(dict(path=str(p),size_bytes=p.stat().st_size,sha256=h.hexdigest()))
(O/'input_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(dict(output=str(O),statuses=len(status_rows),camera_stage_rows=len(camera),image_rows=len(image_rows),objects=len(objects),measurement_messages=len(acks),joints=len(joints)),indent=2))
