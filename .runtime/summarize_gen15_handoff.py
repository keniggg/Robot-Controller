#!/usr/bin/env python3
from pathlib import Path
from collections import Counter
import json,hashlib
import rosbag
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');NEW=BASE/'ros_log/full_latest_ros_20261009T003407Z';REC=BASE/'grasp_records/20261009T052127.805554Z_6b8770587c5c4cd8abc679178eaa1c70';OUT=NEW/'gen15_handoff_analysis'
manifest=json.loads((REC/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed'] is True
req=[];ack=[];status=[];phases=[];plans=[];images={}
topics=['/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/perception/unknown/detector_status','/grasp_6d/status','/grasp/state','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']
with rosbag.Bag(str(REC/'keyframes.bag')) as b:
 for topic,m,t in b.read_messages(topics=topics):
  if topic.endswith('measurement_request'):req.append(dict(received_ns=t.to_nsec(),message=json.loads(m.data)))
  elif topic.endswith('measurement_ack'):ack.append(dict(received_ns=t.to_nsec(),message=json.loads(m.data)))
  elif topic.endswith('detector_status'):status.append(dict(received_ns=t.to_nsec(),message=json.loads(m.data)))
  elif topic=='/grasp_6d/status':plans.append(dict(received_ns=t.to_nsec(),message=m.data))
  elif topic=='/grasp/state':phases.append(dict(received_ns=t.to_nsec(),state=m.state,message=m.message,active=m.active,success=m.success))
  else:
   ns=m.header.stamp.to_nsec()
   if 1791523440000000000<=ns<=1791523448000000000:images.setdefault(ns,[]).append(topic)
arrivals=[]
for r in req:
 x=r['message']
 if x['state']!='requalifying':continue
 aa=[a for a in ack if a['message']['request_id']==x['request_id'] and a['message'].get('qualified') is True]
 assert {a['message']['participant'] for a in aa}=={'unknown','remote'}
 ns=max(a['received_ns'] for a in aa);ss=aa[0]['message']['source_stamps_ns'];assert all(a['message']['source_stamps_ns']==ss for a in aa)
 arrivals.append(dict(view_index=x['view_index'],lease_id=x['lease_id'],view_plan_id=x['view_plan_id'],arrival_stamp_ns=x['arrival_stamp_ns'],both_qualified_ack_ns=ns,qualification_elapsed_sec=(ns-x['arrival_stamp_ns'])*1e-9,source_stamps_ns=ss,latest_source_age_at_ack_sec=(ns-ss[-1])*1e-9,settle=x['settle']))
loss=next(s for s in status if s['message'].get('generation')==15 and s['message'].get('target_lost') is True)
previous=[s for s in status if s['received_ns']<loss['received_ns'] and s['message'].get('state')=='ready']
last=previous[-1]
summary=dict(record=str(REC),manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),sealed_manifest={k:manifest.get(k) for k in ['state','result','recorder_closed','failure_reason','completed_at']},selection_generation=15,arrivals=arrivals,measurement_requests=req,measurement_acks=ack,task_states=phases,planner_status=plans,first_loss=loss,last_ready=last,third_view_post_arrival_statuses=[s for s in status if arrivals[-1]['arrival_stamp_ns']<=s['received_ns']<=loss['received_ns']],saved_third_arrival_images=[dict(source_stamp_ns=k,topics=v) for k,v in sorted(images.items())],first_loss_exact_rgbd_saved=all(any(t.endswith('/color') or '/color/' in t for t in images.get(loss['message']['source_stamp_ns'],[])) for _ in [0]) and any(t.endswith('/depth') or '/depth/' in t for t in images.get(loss['message']['source_stamp_ns'],[])),no_ros_calls=True,production_modified=False,physical_grasp_verified=False,interpretation=['Three real observation motions reached and all three original RGB/support/identity confirmation handshakes passed.','After view3 qualified and normal tracking resumed, metric ambiguity RGB identity failed; do not reclassify it as arrival confirmation or supplemental-plan handoff failure.','Exact failing RGB-D availability is recorded explicitly; absent raw frames cannot be reconstructed or replaced silently by neighbouring images.'])
OUT.mkdir(exist_ok=True);p=OUT/'arrivals_and_terminal.json';p.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),arrivals=arrivals,first_loss=dict(source_stamp_ns=loss['message']['source_stamp_ns'],received_ns=loss['received_ns'],detail=loss['message']['detail'],failed_stage=loss['message'].get('frame_timing',{}).get('failed_stage'),source_age_sec=loss['message'].get('frame_timing',{}).get('status_source_age_sec')),last_ready_ns=last['message']['source_stamp_ns'],first_loss_exact_rgbd_saved=summary['first_loss_exact_rgbd_saved'],saved_stamps=sorted(images)),ensure_ascii=False,indent=2))
