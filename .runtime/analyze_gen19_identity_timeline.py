#!/usr/bin/env python3
from pathlib import Path
from collections import Counter
import json, hashlib, rosbag

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=BASE/'ros_log/full_latest_ros_20261009T003407Z'; OUT=NEW/'gen19_handoff_analysis'
base=json.loads((OUT/'first_loss_analysis.json').read_text()); REC=Path(base['record'])
start=base['selection_stamp_ns'];end=base['first_loss']['received_ns']
events=[];objects={};masks={};gui=[];epoch=[];geometry=[]
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/gui/joint_direct_mode','/alicia_d/control_reference_epoch','/perception/object','/perception/object_mask','/grasp_6d/object_geometry']):
  received=t.to_nsec()
  if topic=='/gui/joint_direct_mode':gui.append(dict(received_ns=received,value=m.data));continue
  if topic=='/alicia_d/control_reference_epoch':epoch.append(dict(received_ns=received,epoch_ns=m.stamp.to_nsec(),frame_id=m.frame_id));continue
  ns=m.header.stamp.to_nsec()
  if topic=='/grasp_6d/object_geometry':
   if start<=received<=end:geometry.append(dict(received_ns=received,source_stamp_ns=ns,valid=m.valid,track_id=m.target_track_id,reason=m.failure_reason))
  elif start<=ns<=end:
   if topic.endswith('/object'):objects[ns]=dict(joined_bag_received_ns=received,detected=m.detected,source_stamp_ns=ns)
   else:masks[ns]=dict(joined_bag_received_ns=received,source_stamp_ns=ns,frame_id=m.header.frame_id,nonempty=any(m.data))
rows=[]
for item in base['all_gen19_status_before_first_loss']:
 m=item['message'];ns=m.get('source_stamp_ns');q=m.get('source_qualification')
 rows.append(dict(received_ns=item['received_ns'],source_stamp_ns=ns,state=m['state'],detail=m['detail'],target_lost=m.get('target_lost'),initial_station_identity_template=m.get('initial_station_identity_template'),initial_station_measurement=m.get('initial_station_measurement'),source_qualification=q,object=objects.get(ns),mask=masks.get(ns),geometry_metrics=m.get('geometry_metrics'),failed_stage=m.get('frame_timing',{}).get('failed_stage')))
counter=Counter((r['track_id'],r['valid'],r['reason']) for r in geometry)
report=dict(record=str(REC),sealed_manifest=base['sealed_manifest'],selection_stamp_ns=start,gui_events=gui,hardware_epoch_events=epoch,accepted_joint_evidence=base['accepted_joint_feedback'],perception_status_rows=rows,remote_geometry_events=geometry,remote_geometry_reason_counts=[dict(track_id=k[0],valid=k[1],reason=k[2],count=v) for k,v in counter.items()],first_false=next(r for r in rows if r.get('object') and r['object']['detected'] is False and r.get('source_qualification')),typed_recoveries=[r for r in rows if (r.get('source_qualification') or {}).get('state')=='recovered_unique_measured_frame'],first_loss=base['first_loss'],current_lastgood_ages=base['age_at_terminal_observation'],limitations=['Object/Mask bag timestamps are the recorder exact-four-source join time, not individual remote callback timestamps.','The producer marker underlying invalidated_reason is not recorded; .8-second expiry is a code-and-timestamp inference, not an extracted private field.','Remote target epoch changes are distinct from the unchanged hardware control_reference_epoch.'],ros_calls=False,temperature_read=False,production_modified=False)
p=OUT/'identity_timeline.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),geometry_reason_counts=report['remote_geometry_reason_counts'],gui=gui,epochs=epoch,first_false=report['first_false'],recovery_sources=[r['source_stamp_ns'] for r in report['typed_recoveries']]),ensure_ascii=False,indent=2))
