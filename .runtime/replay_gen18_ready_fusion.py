#!/usr/bin/env python3
"""Actual sealed five-frame source joins and original pure stable fusion gates."""
from pathlib import Path
from dataclasses import asdict
import json, hashlib, re, time
import numpy as np
import cv2, rosbag, yaml
from cv_bridge import CvBridge
from alicia_flexible_grasp.vision.rgbd_snapshot import RgbdSample, fuse_stable_samples, mask_iou
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=BASE/'ros_log/full_latest_ros_20261009T003407Z'
REC=BASE/'grasp_records/20261009T061104.522156Z_f598aaa2d91444a99bdf1a87af163419'
OUT=NEW/'gen18_handoff_analysis/ready_fusion_replay'
manifest=json.loads((REC/'manifest.json').read_text());assert manifest['recorder_closed'] and manifest['state']=='complete'
analysis=json.loads((NEW/'gen18_handoff_analysis/first_loss_analysis.json').read_text())
status={r['message']['source_stamp_ns']:r for r in analysis['all_gen18_status_before_first_loss'] if r['message'].get('source_stamp_ns')}
ready=sorted(n for n,r in status.items() if r['message']['state']=='ready')
recoveries=[n for n in ready if status[n]['message'].get('source_qualification',{}).get('state')=='recovered_unique_measured_frame']
frames={};joints=[];cameras=[];geometries=[]
topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object','/perception/object_mask']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=topics+['/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata','/grasp_6d/object_geometry']):
  if topic in topics:
   n=m.header.stamp.to_nsec()
   if n in ready:frames.setdefault(n,{})[topic]=(m,t.to_nsec())
  elif topic.endswith('accepted_joint_states'):
   if m.header.frame_id=='sdk_measured':joints.append(m)
  elif topic.endswith('camera_runtime_metadata'):
   p=json.loads(m.data).get('parameters')
   if p:cameras.append((t.to_nsec(),p))
  elif m.valid and analysis['selection_stamp_ns']<=t.to_nsec()<=analysis['first_loss']['received_ns']:
   geometries.append(dict(stamp_ns=m.header.stamp.to_nsec(),track_id=m.target_track_id))
assert {g['track_id'] for g in geometries}=={'g1-t7'}
identity=TargetTrackIdentity(7,'g1-t7')
cfg=yaml.safe_load(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_text())['grasp_6d']['remote']
camera=cameras[0][1]
fusion=dict(require_mask=True,min_mask_iou=cfg['planning_mask_min_iou'],max_centroid_shift_px=cfg['planning_mask_max_centroid_shift_px'],max_joint_delta_rad=cfg['planning_max_joint_delta_rad'],erosion_px=cfg['mask_erosion_px'],depth_scale=camera['depth_scale'],depth_min_m=.03,depth_max_m=2.,mad_scale=cfg['depth_mad_scale'],mad_absolute_floor_m=cfg['depth_mad_absolute_floor_m'],internal_hole_max_area_px=cfg['mask_internal_hole_max_area_px'])
bridge=CvBridge();cv2.setNumThreads(1);samples={};joins={}
for n in ready:
 f=frames.get(n,{})
 if set(f)!=set(topics):continue
 col,dep,obj,mask=[f[t][0] for t in topics]
 assert obj.detected and obj.header.stamp.to_nsec()==obj.pose_camera.header.stamp.to_nsec()==obj.pose_base.header.stamp.to_nsec()==n
 assert col.header.stamp.to_nsec()==dep.header.stamp.to_nsec()==mask.header.stamp.to_nsec()==n
 q=min(joints,key=lambda j:abs(j.header.stamp.to_nsec()-n))
 samples[n]=RgbdSample(bridge.imgmsg_to_cv2(col,'bgr8'),bridge.imgmsg_to_cv2(dep,'passthrough'),bridge.imgmsg_to_cv2(mask,'mono8'),(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj,n/1e9,dep.header.frame_id,np.array(q.position[:6]),n,identity.epoch,identity)
 join=max(t for _,t in f.values());st=status[n]
 joins[n]=dict(source_stamp_ns=n,ready_status_received_ns=st['received_ns'],ready_detail=st['message']['detail'],qualification=st['message'].get('source_qualification'),all_original_headers_exact=True,object_detected=True,joined_bag_write_ns=join,bag_join_source_age_sec=(join-n)*1e-9,ready_status_source_age_sec=(st['received_ns']-n)*1e-9,joint_feedback_stamp_ns=q.header.stamp.to_nsec(),joint_source_distance_sec=abs(q.header.stamp.to_nsec()-n)*1e-9,q=q.position[:6],object_frame=obj.header.frame_id,image_frame=dep.header.frame_id,mask_sha256=hashlib.sha256(samples[n].object_mask.tobytes()).hexdigest())
requests=[]
for line in (NEW/'initial_station_current_support_deployment/consumers.log').read_text().splitlines():
 if 'remote 6D pipeline metrics: ' in line:
  x=json.loads(line.split('remote 6D pipeline metrics: ',1)[1])
  if x.get('request_id') in (2,4):
   complete=float(re.search(r'\[(179\d+\.\d+)\]',line).group(1))
   requests.append(dict(request_id=x['request_id'],source_stamp_ns=x['snapshot_evidence']['source_stamp_ns'],completion_sec=complete,submission_approx_ns=round((complete-x['end_to_end_ms']/1000.)*1e9),metrics=x))
def run(stamps,at_ns,label):
 missing=[n for n in stamps if n not in samples]
 row=dict(label=label,source_stamp_ns=stamps,evaluation_ros_ns=at_ns,missing_exact_joins=missing,source_join_rows=[joins[n] for n in stamps if n in joins],latest_source_age_sec=(at_ns-stamps[-1])*1e-9,historical_span_sec=(stamps[-1]-stamps[0])*1e-9)
 if missing:return row
 arr=[samples[n] for n in stamps];began=time.perf_counter();f=fuse_stable_samples(arr,**fusion)
 centers=[np.array([np.mean(np.nonzero(s.object_mask)[1]),np.mean(np.nonzero(s.object_mask)[0])]) for s in arr]
 row.update(fusion_ok=f.ok,failure_code=f.failure_code,failure_reason=f.failure_reason,quality=asdict(f.quality),processing_wall_sec=time.perf_counter()-began,pairwise_iou=[mask_iou(a.object_mask,b.object_mask) for a,b in zip(arr,arr[1:])],maximum_centroid_shift_px=max(float(np.linalg.norm(a-b)) for a in centers for b in centers),joint_max_delta_rad=float(np.max(np.ptp(np.stack([s.joint_positions for s in arr]),axis=0))),snapshot_sample_stamp_ns=list(f.sample_stamp_ns))
 return row
actual=[dict(request=x,**run(x['source_stamp_ns'],x['submission_approx_ns'],'actual_request_'+str(x['request_id']))) for x in requests]
windows=[]
for recovery in recoveries:
 idx=ready.index(recovery)
 for end in range(idx,min(idx+3,len(ready))):
  stamps=ready[end-4:end+1]
  if len(stamps)!=5:continue
  n=stamps[-1];observed=max(status[n]['received_ns'],max(joins[x]['joined_bag_write_ns'] for x in stamps if x in joins))
  pending_after_recovery=[r for r in analysis['all_gen18_status_before_first_loss'] if status[recovery]['received_ns']<r['received_ns']<=observed and r['message']['state']!='ready']
  row=run(stamps,observed,'typed_recovery_history_counterfactual');row.update(typed_recovery_source_stamp_ns=recovery,typed_recovery_status_received_ns=status[recovery]['received_ns'],newest_is_recovered_or_later=n>=recovery,pending_intervened=bool(pending_after_recovery),original_all_after_cutoff_would_include=all(s>=recovery for s in stamps),new_station_receipt_present=False)
  windows.append(row)
out=dict(record=str(REC),sealed_manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),original_fusion_parameters=fusion,configuration_provenance='Unchanged deployed repository grasp_params.yaml fusion parameters; camera depth scale from recorded metadata; perception depth range .03..2 is production default and source config. No live parameter read.',target_identity_provenance=dict(actual_valid_geometry=geometries,identity_used=asdict(identity)),actual_requests=actual,recovery_history_counterfactuals=windows,missing_ready_raw_sources=[n for n in ready if n not in samples],fixed_joint_evidence=analysis['accepted_joint_feedback'],source_sha256={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in ['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_snapshot.py','src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml']},limitations=['Old sealed data has no new fixed-station receipt, so post-recovery history windows are pure measured-data counterfactuals and confer no runtime authority.','Four source topics are recorded together on exact join; their bag timestamps are recorder join times, not individual remote callback arrival times. Status receipt is another independently recorded topic.','Request submit time is approximated from completion log minus total latency; no unrecorded per-callback remote arrival time is invented.','No target geometry, source freshness, support or new lifecycle gate is removed by this replay. Passing stable fusion alone is not a feasible or executable plan.'],ros_calls=False,production_modified=False)
OUT.mkdir(parents=True,exist_ok=True);p=OUT/'result.json';p.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),actual=[{k:r.get(k) for k in ['label','fusion_ok','failure_reason','latest_source_age_sec','pairwise_iou','joint_max_delta_rad','missing_exact_joins']} for r in actual],recoveries=[{k:r.get(k) for k in ['typed_recovery_source_stamp_ns','source_stamp_ns','fusion_ok','failure_reason','latest_source_age_sec','historical_span_sec','pairwise_iou','pending_intervened']} for r in windows]),indent=2))
