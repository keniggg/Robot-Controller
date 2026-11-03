"""Closed-record exact-TF/segmentation audit; no ROS master or robot I/O."""
import json,hashlib,os
from pathlib import Path
from types import SimpleNamespace as NS
from collections import OrderedDict
import numpy as np,cv2,rosbag,rospy,tf2_ros
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.tabletop import Config,_audit_current_support,segment_with_support_fallback,_matching_candidates
from alicia_grasp_modes.rgb_instance import refine_candidate
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');assert Path('/media/zhuyupei/SanStand').is_mount()
p=root/'ros_log/feedforward_speed_grasp_20260929T085024Z';out=p/'offline_alignment_replay';out.mkdir(exist_ok=False)
formal=root/'grasp_records/20260929T085024.777917Z_70bd17d5d4454ea48f2cf06c16c36518/keyframes.bag'
buf=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False);accepted=[];tf_arrival=[];ref_status=None;terminal=None
with rosbag.Bag(str(p/'scene_diagnostic.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/tf','/tf_static','/alicia_d/accepted_joint_states','/perception/unknown/detector_status','/grasp/state']):
  if topic in ('/tf','/tf_static'):
   for tr in m.transforms:
    (buf.set_transform_static if topic=='/tf_static' else buf.set_transform)(tr,'recorded_accepted_rsp')
    if tr.header.frame_id=='base_link':tf_arrival.append((tr.header.stamp.to_nsec(),t.to_nsec()))
  elif topic.endswith('accepted_joint_states'):accepted.append((m.header.stamp.to_nsec(),t.to_nsec()))
  elif topic.endswith('detector_status'):
   s=json.loads(m.data)
   if ref_status is None and s.get('generation')==13 and s.get('first_loss'):ref_status=s
  elif not m.active and 'FINAL_REFINE_REFERENCE_UNAVAILABLE' in m.message:terminal=m.header.stamp.to_nsec()
assert ref_status and terminal
with rosbag.Bag(str(formal)) as bag:
 camera=json.loads(next(bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']))[1].data)['parameters']
anchor=NS(position_base=np.array(ref_status['anchor_position_base']),extent_m=np.array(ref_status['anchor_extent_m']),support_footprint_base=np.array(ref_status['reference_support_footprint_base']));ref=np.array(ref_status['reference_support_plane_base']);K=[camera[n] for n in ['fx','fy','cx','cy']];cfg=Config();last=anchor
bridge=CvBridge();cv2.setNumThreads(1);rgbs=OrderedDict();rows=[];loss=ref_status['first_loss']['source_stamp_ns'];tf_stamps=np.array([a for a,b in tf_arrival],dtype=np.int64)
with rosbag.Bag(str(p/'scene_diagnostic.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
  ns=m.header.stamp.to_nsec()
  if ns<loss-1000000000:continue
  if 'color' in topic:
   rgbs[ns]=m
   while len(rgbs)>3:rgbs.popitem(last=False)
   continue
  rgb=rgbs.pop(ns,None)
  if rgb is None:continue
  row=dict(source_ns=ns,seconds_after_terminal=(ns-terminal)*1e-9)
  try:
   tr=buf.lookup_transform_core('base_link','camera_link',m.header.stamp);q=tr.transform.rotation;z=tr.transform.translation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[z.x,z.y,z.z]
   depth=bridge.imgmsg_to_cv2(m,'passthrough').astype(np.float32)*camera['depth_scale'];color=bridge.imgmsg_to_cv2(rgb,'bgr8')
   i=int(np.searchsorted(tf_stamps,ns));row['exact_tf_ready_source_age_sec']=(tf_arrival[i][1]-ns)*1e-9;row['tf_sample_gap_sec']=(tf_stamps[i]-tf_stamps[i-1])*1e-9
   row['support']=_audit_current_support(depth,K,F,cfg,anchor,ref)
   seg=segment_with_support_fallback(depth,K,F,cfg,anchor=anchor,last=last,reference_plane_base=ref);matching=_matching_candidates(seg.candidates,anchor,last,cfg);row['matching_count']=len(matching)
   if len(matching)==1:
    c=refine_candidate(color,depth,K,F,matching[0][1],cfg);row.update(rgb_accepted=True,position_base=c.position_base.tolist(),quality=c.quality);last=c
  except Exception as e:row['error']=str(e)
  rows.append(row)
summary=dict(frames=len(rows),rgb_accepted=sum(bool(r.get('rgb_accepted')) for r in rows),support_available=sum(bool(r.get('support',{}).get('support_current_audit_available')) for r in rows),errors=[{k:r[k] for k in ('source_ns','error')} for r in rows if 'error' in r],max_support_error_m=max(r.get('support',{}).get('support_reference_max_distance_m',0) for r in rows),max_exact_tf_ready_source_age_sec=max(r.get('exact_tf_ready_source_age_sec',0) for r in rows),max_accepted_gap_sec=float(np.diff(np.array(accepted,dtype=np.int64)[:,0]).max()*1e-9))
report=dict(scope='Recorded actual accepted-sample TF, same exact RGB-D images, unchanged anchor and original support/RGB gates; no failure reset or live actions',summary=summary,original_first_loss=ref_status['first_loss'],original_source_max_age_sec=.8,original_support_max_distance_m=.004,rows=rows,hardware_commands_sent=0,production_changed=False,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary,indent=2))
