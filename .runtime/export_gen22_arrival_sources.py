#!/usr/bin/env python3
"""Original sealed RGB/depth/mask and TF, no segmentation or live ROS."""
from pathlib import Path
import json, hashlib
import cv2, numpy as np, rosbag, rospy, tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=EXT/'ros_log/full_latest_ros_20261009T003407Z'
REC=EXT/'grasp_records/20261009T095400.350634Z_f3717a52509b4eb4a1e6354512f1b17d'
OUT=NEW/'gen22_handoff_analysis/source_images'
summary=json.loads((OUT.parent/'arrival_and_terminal.json').read_text())
status=json.loads((OUT.parent/'detector_status.json').read_text())
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
start=summary['arrival_window']['start_ns']; deadline=summary['arrival_window']['deadline_ns']
request=summary['arrival_request']; executing=int(request['lease_id'].rsplit(':',1)[1])
acquired=next(r['message'] for r in status if r['message'].get('detail')=='acquired')
anchor_ns=acquired['anchor_source_stamp_ns']
last_ns=summary['first_loss']['message']['first_loss']['last_good_source_stamp_ns']
last=next(r for r in status if r['message'].get('state')=='ready' and r['message']['source_stamp_ns']==last_ns)
assert last_ns < executing
refs={anchor_ns,last_ns}
images={};masks={};static=[];dynamic=[];camera=[]
topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/supervisor/camera/color/image_raw',
 '/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,msg,received in bag.read_messages(topics=topics):
  if topic=='/tf':dynamic.extend(msg.transforms)
  elif topic=='/tf_static':static.extend(msg.transforms)
  elif topic.endswith('camera_runtime_metadata'):
   row=json.loads(msg.data)
   if row.get('parameters'):camera.append(row['parameters'])
  else:
   ns=msg.header.stamp.to_nsec()
   if ns not in refs and not start<ns<deadline:continue
   if topic.endswith('object_mask'):masks[ns]=msg
   else:
    key='rgb' if '/color/' in topic or topic.endswith('/color') else 'depth'
    images.setdefault(ns,{})[key]=(msg,topic,received.to_nsec())
cam=camera[0]
assert all(all(c[k]==cam[k] for k in ('fx','fy','cx','cy','width','height','depth_scale')) for c in camera)
buf=tf2_py.BufferCore(rospy.Duration(600.))
for t in static:buf.set_transform_static(t,'sealed_gen22')
for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(t,'sealed_gen22')
bridge=CvBridge();OUT.mkdir(exist_ok=True);rows=[]
online={r['source_stamp_ns']:r['reason'] for r in summary['arrival_rejections']}
for ns,pair in sorted(images.items()):
 if set(pair)!={'rgb','depth'}:continue
 rgb=bridge.imgmsg_to_cv2(pair['rgb'][0],'bgr8');raw=bridge.imgmsg_to_cv2(pair['depth'][0],'passthrough')
 tf=buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
 q=tf.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
 payload=dict(rgb=rgb,depth_raw=raw,T_base_camera=T)
 role='anchor' if ns==anchor_ns else 'last_pre_motion' if ns==last_ns else 'arrival'
 prefix=role+'_'+str(ns)
 row=dict(role=role,source_stamp_ns=ns,source_shape=list(rgb.shape),online_reason=online.get(ns),
  online_evaluated=ns in online,rgb_topic=pair['rgb'][1],depth_topic=pair['depth'][1],
  rgb_received_ns=pair['rgb'][2],depth_received_ns=pair['depth'][2],
  rgb_frame_id=pair['rgb'][0].header.frame_id,depth_frame_id=pair['depth'][0].header.frame_id,
  rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),raw_depth_sha256=hashlib.sha256(raw.tobytes()).hexdigest(),
  rgb_png=prefix+'_rgb.png',npz=prefix+'.npz')
 if ns in masks:
  mask=bridge.imgmsg_to_cv2(masks[ns],'mono8');payload['refined_mask']=mask
  y,x=np.nonzero(mask);row.update(mask_pixels=len(x),mask_bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],
   mask_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),mask_png=prefix+'_mask.png')
  cv2.imwrite(str(OUT/row['mask_png']),mask)
  over=rgb.copy();cv2.drawContours(over,cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,(0,255,0),1)
  cv2.imwrite(str(OUT/(prefix+'_overlay.png')),over)
 cv2.imwrite(str(OUT/row['rgb_png']),rgb);np.savez_compressed(str(OUT/row['npz']),**payload);rows.append(row)
arrivals=[r for r in rows if r['role']=='arrival']
report=dict(record=str(REC),camera=cam,anchor_source_stamp_ns=anchor_ns,last_pre_motion_source_stamp_ns=last_ns,
 anchor_status=acquired,last_pre_motion_status=last,original_arrival_request=request,files=rows,
 online_sources_missing_rgbd=sorted(set(online)-{r['source_stamp_ns'] for r in arrivals}),
 first_arrival_png=str(OUT/arrivals[0]['rgb_png']),last_arrival_png=str(OUT/arrivals[-1]['rgb_png']),
 no_segmentation=True,no_ros_calls=True,limitation='Original exact 5Hz saved RGB-D pairs; missing online evaluated sources are listed. Arrival masks are not fabricated.')
(OUT/'manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('anchor_source_stamp_ns','last_pre_motion_source_stamp_ns','online_sources_missing_rgbd','first_arrival_png','last_arrival_png')},ensure_ascii=False))
print('PAIRS',len(arrivals),'FILES',str(OUT/'manifest.json'))
