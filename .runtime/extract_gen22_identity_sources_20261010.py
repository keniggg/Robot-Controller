#!/usr/bin/env python3
"""Exact sealed GEN22 RGB identity inputs; no ROS graph or device calls."""
from pathlib import Path
import hashlib,json
import cv2,numpy as np,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.rgb_observation_source import bracket_feedback
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=EXT/'ros_log/full_latest_restart_20261010T002347Z'
REC=EXT/'grasp_records/20261010T132545.078718Z_69bac5676950480b80e6cae5cdc4f0ce'
OUT=NEW/'gen22_analysis/replay_inputs'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
executing=1791638832768291234
epoch=1791635727396883411
statuses=[];cams=[];feedback=[];statics=[];dynamic=[];images={};masks={};metadata={};objects={}
topics=['/tf','/tf_static','/grasp_record/camera_runtime_metadata','/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask','/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth','/experiment/observation_rgbd/metadata','/perception/unknown/detector_status','/alicia_d/accepted_joint_states','/perception/object']
def ser(x):
 if hasattr(x,'__slots__'):return {k:ser(getattr(x,k)) for k in x.__slots__}
 if isinstance(x,(list,tuple)):return [ser(v) for v in x]
 return x
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
 for topic,m,t in bag.read_messages(topics=topics):
  received=t.to_nsec()
  if topic in ('/tf','/tf_static'):(statics if topic=='/tf_static' else dynamic).extend(m.transforms)
  elif topic=='/grasp_record/camera_runtime_metadata':
   d=json.loads(m.data)
   if d.get('parameters'):cams.append(dict(received_ns=received,metadata=d))
  elif topic=='/perception/unknown/detector_status':
   d=json.loads(m.data)
   if d.get('generation')==22:statuses.append(dict(received_ns=received,message=d))
  elif topic=='/alicia_d/accepted_joint_states':
   if m.header.frame_id=='sdk_measured':
    q=dict(zip(m.name,m.position));feedback.append((m.header.stamp.to_nsec(),[q['Joint%d'%i] for i in range(1,7)]))
  elif topic=='/experiment/observation_rgbd/metadata':
   d=json.loads(m.data);metadata[d['source_stamp_ns']]=d
  elif topic=='/perception/object_mask':masks[m.header.stamp.to_nsec()]=m
  elif topic=='/perception/object':objects[m.header.stamp.to_nsec()]=ser(m)
  else:
   ns=m.header.stamp.to_nsec();key='color' if '/color/' in topic or topic.endswith('/color') else 'depth'
   images.setdefault(ns,{})[key]=(m,topic,received)
acquired=next(r for r in statuses if r['message'].get('detail')=='acquired')
anchor_ns=acquired['message']['anchor_source_stamp_ns']
first_executing_status=next(r for r in statuses if r['message'].get('observation_measurement',{}).get('state')=='executing' and r['message'].get('observation_measurement',{}).get('request',{}).get('view_index')==2)
last=max((r for r in statuses if r['received_ns']<first_executing_status['received_ns'] and r['message'].get('state')=='ready'),key=lambda r:r['message']['source_stamp_ns'])
last_ns=last['message']['source_stamp_ns']
arrival={r['message']['source_stamp_ns']:r for r in statuses if r['message'].get('observation_measurement',{}).get('state')=='requalifying' and r['message'].get('source_stamp_ns',0)>1791638841662415266}
targets={anchor_ns:'anchor',last_ns:'last_pre_motion',1791638841740801811:'first_failure'}
for ns in arrival:targets.setdefault(ns,'arrival')
for ns in images:
 if 1791638841662415266 < ns <1791638844662415266:targets.setdefault(ns,'recorded_arrival_not_online_evaluated')
previous=max((r for r in statuses if r['message'].get('state')=='ready' and r['message'].get('source_stamp_ns',0)<last_ns),key=lambda r:r['message']['source_stamp_ns'])
targets.setdefault(previous['message']['source_stamp_ns'],'previous_ready_reference_probe')
buf=tf2_py.BufferCore(rospy.Duration(600))
for t in statics:buf.set_transform_static(t,'sealed_gen22')
for t in sorted(dynamic,key=lambda t:t.header.stamp.to_nsec()):buf.set_transform(t,'sealed_gen22')
bridge=CvBridge();OUT.mkdir(exist_ok=True);rows=[];missing=[]
def save(name,data):
 p=OUT/name;p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
for ns,role in sorted(targets.items()):
 if set(images.get(ns,{}))!={'color','depth'}:
  missing.append(dict(source_stamp_ns=ns,role=role,available_components=list(images.get(ns,{}))));continue
 pair=images[ns];c,d=pair['color'][0],pair['depth'][0]
 rgb=bridge.imgmsg_to_cv2(c,'bgr8');raw=bridge.imgmsg_to_cv2(d,'passthrough')
 camrow=max((r for r in cams if r['received_ns']<=pair['color'][2]),key=lambda r:r['received_ns'],default=cams[0]);cam=camrow['metadata']['parameters']
 K=np.asarray([cam[k] for k in ('fx','fy','cx','cy')]);valid=rectification_rgb_valid_mask(cam,rgb.shape[:2])
 tr=buf.lookup_transform_core('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
 T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w]);T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
 q,before,after=bracket_feedback(feedback,ns,epoch)
 payload=dict(color=rgb,depth_raw=raw,depth_scale=float(cam['depth_scale']) if d.encoding!='32FC1' else 1.,intrinsics=K,K=K,T_base_camera=T,q=q,source_stamp_ns=np.int64(ns))
 if valid is not None:payload['rgb_valid']=valid
 prefix=role+'_'+str(ns);path=OUT/(prefix+'.npz')
 row=dict(role=role,source_stamp_ns=ns,color_topic=pair['color'][1],depth_topic=pair['depth'][1],color_received_ns=pair['color'][2],depth_received_ns=pair['depth'][2],color_original_encoding=c.encoding,depth_encoding=d.encoding,feedback_before=before,feedback_after=after,color_bytes_sha256=hashlib.sha256(c.data).hexdigest(),depth_bytes_sha256=hashlib.sha256(d.data).hexdigest(),recorded_refined_mask_available=ns in masks,rgb_png=str(OUT/(prefix+'_rgb.png')),path=str(path),camera_metadata_received_ns=camrow['received_ns'])
 if ns in masks:
  mask=bridge.imgmsg_to_cv2(masks[ns],'mono8');payload['recorded_refined_mask']=mask
  y,x=np.nonzero(mask);row.update(mask_pixels=len(x),mask_bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)],mask_bytes_sha256=hashlib.sha256(mask.tobytes()).hexdigest(),mask_png=str(OUT/(prefix+'_recorded_mask.png')))
  cv2.imwrite(row['mask_png'],mask)
 cv2.imwrite(row['rgb_png'],rgb);np.savez_compressed(str(path),**payload);row['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
 if ns in arrival:row['detector_status']=arrival[ns]
 if ns in metadata:row['recording_metadata']=metadata[ns]
 if ns in objects:row['recorded_object']=objects[ns]
 rows.append(row)
config=json.loads((REC/'perception_execution_configuration.json').read_text())
report=dict(record=str(REC),recorder_closed=True,selection_stamp_ns=1791638746764785051,executing_stamp_ns=executing,first_producer_executing_status_received_ns=first_executing_status['received_ns'],reference_selection_basis='Last READY on producer status connection before first executing status, because request callback takes producer lock after in-flight frame finishes; request publication time is not producer receipt time.',control_epoch_ns=epoch,anchor_source_stamp_ns=anchor_ns,last_pre_motion_source_stamp_ns=last_ns,files=rows,missing_exact_sources=missing,metadata=dict(anchor_status=save('anchor_status.json',acquired),last_pre_motion_status=save('last_pre_motion_status.json',last),camera_configuration=save('camera_configuration.json',cams[0]['metadata']['parameters']),camera_metadata=save('camera_metadata.json',cams),unknown_perception_configuration=save('unknown_perception_configuration.json',dict(source=str(REC/'perception_execution_configuration.json'),read_start_ns=config['read_start_ns'],read_end_ns=config['read_end_ns'],parameters=config['parameters']['/unknown_perception'])),statuses=save('detector_statuses.json',statuses)),no_ros_calls=True,no_hardware_access=True,no_temperature_read=True,limitations=['Missing exact source frames are listed; no nearest-source substitution.','Masks are recorded outputs only where published; failed-arrival derived masks require production replay.','Recorded camera configuration snapshots are metadata, not image-synchronized calibration measurement.'])
final=save('manifest.json',report)
print(json.dumps(dict(manifest=final,anchor_ns=anchor_ns,last_ns=last_ns,frames=[{k:r[k] for k in ('role','source_stamp_ns','recorded_refined_mask_available')} for r in rows],missing=missing),ensure_ascii=False))
