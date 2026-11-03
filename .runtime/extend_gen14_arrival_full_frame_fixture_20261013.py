#!/usr/bin/env python3
"""One offline range extraction for actual latest-frame queue qualification replay."""
from pathlib import Path
import json,hashlib,importlib.util
import cv2,numpy as np,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade');spec=importlib.util.spec_from_file_location('extract',W/'.runtime/extract_gen14_arrival_independent_20261013.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def main():
 O=m.O/'full_arrival_frames';O.mkdir(exist_ok=False);f=json.loads((m.O/'fixture.json').read_text());camera=f['camera'];raw={};dynamic=[];static=[];sdk=[];bridge=CvBridge();sidecar=m.N/'attempt_2_full_frames';closed=json.loads((sidecar/'result.json').read_text());assert closed['recording_closed']
 # Read only the already indexed source window and whitelisted pose/TF data.
 with rosbag.Bag(str(m.R/'keyframes.bag'))as bag:
  for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static','/alicia_d/accepted_joint_states']):
   if topic=='/tf':dynamic.extend(dict(received_ns=t.to_nsec(),transform=x)for x in msg.transforms)
   elif topic=='/tf_static':static.extend(dict(received_ns=t.to_nsec(),transform=x)for x in msg.transforms)
   else:sdk.append(dict(received_ns=t.to_nsec(),stamp_ns=msg.header.stamp.to_nsec(),names=msg.name,positions=msg.position))
 with rosbag.Bag(str(sidecar/'full_frames.bag'))as bag:
  for topic,msg,t in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/tf','/alicia_d/accepted_joint_states'],start_time=rospy.Time(m.ARR//10**9-1,0),end_time=rospy.Time(m.DEAD//10**9+2,0)):
   if topic=='/tf':dynamic.extend(dict(received_ns=t.to_nsec(),transform=x)for x in msg.transforms);continue
   if topic.endswith('accepted_joint_states'):sdk.append(dict(received_ns=t.to_nsec(),stamp_ns=msg.header.stamp.to_nsec(),names=msg.name,positions=msg.position));continue
   ns=msg.header.stamp.to_nsec()
   if m.ARR<ns<m.DEAD+1000000000:raw.setdefault(ns,{})['rgb'if'/color/'in topic else'depth']=(msg,t.to_nsec(),topic)
 # Duplicate transport deliveries are identical source observations; retain earliest receive.
 unique={}
 for x in sdk:
  key=x['stamp_ns'];old=unique.get(key)
  if old is not None:assert old['names']==x['names']and old['positions']==x['positions']
  if old is None or x['received_ns']<old['received_ns']:unique[key]=x
 sdk=sorted(unique.values(),key=lambda x:x['received_ns']);unique={}
 for row in dynamic:
  x=row['transform'];key=(x.header.stamp.to_nsec(),x.header.frame_id,x.child_frame_id);old=unique.get(key)
  if old is not None:assert m.plain(old['transform'])==m.plain(x)
  if old is None or row['received_ns']<old['received_ns']:unique[key]=row
 dynamic=sorted(unique.values(),key=lambda x:x['received_ns']);buffer=tf2_py.BufferCore(rospy.Duration(600))
 for row in static:buffer.set_transform_static(row['transform'],'sealed_input')
 for row in dynamic:buffer.set_transform(row['transform'],'sealed_input')
 frames=[]
 for ns,pair in sorted(raw.items()):
  assert set(pair)=={'rgb','depth'}
  rgb_msg,rns,rtopic=pair['rgb'];depth_msg,dns,dtopic=pair['depth'];rgb=bridge.imgmsg_to_cv2(rgb_msg,'bgr8');rd=bridge.imgmsg_to_cv2(depth_msg,'passthrough');depth=np.asarray(rd,dtype=np.float32)*camera['depth_scale']if depth_msg.encoding in('16UC1','mono16')else np.asarray(rd,dtype=np.float32)
  exact=buffer.lookup_transform_core('base_link','camera_link',rgb_msg.header.stamp);t=exact.transform;q=t.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
  data=dict(rgb=rgb,raw_depth=rd,depth=depth,T_base_camera=T);valid=rectification_rgb_valid_mask(camera,depth.shape)
  if valid is not None:data['rgb_valid']=valid
  path=O/f'{ns}_source.npz';np.savez_compressed(path,**data)
  frames.append(dict(source_ns=ns,file=str(path),sha256=m.sha(path),rgb_header=m.plain(rgb_msg.header),depth_header=m.plain(depth_msg.header),depth_encoding=depth_msg.encoding,rgb_received_ns=rns,depth_received_ns=dns,pair_available_ns=max(rns,dns),exact_tf=m.plain(exact)))
 fixture=dict(scope='Complete recorded RGBD pairs strictly after original arrival through capture-deadline+1sec, with original source/receive stamps and exact measured TF/SDK; no frame interpolation or source restamping',arrival_ns=m.ARR,capture_deadline_ns=m.DEAD,frames=frames,sdk=sdk,static_tf=m.plain(static),dynamic_tf=m.plain(dynamic),parent_fixture=str(m.O/'fixture.json'),parent_fixture_sha256=m.sha(m.O/'fixture.json'),sidecar_bag_sha256=closed['sha256'],keyframe_bag_sha256=f['record_bag_sha256'],script_sha256=m.sha(Path(__file__)),selected_interval_complete_rgbd=True,no_ros_master=True,motion_commands_sent=False)
 (O/'fixture.json').write_text(json.dumps(fixture,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(output=str(O/'fixture.json'),count=len(frames),first=frames[0]['source_ns'],last=frames[-1]['source_ns'],sdk=len(sdk),dynamic_tf=len(dynamic)),ensure_ascii=False))
if __name__=='__main__':main()
