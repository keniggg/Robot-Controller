#!/usr/bin/env python3
"""Offline original arrival/ACK/physical-stage evidence; selected raw RGBD + TF."""
import hashlib,json,collections
from pathlib import Path
import numpy as np
import cv2,rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
R=N.parents[1]/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
O=N/'verification/gen14_arrival_independent'
ARR=1791860212162192583;DEAD=1791860215162192583;PRE=1791860176945733070;ANCHOR=1791860167282787322

def plain(x):
 if hasattr(x,'to_nsec'):return x.to_nsec()
 if hasattr(x,'__slots__'):return {k:plain(getattr(x,k))for k in x.__slots__}
 if isinstance(x,(list,tuple)):return [plain(v)for v in x]
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x

def sha(p):
 h=hashlib.sha256()
 with p.open('rb')as f:
  for b in iter(lambda:f.read(8*2**20),b''):h.update(b)
 return h.hexdigest()

def write(name,data):
 p=O/name;p.write_text(json.dumps(plain(data),ensure_ascii=False,indent=2)+'\n');return p

def main():
 O.mkdir(exist_ok=True);assert not any(O.iterdir());manifest=json.loads((R/'manifest.json').read_text());assert manifest['recorder_closed'] and manifest['result']=='failure'
 expected=next(x['sha256']for x in manifest['files']if x['path']=='keyframes.bag');actual=sha(R/'keyframes.bag');assert actual==expected
 topics=['/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack','/perception/unknown/detector_status','/grasp/state','/grasp/near_field_phase','/alicia_controller/follow_joint_trajectory/goal','/alicia_controller/follow_joint_trajectory/result','/hand_controller/follow_joint_trajectory/goal','/hand_controller/follow_joint_trajectory/result','/grasp_mode/initial_preflight_attempt','/grasp_record/camera_runtime_metadata','/alicia_d/accepted_joint_states','/tf','/tf_static','/perception/object_mask']
 wire=[];statuses=[];cameras=[];sdk=[];masks={};dynamic=[];static=[];bridge=CvBridge()
 with rosbag.Bag(str(R/'keyframes.bag'))as bag:
  counts={k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
  for topic,msg,t in bag.read_messages(topics=topics):
   ns=t.to_nsec()
   if topic=='/tf':dynamic.extend(msg.transforms);continue
   if topic=='/tf_static':static.extend(msg.transforms);continue
   if topic.endswith('accepted_joint_states'):sdk.append(dict(received_ns=ns,stamp_ns=msg.header.stamp.to_nsec(),names=msg.name,positions=msg.position));continue
   if topic.endswith('object_mask'):
    if msg.header.stamp.to_nsec()in(PRE,ANCHOR):masks[msg.header.stamp.to_nsec()]=bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough').copy()
    continue
   data=json.loads(msg.data)if hasattr(msg,'data')else plain(msg)
   row=dict(topic=topic,received_ns=ns,message=data)
   if topic.endswith('camera_runtime_metadata'):cameras.append(row)
   elif topic.endswith('detector_status'):statuses.append(row)
   else:wire.append(row)
 requal=[r for r in statuses if (r['message'].get('observation_measurement')or{}).get('state')=='requalifying' and r['message'].get('source_stamp_ns')]
 stamps=sorted({ANCHOR,PRE,*[r['message']['source_stamp_ns']for r in requal]})
 assert len(requal)==10,(len(requal),[r['message'].get('source_stamp_ns')for r in requal])
 camera=cameras[0]['message']['parameters'];config=json.loads((R/'perception_execution_configuration.json').read_text())['parameters']['/unknown_perception']
 sidecar=N/'attempt_2_full_frames';closed=json.loads((sidecar/'result.json').read_text());assert closed['recording_closed'] and not(sidecar/'full_frames.bag.active').exists()
 exact={}
 with rosbag.Bag(str(sidecar/'full_frames.bag'))as bag:
  scounts={k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
  for topic,msg,t in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw'],start_time=rospy.Time(ANCHOR//10**9-1,0),end_time=rospy.Time(DEAD//10**9+1,0)):
   ns=msg.header.stamp.to_nsec()
   if ns in stamps:exact.setdefault(ns,{})['rgb'if'/color/'in topic else'depth']=(msg,t.to_nsec(),topic)
 buffer=tf2_py.BufferCore(rospy.Duration(600))
 for tf in static:buffer.set_transform_static(tf,'sealed_record')
 for tf in sorted(dynamic,key=lambda x:x.header.stamp.to_nsec()):buffer.set_transform(tf,'sealed_record')
 frames=[]
 for ns in stamps:
  pair=exact[ns];assert set(pair)=={'rgb','depth'}
  rgb_msg,rreceived,rtopic=pair['rgb'];depth_msg,dreceived,dtopic=pair['depth'];rgb=bridge.imgmsg_to_cv2(rgb_msg,'bgr8');raw_depth=bridge.imgmsg_to_cv2(depth_msg,'passthrough')
  depth=np.asarray(raw_depth,dtype=np.float32)*camera['depth_scale']if depth_msg.encoding in('16UC1','mono16')else np.asarray(raw_depth,dtype=np.float32)
  t=buffer.lookup_transform_core('base_link','camera_link',rgb_msg.header.stamp).transform;q=t.rotation;T=quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
  data=dict(rgb=rgb,raw_depth=raw_depth,depth=depth,T_base_camera=T)
  if ns in masks:data['recorded_mask']=masks[ns]
  file=O/f'{ns}_source.npz';np.savez_compressed(file,**data);cv2.imwrite(str(O/f'{ns}_rgb.png'),rgb)
  if ns in masks:cv2.imwrite(str(O/f'{ns}_recorded_mask.png'),masks[ns])
  frames.append(dict(source_stamp_ns=ns,file=str(file),sha256=sha(file),rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),raw_depth_sha256=hashlib.sha256(raw_depth.tobytes()).hexdigest(),rgb_header=plain(rgb_msg.header),depth_header=plain(depth_msg.header),rgb_received_ns=rreceived,depth_received_ns=dreceived,raw_topics=[rtopic,dtopic],raw_bag=str(sidecar/'full_frames.bag'),recorded_mask_available=ns in masks,recorded_mask_sha256=None if ns not in masks else hashlib.sha256(masks[ns].tobytes()).hexdigest(),T_base_camera=T.tolist()))
 compact=[]
 for r in requal:
  d=r['message'];o=d['observation_measurement'];a=o.get('appearance_identity',[]);support=o.get('support_audit',{})
  compact.append(dict(source_ns=d['source_stamp_ns'],received_ns=r['received_ns'],publication_age_sec=(r['received_ns']-d['source_stamp_ns'])*1e-9,remaining_capture_at_publication_sec=(DEAD-r['received_ns'])*1e-9,reason=o['reason'],qualified=o['qualified'],accepted_source_stamps=o['source_stamps_ns'],appearance=[{k:e.get(k)for k in('code','appearance_match','colour_distance','contour_distance','reference_source_stamp_ns','current_mask_sha256','reference_mask_sha256','elapsed_sec','dense_metrics','proposal')}for e in a],support={k:v for k,v in support.items()if k in('support_current_audit_available','support_current_audit_quadrants','support_current_audit_inlier_ratio','support_current_audit_residual_m','support_reference_eligible_candidates')},frame_timing_present='frame_timing'in d))
 goals=[r for r in wire if r['topic'].endswith('/goal')];results=[r for r in wire if r['topic'].endswith('/result')];tasks=[r for r in wire if r['topic']=='/grasp/state'];phase=[r for r in wire if r['topic']=='/grasp/near_field_phase'];assert len(goals)==len(results)==1 and goals[0]['topic'].startswith('/alicia_') and results[0]['message']['status']['status']==3 and results[0]['message']['result']['error_code']==0;assert not any(r['message']['active']for r in phase)
 report=dict(passed=True,classification='Closed failed observation acquisition; both requalifying request ACKs timely; zero qualified frames because original premotion appearance rejects all nine evaluated frames, tenth reaches three-second capture deadline',record=str(R),record_bag_sha256=actual,full_sidecar_bag=str(sidecar/'full_frames.bag'),full_sidecar_bag_sha256=closed['sha256'],full_sidecar_hash_scope='Root-owned sealed result; bag content independently indexed and selected original pixels decoded; no duplicate 4.2GB hashing in this pass',arrival_ns=ARR,capture_deadline_ns=DEAD,premotion_reference_ns=PRE,anchor_ns=ANCHOR,rows=compact,reason_counts=dict(collections.Counter(x['reason']for x in compact)),physical_stage=dict(arm_goal_count=len(goals),arm_result=results[0],hand_goal_count=scounts.get('/hand_controller/follow_joint_trajectory/goal',0),hand_result_count=scounts.get('/hand_controller/follow_joint_trajectory/result',0),active_contact_phase_count=sum(x['message']['active']for x in phase),task_states=[r['message']['state']for r in tasks],classification='One successful initial far-field observation arm trajectory; no contact phase, approach or gripper action recorded'),source_config_sha256=sha(R/'perception_execution_configuration.json'),script_sha256=sha(Path(__file__)),no_ros_master=True,motion_commands_sent=False,temperature_queried=False,limits=['Pending observation messages lack full frame_timing. Publication age is computed from source to observer receive and is not claimed as CPU duration.','Exact TF is reconstructed from sealed measured transforms. status exact_tf_consumed=false/measurement_evaluated=false reflects rejected measurement acceptance, not proof TF was unavailable.','Arrival rejected masks are not published and must be reconstructed with original algorithm; only actual recorded premotion and anchor masks are exported.'])
 write('wire.json',wire);write('arrival_statuses.json',requal);write('all_detector_statuses.json',statuses);write('camera_metadata.json',cameras);write('fixture.json',dict(camera=camera,unknown_parameters=config,frames=frames,sdk=sdk,record_bag_sha256=actual,full_sidecar_bag_sha256=closed['sha256'],wire=wire));write('result.json',report);print(json.dumps(dict(output=str(O),reason_counts=report['reason_counts'],frames=stamps,premotion_mask=next(x for x in frames if x['source_stamp_ns']==PRE)['recorded_mask_available']),ensure_ascii=False))
if __name__=='__main__':main()
