"""Export three exact sealed generation 33 frames; no ROS calls/control."""
from pathlib import Path
import json,hashlib
import numpy as np
import rosbag,rospy,tf2_py
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen33_handoff_analysis')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T144532.807783Z_f42fde5ce926439082043f73f1a4715e')
m=json.loads((R/'manifest.json').read_text());assert m['state']=='complete' and m['recorder_closed']
stamps={1791557134886816740:'original_anchor',1791557138989239692:'last_good',1791557139297346353:'first_loss'}
T=lambda ns:rospy.Time(ns//10**9,ns%10**9)
rgbtopic='/supervisor/camera/color/image_raw';depthtopic='/supervisor/camera/depth/image_raw';masktopic='/perception/object_mask';objecttopic='/perception/object'
special={'/experiment/observation_rgbd/color':rgbtopic,'/experiment/observation_rgbd/depth':depthtopic}
frames={ns:{} for ns in stamps};statuses={ns:[] for ns in stamps};camera=[];joints=[];buf=tf2_py.BufferCore(rospy.Duration(100.))
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for _,msg,_ in bag.read_messages(topics=['/tf_static']):
  for tf in msg.transforms:buf.set_transform_static(tf,'sealed_gen33')
 topics=['/tf','/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status',rgbtopic,depthtopic,masktopic,objecttopic]+list(special)
 for topic,msg,receipt in bag.read_messages(topics=topics,start_time=T(min(stamps)-1000000000),end_time=T(max(stamps)+2000000000)):
  if topic=='/tf':
   for tf in msg.transforms:buf.set_transform(tf,'sealed_gen33')
  elif topic.endswith('accepted_joint_states'):
   if msg.header.frame_id=='sdk_measured':joints.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(),positions=list(msg.position),names=list(msg.name),bag_received_ns=receipt.to_nsec()))
  elif topic.endswith('camera_runtime_metadata'):camera.append(dict(received_ns=receipt.to_nsec(),message=json.loads(msg.data)))
  elif topic.endswith('detector_status'):
   x=json.loads(msg.data);ns=x.get('source_stamp_ns')
   if ns in stamps:statuses[ns].append(dict(received_ns=receipt.to_nsec(),message=x))
  elif msg.header.stamp.to_nsec() in stamps:frames[msg.header.stamp.to_nsec()][special.get(topic,topic)]=(msg,receipt.to_nsec(),topic)
def matrix(parent,child,ns):
 tf=buf.lookup_transform_core(parent,child,T(ns)).transform;q=tf.rotation
 out=quaternion_matrix([q.x,q.y,q.z,q.w]);out[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z];return out
out=D/'exact_source_images';out.mkdir(exist_ok=False);bridge=CvBridge();rows=[]
for ns,f in frames.items():
 row=dict(source_stamp_ns=ns,role=stamps[ns]);row['detector_status']=next((s for s in statuses[ns] if s['message'].get('state')=='ready'),statuses[ns][0] if statuses[ns] else None)
 assert row['detector_status'] is not None,(ns,'missing source status')
 row['camera_metadata']=min(camera,key=lambda c:abs(c['received_ns']-ns))
 arrays=dict(T_base_camera=matrix('base_link','camera_link',ns),tool_from_camera=matrix('tool0','camera_link',ns))
 for topic,key,encoding in [(rgbtopic,'rgb','bgr8'),(depthtopic,'depth_raw','passthrough'),(masktopic,'refined_mask','mono8')]:
  if topic in f:arrays[key]=bridge.imgmsg_to_cv2(f[topic][0],encoding)
 assert 'rgb' in arrays and 'depth_raw' in arrays,(ns,'missing exact RGBD')
 path=out/(str(ns)+'.npz');np.savez_compressed(str(path),**arrays)
 row.update(npz=str(path),npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),array_sha256={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()},available_topics={k:v[1] for k,v in f.items()},original_topics={k:v[2] for k,v in f.items()})
 rows.append(row)
report=dict(record=str(R),record_manifest_sha256=hashlib.sha256((R/'manifest.json').read_bytes()).hexdigest(),recorder_closed=True,no_ros_calls=True,sources=rows,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),limitations=['Exact saved input sources only. TF reconstructed at original timestamps; in-memory tracker not serialized.'])
p=out/'manifest.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(dict(manifest=str(p),sources=[(r['source_stamp_ns'],r['role']) for r in rows])))
