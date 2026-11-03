"""Replay recorded accepted samples through an isolated real RSP; no live ROS I/O."""
import os, socket, subprocess, time, signal, json, hashlib
from pathlib import Path
from datetime import datetime, timezone
mount=Path('/media/zhuyupei/SanStand')
if not mount.is_mount(): raise RuntimeError('SanStand unavailable')
source=mount/'实验记录/Robot-Controller/ros_log/feedforward_speed_grasp_20260929T082708Z'
out=source/('isolated_accepted_tf_'+datetime.now(timezone.utc).strftime('%H%M%S'))
out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
s=socket.socket();s.bind(('127.0.0.1',0));port=s.getsockname()[1];s.close()
assert port!=11311
os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%port
os.environ['ROS_HOSTNAME']='localhost';os.environ.pop('ROS_IP',None)
import rospy,rosbag,numpy as np,tf2_ros
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix
root=Path('.runtime/candidate_models/expanded_metric_20260929')
model=(root/'alicia_expanded_metric_candidate.urdf').read_text()
processes=[];logs=[]
def spawn(cmd,name):
 f=(out/name).open('w');logs.append(f)
 p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy());processes.append(p);return p
def matrix(m):
 q=m.transform.rotation;t=m.transform.translation;F=quaternion_matrix([q.x,q.y,q.z,q.w]);F[:3,3]=[t.x,t.y,t.z];return F
try:
 master=spawn(['/opt/ros/noetic/bin/roscore','-p',str(port)],'master.log')
 import xmlrpc.client
 deadline=time.monotonic()+15
 while time.monotonic()<deadline:
  try:
   if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/replay')[0]==1:break
  except Exception:time.sleep(.05)
 else:raise RuntimeError('isolated master unavailable')
 rospy.init_node('accepted_tf_transport_replay',anonymous=True,disable_signals=True)
 rospy.set_param('/robot_description',model);rospy.set_param('/use_sim_time',True)
 buffer=tf2_ros.Buffer(cache_time=rospy.Duration(180),debug=False);received=[]
 def tf_cb(msg,static=False):
  for tr in msg.transforms:
   (buffer.set_transform_static if static else buffer.set_transform)(tr,'isolated_rsp')
   if not static:received.append(tr.header.stamp.to_nsec())
 rospy.Subscriber('/tf',TFMessage,tf_cb,queue_size=10000)
 rospy.Subscriber('/tf_static',TFMessage,lambda m:tf_cb(m,True),queue_size=100)
 pub=rospy.Publisher('/accepted_replay',JointState,queue_size=10000);clock=rospy.Publisher('/clock',Clock,queue_size=10000)
 rsp=spawn(['/opt/ros/noetic/lib/robot_state_publisher/robot_state_publisher','joint_states:=/accepted_replay','__name:=isolated_rsp'],'rsp.log')
 deadline=time.monotonic()+10
 while time.monotonic()<deadline and pub.get_num_connections()<1:time.sleep(.05)
 assert pub.get_num_connections()==1
 samples=[];statics=[]
 with rosbag.Bag(str(source/'scene_diagnostic.bag')) as b:
  for topic,m,t in b.read_messages(topics=['/alicia_d/accepted_joint_states','/tf_static']):
   if topic.endswith('accepted_joint_states'):samples.append(m)
   else:
    for tr in m.transforms:
     if tr.child_frame_id=='camera_link':statics.append(tr)
 assert statics
 for tr in statics:buffer.set_transform_static(tr,'frozen_recorded_handeye')
 for m in samples:
  clock.publish(Clock(clock=m.header.stamp));pub.publish(m);time.sleep(.004)
 deadline=time.monotonic()+5
 while time.monotonic()<deadline and (not received or max(received)<samples[-1].header.stamp.to_nsec()):time.sleep(.02)
 report=json.loads((source/'offline_support_motion_replay/report.json').read_text())
 reference=np.array(report['reference_plane_base']);foot=np.array(report['reference_footprint_base']);anchor=np.array(report['anchor_position_base']);rows=[]
 for row in report['rows']:
  if not row.get('original_fit_evidence_available'):continue
  ns=row['source_ns'];stamp=rospy.Time(ns//10**9,ns%10**9)
  F=matrix(buffer.lookup_transform_core('base_link','camera_link',stamp))
  P=np.array(row['T_base_camera']).T@np.array(row['current_plane_base']);current=np.linalg.solve(F.T,P);current/=np.linalg.norm(current[:3]);delta=current-reference
  error=max(abs(float(delta[:3]@anchor+delta[3])),float(np.max(abs(foot@delta[:3]+delta[3]))))
  rows.append(dict(source_ns=ns,seconds_after_terminal=row['seconds_after_terminal'],original_error_m=row['reference_max_distance_m'],accepted_rsp_error_m=error,T_base_camera=F.tolist()))
 unique=sorted(set(received));original=[m.header.stamp.to_nsec() for m in samples]
 assert set(unique)<=set(original),'RSP introduced non-sample timestamp'
 result=dict(schema=1,isolated_master=os.environ['ROS_MASTER_URI'],live_master_contacted=False,hardware_commands_sent=0,model_sha256=hashlib.sha256(model.encode()).hexdigest(),accepted_samples=len(samples),distinct_rsp_sample_stamps=len(unique),all_rsp_stamps_are_accepted_stamps=True,rows=rows,maximum_error_m=max(r['accepted_rsp_error_m'] for r in rows),threshold_m=.004,passed=sum(r['accepted_rsp_error_m']<=.004 for r in rows),limitations=['Host-receipt accepted samples are not MCU acquisition timestamps.','SDK camera global_time is not exposure-to-feedback synchronization proof.','Replay changes TF timing only; original frozen footprint and plane-fit values reused. Full segmentation on corrected projection is a separate check.'])
 (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2));print('OUT',out)
finally:
 for p in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for f in logs:f.close()
