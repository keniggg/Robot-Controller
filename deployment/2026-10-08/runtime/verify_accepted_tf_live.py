"""Read-only TF transport and frame-time verification after the scoped handoff."""
import os,json,time,xmlrpc.client,socket,hashlib
from pathlib import Path
out=Path(json.loads(Path('.runtime/current_accepted_tf_load.json').read_text())['directory']);assert Path('/media/zhuyupei/SanStand').is_mount();os.environ['ROS_LOG_DIR']=str(out/'ros');socket.setdefaulttimeout(3)
import rospy,rosgraph,tf2_ros
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
rospy.init_node('accepted_tf_live_verify',anonymous=True,disable_signals=True)
master=rosgraph.Master('/accepted_tf_live_verify');start=rospy.Time.now().to_nsec();samples=set();transforms=[];images=[]
def accept(m):samples.add(m.header.stamp.to_nsec())
def tf(m):
 for tr in m.transforms:
  if tr.header.frame_id=='base_link':transforms.append((tr.header.stamp.to_nsec(),m._connection_header.get('callerid'),tr.child_frame_id))
rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,accept,queue_size=100)
rospy.Subscriber('/tf',TFMessage,tf,queue_size=100)
rospy.Subscriber('/supervisor/camera/color/image_raw',Image,lambda m:images.append(m.header.stamp.to_nsec()),queue_size=2)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf)
time.sleep(5)
rows=[r for r in transforms if start+800000000<r[0]<rospy.Time.now().to_nsec()-200000000]
assert len(samples)>=20 and len(rows)>=20,(len(samples),len(rows))
assert all(r[0] in samples for r in rows),'non-sample TF timestamp'
assert set(r[1] for r in rows)=={'/accepted_robot_state_publisher'},set(r[1] for r in rows)
lookups=[]
for ns in images:
 if start+1000000000<ns<rospy.Time.now().to_nsec()-300000000:
  tr=buf.lookup_transform('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9),rospy.Duration(.15));lookups.append(tr.header.stamp.to_nsec())
assert len(lookups)>=10,len(lookups)
before=json.loads((out/'before.json').read_text());pids={}
for n,d in before['protected'].items():
 if n=='/robot_state_publisher':continue
 pids[n]=xmlrpc.client.ServerProxy(master.lookupNode(n)).getPid('/accepted_tf_live_verify')[2];assert pids[n]==d['pid']
e=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=3);assert e.stamp.to_nsec()==before['epoch_ns']
sub=xmlrpc.client.ServerProxy(master.lookupNode('/accepted_robot_state_publisher')).getSubscriptions('/accepted_tf_live_verify')[2]
assert ['/alicia_d/accepted_joint_states','sensor_msgs/JointState'] in sub
result=dict(status='live_tf_input_verified',sample_count=len(samples),tf_rows=len(rows),all_tf_stamps_are_real_samples=True,tf_authorities=sorted(set(r[1] for r in rows)),exact_camera_stamp_lookups=len(lookups),protected_pids=pids,epoch_ns=e.stamp.to_nsec(),model_sha256=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest(),robot_commands_sent=0)
(out/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
p=Path('.runtime/latest_work_state.json');d=json.loads(p.read_text());d.update(status='active',stage='accepted_tf_loaded_fresh_target_pending',last_user_instruction='已调好并对准，可以继续',updated_ns=time.time_ns(),latest_tf_load_directory=str(out),pending='TF sample-time remap loaded and read-only verified; fresh target selection and recorded grasp next; old generation 12 plan retired.');p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n');(out.parents[1]/'文档/2026-09-29-grasp-current-state.json').write_text(p.read_text())
