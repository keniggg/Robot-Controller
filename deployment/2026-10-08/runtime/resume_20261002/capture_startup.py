import concurrent.futures,datetime,hashlib,json,pathlib,subprocess,threading,time,xmlrpc.client
import rospy,rosnode,rosgraph
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Bool,String
from alicia_flexible_grasp_supervisor.msg import GraspState
ROOT=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
if not pathlib.Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand not mounted')
OUT=ROOT/'ros_log'/'full_ros_latest_start_20261002'
OUT.mkdir(exist_ok=True)
rospy.init_node('startup_evidence_readonly',anonymous=True,disable_signals=True)
lock=threading.Lock(); rows={}; subscribers=[]
def frame(topic,m):
 now=rospy.Time.now().to_nsec()
 with lock:rows.setdefault(topic,[]).append((now,m.header.stamp.to_nsec()))
for t,typ in [('/supervisor/camera/color/image_raw',Image),('/supervisor/camera/depth/image_raw',Image),('/alicia_d/accepted_joint_states',JointState)]:
 subscribers.append(rospy.Subscriber(t,typ,lambda m,t=t:frame(t,m),queue_size=4))
def read_one(t):
 name,typ=t
 try:
  m=rospy.wait_for_message(name,typ,timeout=4)
  return name,dict(state=m.state,active=m.active,message=m.message) if typ==GraspState else m.data
 except Exception as e:return name,{'error':str(e)}
with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
 messages=dict(pool.map(read_one,[('/alicia_d/motion_enabled',Bool),('/alicia_d/feedback_ready',Bool),('/alicia_d/actuation_status',String),('/gui/joint_direct_mode',Bool),('/grasp/state',GraspState)]))
rospy.sleep(12.)
for sub in subscribers:sub.unregister()
stats={}
for t,r in rows.items():
 unique=sorted(set(s for _,s in r));gaps=[(b-a)/1e6 for a,b in zip(unique,unique[1:])]
 stats[t]={'received':len(r),'unique_stamps':len(unique),'max_source_gap_ms':max(gaps,default=None),'max_arrival_age_ms':max((a-b)/1e6 for a,b in r)}
nodes={}
for node in rosnode.get_node_names():
 if 'readonly' in node:continue
 try:
  uri=rosgraph.Master("/startup_evidence_readonly").lookupNode(node);pid=xmlrpc.client.ServerProxy(uri).getPid('/startup_evidence_readonly')[2]
  cmd=(pathlib.Path('/proc')/str(pid)/'cmdline').read_bytes().decode().replace('\0',' ')
  nodes[node]={'pid':pid,'cmd':cmd}
 except Exception as e:nodes[node]={'error':str(e)}
files=['src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py','src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py','devel/lib/alicia_d_driver/alicia_d_driver_node']
evidence={'recorded_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'messages':messages,'sample_window_sec':12,'telemetry':stats,'nodes':nodes,'files':{p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() for p in files},'model_sha256':hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest(),'selection':rospy.get_param('/grasp_mode/selection'),'handeye':rospy.get_param('/handeye'),'remote_url':rospy.get_param('/grasp_6d/remote/server_url'),'simulation_url':rospy.get_param('/mujoco_digital_twin/server_url'),'robot_control_commands_sent_by_this_capture':False,'automatic_grasp_started_this_turn':False,'torque_off_command_sent':False}
(OUT/'startup_evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'path':str(OUT/'startup_evidence.json'),'messages':messages,'telemetry':stats,'nodes':{n:x.get('pid') for n,x in nodes.items()}},ensure_ascii=False,indent=2))
