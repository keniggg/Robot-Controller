import os,json,time,subprocess,threading,hashlib
from pathlib import Path
from datetime import datetime,timezone
import rospy,cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Header,String
from alicia_flexible_grasp_supervisor.msg import GraspState
root=Path.cwd();base=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
assert os.path.ismount('/media/zhuyupei/SanStand')
rospy.init_node('prepare_two_stage_current_diagnostic',anonymous=True)
state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
if state.active:raise RuntimeError('Task is already active')
previous=json.loads((root/'.runtime/current_grasp_diagnostic.json').read_text())['directory']
out=base/'ros_log'/('grasp_two_stage_diagnostic_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir()
(root/'.runtime/current_grasp_diagnostic.json').write_text(json.dumps({'directory':str(out)}))
cache={}; lock=threading.Lock()
def image(topic,msg):
 with lock:cache.setdefault(msg.header.stamp.to_nsec(),{})[topic]=msg
subs=[rospy.Subscriber('/supervisor/camera/'+topic+'/image_raw',Image,lambda m,t=topic:image(t,m),queue_size=2) for topic in ('color','depth')]
end=time.monotonic()+6;pair=None
while time.monotonic()<end:
 with lock:
  matches=[k for k,v in cache.items() if len(v)==2]
  if matches: ns=max(matches);pair=cache[ns];break
 time.sleep(.02)
for s in subs:s.unregister()
if pair is None:raise RuntimeError('No exact current RGB-D pair')
bridge=CvBridge();cv2.imwrite(str(out/'initial_rgb.png'),bridge.imgmsg_to_cv2(pair['color'],'bgr8'));cv2.imwrite(str(out/'initial_depth.png'),bridge.imgmsg_to_cv2(pair['depth'],'passthrough'))
j=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
camera=rospy.get_param('/camera');handeye=rospy.get_param('/handeye');urdf=rospy.get_param('/robot_description')
context=dict(camera=camera,handeye=handeye,rgb_source_stamp_ns=ns,depth_source_stamp_ns=ns,joint_source_stamp_ns=j.header.stamp.to_nsec(),joint_names=list(j.name),joint_positions=list(j.position),selection_before=rospy.get_param('/grasp_mode/selection'),previous_failed_experiment=previous,authorization='用户要求重启节点并解决问题，已确认重新对准目标；延续自主抓取诊断实验授权',strategy='two_stage')
(out/'initial_context.json').write_text(json.dumps(context,ensure_ascii=False,indent=2));(out/'robot_description.urdf').write_text(urdf)
params={k:rospy.get_param(k) for k in ('/robot','/grasp','/grasp_6d','/gripper','/mujoco_digital_twin')};params['robot_description_sha256']=hashlib.sha256(urdf.encode()).hexdigest()
(out/'execution_parameters.json').write_text(json.dumps(params,ensure_ascii=False,indent=2))
(out/'experiment_protocol.json').write_text(json.dumps(dict(purpose='Compare reached-view two-stage reobservation while retaining one generation and original support contradiction contract',previous_failed_experiment=previous,changes=['support current-ring independent audit loaded','failed lift cannot request new contact precision lease','existing two_stage strategy used for this diagnostic'],unchanged=['validated handeye','current SDK camera model','20g operator upper bound, not measured mass','lift simulation diagnostic only','physical lift enabled','no target relocation','no torque-disable commands'],failure_policy='Do not clear a support contradiction by selecting again at the reached pose',limits=['Physical height correction not proven','Lift command-reference handoff remains under separate investigation']),ensure_ascii=False,indent=2))
env=os.environ.copy();env['ROS_LOG_DIR']=str(out/'ros');env['GRASP_STRATEGY']='two_stage';env['PYTHONUNBUFFERED']='1';env['OPENBLAS_NUM_THREADS']='1'
(out/'ros').mkdir(exist_ok=True)
for key in ('GRASP_OPERATOR_MASS_KG','GRASP_OPERATOR_MASS_STATEMENT'):env.pop(key,None)
log=(out/'scene_recorder_stdout.log').open('ab',buffering=0)
rec=subprocess.Popen(['/usr/bin/python3','-u',str(root/'.runtime/record_grasp_scene_diagnostic.py')],cwd=str(root),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'scene_recorder_process.json').write_text(json.dumps({'pid':rec.pid}))
for _ in range(80):
 if (out/'scene_recorder_ready.json').exists():break
 if rec.poll() is not None:raise RuntimeError('Scene recorder exited')
 time.sleep(.1)
else:raise RuntimeError('Scene recorder did not become ready')
log=(out/'recorded_attempt_stdout.log').open('ab',buffering=0)
run=subprocess.Popen(['/usr/bin/python3','-u','tools/run_recorded_grasp.py','--duration','900','--','/usr/bin/python3','-u','.runtime/start_current_grasp_experiment.py'],cwd=str(root),env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'attempt_process.json').write_text(json.dumps({'pid':run.pid,'started':time.time(),'strategy':'two_stage'}))
print(json.dumps({'directory':str(out),'attempt_pid':run.pid,'scene_recorder_pid':rec.pid},ensure_ascii=False),flush=True)
