"""Fresh target generation for the user's resumed motor grasp; no motion."""
import os,json,time,hashlib
from pathlib import Path
from datetime import datetime,timezone
if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand absent')
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')/('fresh_target_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'));out.mkdir();(out/'ros').mkdir();os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import Header,String
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.srv import SetGraspMode
rospy.init_node('fresh_resumed_motor',anonymous=True)
if rospy.wait_for_message('/grasp/state',GraspState,timeout=4).active or rospy.get_param('/gui/joint_direct_mode',False):raise RuntimeError('task or manual active')
old=rospy.get_param('/grasp_mode/selection')
strategy=os.environ.get('TASK_GRASP_STRATEGY','observe_return')
if strategy not in ('observe_return','two_stage','direct'):raise RuntimeError('invalid requested strategy')
r=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown',strategy)
if not r.success:raise RuntimeError(r.message)
deadline=time.monotonic()+20
while time.monotonic()<deadline:
 new=rospy.get_param('/grasp_mode/selection')
 if new['generation']>old['generation']:break
 time.sleep(.1)
else:raise RuntimeError('no new selection')
current=None
while time.monotonic()<deadline:
 m=json.loads(rospy.wait_for_message('/perception/unknown/detector_status',String,timeout=4).data)
 if m.get('generation')==new['generation'] and m.get('state')=='ready' and m.get('target_locked') and not m.get('target_lost'):current=m;break
if current is None:raise RuntimeError('current target did not acquire')
q=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
e=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4)
record=dict(user_change=os.environ.get('TASK_SCENE_CHANGE','User reposed motor and authorized grasp after timing repair; user resumed after pause. Fresh observed target only.'),old_selection=old,new_selection=new,epoch_ns=e.stamp.to_nsec(),joint_names=q.name,joint_positions=q.position,joint_source_stamp_ns=q.header.stamp.to_nsec(),handeye=rospy.get_param('/handeye'),model_sha256=hashlib.sha256(rospy.get_param('/robot_description').encode()).hexdigest(),current_status=current,hardware_commands_sent=0)
(out/'context.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
Path('.runtime/current_moved_motor.json').write_text(json.dumps({'directory':str(out)},ensure_ascii=False)+'\n')
print(json.dumps({'directory':str(out),'selection':new,'target_position':current.get('position_base'),'frame':current['source_stamp_ns']},ensure_ascii=False))
