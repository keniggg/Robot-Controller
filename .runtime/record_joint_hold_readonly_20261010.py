"""Passive current holding-state capture. No publishers, services or queries to device."""
from pathlib import Path
import json,os,threading,time
r=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=r/'gen13_handoff_analysis/current_hold_readonly';out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Header,Bool
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('joint_hold_readonly',anonymous=True,disable_rosout=True)
lock=threading.Lock();rows=[]
def cb(kind,m):
 d=dict(topic=kind,receipt_ns=rospy.Time.now().to_nsec())
 if isinstance(m,JointState):d.update(stamp_ns=m.header.stamp.to_nsec(),frame=m.header.frame_id,names=list(m.name),positions=list(m.position))
 elif isinstance(m,Header):d.update(stamp_ns=m.stamp.to_nsec(),frame=m.frame_id)
 elif isinstance(m,GraspState):d.update(active=bool(m.active),state=str(m.state),detail=str(getattr(m,'detail','')))
 else:d['value']=m.data
 with lock:rows.append(d)
subs=[]
for topic,cls in [('/alicia_d/accepted_joint_states',JointState),('/alicia_d/sdk_command_feedback',JointState),('/alicia_d/control_reference',JointState),('/alicia_d/control_reference_epoch',Header),('/alicia_d/actuation_status',String),('/gui/joint_direct_mode',Bool),('/grasp/state',GraspState)]:
 subs.append(rospy.Subscriber(topic,cls,lambda msg,k=topic:cb(k,msg),queue_size=20))
started=time.time_ns();deadline=time.monotonic()+10
while time.monotonic()<deadline:time.sleep(.05)
for sub in subs:sub.unregister()
with lock: captured=list(rows)
summary={}
for topic in sorted(set(d['topic'] for d in captured)):
 selected=[d for d in captured if d['topic']==topic];s=dict(count=len(selected),first=selected[0],last=selected[-1])
 if 'positions' in selected[0]:
  names=selected[0]['names'];selected=[d for d in selected if d['names']==names]
  s['peak_to_peak_rad']=[max(d['positions'][i] for d in selected)-min(d['positions'][i] for d in selected) for i in range(len(names))]
 summary[topic]=s
(out/'samples.jsonl').write_text(''.join(json.dumps(d,ensure_ascii=False)+'\n' for d in captured))
result=dict(started_ns=started,ended_ns=time.time_ns(),summary=summary,gui_manual_param=rospy.get_param('/gui/joint_direct_mode'),motion_sent=False,device_query_sent=False,temperature_accessed=False,limitations=['Passive ten-second interval after user reported prior hand displacement; does not capture the force event and cannot diagnose stiffness/backlash alone.','Host actuation status is not a direct motor torque measurement.'])
(out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(out/'result.json'),summary=summary),ensure_ascii=False))
