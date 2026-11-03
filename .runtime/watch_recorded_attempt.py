"""Passive state/goal observer, independent of the task and its authority."""
from pathlib import Path
import argparse, json, os, threading, time
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=a.output.resolve();assert Path('/media/zhuyupei/SanStand').is_mount()
assert Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log') in r.parents
os.environ['ROS_LOG_DIR']=str(r/'observer_ros')
import rospy
from alicia_flexible_grasp_supervisor.msg import GraspState
from control_msgs.msg import FollowJointTrajectoryActionGoal,FollowJointTrajectoryActionResult
from rosgraph_msgs.msg import Log
rospy.init_node('recorded_attempt_readonly_observer',anonymous=True,disable_rosout=True)
lock=threading.Lock();f=(r/'live_events.jsonl').open('x')
def receive(kind,m):
    if kind=='margin' and not m.msg.startswith('Contact pregrasp arrival margin:'):return
    d=dict(kind=kind,received_ns=time.time_ns(),source_ns=m.header.stamp.to_nsec(),caller=m._connection_header.get('callerid'))
    if kind=='task':d.update(active=m.active,state=m.state,message=m.message)
    if kind=='margin':d.update(message=m.msg)
    if kind.endswith('goal'):d.update(goal_id=m.goal_id.id,points=len(m.goal.trajectory.points),duration=m.goal.trajectory.points[-1].time_from_start.to_sec() if m.goal.trajectory.points else None)
    if kind.endswith('result'):d.update(goal_id=m.status.goal_id.id,status=m.status.status,error_code=m.result.error_code,error_string=m.result.error_string)
    with lock:f.write(json.dumps(d,ensure_ascii=False)+'\n');f.flush()
    print(json.dumps(d,ensure_ascii=False)[:2400],flush=True)
subs=[rospy.Subscriber('/grasp/state',GraspState,lambda m:receive('task',m),queue_size=30),
      rospy.Subscriber('/rosout_agg',Log,lambda m:receive('margin',m),queue_size=100)]
for prefix,topic in [('arm','/alicia_controller/follow_joint_trajectory'),('hand','/hand_controller/follow_joint_trajectory')]:
    subs.extend([rospy.Subscriber(topic+'/goal',FollowJointTrajectoryActionGoal,lambda m,p=prefix:receive(p+'_goal',m),queue_size=10),
                 rospy.Subscriber(topic+'/result',FollowJointTrajectoryActionResult,lambda m,p=prefix:receive(p+'_result',m),queue_size=10)])
end=time.monotonic()+900
while not (r/'exit.json').exists() and time.monotonic()<end:time.sleep(.2)
for s in subs:s.unregister()
f.close()
