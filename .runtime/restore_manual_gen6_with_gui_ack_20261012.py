"""Restore idle manual control and wait for the GUI's own persistent echo."""
import json,os,threading,time
from pathlib import Path
out=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/gen6_finished_gui_ack')
assert Path('/media/zhuyupei/SanStand').is_mount()
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import Bool,String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('restore_manual_wait_gui_ack',anonymous=True,disable_rosout=True)
assert rospy.get_param('/grasp_mode/selection')['generation']==6
assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
    assert not any(g.status in (0,1,6,7) for g in rospy.wait_for_message(topic,GoalStatusArray,timeout=5).status_list)
received=[]
lock=threading.Lock()
def receive(message):
    with lock:received.append(dict(value=message.data,caller=message._connection_header.get('callerid'),at_ns=rospy.Time.now().to_nsec()))
subscriber=rospy.Subscriber('/gui/joint_direct_mode',Bool,receive,queue_size=30)
publisher=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
end=time.monotonic()+8
while publisher.get_num_connections()<5 and time.monotonic()<end:time.sleep(.05)
assert publisher.get_num_connections()>=5
stamp=rospy.Time.now().to_nsec()
rospy.set_param('/gui/joint_direct_mode',True)
publisher.publish(Bool(data=True))
end=time.monotonic()+8
while time.monotonic()<end:
    with lock:ack=any(r['caller']=='/alicia_supervisor_gui' and r['value'] is True and r['at_ns']>=stamp for r in received)
    if ack and rospy.get_param('/gui/joint_direct_mode') is True:break
    time.sleep(.05)
else:raise RuntimeError('GUI did not acknowledge persistent manual mode')
time.sleep(2)
assert rospy.get_param('/gui/joint_direct_mode') is True
result=dict(manual_restored=True,gui_own_publisher_acknowledged=True,verified_inactive=True,
    generation=6,messages=received,motion_commands_sent=False,temperature_queried=False)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
subscriber.unregister()
