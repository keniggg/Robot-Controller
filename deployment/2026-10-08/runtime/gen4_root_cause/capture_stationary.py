import datetime,json,os,threading,time
from pathlib import Path
import rospy,rosbag
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import String,Bool
from tf2_msgs.msg import TFMessage
from alicia_flexible_grasp_supervisor.msg import GraspState
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
ROOT.mkdir(parents=True,exist_ok=True)
OUT=ROOT/('stationary_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
OUT.mkdir()
rospy.init_node('support_root_cause_readonly',anonymous=True,disable_signals=True)
lock=threading.Lock();active=[True];counts={};states=[]
bag=rosbag.Bag(str(OUT/'samples.bag'),'w',compression='lz4')
def capture(topic):
 def cb(msg):
  with lock:
   if not active[0]:return
   bag.write(topic,msg,rospy.Time.now());counts[topic]=counts.get(topic,0)+1
   if topic=='/alicia_d/accepted_joint_states':
    states.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(),name=list(msg.name),position=list(msg.position)))
 return cb
specs=[('/supervisor/camera/color/image_raw',Image),('/supervisor/camera/depth/image_raw',Image),
('/perception/unknown/detector_status',String),('/alicia_d/accepted_joint_states',JointState),
('/tf',TFMessage),('/tf_static',TFMessage),('/alicia_d/motion_enabled',Bool),
('/alicia_d/feedback_ready',Bool),('/grasp/state',GraspState)]
subs=[rospy.Subscriber(topic,kind,capture(topic),queue_size=32) for topic,kind in specs]
time.sleep(6.)
with lock:active[0]=False;bag.close()
for sub in subs:sub.unregister()
(OUT/'summary.json').write_text(json.dumps(dict(readonly=True,counts=counts,
first_feedback=states[0] if states else None,last_feedback=states[-1] if states else None),indent=2)+'\n')
print(json.dumps(dict(directory=str(OUT),counts=counts,first_q=states[0]['position'] if states else None,
last_q=states[-1]['position'] if states else None)))
