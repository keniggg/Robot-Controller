"""Read current pose for continuation of an existing real user confirmation."""
import argparse
import json
import os
from pathlib import Path
import threading
import time

root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--session-dir',type=Path,required=True)
args=parser.parse_args()
out=args.output.resolve()
session=args.session_dir.resolve()
assert root in session.parents
assert Path('/media/zhuyupei/SanStand').is_mount()
assert root in out.parents
out.mkdir(exist_ok=True)
os.environ['ROS_LOG_DIR']=str(out/'continuation_ros')
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header, String
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('same_pose_continuation_readonly',anonymous=True,disable_rosout=True)
receipt=json.loads((session/'alignment_receipt.json').read_text())
lock=threading.Lock(); latest={}; history=[]
def callback(key,msg):
    with lock:
        latest[key]=msg
        if key=='q': history.append(msg)
subs=[]
for key,topic,kind in [('q','/alicia_d/accepted_joint_states',JointState),
    ('epoch','/alicia_d/control_reference_epoch',Header),
    ('image','/supervisor/camera/color/image_raw',Image),
    ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String)]:
    subs.append(rospy.Subscriber(topic,kind,lambda msg,k=key: callback(k,msg),queue_size=2,buff_size=2**24))
deadline=time.monotonic()+8
while time.monotonic()<deadline:
    with lock:
        ready=len(latest)==5 and len(history)>=20
    if ready: break
    time.sleep(.05)
with lock: state=dict(latest); samples=list(history)
assert len(state)==5 and len(samples)>=20, 'fresh read incomplete'
old=receipt['topics']['/alicia_d/accepted_joint_states']
expected=dict(zip(old['names'],old['positions']))
q=state['q']; now=rospy.Time.now().to_nsec()
def delta(msg): return max(abs(expected[n]-v) for n,v in zip(msg.name,msg.position) if n in expected)
assert q.header.frame_id=='sdk_measured' and 0<=now-q.header.stamp.to_nsec()<200000000
assert all(set(expected)<=set(msg.name) and delta(msg)<=.004 for msg in samples)
assert state['epoch'].stamp.to_nsec()==receipt['epoch_ns']
assert 0<=now-state['image'].header.stamp.to_nsec()<800000000
assert not state['task'].active
mode=json.loads(state['mode'].data)
assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
result=dict(same_pose_continuation=True,original_confirmation=str(session/'alignment_receipt.json'),
    original_confirmation_message_id=receipt['message_id'],original_confirmation_received_ns=receipt['receipt_ns'],
    new_user_confirmation_claimed=False,measured_ns=now,epoch_ns=state['epoch'].stamp.to_nsec(),
    measured_joint_stamp_ns=q.header.stamp.to_nsec(),names=list(q.name),positions=list(q.position),
    maximum_delta_from_confirmed_pose_rad=max(delta(msg) for msg in samples),samples=len(samples),
    current_image_stamp_ns=state['image'].header.stamp.to_nsec(),
    previous_selection=rospy.get_param('/grasp_mode/selection'),task_state=str(state['task'].state),
    manual_mode=rospy.get_param('/gui/joint_direct_mode'),motion_sent=False,temperature_queried=False)
assert cv2.imwrite(str(out/'same_pose_scene.png'),CvBridge().imgmsg_to_cv2(state['image'],'bgr8'))
(out/'same_pose_retry_context.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
for sub in subs: sub.unregister()
