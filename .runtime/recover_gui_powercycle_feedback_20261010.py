"""One positive-enable recovery after the user's reported reboot and GUI failure."""
from pathlib import Path
import json
import os
import threading
import time

assert os.path.ismount('/media/zhuyupei/SanStand')
out = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gui_powercycle_recovery')
out.mkdir(exist_ok=True)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
import rospy
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('authorized_gui_powercycle_feedback_recovery', anonymous=True, disable_rosout=True)
latest = {}
q_rows = []
lock = threading.Lock()
def receive(key, msg):
    with lock:
        latest[key] = msg
        if key == 'q':
            q_rows.append(msg)
subs = [rospy.Subscriber(topic, cls, lambda msg,key=key:receive(key,msg), queue_size=30)
        for key,topic,cls in [('q','/alicia_d/accepted_joint_states',JointState),
        ('status','/alicia_d/actuation_status',String), ('epoch','/alicia_d/control_reference_epoch',Header),
        ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String)]]
deadline = time.monotonic()+4
while time.monotonic()<deadline and not {'status','epoch','task','mode'}.issubset(latest):
    time.sleep(.03)
with lock: before = dict(latest)
assert {'status','epoch','task','mode'}.issubset(before)
assert not before['task'].active
assert not any(json.loads(before['mode'].data).get(k) for k in ('active','pending','start_reserved'))
assert rospy.get_param('/gui/joint_direct_mode') is True
assert before['status'].data == 'UNCONFIRMED:ENCODER_FEEDBACK_NOT_READY', before['status'].data
epoch = before['epoch'].stamp.to_nsec()
receipt = dict(user_messages=['我刚刚给机械臂上了电，但是上位机中的滑条无法控制机械臂移动',
                              '电机同步当前关节后，滑条也没有反应'],
               prior_authorization='User requested positive joint enable and forbids disable; latest request authorizes restoring manual operation after power-on.',
               started_ns=time.time_ns(), before_status=before['status'].data,before_epoch_ns=epoch,
               positive_enable_sent=False, joint_targets_sent=False, disable_sent=False, temperature_accessed=False)
dest = out / 'positive_enable_result.json'
assert not dest.exists(), 'single recovery only'
def save():dest.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
save()
pub = rospy.Publisher('/demonstration', Bool, queue_size=1, latch=False)
deadline = time.monotonic()+3
while time.monotonic()<deadline and not pub.get_num_connections():time.sleep(.03)
assert pub.get_num_connections()
receipt.update(write_intent_ns=time.time_ns(), topic='/demonstration', bool_value=False)
save()
pub.publish(Bool(data=False))
receipt.update(positive_enable_sent=True,sent_ns=time.time_ns())
save()
deadline=time.monotonic()+5
while time.monotonic()<deadline:
    with lock:
        after=dict(latest)
        fresh=[q for q in q_rows if q.header.stamp.to_nsec()>receipt['sent_ns']]
    if len(fresh)>=10 and after['epoch'].stamp.to_nsec()>epoch:break
    time.sleep(.03)
receipt.update(ended_ns=time.time_ns(),after_epoch_ns=after['epoch'].stamp.to_nsec(),
               after_status=after['status'].data, accepted_samples=len(fresh),
               feedback_restored=len(fresh)>=10,
               gui_manual=rospy.get_param('/gui/joint_direct_mode'),physical_motion_verified=False)
if fresh:
    q=fresh[-1]
    receipt.update(names=list(q.name),positions_rad=list(q.position),feedback_stamp_ns=q.header.stamp.to_nsec(),
                   sample_span_rad=[max(m.position[i] for m in fresh)-min(m.position[i] for m in fresh) for i in range(6)])
save()
for sub in subs:sub.unregister()
print(json.dumps(receipt,ensure_ascii=False),flush=True)
assert receipt['feedback_restored'] and receipt['after_epoch_ns']>epoch
