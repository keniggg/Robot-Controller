"""One authorized positive-enable handshake; never send joint motion/disable."""
from pathlib import Path
import json
import os
import threading
import time

root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
out = root / 'gen10_handoff_analysis/positive_enable_recovery'
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(out / 'ros')
import rospy
from std_msgs.msg import Bool, String, Header
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('authorized_positive_enable_recovery', anonymous=True, disable_rosout=True)
lock = threading.Lock()
latest = {}
history = []

def receive(key, msg):
    with lock:
        latest[key] = msg
        if key == 'q':
            history.append(msg)

subs = []
for key, topic, kind in (
    ('q', '/alicia_d/accepted_joint_states', JointState),
    ('epoch', '/alicia_d/control_reference_epoch', Header),
    ('status', '/alicia_d/actuation_status', String),
    ('task', '/grasp/state', GraspState),
    ('mode', '/grasp_mode/status', String),
):
    subs.append(rospy.Subscriber(topic, kind, lambda msg, k=key: receive(k, msg), queue_size=10))
deadline = time.monotonic() + 6
while time.monotonic() < deadline:
    with lock:
        ready = len(latest) == 5 and len(history) >= 10
    if ready:
        break
    time.sleep(.03)
with lock:
    before = dict(latest)
assert len(before) == 5 and len(history) >= 10
assert not before['task'].active
assert not any(json.loads(before['mode'].data).get(k) for k in ('active', 'start_reserved', 'pending'))
assert rospy.get_param('/gui/joint_direct_mode') is True
q = before['q']
assert q.header.frame_id == 'sdk_measured'
assert 0 <= (rospy.Time.now() - q.header.stamp).to_sec() < .2
status = before['status'].data
assert status == 'UNCONFIRMED:CONFIRMED_FEEDBACK_STALE', status
epoch = before['epoch'].stamp.to_nsec()
receipt = dict(started_ns=time.time_ns(), reason=status, before_epoch_ns=epoch,
    names=list(q.name), before_positions=list(q.position),
    positive_enable_sent=False, negative_enable_sent=False, joint_command_sent=False,
    temperature_queried=False, physical_success_verified=False,
    authorization='User explicitly requested positive joint enable and continued autonomous grasp; no torque-disable authorization.')

def save():
    (out / 'result.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')

save()
pub = rospy.Publisher('/demonstration', Bool, queue_size=1, latch=False)
deadline = time.monotonic() + 5
while not pub.get_num_connections() and time.monotonic() < deadline:
    time.sleep(.03)
assert pub.get_num_connections()
# Driver demonstration=false calls request_positive_enable; true is forbidden.
receipt.update(topic='/demonstration', bool_value=False, write_intent_ns=time.time_ns())
save()
pub.publish(Bool(data=False))
receipt.update(positive_enable_sent=True, sent_ns=time.time_ns())
save()
deadline = time.monotonic() + 6
while time.monotonic() < deadline:
    with lock:
        after = dict(latest)
    if after['epoch'].stamp.to_nsec() > epoch and after['status'].data.startswith(('PENDING:', 'CONFIRMED:')):
        break
    time.sleep(.03)
time.sleep(.5)
with lock:
    after = dict(latest)
q2 = after['q']
actual = dict(zip(q2.name, q2.position))
maximum = max(abs(actual[n] - v) for n, v in zip(q.name, q.position))
receipt.update(after_epoch_ns=after['epoch'].stamp.to_nsec(),
    after_status=after['status'].data, after_positions=list(q2.position),
    after_feedback_ns=q2.header.stamp.to_nsec(), maximum_position_change=maximum,
    acknowledged=after['epoch'].stamp.to_nsec() > epoch and after['status'].data.startswith(('PENDING:', 'CONFIRMED:')),
    confirmation_pending=not after['status'].data.startswith('CONFIRMED:'), ended_ns=time.time_ns())
save()
assert receipt['acknowledged'], receipt
assert q2.header.frame_id == 'sdk_measured' and 0 <= (rospy.Time.now() - q2.header.stamp).to_sec() < .2
assert maximum <= .004, receipt
print(json.dumps(receipt, ensure_ascii=False), flush=True)
for sub in subs:
    sub.unregister()
