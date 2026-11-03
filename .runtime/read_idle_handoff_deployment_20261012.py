"""Read-only idle and measured-feedback evidence around a compute deployment."""
import argparse
import json
import os
from pathlib import Path
import threading
import time

ap = argparse.ArgumentParser()
ap.add_argument('--output', type=Path, required=True)
ap.add_argument('--compare', type=Path)
a = ap.parse_args()
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and mount in a.output.resolve().parents
a.output.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(a.output/'ros')
import rospy
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('handoff_deployment_readonly', anonymous=True, disable_rosout=True)
lock = threading.Lock()
values, samples = {}, []
def receive(key, msg):
    with lock:
        values[key] = msg
        if key == 'joint':
            samples.append(msg)
topics = [('joint','/alicia_d/accepted_joint_states',JointState),
          ('epoch','/alicia_d/control_reference_epoch',Header),
          ('task','/grasp/state',GraspState), ('mode','/grasp_mode/status',String),
          ('arm_action','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
          ('hand_action','/hand_controller/follow_joint_trajectory/status',GoalStatusArray)]
subscriptions = [rospy.Subscriber(topic, kind, lambda msg,k=key: receive(k,msg),queue_size=20)
                 for key, topic, kind in topics]
end = time.monotonic()+8
while time.monotonic()<end:
    with lock:
        ready = len(values)==len(topics) and len(samples)>=20
    if ready:
        break
    time.sleep(.05)
with lock:
    state, history = dict(values), list(samples)
assert len(state)==len(topics) and len(history)>=20
joint = state['joint']
now = rospy.Time.now().to_nsec()
assert joint.header.frame_id=='sdk_measured' and 0<=now-joint.header.stamp.to_nsec()<200_000_000
assert not state['task'].active
mode = json.loads(state['mode'].data)
assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
assert all(not any(g.status in (0,1,6,7) for g in state[k].status_list)
           for k in ('arm_action','hand_action'))
assert rospy.get_param('/gui/joint_direct_mode') is True
result = dict(passed=True, recorded_ns=now, names=list(joint.name), positions=list(joint.position),
    sample_count=len(history), epoch_ns=state['epoch'].stamp.to_nsec(),
    task_state=state['task'].state, task_active=False, mode=mode,
    selection=rospy.get_param('/grasp_mode/selection'), manual_mode=True,
    motion_commands_sent=False, temperature_queried=False, new_grasp_authorized=False)
if a.compare:
    old = json.loads(a.compare.read_text())
    expected = dict(zip(old['names'],old['positions']))
    assert old['epoch_ns']==result['epoch_ns'] and old['selection']==result['selection']
    assert all(set(expected)<=set(q.name) for q in history)
    maximum = max(abs(expected[n]-v) for q in history for n,v in zip(q.name,q.position) if n in expected)
    result.update(previous=str(a.compare), maximum_joint_delta_rad=maximum)
    assert maximum<=.004
(a.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
for subscription in subscriptions:
    subscription.unregister()
