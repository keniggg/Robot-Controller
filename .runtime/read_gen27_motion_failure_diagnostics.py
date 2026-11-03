"""One idle, read-only velocity/self-check batch; never send a motion command."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import threading
import time

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--output', type=Path, required=True)
a = ap.parse_args()
mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log'
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert root in a.output.resolve().parents
a.output.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(a.output / 'ros')
import rospy
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Trigger
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('readonly_gen27_motion_failure_diagnostics', anonymous=True, disable_rosout=True)
lock = threading.Lock()
latest, samples, raw = {}, [], []

def receive(key, msg):
    with lock:
        latest[key] = msg
        if key == 'q':
            samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
                                names=list(msg.name), positions=list(msg.position)))
        if key == 'raw':
            for status in msg.status:
                values = {v.key: v.value for v in status.values}
                # Exclude unsolicited temperature packets without decoding them.
                if values.get('command_function_hex') not in ('06 00', '06 02', 'FE 00'):
                    continue
                raw.append(dict(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
                                name=status.name, values=values))

spec = [('q','/alicia_d/accepted_joint_states',JointState),
        ('epoch','/alicia_d/control_reference_epoch',Header),
        ('status','/alicia_d/actuation_status',String),
        ('enabled','/alicia_d/motion_enabled',Bool),
        ('ready','/alicia_d/feedback_ready',Bool),
        ('task','/grasp/state',GraspState), ('mode','/grasp_mode/status',String),
        ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('raw','/alicia_d/sdk_diagnostics',DiagnosticArray)]
subs = [rospy.Subscriber(topic, kind, lambda msg,k=key:receive(k,msg), queue_size=100)
        for key,topic,kind in spec]
end = time.monotonic()+5
required = {'q','epoch','status','enabled','ready','task','mode','arm','hand'}
while time.monotonic()<end:
    with lock:
        if required <= latest.keys(): break
    time.sleep(.02)
with lock: before = dict(latest)
assert required <= before.keys()
assert not before['task'].active
assert not any(json.loads(before['mode'].data).get(k) for k in ('active','pending','start_reserved'))
assert all(not any(g.status in (0,1,6,7) for g in before[k].status_list) for k in ('arm','hand'))
assert rospy.get_param('/gui/joint_direct_mode') is True
assert rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
assert rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz') == 0.0
start = rospy.Time.now().to_nsec()
service = '/alicia_d_driver_node/query_motion_diagnostics'
rospy.wait_for_service(service, timeout=3)
response = rospy.ServiceProxy(service, Trigger)()
end = time.monotonic()+6
while time.monotonic()<end: time.sleep(.05)
with lock:
    after = dict(latest)
    result = dict(start_ns=start, end_ns=rospy.Time.now().to_nsec(),
                  query=dict(service=service, success=response.success, message=response.message),
                  before_status=before['status'].data, after_status=after['status'].data,
                  motion_enabled=after['enabled'].data, feedback_ready=after['ready'].data,
                  epoch_before=before['epoch'].stamp.to_nsec(), epoch_after=after['epoch'].stamp.to_nsec(),
                  task_active=after['task'].active, manual_mode=rospy.get_param('/gui/joint_direct_mode'),
                  joint_samples=list(samples), diagnostics=list(raw),
                  motion_commands_sent=False, temperature_queried=False, disable_sent=False,
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
for sub in subs: sub.unregister()
(a.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
summary={k:v for k,v in result.items() if k not in ('joint_samples','diagnostics')}
summary.update(joint_sample_count=len(samples), diagnostic_count=len(raw),
               non_joint_diagnostics=[r for r in raw if r['values'].get('command_function_hex')!='06 00'])
print(json.dumps(summary,ensure_ascii=False))
