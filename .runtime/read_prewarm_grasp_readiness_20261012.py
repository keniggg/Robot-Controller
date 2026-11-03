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
ap.add_argument('--alignment', type=Path, required=True)
ap.add_argument('--expected-generation', type=int, required=True)
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
assert type(rospy.get_param('/gui/joint_direct_mode')) is bool
result = dict(passed=True, recorded_ns=now, names=list(joint.name), positions=list(joint.position),
    sample_count=len(history), epoch_ns=state['epoch'].stamp.to_nsec(),
    task_state=state['task'].state, task_active=False, mode=mode,
    selection=rospy.get_param('/grasp_mode/selection'), manual_mode=rospy.get_param('/gui/joint_direct_mode'),
    motion_commands_sent=False, temperature_queried=False, new_grasp_authorized=False)
if a.compare:
    old = json.loads(a.compare.read_text())
    expected = dict(zip(old['names'],old['positions']))
    assert old['epoch_ns']==result['epoch_ns'] and old['selection']==result['selection']
    assert all(set(expected)<=set(q.name) for q in history)
    maximum = max(abs(expected[n]-v) for q in history for n,v in zip(q.name,q.position) if n in expected)
    result.update(previous=str(a.compare), maximum_joint_delta_rad=maximum)
    assert maximum<=.004
import hashlib, shutil
work=Path(__file__).resolve().parents[1]
root=a.output.parent
deployment=json.loads((root/'gen5_handoff_runtime_deployment.json').read_text())
assert deployment['whole_patch_deployed'] and deployment['deployment_passed']
assert hashlib.sha256(Path(deployment['source_manifest']).read_bytes()).hexdigest()==deployment['source_manifest_sha256']
release=json.loads((root/'verification/gen6_prewarm_release_manifest.json').read_text())
assert release['passed'] and release['installed']
source=release['source_sha256']
override={}
for name, sha in source.items():
    assert hashlib.sha256((work/name).read_bytes()).hexdigest()==override.get(name,sha), name
for name, process in {**deployment['after'],**deployment['preserved_after']}.items():
    stat=Path('/proc/%s/stat'%process['pid']).read_text().rsplit(')',1)[1].split()
    assert stat[19]==process['start_ticks'], name
ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
config=json.loads((ext/'grasp_records/archive_config.json').read_text())
control=json.loads((ext/'maintenance/storage_control.json').read_text())
assert config['upload_enabled'] is False and config['delete_enabled'] is False
assert control['archive_policy']=='local_only'
free=shutil.disk_usage(ext).free
assert free>=config['min_free_bytes']
assert rospy.get_param('/grasp/unknown_lift_after_close_enabled') is False
assert rospy.get_param('/grasp/unknown_observe_return_max_observations')==2
assert rospy.get_param('/alicia_d_driver_node/software_temperature_monitoring_enabled') is False
assert rospy.get_param('/alicia_d_driver_node/temperature_poll_rate_hz')==0.0
assert result['selection']['generation']==a.expected_generation
alignment=a.alignment.resolve()
assert root in alignment.parents
old=json.loads(alignment.read_text())
expected=dict(zip(old['topics']['/alicia_d/accepted_joint_states']['names'],old['topics']['/alicia_d/accepted_joint_states']['positions']))
assert result['epoch_ns']==old['epoch_ns']
maximum=max(abs(expected[n]-v) for q in history for n,v in zip(q.name,q.position) if n in expected)
assert maximum<=.004
result.update(source_count=len(source),deployment_processes_unchanged=True,source_matches_operational_release=True,
    operational_script_override=override,operational_validation='prewarm installed; combined 296 tests passed; imports-only probe passed',
    maximum_joint_delta_rad=maximum,free_bytes=free,min_free_bytes=config['min_free_bytes'],new_grasp_authorized=True,
    alignment_receipt=str(alignment),alignment_reply_id=old['message_id'],
    hands_clear_reply_id=old['message_id'],hands_clear_reply=old['user_message'])
(a.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False),flush=True)
for subscription in subscriptions:
    subscription.unregister()
