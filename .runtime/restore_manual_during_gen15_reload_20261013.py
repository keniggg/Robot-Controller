"""Idle manual restoration for the explicitly expected generation; verify GUI own current latched true delivery."""
import json,os,threading,time
from pathlib import Path
import argparse
ap=argparse.ArgumentParser()
ap.add_argument('--output',type=Path,required=True)
ap.add_argument('--expected-generation',type=int,required=True)
a=ap.parse_args()
out=a.output
assert Path('/media/zhuyupei/SanStand') in out.resolve().parents
assert Path('/media/zhuyupei/SanStand').is_mount()
out.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros')
import rospy
from std_msgs.msg import Bool,String
from actionlib_msgs.msg import GoalStatusArray
from alicia_flexible_grasp_supervisor.msg import GraspState
rospy.init_node('restore_manual_wait_gui_ack',anonymous=True,disable_rosout=True)
assert rospy.get_param('/grasp_mode/selection')['generation']==a.expected_generation
# The task is already absent in this recorded partial compute reload.
# Verify its exact prior identity exited and all hardware identities persist.
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
partial=json.loads((root/'gen15_reload_r2/result.json').read_text())
initial=json.loads((root/'gen15_reload_initial_state/result.json').read_text())
assert partial['deployment_passed'] is False and 'after' not in partial
assert initial['task']['state']=='FAILED' and not initial['task']['active'] and not initial['hand']
assert initial['selection']['generation']==a.expected_generation
old_task=partial['before']['/grasp_task_node']
assert not (Path('/proc')/str(old_task['pid'])).exists(), 'Original task must already be absent'
for name, expected in partial['preserved_before'].items():
    stat=(Path('/proc')/str(expected['pid'])/'stat').read_text().rsplit(')',1)[1].split()
    assert stat[0]!='Z' and stat[19]==expected['start_ticks'] and int(stat[1])==expected['parent_pid'], name

mode=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
assert not any(mode.get(k) for k in ('active','start_reserved','pending'))
for topic in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
    assert not any(g.status in (0,1,6,7) for g in rospy.wait_for_message(topic,GoalStatusArray,timeout=5).status_list)
received=[]
lock=threading.Lock()
def receive(message):
    with lock:received.append(dict(value=message.data,caller=message._connection_header.get('callerid'),latched=message._connection_header.get('latching'),at_ns=rospy.Time.now().to_nsec()))
subscriber=rospy.Subscriber('/gui/joint_direct_mode',Bool,receive,queue_size=30)
publisher=rospy.Publisher('/gui/joint_direct_mode',Bool,queue_size=1,latch=True)
import rosgraph
required_consumers={'/alicia_d_driver_node','/motion_gateway','/alicia_supervisor_gui'}
_, mode_subscriptions, _=rosgraph.Master(rospy.get_name()).getSystemState()
mode_consumers=set(dict(mode_subscriptions).get('/gui/joint_direct_mode', []))
assert required_consumers | {rospy.get_name()} <= mode_consumers
required_connections=len(required_consumers)+1  # includes this receipt subscriber
end=time.monotonic()+8
while publisher.get_num_connections()<required_connections and time.monotonic()<end:time.sleep(.05)
assert publisher.get_num_connections()>=required_connections
stamp=rospy.Time.now().to_nsec()
rospy.set_param('/gui/joint_direct_mode',True)
publisher.publish(Bool(data=True))
end=time.monotonic()+8
while time.monotonic()<end:
    with lock:
        gui=[r for r in received if r['caller']=='/alicia_supervisor_gui']
        ack=bool(gui) and gui[-1]['value'] is True and gui[-1]['latched']=='1'
    if ack and rospy.get_param('/gui/joint_direct_mode') is True:break
    time.sleep(.05)
else:raise RuntimeError('GUI did not acknowledge persistent manual mode')
time.sleep(2)
assert rospy.get_param('/gui/joint_direct_mode') is True
with lock:
    gui=[r for r in received if r['caller']=='/alicia_supervisor_gui']
    assert gui and gui[-1]['value'] is True and gui[-1]['latched']=='1'
    assert not any(r['value'] is False and r['at_ns']>=stamp for r in received)
result=dict(manual_restored=True,gui_own_publisher_acknowledged=True,verified_inactive=True,
    generation=a.expected_generation,messages=received,
    required_mode_consumers=sorted(required_consumers),mode_consumers=sorted(mode_consumers),
    publisher_connections=publisher.get_num_connections(),task_previously_exited=True,
    gui_ack_kind='gui_current_latched_true_delivery',
    motion_commands_sent=False,temperature_queried=False)
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result),flush=True)
subscriber.unregister()
