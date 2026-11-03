"""Align the idle host controller while the driver blocks automatic sources."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--output', type=Path, required=True)
a = ap.parse_args()
mount = Path('/media/zhuyupei/SanStand')
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert (mount/'实验记录/Robot-Controller/ros_log') in a.output.resolve().parents
a.output.mkdir(exist_ok=False)
os.environ['ROS_LOG_DIR'] = str(a.output/'ros')
import rospy
from actionlib_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from alicia_flexible_grasp_supervisor.msg import GraspState

rospy.init_node('gen27_host_controller_alignment_manual', anonymous=True, disable_rosout=True)
lock = threading.Lock()
latest, events, samples, sdk = {}, [], [], []
received, busy_events = {}, []
names = ['Joint%d' % i for i in range(1,7)]

def receive(key, msg):
    with lock:
        latest[key] = msg
        received[key] = time.monotonic()
        busy = ((key=='task' and msg.active)
                or (key=='mode' and any(json.loads(msg.data).get(k) for k in ('active','pending','start_reserved')))
                or (key in ('arm','hand') and any(g.status in (0,1,6,7) for g in msg.status_list)))
        if busy:
            busy_events.append(dict(key=key,receipt_ns=time.time_ns()))
        if key == 'q':
            samples.append(dict(stamp_ns=msg.header.stamp.to_nsec(), names=list(msg.name),
                                positions=list(msg.position)))
        if key in ('manual','epoch','actuation'):
            value = msg.stamp.to_nsec() if key=='epoch' else msg.data
            events.append(dict(key=key,value=value,receipt_ns=time.time_ns()))
        if key == 'sdk':
            sdk.append(dict(stamp_ns=msg.header.stamp.to_nsec(),frame_id=msg.header.frame_id,
                            names=list(msg.name),positions=list(msg.position)))

topics = [('q','/alicia_d/accepted_joint_states',JointState),
          ('sdk','/alicia_d/sdk_command',JointState),
          ('epoch','/alicia_d/control_reference_epoch',Header),
          ('manual','/gui/joint_direct_mode',Bool),
          ('actuation','/alicia_d/actuation_status',String),
          ('task','/grasp/state',GraspState),('mode','/grasp_mode/status',String),
          ('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
          ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),
          ('controller','/alicia_controller/state',JointTrajectoryControllerState)]
subs=[rospy.Subscriber(topic,kind,lambda msg,k=k:receive(k,msg),queue_size=100)
      for k,topic,kind in topics]
required={k for k,_,_ in topics if k!='sdk'}
deadline=time.monotonic()+6
while time.monotonic()<deadline:
    with lock:
        if required<=latest.keys() and len(samples)>=20:break
    time.sleep(.02)

def check():
    with lock:
        state=dict(latest)
        assert required<=state.keys()
        assert not busy_events, 'Observed task/router/action activity'
        assert all(time.monotonic()-received[k]<2.5 for k in ('q','controller','task','arm','hand'))
        assert all(sub.get_num_connections()>0 for sub in subs)
        assert state['manual'].data is True and rospy.get_param('/gui/joint_direct_mode') is True
        assert not state['task'].active
        assert not any(json.loads(state['mode'].data).get(k) for k in ('active','pending','start_reserved'))
        assert all(not any(g.status in (0,1,6,7) for g in state[k].status_list) for k in ('arm','hand'))
        assert state['actuation'].data=='PENDING:POSITIVE_ENABLE_REQUESTED'
        assert state['epoch'].stamp.to_nsec()==1792043953113863243
        assert not sdk, 'Unexpected SDK write during manual-only alignment'
        assert all(e['value'] is True for e in events if e['key']=='manual')
        assert all(e['value']==1792043953113863243 for e in events if e['key']=='epoch')
        assert all(e['value']=='PENDING:POSITIVE_ENABLE_REQUESTED' for e in events if e['key']=='actuation')
    q=state['q']; c=state['controller']; now=rospy.Time.now().to_nsec()
    assert len(q.name)==len(q.position)==7 and set(q.name)==set(names+['right_finger'])
    assert all(math.isfinite(v) for v in q.position)
    assert q.header.frame_id=='sdk_measured' and 0<=now-q.header.stamp.to_nsec()<200_000_000
    assert 0<=now-c.header.stamp.to_nsec()<200_000_000 and list(c.joint_names)==names
    assert len(c.desired.positions)==len(c.actual.positions)==6
    assert all(math.isfinite(v) for v in list(c.desired.positions)+list(c.actual.positions))
    assert len(c.desired.velocities)==6 and max(map(abs,c.desired.velocities))<1e-9
    assert len(c.desired.accelerations)==6 and max(map(abs,c.desired.accelerations))<1e-9
    return state

result=dict(host_controller_hold_published=False, sdk_joint_commands_observed=0,
            temperature_queried=False, disable_sent=False, manual_mode_changed=False,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
def save():
    with lock:
        result.update(joint_samples=list(samples),sdk_commands=list(sdk),events=list(events),
                      sdk_joint_commands_observed=len(sdk),busy_events=list(busy_events))
    (a.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')

try:
    before=check()
    assert len(samples)>=20
    result.update(started_ns=time.time_ns())
    save()
    pub=rospy.Publisher('/alicia_controller/command',JointTrajectory,queue_size=1,latch=False)
    deadline=time.monotonic()+3
    while not pub.get_num_connections() and time.monotonic()<deadline:time.sleep(.02)
    assert pub.get_num_connections()
    before=check()
    q=before['q']; target=dict(zip(q.name,q.position))
    assert all(max(abs(target[n]-v) for n,v in zip(row['names'],row['positions']))<=.004 for row in samples)
    values=[target[n] for n in names]
    result.update(target_positions=values,previous_desired=list(before['controller'].desired.positions),
                  epoch_ns=before['epoch'].stamp.to_nsec())
    hold=JointTrajectory();hold.joint_names=names
    point=JointTrajectoryPoint();point.positions=values;point.velocities=[0.]*6
    point.time_from_start=rospy.Duration.from_sec(.2);hold.points=[point]
    result['publish_intent_ns']=time.time_ns();save()
    checked=check()
    assert max(abs(target[n]-v) for n,v in zip(checked['q'].name,checked['q'].position))<1e-9
    pub.publish(hold)
    result['host_controller_hold_published']=True;save()
    # The host spline can have nonzero derivatives during its .2s transition;
    # require stationary desired only after it has finished. Driver ownership
    # remains manual throughout, so automatic targets must not reach the SDK.
    time.sleep(.4)
    deadline=time.monotonic()+3
    stable_since=None
    while time.monotonic()<deadline:
        state=check();c=state['controller']
        error=max(abs(x-y) for x,y in zip(c.desired.positions,values))
        current=dict(zip(state['q'].name,state['q'].position))
        assert max(abs(current[n]-target[n]) for n in target)<=.004
        if error<1e-8:
            if stable_since is None:stable_since=time.monotonic()
            if time.monotonic()-stable_since>=.5:break
        else:stable_since=None
        time.sleep(.02)
    assert stable_since is not None and time.monotonic()-stable_since>=.5
    result.update(passed=True,ended_ns=time.time_ns(),final_desired=list(c.desired.positions),
                  final_accepted=[current[n] for n in names],manual_mode=True,
                  maximum_accepted_delta_rad=max(abs(target[n]-v) for row in samples
                      for n,v in zip(row['names'],row['positions'])))
except Exception as exc:
    result.update(passed=False,error=repr(exc),ended_ns=time.time_ns())
    raise
finally:
    for sub in subs:sub.unregister()
    # Subscribers are now closed; revoke success if the last callbacks added
    # conflicting evidence while final bookkeeping was in progress.
    with lock:
        valid=(not sdk and not busy_events
               and all(e['value'] is True for e in events if e['key']=='manual')
               and all(e['value']==1792043953113863243 for e in events if e['key']=='epoch')
               and all(e['value']=='PENDING:POSITIVE_ENABLE_REQUESTED' for e in events if e['key']=='actuation'))
        if result.get('passed'):
            final_q=latest['q']; final_c=latest['controller']
            valid=valid and len(final_q.name)==len(final_q.position)==7 and set(final_q.name)==set(target)
            valid=valid and all(math.isfinite(v) and abs(target[n]-v)<=.004 for n,v in zip(final_q.name,final_q.position))
            valid=valid and list(final_c.joint_names)==names and len(final_c.desired.positions)==6
            valid=valid and all(math.isfinite(v) and abs(v-values[i])<1e-8 for i,v in enumerate(final_c.desired.positions))
            valid=valid and len(final_c.desired.velocities)==len(final_c.desired.accelerations)==6
            valid=valid and all(math.isfinite(v) and abs(v)<1e-9 for v in list(final_c.desired.velocities)+list(final_c.desired.accelerations))
            valid=valid and all(max(abs(target[n]-v) for n,v in zip(row['names'],row['positions']))<=.004 for row in samples)
        if not valid:
            result.update(passed=False,error='Final evidence changed during subscriber shutdown')
    save()
    print(json.dumps({k:v for k,v in result.items() if k not in ('joint_samples','sdk_commands','events')},ensure_ascii=False))
    if not valid:
        raise RuntimeError(result['error'])
