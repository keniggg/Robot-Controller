#!/usr/bin/env python3
"""Read-only initial preflight / SDK motion boundary observer; never commands."""
import argparse,json,math,threading,time
from pathlib import Path
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import ObjectGeometry

p=argparse.ArgumentParser();p.add_argument('--generation',type=int,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--duration',type=float,default=900.)
a=p.parse_args();mount=Path('/media/zhuyupei/SanStand');storage=mount/'实验记录/Robot-Controller'
assert mount.is_mount() and storage.resolve() in a.output_dir.resolve().parents and 0<a.duration<=1200
a.output_dir.mkdir(parents=True,exist_ok=True)
path=a.output_dir/('motion_boundary_%d_%d.jsonl'%(a.generation,time.time_ns()));handle=path.open('x',buffering=1)
lock=threading.RLock();selection=[None];q0=[None];maximum=[0.];started=time.monotonic();counts={}
def write(kind,**data):
    with lock:
        handle.write(json.dumps(dict(kind=kind,received_ros_ns=rospy.Time.now().to_nsec(),received_monotonic=time.monotonic(),**data),sort_keys=True,allow_nan=False)+'\n');counts[kind]=counts.get(kind,0)+1

def selected(msg):
    value=json.loads(msg.data)
    if value.get('generation')==a.generation:
        selection[0]=value;write('selection',value=value)
    else:selection[0]=None

def structured(topic,msg):
    try:
        value=json.loads(msg.data)
        if topic=='detector':
            if value.get('generation')!=a.generation:return
            value={k:value.get(k) for k in ('state','detail','source_stamp_ns','source_qualification','target_lost','first_loss','frame_timing','selection_stamp_ns')}
        elif selection[0] is None:return
        write(topic,payload=value)
    except (ValueError,TypeError,AttributeError):write('invalid_json',topic=topic)

def joints(msg):
    if selection[0] is None or msg.header.frame_id!='sdk_measured':return
    by=dict(zip(msg.name,msg.position))
    try:q=[float(by['Joint%d'%i]) for i in range(1,7)]
    except (KeyError,ValueError,TypeError):return
    if not all(math.isfinite(v) for v in q):return
    if q0[0] is None:q0[0]=q
    delta=max(abs(x-y) for x,y in zip(q,q0[0]));maximum[0]=max(maximum[0],delta)
    write('accepted_q',source_ns=msg.header.stamp.to_nsec(),q=q,baseline_q=q0[0],max_delta_rad=delta,maximum_delta_rad=maximum[0])

def geometry(msg):
    if selection[0] is not None:write('geometry',source_ns=msg.header.stamp.to_nsec(),valid=msg.valid,target_track_id=msg.target_track_id,reason=msg.failure_reason)

rospy.init_node('readonly_preflight_motion_boundary',anonymous=True,disable_rosout=True)
subs=[rospy.Subscriber('/grasp_mode/selection',String,selected,queue_size=4),rospy.Subscriber('/alicia_d/accepted_joint_states',JointState,joints,queue_size=128),rospy.Subscriber('/grasp_6d/object_geometry',ObjectGeometry,geometry,queue_size=32)]
for topic,kind in [('/perception/unknown/detector_status','detector'),('/grasp_mode/observe_return_measurement_request','measurement_request'),('/grasp_mode/observe_return_measurement_ack','measurement_ack'),('/grasp_mode/observe_return_session','session')]:
    subs.append(rospy.Subscriber(topic,String,lambda msg,k=kind:structured(k,msg),queue_size=64))
print(json.dumps(dict(observer_ready=True,output=str(path))),flush=True)
try:
    while not rospy.is_shutdown() and time.monotonic()-started<a.duration:time.sleep(.2)
finally:
    for sub in subs:sub.unregister()
    write('end',counts=counts,maximum_q_delta_rad=maximum[0]);handle.close()
    print(json.dumps(dict(observer_finished=True,output=str(path),counts=counts,maximum_q_delta_rad=maximum[0])),flush=True)
