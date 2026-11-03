#!/usr/bin/env python3
"""Extract sealed camera/TF/joint observations; no runtime ROS connection."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
import rosbag
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.rgb_observation_source import bracket_feedback

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z'
RECORD=ROOT/'grasp_records/20261010T060144.544180Z_94750fef3cae477792d13cdcb9d4ade0'
OUT=NEW/'gen14_analysis/replay_inputs'
assert json.loads((RECORD/'manifest.json').read_text())['recorder_closed']
OUT.mkdir(exist_ok=True)
targets={'initial_anchor':1791612106454440355, 'last_good':1791612195990879774,
         'failed_frame':1791612196247022628, 'after_200ms':1791612196447022628,
         'after_400ms':1791612196647022628, 'after_800ms':1791612197047022628}
additional = '--additional' in sys.argv
if additional:
    motion_start = 1791612194667747020
    approach_transition = 1791612246030165672
    targets = {'motion_plus_%ds' % seconds: motion_start + seconds*10**9 for seconds in (2,10,30,50)}
    targets.update(pregrasp_before_transition=approach_transition-200_000_000,
                   pregrasp_after_transition=approach_transition+200_000_000)
color_topic='/supervisor/camera/color/image_raw'
depth_topic='/supervisor/camera/depth/image_raw'
images={color_topic:{}, depth_topic:{}}
buffer=tf2_ros.Buffer(cache_time=rospy.Duration(600), debug=False)
statuses=[]; feedback=[]; requests=[]; sessions=[]; phases=[]; camera_meta=[]

def serialize(value):
    if hasattr(value,'__slots__'):
        return {key:serialize(getattr(value,key)) for key in value.__slots__}
    if isinstance(value,(list,tuple)):return [serialize(x) for x in value]
    return value

with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    topics=[color_topic,depth_topic,'/tf','/tf_static','/perception/unknown/detector_status',
            '/alicia_d/accepted_joint_states','/grasp_mode/observe_return_measurement_request',
            '/grasp_mode/observe_return_session','/grasp/near_field_phase','/grasp_record/camera_runtime_metadata']
    for topic,msg,received in bag.read_messages(topics=topics):
        if topic in images:
            ns=msg.header.stamp.to_nsec()
            for label,target in targets.items():
                old=images[topic].get(label)
                if old is None or abs(ns-target)<abs(old.header.stamp.to_nsec()-target):
                    images[topic][label]=msg
        elif topic in ('/tf','/tf_static'):
            for transform in msg.transforms:
                (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'sealed_gen14')
        elif topic=='/alicia_d/accepted_joint_states':
            if msg.header.frame_id=='sdk_measured':
                q=dict(zip(msg.name,msg.position))
                feedback.append((msg.header.stamp.to_nsec(),[q['Joint%d'%i] for i in range(1,7)]))
        elif topic=='/perception/unknown/detector_status':
            d=json.loads(msg.data)
            if d.get('generation')==14:statuses.append(d)
        elif topic=='/grasp_mode/observe_return_measurement_request':requests.append(json.loads(msg.data))
        elif topic=='/grasp_mode/observe_return_session':sessions.append(json.loads(msg.data))
        elif topic=='/grasp/near_field_phase':phases.append(serialize(msg))
        elif topic=='/grasp_record/camera_runtime_metadata':camera_meta.append(json.loads(msg.data))
assert camera_meta
camera=camera_meta[-1]['parameters']
K=np.asarray([camera[k] for k in ('fx','fy','cx','cy')],dtype=float)
qualified=next(d for d in requests if d['state']=='qualified')
epoch=qualified['control_epoch_ns']
frames=[]
for label,target in targets.items():
    c,d=images[color_topic][label],images[depth_topic][label]
    ns=c.header.stamp.to_nsec()
    assert ns==d.header.stamp.to_nsec()
    assert c.header.frame_id==d.header.frame_id=='camera_link'
    assert c.encoding in ('rgb8','bgr8')
    color=np.ndarray((c.height,c.width,3),dtype=np.uint8,buffer=c.data,strides=(c.step,3,1)).copy()
    if c.encoding=='rgb8':color=color[:,:,::-1].copy()
    assert d.encoding in ('16UC1','mono16','32FC1')
    dtype=np.dtype(('>' if d.is_bigendian else '<')+('u2' if d.encoding in ('16UC1','mono16') else 'f4'))
    depth=np.ndarray((d.height,d.width),dtype=dtype,buffer=d.data,strides=(d.step,dtype.itemsize)).copy()
    stamp=rospy.Time(ns//10**9,ns%10**9)
    tr=buffer.lookup_transform('base_link','camera_link',stamp).transform
    T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
    T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
    q,before,after=bracket_feedback(feedback,ns,epoch)
    path=OUT/(label+'.npz')
    np.savez_compressed(path,color=color,depth_raw=depth,
        depth_scale=float(camera['depth_scale']) if d.encoding in ('16UC1','mono16') else 1.,
        K=K,intrinsics=K,T_base_camera=T,q=q,source_stamp_ns=np.int64(ns))
    frames.append(dict(label=label,requested_source_stamp_ns=target,source_stamp_ns=ns,
        exact_requested_source_available=ns==target,nearest_source_delta_ns=ns-target,
        color_encoding='bgr8',original_color_encoding=c.encoding,depth_encoding=d.encoding,
        exact_tf_lookup_stamp_ns=ns,feedback_before=before,feedback_after=after,
        color_bytes_sha256=hashlib.sha256(c.data).hexdigest(),depth_bytes_sha256=hashlib.sha256(d.data).hexdigest(),
        path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))

def save(name,data):
    p=OUT/name;p.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())

if additional:
    rows = [dict(source_stamp_ns=ns,q=q) for ns,q in feedback
            if any(abs(ns-f['source_stamp_ns']) <= 200_000_000 for f in frames)]
    evidence = save('accepted_feedback_additional_windows.json', rows)
    report = dict(record=str(RECORD), sealed_record=True, frames=frames,
        accepted_feedback_windows=evidence,
        time_basis=dict(contact_motion_start_ns=motion_start, pregrasp_to_approach_task_transition_ns=approach_transition),
        original_manifest_preserved=True,
        restrictions=dict(ros_node_initialized=False,ros_master_access=False,device_access=False,temperature_read=False),
        limitations=['The before/after targets bracket the task transition to APPROACH_TARGET. This timestamp is a phase boundary, not an independent physical-settle certificate.',
                     'Nearest saved source times and exact source TF lookup are reported; no synthetic READY is created.',
                     'Sparse frames with long gaps do not represent uninterrupted real-time tracking and cannot waive ordinary source/reference freshness.'])
    final = save('manifest_additional.json', report)
    print(json.dumps(dict(manifest=final, frames=[{k:v for k,v in f.items() if k in ('label','source_stamp_ns','nearest_source_delta_ns')} for f in frames]),ensure_ascii=False))
    raise SystemExit(0)

last_ready=max((d for d in statuses if d.get('state')=='ready' and d.get('source_stamp_ns',0)<=targets['last_good']),key=lambda d:d['source_stamp_ns'])
initial=next(d for d in statuses if d.get('detail')=='acquired')
failure=next(d for d in statuses if d.get('target_lost'))
metadata={
 'camera_runtime_metadata':save('camera_runtime_metadata.json',camera_meta),
 'camera_configuration':save('camera_configuration.json',camera),
 'initial_acquisition_status':save('initial_acquisition_status.json',initial),
 'last_ready_status':save('last_ready_status.json',last_ready),
 'first_failure_status':save('first_failure_status.json',failure),
 'qualified_measurement_request':save('qualified_measurement_request.json',qualified),
 'sessions':save('sessions.json',sessions),
 'near_field_phases':save('near_field_phases.json',phases),
 'accepted_feedback_window':save('accepted_feedback_window.json',[
      dict(source_stamp_ns=ns,q=q) for ns,q in feedback if targets['last_good']-800_000_000<=ns<=targets['after_800ms']+800_000_000]),
}
report=dict(record=str(RECORD),sealed_record=True,frames=frames,metadata=metadata,
    restrictions=dict(ros_node_initialized=False,ros_master_access=False,device_access=False,temperature_read=False),
    extraction_audit_note='Final extraction explicitly disables tf2 Buffer debug services. The first extraction used the library default debug=True, which performs a master service lookup; that metadata-only lookup was not a hardware action. This final report and inputs were regenerated without it.',
    limitations=['Nearest saved source is explicitly identified where the requested timestamp was not recorded.',
                 'TF is looked up at each saved source timestamp; it establishes the recorded coordinate chain, not physical calibration accuracy.',
                 'These are replay inputs only. No synthetic READY or claim of successful tracking is made.'])
manifest=save('manifest.json',report)
print(json.dumps(dict(manifest=manifest,frames=[{k:v for k,v in f.items() if k in ('label','source_stamp_ns','exact_requested_source_available','nearest_source_delta_ns')} for f in frames]),ensure_ascii=False))
