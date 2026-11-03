#!/usr/bin/env python3
"""Bounded-memory extraction of real sealed pairs near a 6 Hz source grid."""
from pathlib import Path
import hashlib,json
import numpy as np
import rosbag,rospy,tf2_ros
from tf.transformations import quaternion_matrix
from alicia_grasp_modes.rgb_observation_source import bracket_feedback

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z'
REC=ROOT/'grasp_records/20261010T060144.544180Z_94750fef3cae477792d13cdcb9d4ade0'
OUT=NEW/'gen14_analysis/replay_inputs/continuous'
OUT.mkdir(exist_ok=True)
assert json.loads((REC/'manifest.json').read_text())['recorder_closed']
START,END=1791612194600000000,1791612246400000000
TOPICS=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']
buffer=tf2_ros.Buffer(cache_time=rospy.Duration(600),debug=False)
feedback=[]
camera=json.loads((OUT.parent/'camera_configuration.json').read_text())
request=json.loads((OUT.parent/'qualified_measurement_request.json').read_text())
K=np.asarray([camera[k] for k in ('fx','fy','cx','cy')],dtype=float)
epoch=request['control_epoch_ns']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,_ in bag.read_messages(topics=['/tf','/tf_static','/alicia_d/accepted_joint_states']):
        if topic in ('/tf','/tf_static'):
            for transform in msg.transforms:
                (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(transform,'sealed_gen14')
        elif msg.header.frame_id=='sdk_measured':
            values=dict(zip(msg.name,msg.position))
            feedback.append((msg.header.stamp.to_nsec(),[values['Joint%d'%i] for i in range(1,7)]))

def window(bag):
    return bag.read_messages(topics=TOPICS,
        start_time=rospy.Time((START-10**9)//10**9,(START-10**9)%10**9),
        end_time=rospy.Time((END+10**9)//10**9,(END+10**9)%10**9))

stamps={topic:set() for topic in TOPICS}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,_ in window(bag):
        ns=msg.header.stamp.to_nsec()
        if START<=ns<=END:stamps[topic].add(ns)
available=sorted(stamps[TOPICS[0]]&stamps[TOPICS[1]])
assert available
grid=range(START,END+1,round(1e9/6))
selected=sorted({min(available,key=lambda ns:abs(ns-goal)) for goal in grid})
selected_set=set(selected)
frames=[]; pending={}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic,msg,received in window(bag):
        ns=msg.header.stamp.to_nsec()
        if ns not in selected_set:continue
        pair=pending.setdefault(ns,{})
        pair[topic]=(msg,received.to_nsec())
        if len(pair)<2:continue
        c,cr=pair[TOPICS[0]];d,dr=pair[TOPICS[1]]
        assert c.header.frame_id==d.header.frame_id=='camera_link'
        assert c.encoding in ('rgb8','bgr8') and d.encoding in ('16UC1','mono16','32FC1')
        color=np.ndarray((c.height,c.width,3),dtype=np.uint8,buffer=c.data,strides=(c.step,3,1)).copy()
        if c.encoding=='rgb8':color=color[:,:,::-1].copy()
        dtype=np.dtype(('>' if d.is_bigendian else '<')+('u2' if d.encoding in ('16UC1','mono16') else 'f4'))
        depth=np.ndarray((d.height,d.width),dtype=dtype,buffer=d.data,strides=(d.step,dtype.itemsize)).copy()
        tr=buffer.lookup_transform('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
        T=quaternion_matrix([tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w])
        T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
        q,before,after=bracket_feedback(feedback,ns,epoch)
        path=OUT/('source_%d.npz'%ns)
        np.savez_compressed(path,color=color,depth_raw=depth,
            depth_scale=float(camera['depth_scale']) if d.encoding in ('16UC1','mono16') else 1.,
            K=K,intrinsics=K,T_base_camera=T,q=q,source_stamp_ns=np.int64(ns))
        frames.append(dict(label=path.stem,source_stamp_ns=ns,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            exact_tf_lookup_stamp_ns=ns,feedback_before=before,feedback_after=after,
            color_receipt_ns=cr,depth_receipt_ns=dr,color_encoding='bgr8',depth_encoding=d.encoding,
            color_bytes_sha256=hashlib.sha256(c.data).hexdigest(),depth_bytes_sha256=hashlib.sha256(d.data).hexdigest()))
        del pending[ns]
assert len(frames)==len(selected) and not pending
frames.sort(key=lambda d:d['source_stamp_ns'])
gaps=[b-a for a,b in zip(selected,selected[1:])]
qfile=OUT/'accepted_feedback.json'
qfile.write_text(json.dumps([dict(source_stamp_ns=ns,q=q) for ns,q in feedback if START-300_000_000<=ns<=END+300_000_000],ensure_ascii=False,indent=2)+'\n')
report=dict(record=str(REC),start_ns=START,end_ns=END,requested_rate_hz=6,
    selection_policy='Nearest real exact RGB-D pair to each fixed 6 Hz grid point, deduplicated; unavailable sources are never interpolated or invented.',
    available_exact_pairs=len(available),selected_pair_count=len(selected),first_source_ns=selected[0],last_source_ns=selected[-1],
    maximum_gap_ns=max(gaps),minimum_gap_ns=min(gaps),source_gaps_ns=gaps,
    gaps_over_800ms=[dict(before_ns=a,after_ns=b,gap_ns=b-a) for a,b in zip(selected,selected[1:]) if b-a>800_000_000],
    frames=frames,accepted_feedback=dict(path=str(qfile),sha256=hashlib.sha256(qfile.read_bytes()).hexdigest()),
    restrictions=dict(ros_node_initialized=False,ros_master_access=False,device_access=False,temperature_read=False),
    limitations=['Saved camera samples do not necessarily include every frame processed by the original 6 Hz detector.',
                 'Every saved sample retains its original exact source stamp, recorded TF and measured feedback brackets.',
                 'This extraction asserts no tracking, collision, motion or grasp success.'])
path=OUT/'manifest.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),available_pairs=len(available),selected_pairs=len(selected),max_gap_ns=max(gaps),gap_over_800ms_count=len(report['gaps_over_800ms'])),ensure_ascii=False))
