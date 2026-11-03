"""Extract one indexed, finite gen13 window; all native evidence stays external."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag
import rospy
import tf2_ros
from cv_bridge import CvBridge

from alicia_grasp_modes.rgb_observation_source import bracket_feedback

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
BAG = ROOT/'grasp_records/20261004T114247.984592Z_7c99754a8ef14082bf57c5417afb9499/keyframes.bag'
RUN = ROOT/'ros_log/rgb_acquisition_run_20261004T114249Z/result.json'
REF = ROOT/'ros_log/rgb_acquisition_gateway_20261004T114259.948391Z_1541ec9e/result.json'
OUT = ROOT/'ros_log/gen13_rgb_support_exact_20261004T122355Z'
STATUS = '/perception/unknown/detector_status'
RGB = '/supervisor/camera/color/image_raw'
DEPTH = '/supervisor/camera/depth/image_raw'
ACCEPTED = '/alicia_d/accepted_joint_states'
NAMES = tuple('Joint%d' % i for i in range(1, 7))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ns(iso):
    return int(datetime.fromisoformat(iso).timestamp()*1e9)


def stamp(value):
    return rospy.Time(value//10**9, value%10**9)


def matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = float(quaternion@quaternion)
    if not np.isfinite(norm) or abs(norm-1.) > .01:
        raise ValueError('invalid_exact_tf_quaternion')
    x, y, z, w = quaternion/np.sqrt(norm)
    result = np.eye(4)
    result[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    result[:3,3] = [t.x,t.y,t.z]
    return result


def reason(row):
    return row['message'].get('rgb_acquisition_handoff',{}).get('reason',row['message'].get('detail',''))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    run, reference = json.loads(RUN.read_text()), json.loads(REF.read_text())
    run_start, run_finish = ns(run['started_at_utc']), ns(run['finished_at_utc'])
    context = reference['frames']['initial']['source_context']
    camera = context['camera']; K = [camera[k] for k in ('fx','fy','cx','cy')]
    reference_id = reference['outcome']['episode']['reference_id']
    start, finish = stamp(1791114168*10**9), stamp(1791114217*10**9)
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(120.), debug=False)
    statuses, accepted, controller, event_rows = [], [], [], []
    topics = [STATUS,ACCEPTED,'/tf','/tf_static','/alicia_controller/state',
        '/alicia_d/actuation_status','/alicia_d/control_reference_epoch',
        '/alicia_controller/follow_joint_trajectory/goal',
        '/alicia_controller/follow_joint_trajectory/result',
        '/grasp_record/camera_runtime_metadata','/gui/joint_direct_mode']
    counts = Counter()
    with rosbag.Bag(str(BAG)) as bag, rosbag.Bag(str(OUT/'context.bag'),'w') as context_bag:
        for topic,msg,t in bag.read_messages(topics=topics,start_time=start,end_time=finish):
            counts[topic] += 1
            context_bag.write(topic,msg,t)
            receipt = t.to_nsec()
            if topic in ('/tf','/tf_static'):
                for transform in msg.transforms:
                    if topic == '/tf_static':buffer.set_transform_static(transform,'archived_exact')
                    else:buffer.set_transform(transform,'archived_exact')
            elif topic == STATUS:
                value=json.loads(msg.data)
                statuses.append(dict(bag_arrival_ns=receipt,message=value))
            elif topic == ACCEPTED:
                values = dict(zip(msg.name,msg.position))
                accepted.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(),bag_arrival_ns=receipt,
                    frame_id=msg.header.frame_id,names=list(msg.name),position=list(msg.position),
                    arm_position=[values[n] for n in NAMES] if all(n in values for n in NAMES) else None))
            elif topic == '/alicia_controller/state':
                controller.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(),bag_arrival_ns=receipt,
                    names=list(msg.joint_names),desired_positions=list(msg.desired.positions),
                    desired_velocities=list(msg.desired.velocities),actual_positions=list(msg.actual.positions),
                    actual_velocities=list(msg.actual.velocities)))
            elif topic in ('/alicia_d/actuation_status','/gui/joint_direct_mode'):
                event_rows.append(dict(topic=topic,bag_arrival_ns=receipt,value=msg.data))
    in_run = [r for r in statuses if run_start <= r['bag_arrival_ns'] <= run_finish
              and r['message'].get('rgb_acquisition_handoff',{}).get('reference_id') == reference_id]
    failures = [r for r in in_run if reason(r)=='no_cross_view_rgb_match' or reason(r).startswith('support_inconsistent:')]
    controls = [r for r in in_run if reason(r)=='associated_region_lacks_measured_3d']
    chosen = {r['message']['source_stamp_ns']:dict(tags=[reason(r)],status=r) for r in failures}
    for failure in failures:
        obs = failure['message']['rgb_acquisition_handoff']['observation_index']
        same = [r for r in controls if r['message']['rgb_acquisition_handoff'].get('observation_index')==obs]
        before = [r for r in same if r['bag_arrival_ns'] < failure['bag_arrival_ns']]
        after = [r for r in same if r['bag_arrival_ns'] > failure['bag_arrival_ns']]
        for row in ([before[-1]] if before else [])+([after[0]] if after else []):
            source=row['message']['source_stamp_ns']
            chosen.setdefault(source,dict(tags=['adjacent_associated_rgb_control'],status=row))
    # The recorder stores source-stamped native pairs, not the callback's wall time.
    pending, raw_rows = {}, []
    bridge=CvBridge()
    with rosbag.Bag(str(BAG)) as bag:
        for topic,msg,t in bag.read_messages(topics=[RGB,DEPTH],start_time=start,end_time=finish):
            source=msg.header.stamp.to_nsec()
            if source not in chosen:continue
            item=pending.setdefault(source,{})
            item['rgb' if topic==RGB else 'depth']=(msg,t.to_nsec())
            if set(item)!={'rgb','depth'}:continue
            rgb=bridge.imgmsg_to_cv2(item['rgb'][0],desired_encoding='bgr8').copy()
            depth=bridge.imgmsg_to_cv2(item['depth'][0],desired_encoding='passthrough').copy()
            T=matrix(buffer.lookup_transform('base_link','camera_link',stamp(source)))
            tool=matrix(buffer.lookup_transform('tool0','camera_link',stamp(source)))
            record=dict(source_stamp_ns=source,tags=chosen[source]['tags'],status=chosen[source]['status'],
                images={key:dict(encoding=m.encoding,frame_id=m.header.frame_id,seq=m.header.seq,
                    source_stamp_ns=m.header.stamp.to_nsec(),bag_arrival_ns=receipt,
                    array_sha256=hashlib.sha256(array.tobytes()).hexdigest())
                    for key,(m,receipt),array in [('rgb',item['rgb'],rgb),('depth',item['depth'],depth)]})
            completion=record['status']['message'].get('frame_timing',{}).get('status_build_ros_ns',record['status']['bag_arrival_ns'])
            records=[(a['source_stamp_ns'],a['arm_position']) for a in accepted
                if a['frame_id']=='sdk_measured' and a['arm_position'] is not None and a['bag_arrival_ns'] <= completion]
            try:
                q,lo,hi=bracket_feedback(records,source,max(context['epoch_ns'],context['selection']['stamp_ns']))
                record.update(q_source=q.tolist(),accepted_before=[lo[0],list(lo[1])],accepted_after=[hi[0],list(hi[1])])
            except ValueError as exc:
                record['feedback_error']=str(exc)
                q=np.full(6,np.nan)
            path=OUT/('%d_source.npz'%source)
            np.savez_compressed(path,color_bgr=rgb,depth_raw=depth,depth_scale=float(camera['depth_scale']),
                K=np.array(K),T_base_camera=T,T_tool_camera=tool,q_source=q,source_stamp_ns=np.int64(source))
            record.update(file=str(path),sha256=sha(path))
            raw_rows.append(record)
            del pending[source]
    found={r['source_stamp_ns'] for r in raw_rows}
    manifest=json.loads((BAG.parent/'manifest.json').read_text())
    declared=next(item for item in manifest['files'] if item['path']=='keyframes.bag')
    report=dict(schema=1,scope=__doc__,ros_io=False,robot_commands_sent=False,
        bag=str(BAG),bag_manifest_sha256=declared['sha256'],bag_size_bytes=BAG.stat().st_size,
        bag_hash_recomputed=False,finite_bag_window_ns=[start.to_nsec(),finish.to_nsec()],
        run_path=str(RUN),run_sha256=sha(RUN),run_window_ns=[run_start,run_finish],
        reference_path=str(REF),reference_sha256=sha(REF),reference_id=reference_id,source_context=context,
        metadata_topic_counts=dict(counts),in_run_reason_counts=dict(Counter(reason(r) for r in in_run)),
        selected_reason_counts=dict(Counter(reason(r) for r in failures)),
        selected_source_count=len(chosen),raw_frame_count=len(raw_rows),
        missing_source_stamps=sorted(set(chosen)-found),
        statuses=statuses,accepted_feedback=accepted,controller_states=controller,events=event_rows,
        raw_frames=sorted(raw_rows,key=lambda r:r['source_stamp_ns']),
        context_bag=str(OUT/'context.bag'),context_bag_sha256=sha(OUT/'context.bag'),script_sha256=sha(__file__))
    destination=OUT/'report.json'
    destination.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(destination),sha256=sha(destination),
        selected_reason_counts=report['selected_reason_counts'],raw_frame_count=len(raw_rows),
        missing=report['missing_source_stamps']),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
