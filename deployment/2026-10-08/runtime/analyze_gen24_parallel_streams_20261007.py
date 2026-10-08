"""Indexed, bounded metadata-only bag analysis. No ROS node or hardware access."""
from collections import Counter
import hashlib
import io
import json
from pathlib import Path
import statistics
import rosbag
import rospy

W=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
TOPICS=['/alicia_d/accepted_joint_states','/tf','/joint_states']
CAMERA_WAIT=[1791376178046740800,1791376179687992300]

def sha(raw):return hashlib.sha256(raw).hexdigest()
def enc(v):return (json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
def serialize_sha(m):
    s=io.BytesIO();m.serialize(s);return sha(s.getvalue())
def stats(v):
    v=sorted(v)
    if not v:return dict(count=0)
    def q(f):
        x=f*(len(v)-1);i=int(x);return v[i]+(v[min(i+1,len(v)-1)]-v[i])*(x-i)
    return dict(count=len(v),minimum_ms=v[0]*1e-6,p50_ms=q(.5)*1e-6,p95_ms=q(.95)*1e-6,maximum_ms=v[-1]*1e-6)
def intervals(stamps):return stats([b-a for a,b in zip(stamps,stamps[1:])])

def analyze_bag(folder,start,end,camera_interval=None):
    p=EXT/'grasp_records'/folder;bagpath=p/'keyframes.bag';manifestraw=(p/'manifest.json').read_bytes();manifest=json.loads(manifestraw)
    assert manifest['recorder_closed'] and manifest['state']=='complete'
    baginfo=next(v for v in manifest['files'] if v['path']=='keyframes.bag');st=bagpath.stat();assert st.st_size==baginfo['size_bytes']
    rows={t:[] for t in TOPICS}
    with rosbag.Bag(str(bagpath)) as b:
        for topic,m,t,connection in b.read_messages(topics=TOPICS,start_time=rospy.Time(start),end_time=rospy.Time(end),return_connection_header=True):
            r=dict(bag_time_ns=t.to_nsec(),callerid=connection.get('callerid'),serialized_sha256=serialize_sha(m))
            if topic=='/tf':
                r['transforms']=[dict(source_stamp_ns=v.header.stamp.to_nsec(),parent=v.header.frame_id,child=v.child_frame_id) for v in m.transforms]
                r['source_stamps_ns']=sorted(set(v['source_stamp_ns'] for v in r['transforms']))
            else:r.update(source_stamps_ns=[m.header.stamp.to_nsec()],frame_id=m.header.frame_id,joint_names=list(m.name))
            rows[topic].append(r)
    assert bagpath.stat().st_size==st.st_size and bagpath.stat().st_mtime_ns==st.st_mtime_ns
    report=dict(bag_path=str(bagpath),manifest_sha256=sha(manifestraw),bag_sha256_from_sealed_manifest=baginfo['sha256'],
        whole_bag_hash_recomputed=False,read_interval_ros_ns=[start*10**9,end*10**9],topics={})
    accepted=set(s for r in rows[TOPICS[0]] for s in r['source_stamps_ns'])
    for topic,rr in rows.items():
        source=sorted(set(s for r in rr for s in r['source_stamps_ns']));receipt=[r['bag_time_ns'] for r in rr]
        v=dict(message_count=len(rr),unique_source_count=len(source),source_interval=intervals(source),bag_receipt_interval=intervals(receipt),
            callerids=dict(Counter(r['callerid'] for r in rr)),source_equals_accepted_count=sum(s in accepted for s in source),
            rows=rr)
        if camera_interval:
            lo,hi=camera_interval
            inside=[r for r in rr if lo<=r['bag_time_ns']<=hi]
            ss=[s for s in source if lo<=s<=hi]
            surrounding=[s for s in source if lo-300000000<=s<=hi+300000000]
            v['during_camera_wait']=dict(bag_message_count=len(inside),unique_source_count=len(ss),
                source_first_ns=ss[0] if ss else None,source_last_ns=ss[-1] if ss else None,
                source_interval=intervals(ss),source_interval_with_300ms_margin=intervals(surrounding),
                bag_receipt_interval=intervals([r['bag_time_ns'] for r in inside]))
        if topic=='/tf':
            links={}
            for r in rr:
                for t in r['transforms']:links.setdefault(t['parent']+'->'+t['child'],[]).append(t['source_stamp_ns'])
            v['per_link']={k:dict(count=len(vv),unique_count=len(set(vv)),source_interval=intervals(sorted(set(vv)))) for k,vv in links.items()}
        report['topics'][topic]=v
    return report


def main():
    assert not rospy.core.is_initialized()
    gen24=analyze_bag('20261007T122934.991248Z_36ac12c5aa6c40f885fa8ae37760b630',1791376176,1791376181,CAMERA_WAIT)
    gen23=analyze_bag('20261007T120119.114132Z_74e95ad0103a48b6991dd20eb95d279d',1791374505,1791374510)
    source=1791374508742347478
    tfrows=gen23['topics']['/tf']['rows']
    future=[r for r in tfrows if r['source_stamps_ns'] and max(r['source_stamps_ns'])>=source]
    nexttf=min(future,key=lambda r:max(r['source_stamps_ns'])) if future else None
    gen23['first_loss_tf_query_bracket_diagnostic']=dict(image_source_ns=source,processing_start_ros_ns=1791374508786808013,
        tf_lookup_reported_wall_ms=42.20606999297161,first_saved_tf_at_or_after_image_source=nexttf,
        limitation='Bag receipt is a recorder timestamp, not the detector TF listener delivery time.')
    poll=W/'src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch';driver=W/'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp'
    report=dict(diagnostic_only=True,hardware_commands_sent=False,ros_node_initialized=rospy.core.is_initialized(),production_modified=False,
        script_sha256=sha(Path(__file__).read_bytes()),camera_sdk_wait_interval_ns=CAMERA_WAIT,
        camera_interval_provenance='Exact interval provided by independent metadata read analysis; no device query.',gen24=gen24,gen23=gen23,
        config_evidence=dict(launch_path=str(poll),launch_sha256=sha(poll.read_bytes()),driver_path=str(driver),driver_sha256=sha(driver.read_bytes()),
            launch_state_poll_rate_hz=10.,code_default_only_hz=20.,launch_overrides_code_default=True,
            accepted_publish_rule='Each accepted SDK sample publishes /alicia_d/accepted_joint_states with actual host parser time; heartbeat does not publish this topic.',
            robot_state_publisher_input='/alicia_d/accepted_joint_states',heartbeat_joint_states_hz=50.,
            runtime_parameter_snapshot_not_collected=True),
        limitations=['These headers use guest ROS/host times; a guest clock discontinuity or queues cannot be excluded merely from nominal source frequency.',
            'Continued parser-stamped feedback and TF during the camera gap argue against a whole-VM pause lasting the full gap; shorter scheduler stalls remain possible.',
            'Current launch is static config evidence; no ROS parameter was queried or changed.',
            'The 50Hz joint_states heartbeat may repeat retained positions and is not fresh measured feedback.',
            'No source deadlines, support thresholds, TF timestamps, identity or hardware state were changed.'])
    out=EXT/'ros_log/explicit_grasp_20261007T122934Z/raw_support/parallel_stream_gap_analysis';out.mkdir(exist_ok=False)
    raw=enc(report);(out/'report.json').write_bytes(raw);(out/'SHA256SUMS').write_text(sha(raw)+'  report.json\n')
    print(json.dumps(dict(output=str(out),sha256=sha(raw),gen24={t:{k:v for k,v in r.items() if k in ('message_count','source_interval','bag_receipt_interval','during_camera_wait','source_equals_accepted_count')} for t,r in gen24['topics'].items()},gen23={t:{k:v for k,v in r.items() if k in ('message_count','source_interval','source_equals_accepted_count')} for t,r in gen23['topics'].items()},nexttf=nexttf),ensure_ascii=False))

if __name__=='__main__':main()
