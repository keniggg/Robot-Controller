"""Bounded offline metadata-only failure chain analysis; no ROS initialization."""
import argparse
from collections import Counter
import hashlib
import io
import json
from pathlib import Path
import zipfile

EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')

def sha(b):return hashlib.sha256(b).hexdigest()
def encoded(v):return (json.dumps(v,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
def save(path,report):
    path.mkdir(exist_ok=False)
    report['script_sha256']=sha(Path(__file__).read_bytes())
    payload=encoded(report);(path/'report.json').write_bytes(payload)
    (path/'SHA256SUMS').write_text(sha(payload)+'  report.json\n')
    print(json.dumps({'report':str(path/'report.json'),'sha256':sha(payload),'conclusion':report['conclusion']},ensure_ascii=False))

def gen24():
    p=EXT/'ros_log/explicit_grasp_20261007T122934Z/raw_support'
    summary=json.loads((p/'summary.json').read_text())
    if not summary['archive_closed'] or not summary['complete_without_writer_error']:raise ValueError('not_sealed')
    rows=[]
    with zipfile.ZipFile(p/'diagnostic.zip') as z:
        mr=z.read('entry_manifest.json');entries={e['name']:e for e in json.loads(mr)}
        for name in sorted(entries):
            if not (name.startswith('status/') and name.endswith('.json')):continue
            raw=z.read(name);entry=entries[name]
            if sha(raw)!=entry['sha256'] or len(raw)!=entry['bytes']:raise ValueError('bad_entry')
            envelope=json.loads(raw);original=envelope['original_status_data']
            if sha(original.encode())!=envelope['original_status_utf8_sha256']:raise ValueError('bad_original')
            data=json.loads(original)
            if data.get('generation')==24:
                rows.append(dict(entry=name,entry_sha256=sha(raw),receipt_ros_ns=envelope['receipt_ros_ns'],status=data,
                                 original_status_data=original,original_status_utf8_sha256=sha(original.encode())))
    lostindex=next(i for i,r in enumerate(rows) if r['status'].get('first_loss'))
    loss=rows[lostindex]['status']['first_loss'];before=rows[:lostindex]
    ready=[r for r in before if r['status']['state']=='ready']
    after=next(r for r in rows[lostindex+1:] if r['status'].get('source_stamp_ns'))
    last=ready[-1]['status'];ft=last['frame_timing'];lastns=loss['last_good_source_stamp_ns']
    save(p/'first_loss_metadata_analysis',dict(diagnostic_only=True,hardware_commands_sent=False,
        production_modified=False,thresholds_modified=False,target_recovered=False,
        capture_summary_sha256=sha((p/'summary.json').read_bytes()),entry_manifest_sha256=sha(mr),
        generation=24,first_loss=loss,
        status_counts=dict(Counter(r['status']['state']+':'+r['status']['detail'] for r in rows)),
        ready_count_before_loss=len(ready),support_pending_before_loss=sum(r['status'].get('detail')=='support_measurement_pending_revalidation' for r in before),
        last_ready_timing=ft,last_exact_pair_callback_to_loss_sec=(loss['observed_stamp_ns']-ft['exact_pair_callback_ros_ns'])/1e9,
        last_good_source_to_loss_sec=(loss['observed_stamp_ns']-lastns)/1e9,
        next_processed_source_ns=after['status']['source_stamp_ns'],
        processed_source_gap_sec=(after['status']['source_stamp_ns']-lastns)/1e9,
        last_status_publish_to_recorder_receipt_sec=(ready[-1]['receipt_ros_ns']-ft['status_build_ros_ns'])/1e9,
        selected_original_statuses=rows[:lostindex+1]+[after],
        conclusion='Six qualified ready frames followed by a missing exact RGB-D stream and the unchanged freshness timeout, not support recovery rejection. Later image arrival did not revive the generation.',
        limitations=['Metadata-only: no image decoding or fit was run.',
            'Saved source pairs are status-referenced and do not constitute a complete image-arrival stream.',
            'Original producer timestamps are distinguished from recorder receipt timestamps; recorder delay is not called camera latency.',
            'SDK/host diagnostics are analyzed independently; this status evidence alone cannot localize the upstream missing-stream cause.']))

def gen23():
    import rosbag
    import rospy
    if rospy.core.is_initialized():raise ValueError('offline_only')
    p=EXT/'ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support'
    bagpath=EXT/'grasp_records/20261007T120119.114132Z_74e95ad0103a48b6991dd20eb95d279d/keyframes.bag'
    stat=bagpath.stat();rows=[];feedback=[]
    with rosbag.Bag(str(bagpath)) as b:
        for topic,msg,t in b.read_messages(topics=['/perception/unknown/detector_status','/alicia_d/accepted_joint_states'],
                start_time=rospy.Time(1791374505),end_time=rospy.Time(1791374510)):
            if topic.endswith('detector_status'):
                d=json.loads(msg.data)
                if d.get('generation')==23:rows.append(dict(bag_time_ns=t.to_nsec(),status=d,
                    original_status_data=msg.data,original_status_utf8_sha256=sha(msg.data.encode())))
            else:
                stream=io.BytesIO();msg.serialize(stream)
                feedback.append(dict(bag_time_ns=t.to_nsec(),source_stamp_ns=msg.header.stamp.to_nsec(),
                    names=list(msg.name),position=list(msg.position),serialized_sha256=sha(stream.getvalue())))
    if bagpath.stat().st_size!=stat.st_size or bagpath.stat().st_mtime_ns!=stat.st_mtime_ns:raise ValueError('bag_changed')
    idx=next(i for i,r in enumerate(rows) if r['status'].get('first_loss'))
    status=rows[idx]['status'];loss=status['first_loss'];ft=status['frame_timing'];last=loss['last_good_source_stamp_ns']
    pending=[r for r in rows[:idx] if (r['status'].get('source_qualification') or {}).get('reference_source_stamp_ns')==last]
    sources=[last]+[r['status']['source_stamp_ns'] for r in pending]+[loss['source_stamp_ns']]
    brackets=[]
    for source in sources:
        earlier=[r for r in feedback if r['source_stamp_ns']<=source];later=[r for r in feedback if r['source_stamp_ns']>=source]
        before=max(earlier,key=lambda r:r['source_stamp_ns'],default=None);after=min(later,key=lambda r:r['source_stamp_ns'],default=None)
        brackets.append(dict(query_source_ns=source,before=before,after=after,
            bracket_seconds=(after['source_stamp_ns']-before['source_stamp_ns'])/1e9 if before and after else None,
            independent_source_qualification_not_asserted=True))
    save(p/'analysis/first_loss_bag_metadata',dict(diagnostic_only=True,hardware_commands_sent=False,
        production_modified=False,thresholds_modified=False,target_recovered=False,generation=23,
        bag_path=str(bagpath),bag_size=stat.st_size,bag_mtime_ns=stat.st_mtime_ns,
        bounded_bag_interval_ns=[1791374505000000000,1791374510000000000],
        first_loss=loss,first_loss_timing=ft,original_statuses=rows[:idx+1],accepted_feedback=feedback,source_feedback_brackets=brackets,
        source_vs_last_good_sec=(loss['source_stamp_ns']-last)/1e9,
        observed_vs_last_good_sec=(loss['observed_stamp_ns']-last)/1e9,
        failure_vs_last_good_sec=(ft['failure_ros_ns']-last)/1e9,
        processing_start_vs_last_good_sec=(ft['processing_start_ros_ns']-last)/1e9,
        after_segmentation_vs_last_good_sec=(ft['processing_start_ros_ns']-last)/1e9+ft['stage_end_elapsed_sec']['post_segment'],
        conclusion='Two support-quality pending frames precede a fresh current frame. The last-good 0.8-second deadline expires inside support_gap_measured_revalidation after about 75ms of computation. Current-frame freshness remains below the gate. This is distinct from gen24 missing-stream failure.',
        limitations=['No first-loss RGB-D fit or identity replay was run; support recovery success is not asserted from timing alone.',
            'Producer instrumentation supplies stage timing; bag receipt delay is not counted as camera latency.',
            'No threshold extension, old-source renewal, target resurrection or motion is authorized.',
            'Feedback bracketing is reported without inferring unchanged epoch/config or full production qualification.',
            'Original bag is read in a bounded interval; file size/mtime stable, whole large bag is not rehashed.'],ros_node_initialized=rospy.core.is_initialized()))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--generation',required=True,type=int,choices=[23,24]);args=parser.parse_args()
    (gen23 if args.generation==23 else gen24)()
