"""Read only two sealed bag windows; preserve source timing, not image payloads."""
import argparse
import hashlib
import json
from pathlib import Path
import rosbag
import rospy


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', type=Path, required=True)
    ap.add_argument('--case', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    manifest = json.loads((a.record/'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    bagpath = a.record/'keyframes.bag'
    bagentry = next(row for row in manifest['files'] if row['path'] == 'keyframes.bag')
    assert bagpath.stat().st_size == bagentry['size_bytes']
    ingresspath = a.case/'selection/raw_support/unknown_ingress_review/report.json'
    ingress = json.loads(ingresspath.read_text())
    windows = [('raw_delay', 1791426945.8, 1791426952.),
               ('lock_and_first_loss', 1791426972.5, 1791426978.)]
    statuses, logs = [], []
    with rosbag.Bag(str(bagpath), 'r') as bag:
        for label, start, end in windows:
            for topic, message, stamp in bag.read_messages(
                    topics=['/perception/unknown/detector_status', '/rosout_agg'],
                    start_time=rospy.Time.from_sec(start), end_time=rospy.Time.from_sec(end)):
                if topic.endswith('detector_status'):
                    payload = json.loads(message.data)
                    if payload.get('generation') != 25:
                        continue
                    statuses.append(dict(window=label, bag_receipt_ns=stamp.to_nsec(),
                        payload=payload, raw_payload=message.data,
                        raw_payload_sha256=hashlib.sha256(message.data.encode()).hexdigest()))
                elif message.name == '/unknown_tabletop_perception':
                    logs.append(dict(window=label, bag_receipt_ns=stamp.to_nsec(),
                                     original_header_ns=message.header.stamp.to_nsec(), text=message.msg))
    early = [row['payload'] for row in statuses if row['window'] == 'raw_delay'
             and row['payload'].get('frame_timing')]
    gaps = []
    for previous, current in zip(early, early[1:]):
        p, c = previous['frame_timing'], current['frame_timing']
        gap = c['processing_start_monotonic_sec']-(p['processing_start_monotonic_sec']+p['processing_wall_sec'])
        if gap > .5:
            gaps.append(dict(previous_source_stamp_ns=p['source_stamp_ns'],
                next_source_stamp_ns=c['source_stamp_ns'],
                previous_status_to_next_process_wall_sec=gap,
                same_timer_thread_cpu_delta_sec=c['processing_start_thread_cpu_sec']-
                    (p['processing_start_thread_cpu_sec']+p['processing_thread_cpu_sec']),
                previous_stage_wall_sec=p['stage_wall_sec'],
                interpretation='Includes uninstrumented status publication/logging, timer sleep, and no-source callbacks; does not measure OS scheduler or GIL ownership.'))
    later = [row['payload'] for row in statuses if row['window'] == 'lock_and_first_loss']
    losses = [row for row in later if row.get('first_loss')]
    assert losses, 'actual first loss is required, never infer from a later latch alone'
    first = losses[0]
    loss = first['first_loss']
    observed, current, good = (loss[k] for k in
        ('observed_stamp_ns', 'source_stamp_ns', 'last_good_source_stamp_ns'))
    lockrows = [dict(source_stamp_ns=row.get('source_stamp_ns'), state=row['state'],
        detail=row['detail'], target_locked=row['target_locked'], target_lost=row['target_lost'],
        timing=row.get('frame_timing'), source_qualification=row.get('source_qualification'),
        rgb_identity_hold=row.get('rgb_identity_hold')) for row in later
        if row.get('source_stamp_ns', 0) <= current]
    report = dict(schema=1, offline_only=True, ros_node_or_rpc_calls=False,
        hardware_calls=False, production_changed=False, script_sha256=sha(__file__),
        bag_path=str(bagpath), bag_size_bytes=bagpath.stat().st_size,
        bag_declared_sha256=bagentry['sha256'],
        bag_sha_verification_scope='Manifest declared hash and actual file size checked; whole 3.46 GB bag not rehashed.',
        manifest_sha256=sha(a.record/'manifest.json'), windows=windows,
        ingress_report_path=str(ingresspath), ingress_report_sha256=sha(ingresspath),
        timing_gaps=gaps, first_loss=loss,
        loss_age_seconds=dict(current_source=(observed-current)/1e9,
                              last_good_reference=(observed-good)/1e9,
                              source_to_reference=(current-good)/1e9),
        lock_window_summary=lockrows, unknown_ros_logs=logs,
        proven_and_limits=[
            'Gen25 obtained a metric target after the initial runner acquisition window; the first 15 seconds were not the entire generation.',
            'The later failure has a fresh current source but an expired last-good reference. Read the original qualification and stages to identify which guard failed.',
            'Early same-source independent recorder arrivals localize the seconds of added latency to the unknown receiving/callback path.',
            'The processing Timer CPU did not run for most of the inter-status gap; current recorded segmentation/TF execution cannot account for the whole delay.',
            'No same-window receive/deserialize trace, GIL owner, page faults, per-TID scheduler record or memory sample is available here.',
            'No production blocking bug is established sufficiently to change synchronization, thresholds, queues, or source-age policy.'
        ], original_status_records=statuses)
    report['source_sha256'] = {str(p): sha(p) for p in (
        Path('src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'),
        Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/impl/tcpros_base.py'),
        Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py'))}
    data=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
    assert len(data.encode()) < 4*1024*1024
    a.output.mkdir(exist_ok=False)
    path=a.output/'report.json';path.write_text(data)
    print(json.dumps(dict(path=str(path), sha256=sha(path), timing_gaps=gaps,
        loss_age_seconds=report['loss_age_seconds'], first_loss=loss,
        later=[dict(source_stamp_ns=r['source_stamp_ns'], detail=r['detail'], state=r['state'],
             target_locked=r['target_locked'], target_lost=r['target_lost'],
             source_qualification=r['source_qualification']) for r in lockrows]),ensure_ascii=False))


if __name__ == '__main__':
    main()
