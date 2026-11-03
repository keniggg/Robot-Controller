"""Summarize a sealed retry without treating bag delivery as sensor timing."""
import json
from pathlib import Path
from collections import Counter
import hashlib
import rosbag

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/user_grasp_20261008T090751Z')
OUT = BASE / 'retry_after_plane_fix'


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    record = Path(json.loads((OUT / 'active_record.json').read_text())['record_dir'])
    outcome = json.loads((record / 'recorder_outcome.json').read_text())
    assert outcome['bag_closed']
    statuses, camera_logs, commands = [], [], Counter()
    with rosbag.Bag(str(record / 'keyframes.bag')) as bag:
        for topic, message, stamp in bag.read_messages(topics=[
                '/perception/unknown/detector_status', '/rosout_agg',
                '/joint_commands', '/alicia_controller/command',
                '/alicia_controller/follow_joint_trajectory/goal']):
            if topic.endswith('detector_status'):
                data = json.loads(message.data)
                if data.get('generation') == 28:
                    statuses.append(dict(bag_receipt_ns=stamp.to_nsec(), status=data))
            elif topic == '/rosout_agg':
                if message.name == '/supervisor_camera':
                    camera_logs.append(dict(stamp_ns=message.header.stamp.to_nsec(), message=message.msg))
            else:
                commands[topic] += 1
    lost_row = next(r for r in statuses if r['status'].get('first_loss'))
    first = lost_row['status']['first_loss']
    at = first['observed_stamp_ns']
    ready = [r for r in statuses if r['status'].get('state') == 'ready'
             and r['status'].get('source_stamp_ns', at) <= at]
    recoveries = [dict(source_stamp_ns=r['status']['source_stamp_ns'],
                       qualification=r['status']['source_qualification'])
                  for r in statuses if r['status'].get('source_qualification', {}).get('state')
                  == 'recovered_unique_measured_frame']
    details = Counter(r['status']['detail'] for r in statuses)
    window = []
    for row in statuses:
        d = row['status']
        ns = d.get('source_stamp_ns')
        if ns is None:
            if row is not lost_row:
                continue
        elif not at - 3_000_000_000 <= ns <= at + 2_000_000_000:
            continue
        window.append(dict(bag_receipt_ns=row['bag_receipt_ns'],
                           source_stamp_ns=ns, detail=d['detail'],
                           frame_timing=d.get('frame_timing'),
                           rgbd_ingress_timing=d.get('rgbd_ingress_timing')))
    result = dict(record_dir=str(record), generation=28,
        result_summary=json.loads((record / 'result_summary.json').read_text()),
        recorder_outcome=outcome, status_counts=dict(details), first_loss=first,
        time_from_acquired_to_first_loss_sec=(at-ready[0]['status']['source_stamp_ns'])/1e9,
        first_loss_since_last_good_sec=(at-first['last_good_source_stamp_ns'])/1e9,
        support_recoveries=recoveries, source_window=window,
        plane_identity_error_recurred=any('support_gap_reference_plane_changed' in k for k in details),
        observed_command_topic_counts=dict(commands),
        task_start_submitted='GRASP_START_CALL ' in (record / 'runner.log').read_text(),
        camera_logs_in_failure_window=[r for r in camera_logs
                                      if at-3_000_000_000 <= r['stamp_ns'] <= at+2_000_000_000],
        diagnosis=dict(
            confirmed='No complete eligible RGB-D pair at watchdog decision; depth arrived before decision, matching color afterward; earlier last-good already beyond 0.8 seconds',
            unresolved='SDK acquisition versus asynchronous publication versus transport/scheduling cannot be uniquely attributed from this keyframe bag',
            limitations=['Bag receipt times include recorder scheduling/disk delay',
                         'Image header stamps are host read-completion, not calibrated exposure time',
                         'Publication runtime profile contains policy, not event counters',
                         'No thresholds changed or new motion authorized by this analysis']),
        summary_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (OUT / 'sealed_retry_analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['record_dir','status_counts',
        'time_from_acquired_to_first_loss_sec','first_loss_since_last_good_sec',
        'plane_identity_error_recurred','observed_command_topic_counts','task_start_submitted']},
        ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
