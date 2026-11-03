"""Offline timing and no-action evidence from a bounded immutable bag prefix."""
import hashlib
import io
import json
import os
from pathlib import Path

import numpy as np
import rosbag
import rospy

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
P = N / 'verification/gen26_attempt1_initial_support_prefix'
OUT = N / 'verification/gen27_recorded_feedback_gap_analysis'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert not os.statvfs(E).f_flag & os.ST_RDONLY
    receipt = json.loads((P / 'receipt.json').read_text())
    prefix = P / 'prefix_64MiB.bag'
    assert sha(prefix) == receipt['prefix_sha256']
    rows = json.loads((P / 'detector_status.json').read_text())
    statuses = [r['message'] for r in rows if r['message'].get('generation') == 5]
    failure = next(s for s in statuses if s.get('frame_timing', {}).get(
        'initial_station_invalidated_reason') == 'source_exceeds_existing_frame_limit')
    previous = max((s for s in statuses if s.get('state') == 'ready'
                    and s['source_stamp_ns'] < failure['source_stamp_ns']),
                   key=lambda s: s['source_stamp_ns'])
    gaps = [s for s in statuses if previous['source_stamp_ns'] < s.get('source_stamp_ns', 0)
            < failure['source_stamp_ns'] and s.get('source_qualification', {}).get('kind')]
    timing = failure['frame_timing']
    reference_ns, source_ns = previous['source_stamp_ns'], failure['source_stamp_ns']
    begin, end = timing['processing_start_ros_ns'], timing['status_build_ros_ns']
    start_ns, stop_ns = 1792041464000000000, 1792041480000000000
    topics = ['/alicia_d/accepted_joint_states', '/grasp/state',
              '/alicia_controller/follow_joint_trajectory/goal',
              '/hand_controller/follow_joint_trajectory/goal']
    counts = dict.fromkeys(topics, 0)
    measured, states = [], []
    memory = io.BytesIO(prefix.read_bytes())
    memory.mode = 'rb'
    with rosbag.Bag(memory, mode='r', allow_unindexed=True) as bag:
        list(bag.reindex())
        bag._connection_indexes_read = True
        for topic, msg, stamp in bag.read_messages(topics=topics,
                start_time=rospy.Time(start_ns // 10**9, start_ns % 10**9),
                end_time=rospy.Time(stop_ns // 10**9, stop_ns % 10**9)):
            counts[topic] += 1
            if topic == topics[0]:
                assert msg.header.frame_id == 'sdk_measured'
                measured.append(dict(stamp_ns=msg.header.stamp.to_nsec(),
                    names=list(msg.name), positions=list(msg.position)))
            elif topic == '/grasp/state':
                states.append(dict(record_stamp_ns=stamp.to_nsec(), state=msg.state,
                    active=bool(msg.active), success=bool(msg.success), message=msg.message))
    assert measured and all(row['names'] == measured[0]['names'] for row in measured)
    values = np.asarray([row['positions'] for row in measured])
    report = dict(schema=1, diagnosis='timely_feedback_recovery_exceeds_old_reference_deadline_during_revalidation',
        source_sha256={str(p): sha(p) for p in (prefix, P/'detector_status.json', P/'receipt.json')},
        original_record=receipt['source_path'], original_record_modified=False,
        reference_source_ns=reference_ns, recovery_source_ns=source_ns,
        recovery_processing_start_ns=begin, failure_status_build_ns=end,
        reference_age_at_entry_sec=(begin-reference_ns)*1e-9,
        recovery_source_age_at_entry_sec=(begin-source_ns)*1e-9,
        reference_age_at_failure_status_sec=(end-reference_ns)*1e-9,
        recovery_source_age_at_failure_status_sec=(end-source_ns)*1e-9,
        failed_stage=timing['failed_stage'], original_error=timing['initial_station_invalidated_reason'],
        gap_evidence=[dict(source_stamp_ns=s['source_stamp_ns'],
            qualification=s['source_qualification']) for s in gaps],
        telemetry_window_ns=[start_ns, stop_ns], topic_counts=counts,
        maximum_accepted_joint_change_rad=float(np.max(np.ptp(values[:, :6], axis=0))),
        task_active_seen=any(s['active'] for s in states), task_states=states,
        trajectory_goal_seen=any(counts[t] for t in topics[2:]),
        script_sha256=sha(Path(__file__)), no_ros_master_contact=True,
        no_hardware_access=True, no_temperature_queries=True,
        limitation='Timing analysis and bounded recorded telemetry only; no repair validation or physical grasp success is claimed.')
    OUT.mkdir(exist_ok=False)
    for name, value in [('findings.json', report), ('accepted_feedback.json', measured)]:
        (OUT/name).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('diagnosis', 'reference_age_at_entry_sec',
        'reference_age_at_failure_status_sec', 'recovery_source_age_at_failure_status_sec',
        'maximum_accepted_joint_change_rad', 'task_active_seen', 'trajectory_goal_seen')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
