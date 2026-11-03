#!/usr/bin/env python3
"""Read only sealed gen22 metadata; preserve arrival rejection chronology."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import rosbag

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261009T095400.350634Z_f3717a52509b4eb4a1e6354512f1b17d'
OUT = EXT/'ros_log/full_latest_ros_20261009T003407Z/gen22_handoff_analysis'
manifest = json.loads((REC/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
statuses, requests, acks, tasks, joints = [], [], [], [], []
topics = ['/perception/unknown/detector_status', '/grasp_mode/observe_return_measurement_request',
          '/grasp_mode/observe_return_measurement_ack', '/grasp/state', '/alicia_d/accepted_joint_states']
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic, message, stamp in bag.read_messages(topics=topics):
        row = dict(received_ns=stamp.to_nsec())
        if topic.endswith('accepted_joint_states'):
            if message.header.frame_id == 'sdk_measured':
                joints.append(dict(row, source_stamp_ns=message.header.stamp.to_nsec(),
                    names=list(message.name), positions=list(message.position)))
        elif topic == '/grasp/state':
            tasks.append(dict(row, source_stamp_ns=message.header.stamp.to_nsec(),
                state=message.state, message=message.message, active=message.active,
                success=message.success))
        else:
            payload = json.loads(message.data)
            if topic.endswith('detector_status'):
                if payload.get('generation') == 22:
                    statuses.append(dict(row, message=payload))
            else:
                (requests if topic.endswith('request') else acks).append(dict(row, message=payload))
request = next(r['message'] for r in requests if r['message']['state'] == 'requalifying')
start, deadline = request['arrival_stamp_ns'], request['capture_deadline_ns']
window = [r for r in statuses if start <= r['received_ns'] <= deadline]
pending = [r for r in window if r['message'].get('observation_measurement', {}).get('state') == 'requalifying']
rows = []
for row in pending:
    message = row['message']
    evidence = message['observation_measurement']
    source = message.get('source_stamp_ns')
    rows.append(dict(received_ns=row['received_ns'], source_stamp_ns=source,
        reason=evidence.get('reason'), qualified=evidence.get('qualified'),
        source_stamps_ns=evidence.get('source_stamps_ns'),
        source_to_status_bag_receipt_sec=(row['received_ns']-source)*1e-9 if source else None,
        measurement_evaluated=evidence.get('measurement_evaluated'),
        exact_tf_consumed=evidence.get('exact_tf_consumed')))
stationary = [r for r in joints if start <= r['source_stamp_ns'] <= deadline]
assert stationary
names = stationary[0]['names']
assert all(r['names'] == names for r in stationary)
span = [max(r['positions'][i] for r in stationary)-min(r['positions'][i] for r in stationary)
        for i in range(len(names))]
first_loss = next((r for r in statuses if r['message'].get('target_lost') is True), None)
summary = dict(record=str(REC), manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),
    sealed_manifest={k:manifest[k] for k in ('state','result','failure_reason','recorder_closed','completed_at')},
    selection_generation=22, actual_motion_occurred=True, view_index=1,
    arrival_request=request, arrival_window=dict(start_ns=start,deadline_ns=deadline,
        duration_sec=(deadline-start)*1e-9), arrival_rejections=rows,
    reason_counts=dict(Counter(row['reason'] for row in rows)),
    requalifying_status_count=len(rows), qualified_count=sum(row['qualified'] is True for row in rows),
    protocol_acks=[r for r in acks if r['message']['lease_id']==request['lease_id']],
    first_loss=first_loss, task_states=tasks,
    arrival_stationary_feedback=dict(count=len(stationary),names=names,
        joint_peak_to_peak=span,first=stationary[0],last=stationary[-1]),
    causal_conclusion='See exact in-window measurement reason counts and protocol ACKs; no assumption that the new failure equals gen21. ACK_TIMEOUT and subsequent target retirement are downstream of unfinished arrival qualification.',
    limitations=['Status receipt age is derived from bag receipt, not a claimed exact computation completion timestamp.',
                 'All qualification flags remain false; false exact_tf_consumed flags in failure reports do not identify the low-level operation where processing stopped.',
                 'No pixel segmentation replay has been performed; search-boundary versus depth-seed failure is reported exactly, without an inferred pixel cause.',
                 'No temperature topics read; no ROS commands or source mutations.'])
OUT.mkdir(exist_ok=True)
(OUT/'arrival_and_terminal.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
(OUT/'detector_status.json').write_text(json.dumps(statuses,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(OUT/'arrival_and_terminal.json'),reason_counts=summary['reason_counts'],
    rows=rows,arrival_joint_span=span,first_loss_received_ns=first_loss['received_ns']),ensure_ascii=False,indent=2))
