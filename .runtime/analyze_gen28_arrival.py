#!/usr/bin/env python3
"""Read only sealed gen28 metadata; preserve arrival rejection chronology."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import rosbag

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261009T122642.289598Z_97e4fc00eabf4de2bbac5e2ba35ba119'
OUT = EXT/'ros_log/full_latest_ros_20261009T003407Z/gen28_handoff_analysis'
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
                if payload.get('generation') == 28:
                    statuses.append(dict(row, message=payload))
            else:
                (requests if topic.endswith('request') else acks).append(dict(row, message=payload))
views=[]
for rr in requests:
    request=rr['message']
    if request['state'] != 'requalifying':continue
    start,deadline=request['arrival_stamp_ns'],request['capture_deadline_ns']
    selected=[r for r in statuses if start<=r['received_ns']<=deadline and r['message'].get('observation_measurement',{}).get('request',{}).get('lease_id')==request['lease_id']]
    rows=[]
    for r in selected:
        msg=r['message'];e=msg['observation_measurement'];source=msg.get('source_stamp_ns')
        rows.append(dict(received_ns=r['received_ns'],source_stamp_ns=source,status_state=msg['state'],detail=msg.get('detail'),reason=e.get('reason'),qualified=e.get('qualified'),measurement_state=e.get('state'),source_stamps_ns=e.get('source_stamps_ns'),source_to_status_bag_receipt_sec=(r['received_ns']-source)*1e-9 if source else None,measurement_evaluated=e.get('measurement_evaluated'),exact_tf_consumed=e.get('exact_tf_consumed')))
    feedback=[r for r in joints if start<=r['source_stamp_ns']<=deadline]
    names=feedback[0]['names'] if feedback else []
    span=[max(r['positions'][i] for r in feedback)-min(r['positions'][i] for r in feedback) for i in range(len(names))]
    views.append(dict(view_index=request['view_index'],arrival_request=request,arrival_window=dict(start_ns=start,deadline_ns=deadline,duration_sec=(deadline-start)*1e-9),statuses=rows,reason_counts=dict(Counter(r['reason'] for r in rows)),qualified_count=sum(r['qualified'] is True for r in rows),protocol_acks=[r for r in acks if r['message']['lease_id']==request['lease_id']],arrival_stationary_feedback=dict(count=len(feedback),names=names,joint_peak_to_peak=span,first=feedback[0] if feedback else None,last=feedback[-1] if feedback else None)))
first_loss=next((r for r in statuses if r['message'].get('target_lost') is True),None)
summary=dict(record=str(REC),generation=28,manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),sealed_manifest={k:manifest[k] for k in ('state','result','failure_reason','recorder_closed','completed_at')},views=views,first_loss=first_loss,task_states=tasks,requests=requests,limitations=['Read only sealed non-temperature metadata.','Bag receipt ages are not exact computation completion timestamps.','No pixel replay or inferred low-level cause is claimed by this chronology.'])
OUT.mkdir(exist_ok=True)
(OUT/'arrival_and_terminal.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
(OUT/'detector_status.json').write_text(json.dumps(statuses,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(OUT/'arrival_and_terminal.json'), generation=28, status_count=len(statuses), view_count=len(views), first_loss=first_loss),ensure_ascii=False,indent=2))
