#!/usr/bin/env python3
"""Extract only sealed measurement protocol and detector metadata for gen32."""
from collections import Counter
from pathlib import Path
import hashlib
import json
import rosbag

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261009T141807.628466Z_3d22b7ddd3734f50a5a707e2ffe5aacb'
OUT = EXT/'ros_log/full_latest_ros_20261009T003407Z/gen32_handoff_analysis'
manifest = json.loads((REC/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
topics = ['/perception/unknown/detector_status',
          '/grasp_mode/observe_return_measurement_request',
          '/grasp_mode/observe_return_measurement_ack']
statuses, requests, acks = [], [], []
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        value = json.loads(msg.data)
        row = dict(received_ns=stamp.to_nsec(), message=value)
        if topic == topics[0]:
            if value.get('generation') == 32:
                statuses.append(row)
        elif topic == topics[1]:
            if value.get('selection', {}).get('generation') == 32:
                requests.append(row)
        else:
            acks.append(row)

def counts_in(value, prefix=''):
    """Preserve reported counts, never infer raw candidate counts from errors."""
    found = {}
    if isinstance(value, dict):
        for key, sub in value.items():
            path = prefix+'.'+key if prefix else key
            if isinstance(sub, (int, float)) and ('candidate' in key or 'match' in key or key in ('count', 'measured_point_count')):
                found[path] = sub
            if isinstance(sub, dict):
                found.update(counts_in(sub, path))
    return found

views = []
for req_row in requests:
    req = req_row['message']
    if req['state'] != 'requalifying':
        continue
    start, deadline = req['arrival_stamp_ns'], req['capture_deadline_ns']
    lease = req['lease_id']
    revokes = [r for r in requests if r['message']['lease_id'] == lease and r['message']['state'] == 'revoked']
    stop = max([deadline]+[r['message']['stamp_ns'] for r in revokes])
    full = [r for r in statuses if start-100_000_000 <= r['received_ns'] <= stop+300_000_000]
    chronology = []
    for r in full:
        m = r['message']; e = m.get('observation_measurement', {})
        source = m.get('source_stamp_ns')
        timing = m.get('frame_timing', {})
        chronology.append(dict(received_ns=r['received_ns'], source_stamp_ns=source,
            within_capture_by_bag_receipt=start <= r['received_ns'] <= deadline,
            status_state=m.get('state'), detail=m.get('detail'), target_lost=m.get('target_lost'),
            measurement_state=e.get('state'), reason=e.get('reason'), qualified=e.get('qualified'),
            source_stamps_ns=e.get('source_stamps_ns'), measurement_evaluated=e.get('measurement_evaluated'),
            exact_tf_consumed=e.get('exact_tf_consumed'), candidate_counts_reported=counts_in(m),
            processing_wall_sec=timing.get('processing_wall_sec'),
            status_source_age_sec=timing.get('status_source_age_sec')))
    protocol = [r for r in acks if r['message'].get('lease_id') == lease]
    arrival_acks = [r for r in protocol if r['message'].get('request_id') == req['request_id']]
    views.append(dict(view_index=req['view_index'], arrival_request=req_row,
        capture_start_ns=start, capture_deadline_ns=deadline, revocations=revokes,
        request_ack_summary=[dict(participant=r['message'].get('participant'),
            received_ns=r['received_ns'], accepted=r['message'].get('accepted'),
            qualified=r['message'].get('qualified'), state=r['message'].get('state'),
            reason=r['message'].get('reason'), source_stamps_ns=r['message'].get('source_stamps_ns'))
            for r in arrival_acks],
        all_lease_acks=protocol, arrival_chronology=chronology,
        reason_counts_within_bag_window=dict(Counter(r['reason'] for r in chronology if r['within_capture_by_bag_receipt'])),
        detector_status_full_payloads=full))

result = dict(record=str(REC), generation=32,
    manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),
    sealed_manifest={k:manifest.get(k) for k in ('state','result','failure_reason','recorder_closed','completed_at')},
    recorded_bag_manifest=next(r for r in manifest['files'] if r['path']=='keyframes.bag'),
    topics_read=topics, requests=requests, views=views,
    first_loss=next((r for r in statuses if r['message'].get('target_lost') is True), None),
    limitations=['Only specified metadata topics decoded; no RGB/depth/temperature topics read.',
        'Bag receipt is transport time, not exact producer computation completion; original frame_timing preserved.',
        'Candidate counts absent from source payload are not inferred from reason strings.',
        'This chronology reports original runtime messages, not a pixel replay or physical grasp success.'])
OUT.mkdir(exist_ok=True)
target=OUT/'arrival_details.json'
assert not target.exists(), str(target)
target.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
for view in views:
    print(json.dumps({k:view[k] for k in ('view_index','request_ack_summary','reason_counts_within_bag_window','arrival_chronology')}, ensure_ascii=False))
print('OUTPUT', str(target), hashlib.sha256(target.read_bytes()).hexdigest())
