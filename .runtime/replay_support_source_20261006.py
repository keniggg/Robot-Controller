"""Closed-bag source-window analysis only; no ROS initialization or publishers."""
from collections import deque, Counter
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import rosbag
from alicia_grasp_modes.support_observation import support_contradiction, qualify_support_source

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = ROOT/'ros_log/grasp_close_wire_retry_20261006T032914Z'
BAG = ROOT/'grasp_records/20261006T032914.897053Z_b4aed3dc145a43409c8562c79cdc0fe1/keyframes.bag'
SELECTION = json.loads((RUN/'selection_adapter.json').read_text())['new_selection']
TOPICS = ['/alicia_d/accepted_joint_states','/alicia_d/sdk_command',
          '/alicia_d/control_reference_epoch','/perception/unknown/detector_status','/grasp/state']
sdk, accepted = deque(maxlen=128), deque(maxlen=128)
all_sdk, all_accepted, faults, pending = [], [], [], {}
epoch = None
hard = None
guard = None

def classify(fault, sdk_rows, accepted_rows):
    return qualify_support_source(sdk_rows, accepted_rows,
        source_stamp_ns=fault['source_stamp_ns'], epoch_ns=epoch,
        expected_epoch_ns=fault['observed_epoch_ns'])

def revisit(at):
    global hard
    for key, row in tuple(pending.items()):
        q = classify(row['fault'], sdk, accepted)
        row['latest_qualification'] = q
        if q['state'] in ('moving','stationary'):
            row.update(first_qualified_arrival_ns=at, cached_qualification=q)
            if q['state'] == 'stationary': hard = row['fault']
            pending.pop(key)

with rosbag.Bag(str(BAG)) as bag:
    for topic, message, stamp in bag.read_messages(topics=TOPICS):
        at = stamp.to_nsec()
        if topic.endswith('control_reference_epoch'):
            e = message.stamp.to_nsec()
            epoch = e if message.frame_id == 'sdk_reference_epoch' and e > 0 else None
            sdk.clear();accepted.clear()
        elif topic.endswith('sdk_command'):
            sdk.append(message);all_sdk.append((at,message));revisit(at)
        elif topic.endswith('accepted_joint_states'):
            accepted.append(message);all_accepted.append((at,message));revisit(at)
        elif topic.endswith('detector_status'):
            revisit(at)
            status = json.loads(message.data)
            fault = support_contradiction(status, SELECTION)
            if fault is not None:
                fault['observed_epoch_ns'] = epoch
                q = classify(fault,sdk,accepted)
                row=dict(fault=fault,callback_proxy_arrival_ns=at,callback_qualification=q)
                faults.append(row)
                pending[fault['source_stamp_ns']] = row
                revisit(at)
        elif topic == '/grasp/state' and guard is None and 'SUPPORT_GEOMETRY_CONTRADICTION' in message.message:
            revisit(at)
            guard=dict(arrival_ns=at,stamp_ns=message.header.stamp.to_nsec(),message=message.message,
                pending_source_stamps=list(pending),qualified_hard_source_stamp=(hard or {}).get('source_stamp_ns'),
                sdk=tuple(sdk),accepted=tuple(accepted))

assert len(faults) == 13, len(faults)
assert guard is not None
for row in faults:
    row['guard_qualification_with_retained_128_history'] = classify(row['fault'],guard['sdk'],guard['accepted'])
    row['guard_qualification_with_complete_original_source_history'] = classify(row['fault'],
        [m for at,m in all_sdk if at <= guard['arrival_ns']],
        [m for at,m in all_accepted if at <= guard['arrival_ns']])
    row['cached_before_guard'] = row.get('first_qualified_arrival_ns',10**30) <= guard['arrival_ns']
    row['source_fault_blocks_guard'] = (row['fault']['source_stamp_ns'] in guard['pending_source_stamps']
        or row['fault']['source_stamp_ns'] == guard['qualified_hard_source_stamp'])
    if 'first_qualified_arrival_ns' in row:
        row['qualification_delay_after_status_ms'] = (row['first_qualified_arrival_ns']-row['callback_proxy_arrival_ns'])/1e6

guard.pop('sdk');guard.pop('accepted')
files=[Path('src/alicia_grasp_modes/src/alicia_grasp_modes/support_observation.py'),Path('src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py')]
report=dict(schema_version=1,created_at_utc=datetime.now(timezone.utc).isoformat(),scope='offline sealed bag; bag delivery times approximate callback availability; original image source timestamps determine all windows',bag=str(BAG),selection=SELECTION,source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},fault_count=len(faults),callback_class_counts=dict(Counter(r['callback_qualification']['state'] for r in faults)),cached_class_counts=dict(Counter(r.get('cached_qualification',{}).get('state','unqualified') for r in faults)),guard=guard,faults=faults,source_fault_would_block_guard=any(r['source_fault_blocks_guard'] for r in faults),physical_grip_verified=False,producer_target_loss_unchanged=True,motion_authorized=False,thresholds_changed=False)
if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand unavailable')
out=RUN/('support_source_replay_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'.json')
out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print('REPORT',out)
print(json.dumps({k:report[k] for k in ('fault_count','callback_class_counts','cached_class_counts','source_fault_would_block_guard')},indent=2))
for r in faults:
 print(r['fault']['source_stamp_ns'],r['callback_qualification']['state'],r.get('cached_qualification',{}).get('state'),r.get('qualification_delay_after_status_ms'),r['guard_qualification_with_complete_original_source_history'].get('reason'))
