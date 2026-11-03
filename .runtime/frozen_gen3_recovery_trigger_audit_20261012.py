"""Audit archived branch events; generic recovery receipts are not fault triggers."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import rosbag

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=BASE/'ros_log/full_latest_frozen_no_skin_20261012T021050Z'
RECORD=BASE/'grasp_records/20261012T032110.570728Z_a1a1c737f44d453aa25a5585504da760'
OUT=NEW/'gen3_analysis'
markers=('station_support_gap','station_support_gap_recovery','pending_current_support','STATION_REGISTRATION_REACQUIRE','YAW_BOUND_EXCEEDED')
counts=Counter({k:0 for k in markers})
details=Counter()
transitions=[]
first_recovery=None
last_recovery=None
previous=None
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,message,stamp in bag.read_messages(topics=['/perception/unknown/detector_status','/grasp_6d/status']):
        if topic.endswith('detector_status'):
            data=json.loads(message.data)
            if data.get('generation')!=3:
                continue
        else:
            data=dict(text=message.data)
        counts[topic]+=1
        for marker in markers:
            counts[marker]+=int(marker in message.data)
        if topic.endswith('detector_status'):
            key=(data.get('state'),data.get('detail'))
            details[key]+=1
            if key!=previous:
                transitions.append(dict(receipt_ns=stamp.to_nsec(),source_ns=data.get('source_stamp_ns'),state=key[0],detail=key[1]))
                previous=key
            if data.get('station_compute_recovery') is not None:
                counts['generic_station_compute_recovery']+=1
                row=dict(receipt_ns=stamp.to_nsec(),source_ns=data.get('source_stamp_ns'),state=key[0],detail=key[1],
                    parent_source_ns=data['station_compute_recovery'].get('source_ns'),
                    has_support_gap_recovery=data.get('station_support_gap_recovery') is not None)
                if first_recovery is None:
                    first_recovery=row
                last_recovery=row
core=(NEW/'core.log').read_bytes()
core_matches={key:[] for key in ('STATION_REGISTRATION_REACQUIRE','YAW_BOUND_EXCEEDED')}
for number,line in enumerate(core.decode(errors='replace').splitlines(),1):
    for key in core_matches:
        if key in line:
            core_matches[key].append(dict(line=number,text=line[:1000]))
manifest=json.loads((RECORD/'manifest.json').read_text())
bag_source=next(item for item in manifest['files'] if item['path']=='keyframes.bag')
report=dict(scope='Read-only GEN3 recorded detector/status branch-trigger audit; no ROS operations',
    record_id=RECORD.name,selection_generation=3,bag_source=bag_source,
    counts=dict(counts),state_details=[dict(state=k[0],detail=k[1],count=v) for k,v in details.items()],
    transitions=transitions,first_generic_recovery=first_recovery,last_generic_recovery=last_recovery,
    core_read_snapshot=dict(path=str(NEW/'core.log'),bytes=len(core),sha256=hashlib.sha256(core).hexdigest(),matches=core_matches),
    interpretation=dict(
        gen19_support_gap='No typed support-gap entry or recovery receipt occurred in recorded generation 3; no processing_deadline_pending detector state. Not exercised, not a live fault-recovery pass.',
        gen18_registration_reacquire='No STATION_REGISTRATION_REACQUIRE trigger or YAW_BOUND_EXCEEDED evidence found in the recorded status messages or inspected core log. No evidence of the failed-batch cancellation followed by three newer frames. Not exercised.',
        generic_recovery='Observer increments recovery_frames for any non-null station_compute_recovery, including normal station parent-contract continuation. These counts do not demonstrate either fault-specific branch.',
        normal_arrival='observation_measurement_pending followed by observation_measurement_requalified is ordinary arrival measurement, not failed current-registration reacquisition.'),
    source_checks=dict(
        registration_trigger='src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py:62-128 (requires current YAW_BOUND_EXCEEDED; logs STATION_REGISTRATION_REACQUIRE)',
        support_trigger='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py:630-681 (typed gap receipt and processing_deadline_pending)',
        normal_parent_receipt='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py:566-613',
        observer_counter='.runtime/watch_station_compute_20261009.py:108'))
destination=OUT/'recovery_trigger_audit.json'
destination.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(destination),sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),counts=dict(counts)),ensure_ascii=False))
