#!/usr/bin/env python3
"""Read sealed gen27 initial station metadata only; no ROS calls or temperatures."""
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
REC = EXT/'grasp_records/20261009T121108.702839Z_3a4d30c05d58450cbe43f4b3802a4b1d'
OUT = EXT/'ros_log/full_latest_ros_20261009T003407Z/gen27_handoff_analysis'
manifest = json.loads((REC/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True, 'record not sealed'
topics = ['/perception/unknown/detector_status', '/alicia_d/accepted_joint_states',
          '/alicia_d/sdk_command', '/joint_commands', '/alicia_d/control_reference',
          '/alicia_d/control_reference_epoch', '/gui/joint_direct_mode',
          '/alicia_controller/command', '/alicia_controller/follow_joint_trajectory/goal',
          '/alicia_controller/follow_joint_trajectory/result',
          '/grasp/state', '/grasp_mode/observe_return_measurement_request']

def serialize(value):
    if hasattr(value, 'to_nsec'):
        return {'stamp_ns': value.to_nsec()}
    if hasattr(value, '__slots__'):
        return {key: serialize(getattr(value, key)) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [serialize(x) for x in value]
    return value

rows = {topic: [] for topic in topics}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        if topic.endswith('detector_status'):
            payload = json.loads(msg.data)
            if payload.get('generation') != 27:
                continue
        elif topic.endswith('measurement_request'):
            payload = json.loads(msg.data)
        else:
            payload = serialize(msg)
        rows[topic].append(dict(received_ns=stamp.to_nsec(), message=payload))

statuses = rows[topics[0]]
lost = next(row for row in statuses if row['message'].get('target_lost') is True)
loss = lost['message']['first_loss']
now = loss['observed_stamp_ns']
source = loss['source_stamp_ns']
anchor = lost['message']['anchor_source_stamp_ns']
initial = next((row for row in statuses if row['message'].get('initial_station_measurement')), None)
station = initial['message']['initial_station_measurement'] if initial else None
qstation = np.asarray(station['q_station_rad']) if station else None
before = [row for row in statuses if row['message'].get('source_stamp_ns', 0) <= source]

def jointrow(row):
    msg = row['message']
    return dict(received_ns=row['received_ns'], source_stamp_ns=msg['header']['stamp']['stamp_ns'],
                frame_id=msg['header']['frame_id'], names=msg['name'], positions=msg['position'])

accepted = [jointrow(row) for row in rows['/alicia_d/accepted_joint_states']
            if row['message']['header']['frame_id'] == 'sdk_measured']
qrows = [row for row in accepted if anchor <= row['source_stamp_ns'] <= now]
joint_names = ['Joint%d' % i for i in range(1, 7)]
def q6(row):
    values = dict(zip(row['names'], row['positions']))
    return np.asarray([values[name] for name in joint_names])

station_changes = []
if qstation is not None:
    for row in qrows:
        delta = q6(row)-qstation
        if float(np.max(np.abs(delta))) > .004:
            station_changes.append(dict(row, delta_from_station_rad=delta.tolist()))

def span(items):
    return np.ptp(np.asarray([q6(row) for row in items]), axis=0).tolist() if items else None

def compact(row):
    msg = row['message']
    return dict(received_ns=row['received_ns'], **{key: msg.get(key) for key in (
        'source_stamp_ns', 'state', 'detail', 'target_lost', 'anchor_source_stamp_ns',
        'initial_station_measurement', 'source_qualification', 'metric_ambiguity_hold',
        'rgb_identity_hold', 'support_measurement_hold', 'first_loss', 'frame_timing')})

command_summaries = {}
for topic in ['/alicia_d/sdk_command', '/joint_commands', '/alicia_d/control_reference']:
    selected = [jointrow(row) for row in rows[topic] if row['received_ns'] <= now]
    command_summaries[topic] = dict(count=len(selected), peak_to_peak_rad=span(selected),
        first=selected[0] if selected else None, last=selected[-1] if selected else None,
        unique_position_count=len({tuple(row['positions']) for row in selected}))

timing = lost['message'].get('frame_timing', {})
ref = loss.get('last_good_source_stamp_ns')
summary = dict(record=str(REC), generation=27,
    sealed_manifest={key: manifest[key] for key in ('state','result','recorder_closed','completed_at','failure_reason')},
    manifest_sha256=hashlib.sha256((REC/'manifest.json').read_bytes()).hexdigest(),
    authorization=json.loads((REC/'authorization.json').read_text()),
    alignment_receipt=json.loads((REC/'alignment_receipt.json').read_text()),
    first_loss=lost, initial_station_status=initial,
    initial_station_source_ns=station['station_source_stamp_ns'] if station else None,
    initial_station_q_rad=qstation.tolist() if station else None,
    source_age_at_first_loss_sec=(now-source)*1e-9,
    last_good_age_at_first_loss_sec=(now-ref)*1e-9 if ref else None,
    capture_gap_to_last_good_sec=(source-ref)*1e-9 if ref else None,
    accepted_station_window=dict(start_ns=anchor,end_ns=now,count=len(qrows),
        peak_to_peak_rad=span(qrows),first=qrows[0] if qrows else None,last=qrows[-1] if qrows else None,
        samples_exceeding_station_004_rad=len(station_changes),
        first_exceeding_station_004_rad=station_changes[0] if station_changes else None),
    command_summaries=command_summaries,
    control_epoch=rows['/alicia_d/control_reference_epoch'],
    gui_direct=rows['/gui/joint_direct_mode'],
    controller_command=rows['/alicia_controller/command'],
    action_goals=rows['/alicia_controller/follow_joint_trajectory/goal'],
    action_results=rows['/alicia_controller/follow_joint_trajectory/result'],
    measurement_requests=rows['/grasp_mode/observe_return_measurement_request'],
    task_states=rows['/grasp/state'],
    source_chronology=[compact(row) for row in before],
    status_state_counts=dict(Counter(row['message']['state'] for row in statuses)),
    limitations=['Read only whitelisted sealed metadata; no temperature diagnostics or ROS calls.',
        'accepted sdk_measured is recorded hardware feedback, not proof of why a motion occurred.',
        'Bag arrival order is not assumed equal to callback order inside perception.',
        'Missing motion topics plus runner stage delimit task submission; low-level holds are retained separately.'],
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
OUT.mkdir(exist_ok=True)
(OUT/'initial_station_analysis.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
(OUT/'whitelisted_metadata.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(dict(output=str(OUT/'initial_station_analysis.json'),first_loss=loss,
    failed_stage=timing.get('failed_stage'),initial_station=station,
    q_window=summary['accepted_station_window'],command_summaries=command_summaries,
    gui=summary['gui_direct'],epoch=summary['control_epoch'],
    motion_topic_counts={key:len(summary[key]) for key in ('controller_command','action_goals','measurement_requests')},
    source_age=summary['source_age_at_first_loss_sec'],old_age=summary['last_good_age_at_first_loss_sec'],
    source_qualification=lost['message'].get('source_qualification'),
    frame_timing=timing), ensure_ascii=False, indent=2))
