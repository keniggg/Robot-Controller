"""Read sealed GEN11 joint-only records and non-temperature core log lines."""
from pathlib import Path
import hashlib
import json
import re

import numpy as np
import rosbag
import rospy

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT/'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT/'grasp_records/20261010T045823.108983Z_afb71d5ad1424dad9de6275837bc5f78'
OUT = NEW/'gen11_handoff_analysis'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((REC/'manifest.json').read_text())['recorder_closed'] is True
LOW, HIGH = 1791608310., 1791608322.
names = ['Joint%d'%i for i in range(1, 7)]
topics = ['/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
    '/alicia_d/control_reference', '/alicia_d/control_reference_epoch',
    '/alicia_d/actuation_status', '/joint_commands', '/alicia_controller/state',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result', '/grasp/state']
rows = {topic: [] for topic in topics}
with rosbag.Bag(str(REC/'keyframes.bag'), 'r') as bag:
    for topic, msg, received in bag.read_messages(topics=topics,
            start_time=rospy.Time.from_sec(LOW), end_time=rospy.Time.from_sec(HIGH)):
        row = dict(received_ns=received.to_nsec())
        if topic == '/alicia_d/actuation_status':
            if re.search('temp|overheat', msg.data, re.I):
                continue
            row['status'] = msg.data
        elif topic.endswith('/goal'):
            tr = msg.goal.trajectory
            row.update(goal_id=msg.goal_id.id, stamp_ns=tr.header.stamp.to_nsec(),
                names=list(tr.joint_names), points=[dict(time_from_start_ns=p.time_from_start.to_nsec(),
                    positions=list(p.positions), velocities=list(p.velocities),
                    accelerations=list(p.accelerations)) for p in tr.points],
                path_tolerance=[dict(name=t.name, position=t.position) for t in msg.goal.path_tolerance],
                goal_tolerance=[dict(name=t.name, position=t.position) for t in msg.goal.goal_tolerance])
        elif topic.endswith('/result'):
            row.update(goal_id=msg.status.goal_id.id, status=int(msg.status.status),
                status_text=msg.status.text, error_code=int(msg.result.error_code),
                error_string=msg.result.error_string)
        elif topic == '/alicia_d/control_reference_epoch':
            row.update(stamp_ns=msg.stamp.to_nsec(), frame_id=msg.frame_id)
        elif topic == '/alicia_controller/state':
            row.update(stamp_ns=msg.header.stamp.to_nsec(), names=list(msg.joint_names),
                desired=list(msg.desired.positions), actual=list(msg.actual.positions))
        elif topic == '/grasp/state':
            row.update(stamp_ns=msg.header.stamp.to_nsec(), active=msg.active)
        else:
            values = dict(zip(msg.name, msg.position))
            row.update(stamp_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id,
                q=[values[name] for name in names])
        rows[topic].append(row)

logs = []
for number, raw in enumerate((NEW/'core.log').open(errors='replace'), 1):
    # Exclude these lines before retaining or inspecting their payload.
    if re.search('temp|overheat|温度', raw, re.I):
        continue
    clean = re.sub(r'\x1b\[[0-9;]*m', '', raw).rstrip()
    for match in re.finditer(r'\[(17916083\d\d\.\d+)\]', clean):
        stamp = float(match.group(1))
        if LOW <= stamp <= HIGH and re.search(
                'Actuation state|SDK feedback|SDK command|joint_commands|encoder|'
                'actuation|PREFIX|probe|Submitting proof|Controller reference continuous|'
                'Control ownership|Rejected|reject|INITIAL_PROBE', clean, re.I):
            # A shared terminal may interleave two logger records on one line.
            logs.append(dict(line=number, stamp_sec=stamp, text=clean,
                line_sha256=hashlib.sha256(raw.encode()).hexdigest()))
            break

accepted = rows['/alicia_d/accepted_joint_states']
sdk = rows['/alicia_d/sdk_command']
q0 = np.asarray(accepted[0]['q'])
def summary(samples):
    values = np.asarray([row['q'] for row in samples])
    stamps = np.asarray([row['stamp_ns'] for row in samples], dtype=np.int64)
    return dict(count=len(samples), first=samples[0], last=samples[-1],
        max_abs_delta_from_baseline_rad=np.max(np.abs(values-q0), axis=0).tolist(),
        range_rad=np.ptp(values, axis=0).tolist(),
        maximum_source_gap_sec=float(np.max(np.diff(stamps))*1e-9) if len(stamps)>1 else None)
transitions=[]
for row in rows['/alicia_d/actuation_status']:
    if not transitions or row['status'] != transitions[-1]['status']:
        transitions.append(row)
probe = next(row for row in transitions if row['status']=='PENDING:AWAITING_ENCODER_RESPONSE')
timeout = next(row for row in transitions if row['status']=='UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT')
probe_samples=[row for row in accepted if probe['received_ns'] < row['received_ns'] <= timeout['received_ns']]
report=dict(record=str(REC), bag_topics_read=topics, range_sec=[LOW,HIGH],
    baseline_q6=q0.tolist(),accepted=summary(accepted),sdk_transmitted=summary(sdk),
    probe_status_transitions=transitions,
    probe_receipt_interval_sec=(timeout['received_ns']-probe['received_ns'])*1e-9,
    accepted_samples_during_probe=summary(probe_samples),
    follow_joint_trajectory_results=rows['/alicia_controller/follow_joint_trajectory/result'],
    trajectory_goals=rows['/alicia_controller/follow_joint_trajectory/goal'],
    non_temperature_core_events=logs,
    feedback_rejections_in_selected_window=[row for row in logs if re.search('reject.*feedback|feedback.*reject',row['text'],re.I)],
    source_policy=dict(command_probe_min_delta_rad=.02,measured_response_min_delta_rad=.003,
        response_timeout_sec=1.0,source='driver launch/source configuration; no runtime parameter or temperature query'),
    conclusions=[
        'SDK successful-write telemetry changes while accepted six-joint positions remain fixed in the examined window.',
        'Fresh accepted feedback arrives during the original one-second response window; no selected-window feedback rejection is logged.',
        'The recorded failure is absence of a qualifying measured direction response, not a missing host command or a demonstrated timing/threshold false rejection.',
        'Trajectory action completion under existing tracking tolerances is distinct from measured actuation confirmation; no grasp success is asserted.'],
    limitations=['Successful serial write cannot prove that firmware applied the command.',
        'Recorded SDK feedback establishes no measured motion; internal device or physical cause is not determined.',
        'Accepted joint topic is recorded at its recorder sampling rate, not every raw serial packet.',
        'No temperature values, environment variables, live ROS services, motion, disable command, or driver changes.'])
assert np.max(np.abs(np.asarray([row['q'] for row in accepted])-q0))==0.
OUT.mkdir(exist_ok=True)
def write(name, obj):
    payload=(json.dumps(obj,ensure_ascii=False,indent=2)+'\n').encode();p=OUT/name
    with p.open('xb') as stream:stream.write(payload)
    return dict(path=str(p),sha256=hashlib.sha256(payload).hexdigest())
trace=write('actuation_prefix_joint_only_trace.json',rows)
report['trace']=trace
result=write('actuation_prefix_analysis.json',report)
print(json.dumps(dict(report=result,accepted=report['accepted'],sdk=report['sdk_transmitted'],
    probe=report['accepted_samples_during_probe'],results=report['follow_joint_trajectory_results']),ensure_ascii=False))
