#!/usr/bin/env python3
"""Offline, indexed extraction of the fixed gen11 initial-admission window.

No ROS node, subscriptions, services, parameters, or hardware calls. This is a
workspace diagnostic, not an experiment recorder. It reads only the listed bag
topics in [2026-10-04 10:32:40Z, 10:32:47Z). It does not hash/scan the whole bag.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import genpy
import rosbag


ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT / 'grasp_records/20261004T103233.014422Z_71fe627d78cd4eb5b6669dcdf08d6fbd'
GATEWAY = ROOT / 'ros_log/rgb_acquisition_gateway_20261004T103246.365644Z_76191fbd/result.json'
START_NS, END_NS = 1791109960000000000, 1791109967000000000
ARM_NAMES = ['Joint%d' % n for n in range(1, 7)]
TOPICS = (
    '/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
    '/alicia_d/control_reference', '/alicia_d/control_reference_epoch',
    '/joint_commands', '/alicia_controller/command',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_d/actuation_status',
)


def time_value(ns):
    return genpy.Time(ns // 1000000000, ns % 1000000000)


def canonical(message):
    values = dict(zip(message.name, message.position))
    q = [float(values[n]) for n in ARM_NAMES]
    if not all(math.isfinite(x) for x in q):
        raise ValueError('nonfinite joint evidence')
    return q


def trajectory(message):
    return dict(header_stamp_ns=message.header.stamp.to_nsec(),
                joint_names=list(message.joint_names),
                points=[dict(time_from_start_ns=p.time_from_start.to_nsec(),
                             positions=list(p.positions), velocities=list(p.velocities),
                             accelerations=list(p.accelerations)) for p in message.points])


def extract():
    manifest_path = RECORD / 'manifest.json'
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest.get('state') != 'complete' or manifest.get('recorder_closed') is not True:
        raise ValueError('bag recorder is not confirmed closed')
    entry = next(item for item in manifest['files'] if item['path'] == 'keyframes.bag')
    bag_path = RECORD / entry['path']
    before = bag_path.stat()
    if before.st_size != entry['size_bytes']:
        raise ValueError('bag size differs from closed manifest')
    rows = {topic: [] for topic in TOPICS}
    with rosbag.Bag(str(bag_path), 'r') as bag:
        topic_info = bag.get_type_and_topic_info().topics
        recorded = {t: dict(type=topic_info[t].msg_type,
                            total_recorded_messages=topic_info[t].message_count)
                    for t in TOPICS if t in topic_info}
        for topic, message, stamp in bag.read_messages(
                topics=list(TOPICS), start_time=time_value(START_NS), end_time=time_value(END_NS)):
            arrival = stamp.to_nsec()
            if not START_NS <= arrival < END_NS:
                continue
            row = dict(bag_stamp_ns=arrival)
            if topic == '/alicia_d/actuation_status':
                row['status'] = message.data
            elif topic == '/alicia_d/control_reference_epoch':
                row.update(source_stamp_ns=message.stamp.to_nsec(), frame_id=message.frame_id)
            elif topic == '/alicia_controller/command':
                row['trajectory'] = trajectory(message)
            elif topic.endswith('/goal'):
                row.update(goal_id=message.goal_id.id, trajectory=trajectory(message.goal.trajectory))
            else:
                row.update(source_stamp_ns=message.header.stamp.to_nsec(),
                           frame_id=message.header.frame_id, q=canonical(message),
                           joint_names=list(message.name),
                           other_positions={name:float(value) for name,value in
                                            zip(message.name,message.position)
                                            if name not in ARM_NAMES})
            rows[topic].append(row)
    after = bag_path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('bag changed during read')
    artifact_bytes = GATEWAY.read_bytes()
    artifact = json.loads(artifact_bytes)
    initial = artifact['frames']['initial']
    baseline = initial['interest']['q_source']
    accepted = rows['/alicia_d/accepted_joint_states']
    if not accepted or any(row['frame_id'] != 'sdk_measured' for row in accepted):
        raise ValueError('accepted SDK measurement evidence missing or wrong frame')
    deviations = [max(abs(q-b) for q,b in zip(row['q'], baseline)) for row in accepted]
    first_change = next((dict(row, delta_from_rgb_source_rad=[q-b for q,b in zip(row['q'], baseline)])
                         for row,delta in zip(accepted,deviations) if delta > 1e-9), None)
    first_over_gate = next((dict(row, max_delta_rad=delta) for row,delta in
                           zip(accepted,deviations) if delta > .004), None)
    distinct_accepted = []
    for row in accepted:
        if not distinct_accepted or row['q'] != distinct_accepted[-1]['q']:
            distinct_accepted.append(row)
    sdk = rows['/alicia_d/sdk_command']
    references = rows['/alicia_d/control_reference']
    final_delta = [q-b for q,b in zip(accepted[-1]['q'],baseline)]
    changes = []
    for row in rows['/alicia_d/actuation_status']:
        if not changes or row['status'] != changes[-1]['status']:
            changes.append(row)
    def words(q):
        return [min(4095,max(0,int(math.floor((x+math.pi)/(2*math.pi)*4096+.5)))) for x in q]
    return dict(schema=1, created_at_utc=datetime.now(timezone.utc).isoformat(),
        kind='offline_fixed_window_diagnostic_not_experiment_recording',
        no_ros_node_or_control_calls=True, no_temperature_topics_read=True,
        window=dict(start_ns=START_NS, end_ns_exclusive=END_NS,
                    start_utc='2026-10-04T10:32:40Z', end_utc='2026-10-04T10:32:47Z',
                    selection_basis='bag receipt timestamp; source timestamps preserved'),
        provenance=dict(bag_path=str(bag_path), bag_bytes=before.st_size,
            bag_manifest_sha256=entry['sha256'], whole_bag_hash_recomputed=False,
            whole_bag_hash_note='Existing closed-manifest claim only; no whole-bag scan performed.',
            closed_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
            gateway_artifact=str(GATEWAY), gateway_sha256=hashlib.sha256(artifact_bytes).hexdigest(),
            script_sha256=hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()),
        selected_topics=recorded, window_counts={k:len(v) for k,v in rows.items()},
        summary=dict(generation=11, selection_stamp_ns=1791109957954493999,
            source_stamp_ns=initial['source_stamp_ns'], baseline_q=baseline,
            baseline_sdk_words=words(baseline),
            initial_feedback_bracket_ns=initial['accepted_after'][0]-initial['accepted_before'][0],
            actuation_transitions=changes, first_changed_accepted=first_change,
            first_accepted_over_original_004_gate=first_over_gate,
            maximum_accepted_delta_rad=max(deviations), accepted_value_transitions=distinct_accepted,
            final_delta_rad=final_delta,
            final_delta_in_encoder_counts=[round(x/(2.*math.pi/4096.),6) for x in final_delta],
            first_controller_command=(rows['/alicia_controller/command'] or [None])[0],
            first_joint_command=(rows['/joint_commands'] or [None])[0],
            first_sdk_write=(sdk or [None])[0],
            first_control_reference=(references or [None])[0],
            first_sdk_to_changed_accepted_source_ns=(
                first_change['source_stamp_ns']-sdk[0]['source_stamp_ns']
                if sdk and first_change else None),
            sdk_all_same_wire_as_rgb_source=bool(sdk) and all(words(r['q'])==words(baseline) for r in sdk),
            sdk_unique_wire_targets=sorted({tuple(words(r['q'])) for r in sdk}),
            control_reference_modes=sorted({r['frame_id'] for r in references}),
            sdk_writes_after_last_observed_change=sum(
                r['source_stamp_ns']>distinct_accepted[-1]['source_stamp_ns'] for r in sdk),
            trajectory_goal_messages_in_window=len(rows['/alicia_controller/follow_joint_trajectory/goal']),
            gateway_failure=artifact['outcome']['failure'],
            gateway_captured_stages=list(artifact['frames']), gateway_path_audit_present=bool(artifact['audit']),
            interpretation=[
                'This window concerns gen11 first admission after the earlier positive-enable epoch.',
                'Identical commanded SDK words did not imply zero encoder response during this first admission.',
                'The original 0.004 rad pre-submission rejection remains supported by actual accepted feedback.',
                'A fixed offset after this admission is an observed response, not proof of its physical mechanism.',
                'The one recorded controller command specifies a 20 ms hold endpoint; this extraction '
                'does not reconstruct or certify the controller interpolation or swept volume.',
                'This window does not include the later 10:54:08 user alignment or prove its handoff state.',
                'A later sdk_handoff_required handoff preserves its separately frozen successful SDK target; '
                'do not rebase that target from current feedback or infer repeated same-word writes always move.',
            ]), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().with_suffix('.json'))
    args = parser.parse_args()
    report = extract()
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output.resolve()), counts=report['window_counts'],
                         summary=report['summary']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
