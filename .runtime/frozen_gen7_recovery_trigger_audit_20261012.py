"""Audit only sealed GEN7 bag evidence; never initialize ROS or command hardware."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import rosbag


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    base = mount / '实验记录/Robot-Controller'
    record = base / 'grasp_records/20261012T104800.873840Z_462e869d618f484285a6ca2fcd4f3303'
    output = args.output.resolve()
    assert mount.is_mount() and base.resolve() in output.parents
    assert not output.exists(), 'Preserve existing audits'
    manifest = json.loads((record / 'manifest.json').read_text())
    outcome = json.loads((record / 'recorder_outcome.json').read_text())
    authorization = json.loads((record / 'authorization.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    assert outcome['bag_closed'] is True
    selection = authorization['selection']
    assert selection['generation'] == 7
    expected_stamp = int(selection['stamp_ns'])
    bag_path = record / 'keyframes.bag'
    source = next(row for row in manifest['files'] if row['path'] == 'keyframes.bag')
    bag_digest = sha256(bag_path)
    assert bag_digest == source['sha256']
    counts = Counter()
    details = Counter()
    transitions = []
    support_events = []
    registration_events = []
    marker_counts = Counter()
    generic_first = None
    generic_last = None
    previous = None
    topics = ['/perception/unknown/detector_status', '/grasp_6d/status', '/rosout_agg']
    markers = ('station_support_gap', 'station_support_gap_recovery',
               'pending_current_support', 'STATION_REGISTRATION_REACQUIRE',
               'YAW_BOUND_EXCEEDED')
    with rosbag.Bag(str(bag_path), 'r') as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, stamp in bag.read_messages(topics=topics):
            text = message.msg if topic == '/rosout_agg' else message.data
            if topic == '/perception/unknown/detector_status':
                data = json.loads(text)
                if data.get('generation') != 7:
                    continue
                if data.get('selection_stamp_ns') is not None:
                    assert int(data['selection_stamp_ns']) == expected_stamp
            elif stamp.to_nsec() < expected_stamp:
                continue
            counts[topic] += 1
            for marker in markers:
                marker_counts[topic + ':' + marker] += int(marker in text)
            if topic == '/perception/unknown/detector_status':
                key = (data.get('state'), data.get('detail'))
                details[key] += 1
                row = dict(receipt_ns=stamp.to_nsec(), source_ns=data.get('source_stamp_ns'),
                           state=key[0], detail=key[1])
                if key != previous:
                    transitions.append(row)
                    previous = key
                if data.get('station_compute_recovery') is not None:
                    counts['generic_station_compute_recovery'] += 1
                    generic_row = dict(row, parent_contract=data['station_compute_recovery'])
                    generic_first = generic_first or generic_row
                    generic_last = generic_row
                typed = {name: data[name] for name in (
                    'station_support_gap', 'station_support_gap_recovery')
                    if data.get(name) is not None}
                if typed or 'pending_current_support' in text:
                    counts['typed_support_event_rows'] += 1
                    support_events.append(dict(row, typed=typed,
                        station_compute_wait=data.get('station_compute_wait'),
                        source_qualification=data.get('source_qualification')))
                counts['processing_deadline_pending'] += int(key[0] == 'processing_deadline_pending')
            if 'STATION_REGISTRATION_REACQUIRE' in text or 'YAW_BOUND_EXCEEDED' in text:
                registration_events.append(dict(topic=topic, receipt_ns=stamp.to_nsec(),
                    node=getattr(message, 'name', None), text=text))

    gap_count = sum('station_support_gap' in row['typed'] for row in support_events)
    recovered_count = sum('station_support_gap_recovery' in row['typed'] for row in support_events)
    trigger_count = sum('STATION_REGISTRATION_REACQUIRE' in row['text']
                        for row in registration_events)
    report = dict(
        scope='Offline sealed GEN7 bag audit; no ROS node, live services, commands, or temperature telemetry',
        audited_at=datetime.now(timezone.utc).isoformat(), record_id=record.name,
        selection=selection, bag_source=source, actual_bag_sha256=bag_digest,
        bag_time_range_sec=[bag_start, bag_end], counts=dict(counts), marker_counts=dict(marker_counts),
        state_details=[dict(state=k[0], detail=k[1], count=v) for k, v in details.items()],
        transitions=transitions, support_events=support_events,
        registration_events=registration_events, first_generic_recovery=generic_first,
        last_generic_recovery=generic_last,
        classifications=dict(
            gen19_support_gap=('not_triggered' if not support_events else 'trigger_observed_requires_chain_validation'),
            support_gap_rows=gap_count, support_gap_recovery_rows=recovered_count,
            gen18_registration_reacquire=('not_triggered' if not trigger_count
                else 'trigger_observed_requires_three_new_frames_validation'),
            registration_trigger_rows=trigger_count,
            physical_acceptance_claimed=False),
        interpretation=dict(
            support='Only typed station_support_gap / station_support_gap_recovery and pending_current_support evidence identify this repair. A triggered row alone does not prove successful three-owner recovery or unchanged deadline.',
            registration='STATION_REGISTRATION_REACQUIRE proves branch entry. YAW_BOUND_EXCEEDED alone does not prove reacquisition; three entirely newer frames in the original phase/session deadline must be checked if triggered.',
            generic_recovery='Non-null station_compute_recovery also describes normal station parent-contract continuation and does not prove either repaired fault branch.',
            normal_arrival='observation_measurement_pending and observation_measurement_requalified are ordinary arrival measurement, not failed-current-registration reacquisition.',
            limits='Not-triggered means absent from complete recorded detector/status and bag rosout evidence for this generation; it is not real-hardware acceptance of an unexercised fault recovery.'),
        source_checks=dict(
            registration='src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py:_observe_return_reacquire_failed_registration',
            support='src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py:_station_compute_publish_support_gap',
            watcher='.runtime/watch_station_compute_20261009.py: detector row excludes typed support-gap fields; final audit reads original bag'),
        motion_commands_sent=False, temperature_queried=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(output), sha256=sha256(output), counts=dict(counts),
        classifications=report['classifications']), ensure_ascii=False))


if __name__ == '__main__':
    main()
