"""Audit only sealed GEN8 bag evidence; never initialize ROS or command hardware."""
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


def audit_measurements(rows, selection):
    from alicia_grasp_modes.observe_return_measurement import (
        parse_measurement_request, validate_measurement_ack, PARTICIPANTS)
    requests = [row for row in rows if row['topic'].endswith('_request')]
    acks = [row for row in rows if row['topic'].endswith('_ack')]
    leases = {}
    previous = {}
    validations = []
    request_ids = {row['payload']['request_id'] for row in requests}
    for row in requests:
        request, now = row['payload'], row['receipt_ns']
        lease = request['lease_id']
        entry = leases.setdefault(lease, dict(lease_id=lease,
            view_index=request['view_index'], view_plan_id=request['view_plan_id'],
            executing_dual_ack=False, requalifying_dual_qualified_ack=False,
            qualified_dual_ack=False, matching_three_new_frames=False,
            qualified_request=False, requalification_complete=False))
        validation = dict(request_id=request['request_id'], state=request['state'],
            view_index=request['view_index'], request_valid=False, accepted_participants=[],
            qualified_participants=[], errors=[], source_stamps_ns=None)
        try:
            parse_measurement_request(request, selection, now, previous.get(lease))
            validation['request_valid'] = True
        except (ValueError, TypeError, KeyError) as exc:
            validation['errors'].append(str(exc))
        previous[lease] = request
        accepted, qualified = {}, {}
        for ack_row in acks:
            ack = ack_row['payload']
            if ack.get('request_id') != request['request_id']:
                continue
            try:
                participant = ack.get('participant')
                validate_measurement_ack(ack, request, participant, ack_row['receipt_ns'])
                accepted[participant] = ack_row
                if request['state'] == 'requalifying' and ack.get('qualified') is True:
                    validate_measurement_ack(ack, request, participant,
                        ack_row['receipt_ns'], require_qualified=True)
                    qualified[participant] = ack_row
            except (ValueError, TypeError, KeyError) as exc:
                validation['errors'].append(str(exc))
        validation['accepted_participants'] = sorted(accepted)
        validation['qualified_participants'] = sorted(qualified)
        both = validation['request_valid'] and set(accepted) == set(PARTICIPANTS)
        if request['state'] == 'executing':
            entry['executing_dual_ack'] = both
        elif request['state'] == 'requalifying':
            all_qualified = both and set(qualified) == set(PARTICIPANTS)
            sources = [qualified[p]['payload']['source_stamps_ns'] for p in PARTICIPANTS] if all_qualified else []
            entry['requalifying_dual_qualified_ack'] = all_qualified
            entry['matching_three_new_frames'] = all_qualified and sources[0] == sources[1]
            if entry['matching_three_new_frames']:
                entry['source_stamps_ns'] = sources[0]
                validation['source_stamps_ns'] = sources[0]
        elif request['state'] == 'qualified':
            entry['qualified_dual_ack'] = both
            entry['qualified_request'] = validation['request_valid']
            entry['qualified_request_frames_match'] = request.get('source_stamps_ns') == entry.get('source_stamps_ns')
        validations.append(validation)
    for entry in leases.values():
        entry['requalification_complete'] = all(entry.get(key) is True for key in (
            'executing_dual_ack', 'requalifying_dual_qualified_ack',
            'qualified_dual_ack', 'matching_three_new_frames',
            'qualified_request', 'qualified_request_frames_match'))
    return dict(request_count=len(requests), ack_count=len(acks),
        leases=list(leases.values()), request_validation=validations,
        unbound_ack_count=sum(row['payload'].get('request_id') not in request_ids for row in acks),
        original_rows=rows,
        validation_scope='Original pure request/ACK validators at recorded receipt times; source stamps must match both owners and qualified request. This proves measurement handoff, not contact/grasp success.')


def audit_supplement(record, manifest, selection, sessions, tasks, logs):
    from alicia_grasp_modes.contact_search_yield import contact_search_yield_proof_valid
    files = {row['path']: row for row in manifest['files']}
    reports, attempts, contact_yields = [], [], []
    for path in sorted((record / 'keyframes_audits').glob('*.json')):
        rel = str(path.relative_to(record))
        assert rel in files and sha256(path) == files[rel]['sha256']
        data = json.loads(path.read_text())
        if data.get('grasp_mode_selection') != selection:
            continue
        metadata = data.get('observe_return') or {}
        entry = dict(path=str(path), sha256=sha256(path), request_id=data.get('request_id'),
            internal_generation=data.get('generation'), selected_generation=selection['generation'],
            planning_origin=data.get('planning_origin'), outcome=data.get('outcome'),
            phase=metadata.get('phase'), view_index=metadata.get('view_index'))
        reports.append(entry)
        state = data.get('contact_search_yield_attempt')
        if state is not None:
            contact_yields.append(dict(entry, state=state, moveit_selection=data.get('moveit_selection')))
        if data.get('planning_origin') == 'observe_return_supplemental_observation':
            failure = metadata.get('failed_search')
            attempts.append(dict(entry, view_request_id=metadata.get('view_request_id'),
                previous_view_index=metadata.get('previous_view_index'),
                failed_search=failure, valid_contact_search_yield_proof=contact_search_yield_proof_valid(failure),
                diagnostic_observation_failure=data.get('diagnostic_observation_failure'),
                selected_plan_present=data.get('selected') is not None,
                valid_preview=(data.get('outcome') or {}).get('preview_valid') is True,
                observation_failure_diagnostics_location=rel+'#/observation_failure_diagnostics'))
    second_sessions = [row for row in sessions if row['payload'].get('view_index') == 2]
    admit_tasks = [row for row in tasks if row['message'] == 'admitting supplemental observation view']
    execution_tasks = [row for row in tasks if 'supplemental observation 2' in row['message']]
    completion_logs = [row for row in logs if 'Supplemental observation completed:' in row['text']]
    return dict(gate_audits=reports, contact_yield_attempts=contact_yields,
        second_view_planning_attempts=attempts, second_view_session_rows=second_sessions,
        admission_task_rows=admit_tasks, second_view_execution_task_rows=execution_tasks,
        completion_logs=completion_logs, relevant_logs=logs, task_transitions=tasks,
        classifications=dict(second_view_planning_attempted=bool(attempts),
            second_view_valid_preview=any(row['valid_preview'] for row in attempts),
            second_view_session_published=bool(second_sessions),
            second_view_admission_attempted=bool(admit_tasks),
            second_view_execution_state_observed=bool(execution_tasks),
            second_view_completion_log_observed=bool(completion_logs),
            physical_acceptance_claimed=False),
        interpretation='Planning attempt is distinct from a valid preview, admission and execution. Admission task text alone does not prove gateway admission. Second-view executing dual ACK/requalification are recorded separately by view_index in measurement_handoff.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    base = mount / '实验记录/Robot-Controller'
    record = base / 'grasp_records/20261012T112501.434384Z_ec184365e63145858bb80bea45a03b94'
    output = args.output.resolve()
    assert mount.is_mount() and base.resolve() in output.parents
    assert not output.exists(), 'Preserve existing audits'
    manifest = json.loads((record / 'manifest.json').read_text())
    outcome = json.loads((record / 'recorder_outcome.json').read_text())
    authorization = json.loads((record / 'authorization.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    assert outcome['bag_closed'] is True
    selection = authorization['selection']
    assert selection['generation'] == 8
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
    measurement = []
    task_rows = []
    session_rows = []
    supplemental_logs = []
    topics = ['/perception/unknown/detector_status', '/grasp_6d/status', '/rosout_agg',
              '/grasp_mode/observe_return_measurement_request',
              '/grasp_mode/observe_return_measurement_ack',
              '/grasp_mode/observe_return_session', '/grasp/state']
    markers = ('station_support_gap', 'station_support_gap_recovery',
               'pending_current_support', 'STATION_REGISTRATION_REACQUIRE',
               'YAW_BOUND_EXCEEDED')
    with rosbag.Bag(str(bag_path), 'r') as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, stamp in bag.read_messages(topics=topics):
            if topic == '/grasp/state':
                if stamp.to_nsec() >= expected_stamp:
                    row = dict(receipt_ns=stamp.to_nsec(), source_ns=message.header.stamp.to_nsec(),
                        state=message.state, message=message.message, active=message.active, success=message.success)
                    if not task_rows or any(row[k] != task_rows[-1][k] for k in ('state','message','active','success')):
                        task_rows.append(row)
                    counts[topic] += 1
                continue
            text = message.msg if topic == '/rosout_agg' else message.data
            if topic.startswith('/grasp_mode/'):
                if stamp.to_nsec() < expected_stamp:
                    continue
                payload = json.loads(text)
                if payload.get('selection') is not None and payload['selection'] != selection:
                    continue
                row = dict(topic=topic, receipt_ns=stamp.to_nsec(), payload=payload)
                if topic.endswith('_session'):
                    session_rows.append(row)
                else:
                    measurement.append(row)
                counts[topic] += 1
                continue
            if topic == '/perception/unknown/detector_status':
                data = json.loads(text)
                if data.get('generation') != 8:
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
            if topic == '/rosout_agg' and any(marker in text for marker in (
                    'Supplemental observation', 'supplemental observation', 'OBSERVE_RETURN_VIEW_ADMITTED')):
                supplemental_logs.append(dict(receipt_ns=stamp.to_nsec(), node=message.name, text=text))
            if 'STATION_REGISTRATION_REACQUIRE' in text or 'YAW_BOUND_EXCEEDED' in text:
                registration_events.append(dict(topic=topic, receipt_ns=stamp.to_nsec(),
                    node=getattr(message, 'name', None), text=text))

    gap_count = sum('station_support_gap' in row['typed'] for row in support_events)
    recovered_count = sum('station_support_gap_recovery' in row['typed'] for row in support_events)
    trigger_count = sum('STATION_REGISTRATION_REACQUIRE' in row['text']
                        for row in registration_events)
    report = dict(
        scope='Offline sealed GEN8 bag audit; no ROS node, live services, commands, or temperature telemetry',
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
        measurement_handoff=audit_measurements(measurement, selection),
        supplemental=audit_supplement(record, manifest, selection, session_rows, task_rows, supplemental_logs),
        motion_commands_sent=False, temperature_queried=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(output), sha256=sha256(output), counts=dict(counts),
        classifications=report['classifications']), ensure_ascii=False))


if __name__ == '__main__':
    main()
