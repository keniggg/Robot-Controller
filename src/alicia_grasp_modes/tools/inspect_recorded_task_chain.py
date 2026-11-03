#!/usr/bin/env python3
"""Index a closed local recording without ROS initialization or device access.

Reads an explicit non-temperature topic allowlist. A manifest hash is checked
before and after reading. Output must be a new directory on mounted SanStand.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from alicia_grasp_modes.recorded_task_chain import build_task_chain, normalize_event
import alicia_grasp_modes.recorded_task_chain as chain_module


JSON_TOPICS = (
    '/grasp/task_event',
    '/grasp_mode/initial_preflight_request', '/grasp_mode/initial_preflight_ack',
    '/grasp_mode/initial_preflight_attempt', '/grasp_mode/observe_return_session',
    '/grasp_mode/observe_return_ack', '/grasp_mode/observe_return_measurement_request',
    '/grasp_mode/observe_return_measurement_ack', '/grasp_mode/contact_station_retirement_request',
    '/grasp_mode/contact_station_retirement_ack', '/perception/unknown/detector_status',
    '/grasp_6d/frozen_contact_plan', '/grasp_6d/gate_audit')
TOPICS = JSON_TOPICS + (
    '/grasp/state', '/grasp/near_field_phase', '/grasp_6d/plan_enriched',
    '/grasp_6d/preview_plan_enriched', '/grasp_6d/object_geometry',
    '/perception/object', '/perception/object_mask', '/alicia_d/accepted_joint_states',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result',
    '/hand_controller/follow_joint_trajectory/goal',
    '/hand_controller/follow_joint_trajectory/result')
MOUNT = Path('/media/zhuyupei/SanStand')
STORAGE = MOUNT / '实验记录/Robot-Controller'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_text())


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2,
                               allow_nan=False) + '\n')
    assert read(path) == value


def contained(root, child):
    try:
        child.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def manifest_inventory(record):
    manifest_path = record/'manifest.json'
    manifest = read(manifest_path)
    if manifest.get('state') != 'complete' or manifest.get('recorder_closed') is not True:
        raise ValueError('only closed complete records can be indexed')
    inventory = {str(manifest_path): sha(manifest_path)}
    for item in manifest['files']:
        path = record/item['path']
        if not contained(record, path) or path.is_symlink():
            raise ValueError('manifest path leaves record or is a symlink')
        if path.stat().st_size != item['size_bytes'] or sha(path) != item['sha256']:
            raise ValueError('manifest content changed: '+str(path))
        inventory[str(path)] = item['sha256']
    for name in ('authorization.json', 'runner.log', 'keyframes.bag'):
        if str(record/name) not in inventory:
            raise ValueError('required source is not manifest-bound: '+name)
    return manifest, inventory


def wire_fields(topic, message):
    if topic in JSON_TOPICS:
        payload = json.loads(message.data)
        if not isinstance(payload, dict):
            raise ValueError('JSON topic is not an object')
        return payload
    if topic.endswith('/goal'):
        return dict(goal_id=message.goal_id.id, goal_ns=message.goal_id.stamp.to_nsec(),
                    point_count=len(message.goal.trajectory.points))
    if topic.endswith('/result'):
        return dict(goal_id=message.status.goal_id.id,
            goal_ns=message.status.goal_id.stamp.to_nsec(),
            header_stamp_ns=message.header.stamp.to_nsec(), status=message.status.status,
            error_code=message.result.error_code, error_string=message.result.error_string)
    payload = dict(header_stamp_ns=message.header.stamp.to_nsec(),
                   header_frame_id=message.header.frame_id)
    for name in ('state', 'stage', 'active', 'success', 'message', 'phase_id', 'planning_stage',
                 'plan_id', 'valid', 'diagnostic', 'target_track_id', 'reference_target_track_id',
                 'detected', 'confidence', 'label'):
        if hasattr(message, name):
            payload[name] = getattr(message, name)
    if hasattr(message, 'reference_source_stamp'):
        payload['reference_source_ns'] = message.reference_source_stamp.to_nsec()
    if topic.endswith('/object_mask'):
        payload.update(height=message.height, width=message.width, encoding=message.encoding,
                       data_sha256=hashlib.sha256(bytes(message.data)).hexdigest())
    return payload


def inspect(record, task_run_id=None):
    # rosbag is an offline file reader. No rospy import/init, parameter read,
    # service call, publisher or subscription is performed here.
    import rosbag
    analysis_sources = {str(Path(p).resolve()): sha(Path(p))
                        for p in (__file__, chain_module.__file__)}
    manifest, inventory = manifest_inventory(record)
    selection = read(record/'authorization.json')['selection']
    events, malformed, audit_refs, sdk = [], [], {}, []
    bagpath = record/'keyframes.bag'
    topic_indices = {}; last_recorded_ns = 0
    with rosbag.Bag(str(bagpath), 'r') as bag:
        recorded_topics = set(bag.get_type_and_topic_info().topics)
        for topic, message, stamp in bag.read_messages(topics=TOPICS):
            index = topic_indices.get(topic, 0); topic_indices[topic] = index+1
            last_recorded_ns = max(last_recorded_ns, stamp.to_nsec())
            locator = dict(path=str(bagpath), file_sha256=inventory[str(bagpath)],
                           topic=topic, topic_message_index=index, recorded_ns=stamp.to_nsec())
            if topic == '/alicia_d/accepted_joint_states':
                sdk.append(dict(source_ns=message.header.stamp.to_nsec(),
                                names=list(message.name), positions=list(message.position)))
                continue
            try:
                payload = wire_fields(topic, message)
                events.append(normalize_event(topic, payload, locator))
                if topic == '/grasp_6d/gate_audit':
                    audit_refs.setdefault(payload.get('report_sha256'), []).append(locator)
            except (TypeError, ValueError, AttributeError) as exc:
                malformed.append(dict(locator=locator, reason=str(exc)))
    for name, file_sha in inventory.items():
        path = Path(name)
        if path.parent != record/'keyframes_audits' or path.suffix != '.json':
            continue
        refs = audit_refs.get(file_sha, [])
        locator = dict(path=name, file_sha256=file_sha, json_pointer='/',
                       recorded_ns=min((r['recorded_ns'] for r in refs), default=0),
                       recorded_audit_publications=refs)
        events.append(normalize_event('audit:planning', read(path), locator))
    for line_number, line in enumerate((record/'runner.log').read_text().splitlines(), 1):
        match = re.fullmatch(r'GRASP_RESULT success=(True|False) message=(.*)', line)
        if match:
            # Runner log has no synchronized timestamp. Bind its own result to
            # the manifest-authorized selection but leave time unknown.
            payload = dict(selection=selection, success=match.group(1)=='True', message=match.group(2))
            events.append(normalize_event('runner:result', payload,
                dict(path=str(record/'runner.log'), file_sha256=inventory[str(record/'runner.log')], line=line_number)))
    result = build_task_chain(events, selection, attempt_end_ns=last_recorded_ns,
                             task_run_id=task_run_id)
    current_sdk = [s for s in sdk if s['source_ns'] >= selection['stamp_ns']]
    same_names = bool(current_sdk) and all(s['names']==current_sdk[0]['names'] for s in current_sdk)
    widths = {len(s['positions']) for s in current_sdk}
    span = ([max(s['positions'][i] for s in current_sdk)-min(s['positions'][i] for s in current_sdk)
             for i in range(next(iter(widths)))] if same_names and len(widths)==1 else None)
    result.update(record=str(record), record_result=manifest.get('result'),
        manifest_failure_reason=manifest.get('failure_reason'), malformed_messages=malformed,
        recorded_topic_counts=topic_indices, topic_allowlist=list(TOPICS),
        topic_presence={topic: topic in recorded_topics for topic in TOPICS},
        missing_topic_semantics='Absent topics are unconfirmed evidence, not proof of no physical action.',
        sdk_feedback_summary=dict(samples=len(current_sdk), joint_names=current_sdk[0]['names'] if same_names else None,
            peak_to_peak_positions=span, authority='measurement_only_not_grasp_success'),
        source_sha256=inventory, source_unchanged=all(sha(Path(p))==h for p,h in inventory.items()),
        analysis_source_sha256=analysis_sources,
        analysis_source_unchanged=all(sha(Path(p))==h for p,h in analysis_sources.items()),
        ros_initialized=False, device_io=False, temperature_topics_read=False)
    if not result['source_unchanged'] or not result['analysis_source_unchanged']:
        raise ValueError('record changed during inspection')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task-run-id', help='Exact task_event run ID when a recording contains multiple attempts')
    args = parser.parse_args()
    if not MOUNT.is_mount() or not contained(STORAGE, args.record) or not contained(STORAGE, args.output):
        raise ValueError('record and new output must be on mounted experiment storage')
    if args.output.exists():
        raise ValueError('output must not exist; previous evidence is immutable')
    result = inspect(args.record.resolve(), task_run_id=args.task_run_id)
    args.output.mkdir(parents=True)
    write(args.output/'result.json', result)
    write(args.output/'receipt.json', dict(result_sha256=sha(args.output/'result.json'),
        tool_sha256=sha(Path(__file__)), source_unchanged=result['source_unchanged'],
        first_interruption=result['first_interruption'], original_record_modified=False))
    print(json.dumps(dict(result=str(args.output/'result.json'),
        sha256=sha(args.output/'result.json'), first_interruption=result['first_interruption']), ensure_ascii=False))


if __name__ == '__main__':
    main()
