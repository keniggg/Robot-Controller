#!/usr/bin/env python3
"""Read-only, whitelisted closed-bag state audit; never starts ROS or reads images."""
import argparse
import collections
import hashlib
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import time


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def identity(path):
    s = path.stat()
    return dict(path=str(path), device=s.st_dev, inode=s.st_ino,
                size=s.st_size, mtime_ns=s.st_mtime_ns)


def plain(value):
    if isinstance(value, (str, float, int, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if hasattr(value, 'secs') and hasattr(value, 'nsecs'):
        return dict(secs=value.secs, nsecs=value.nsecs,
                    stamp_ns=value.secs * 1000000000 + value.nsecs)
    if hasattr(value, '__slots__'):
        return {k: plain(getattr(value, k)) for k in value.__slots__}
    raise TypeError(type(value))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    a.record = a.record.resolve(strict=True)
    a.output = a.output.resolve()
    storage = Path('/media/zhuyupei/SanStand')
    assert storage.is_mount(), 'external storage must be mounted'
    assert storage in a.output.parents, 'output must be on SanStand'
    assert not a.output.exists(), 'new output directory required'
    a.output.mkdir(parents=True)
    scratch = a.output / 'genpy_tmp'
    scratch.mkdir()
    tempfile.tempdir = str(scratch)
    denied = []

    def resolved(path, dir_fd=None):
        if isinstance(path, int):
            return Path(os.readlink('/proc/self/fd/' + str(path))).resolve()
        path = Path(os.fsdecode(path))
        if not path.is_absolute() and isinstance(dir_fd, int) and dir_fd >= 0:
            path = Path(os.readlink('/proc/self/fd/' + str(dir_fd))) / path
        return path.resolve()

    def audit(event, args):
        reason = None
        if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.posix_spawn'):
            reason = 'network/process forbidden'
        if event == 'open':
            path, mode, flags = args
            write = (isinstance(mode, str) and any(v in mode for v in 'wax+')) or (isinstance(flags, int) and flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND))
            if write and a.output not in resolved(path).parents:
                reason = 'write outside new output forbidden'
        if event in ('os.remove', 'os.rename', 'os.rmdir', 'os.chmod', 'os.chown', 'os.truncate'):
            if event == 'os.rename':
                paths = [resolved(args[0], args[2]), resolved(args[1], args[3])]
            elif event in ('os.remove', 'os.rmdir'):
                paths = [resolved(args[0], args[1])]
            elif event == 'os.chmod':
                paths = [resolved(args[0], args[2])]
            elif event == 'os.chown':
                paths = [resolved(args[0], args[3])]
            else:
                paths = [resolved(args[0])]
            if any(a.output not in p.parents for p in paths):
                reason = 'mutation outside output forbidden'
        if reason:
            denied.append(dict(event=event, reason=reason))
            raise PermissionError(reason)

    sys.addaudithook(audit)
    import rosbag

    def save(name, data):
        p = a.output / name
        with p.open('x') as f:
            json.dump(data, f, ensure_ascii=False, indent=2, sort_keys=True)
            f.write('\n')
        return dict(path=str(p), sha256=sha(p), bytes=p.stat().st_size)

    def lines(name, data):
        p = a.output / name
        with p.open('x') as f:
            for row in data:
                f.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + '\n')
        return dict(path=str(p), sha256=sha(p), bytes=p.stat().st_size, count=len(data))

    bag_path = a.record / 'keyframes.bag'
    before = identity(bag_path)
    outcome = json.loads((a.record / 'recorder_outcome.json').read_text())
    assert outcome.get('bag_closed') is True, 'record must be closed'
    source_files = {p.name: dict(identity(p), sha256=sha(p)) for p in [bag_path, a.record / 'recorder_outcome.json', a.record / 'result_summary.json', a.record / 'perception_execution_configuration.json']}
    base_topics = ['/grasp/state', '/grasp/task_event', '/grasp/near_field_phase',
                   '/grasp_6d/status', '/grasp_6d/gate_audit', '/grasp_6d/preview_plan_enriched',
                   '/grasp_mode/observe_return_measurement_ack',
                   '/grasp_mode/observe_return_measurement_request',
                   '/grasp_mode/observe_return_session', '/grasp_mode/observe_return_ack',
                   '/experiment/runtime_execution_error_snapshot']
    detector_topic = '/perception/unknown/detector_status'
    object_topics = ['/perception/object', '/grasp_6d/object_geometry']
    whitelist = base_topics + [detector_topic] + object_topics
    rows, detectors, objects, counts = [], [], [], collections.Counter()
    started = time.monotonic()
    with rosbag.Bag(str(bag_path), 'r') as bag:
        metadata = {k: dict(type=v.msg_type, count=v.message_count) for k, v in bag.get_type_and_topic_info().topics.items()}
        for topic, msg, stamp, connection in bag.read_messages(topics=whitelist, return_connection_header=True):
            counts[topic] += 1
            wire = io.BytesIO()
            msg.serialize(wire)
            row = dict(topic=topic, bag_stamp_ns=stamp.to_nsec(), callerid=connection.get('callerid'),
                       message_type=msg._type, message_md5=msg._md5sum,
                       wire_sha256=hashlib.sha256(wire.getvalue()).hexdigest())
            if msg._type == 'std_msgs/String':
                row['raw_string'] = msg.data
                try:
                    row['data'] = json.loads(msg.data)
                except ValueError:
                    row['data'] = msg.data
            else:
                row['data'] = plain(msg)
            (detectors if topic == detector_topic else objects if topic in object_topics else rows).append(row)
    assert before == identity(bag_path), 'source bag changed during audit'
    requests = [r for r in rows if r['topic'].endswith('measurement_request')]
    req = next(r for r in requests if r['data']['state'] == 'requalifying')
    qualified = next(r for r in requests if r['data']['state'] == 'qualified')
    q = qualified['data']
    arrival = req['data']['arrival_stamp_ns']
    sources = q['source_stamps_ns']
    acks = [r for r in rows if r['topic'].endswith('measurement_ack')]
    dual = [r for r in acks if r['data'].get('state') == 'requalifying' and r['data'].get('qualified') and r['data'].get('accepted')]
    states = [r for r in rows if r['topic'] == '/grasp/state']
    first_active = next(r for r in states if r['data']['active'])
    inactive = next(r for r in states if r['bag_stamp_ns'] > first_active['bag_stamp_ns'] and not r['data']['active'])
    state_stamp = lambda r: r['data']['header']['stamp']['stamp_ns']
    terminal_ns = state_stamp(inactive)
    confirmations = [r for r in detectors if r['data'].get('source_stamp_ns') in sources and r['data'].get('detail') in ('observation_measurement_pending', 'observation_measurement_requalified')]
    tracked = [r for r in detectors if r['data'].get('source_stamp_ns', 0) > sources[-1] and r['data'].get('detail') == 'tracked']
    station_tracked = [r for r in tracked if r['data'].get('frame_timing', {}).get('observation_appearance_identity')]

    def detector_small(r):
        d = r['data']
        m = d.get('observation_measurement') or {}
        f = d.get('frame_timing') or {}
        result = {k: r[k] for k in ('topic', 'bag_stamp_ns', 'callerid', 'wire_sha256')}
        result.update({k: d.get(k) for k in ('state', 'detail', 'source_stamp_ns', 'color_stamp_ns', 'depth_stamp_ns', 'exact_rgbd', 'generation', 'selection_stamp_ns', 'target_locked', 'target_lost')})
        result['observation_measurement'] = {k: m.get(k) for k in ('state', 'reason', 'qualified', 'source_stamps_ns', 'appearance_identity', 'exact_tf_consumed', 'measurement_evaluated')}
        result['station_appearance_identity'] = f.get('observation_appearance_identity')
        result['processing_wall_sec'] = f.get('processing_wall_sec')
        metrics = {}

        def collect(v, path=''):
            if isinstance(v, dict):
                for k, val in v.items():
                    here = path + '.' + k if path else k
                    if k.startswith(('rgb_missing_depth_', 'rgb_mask_pixels_without_', 'trusted_core_')) or k in ('rgb_seed_pixels', 'rgb_instance_pixels', 'valid_points', 'depth_seed_retained_fraction'):
                        metrics[here] = val
                    if k != 'reference_binding':
                        collect(val, here)
            elif isinstance(v, list):
                for n, val in enumerate(v):
                    collect(val, path + f'[{n}]')
        collect(d)
        result['mask_metrics'] = metrics
        return result

    small = [detector_small(r) for r in detectors]
    errors = [r for r in rows if r['topic'] == '/experiment/runtime_execution_error_snapshot']
    phase = next(r for r in rows if r['topic'] == '/grasp/near_field_phase' and r['data']['active'])
    gates = [r for r in rows if r['topic'] == '/grasp_6d/gate_audit']
    failed_gate = next(r for r in gates if r['data'].get('summary', {}).get('status') == 'NEAR_FIELD_SURFACE_VIEW_REQUIRED')
    gate_path = a.record / 'keyframes_audits' / (failed_gate['data']['report_sha256'] + '.json')
    assert sha(gate_path) == failed_gate['data']['report_sha256'], 'gate artifact hash mismatch'
    request_digests = {r['data']['request_sha256'] for r in dual}
    conditions = dict(
        exactly_three_unique_sources=len(sources) == len(set(sources)) == 3,
        three_original_appearance_passes=len(confirmations) == 3 and all(r['data']['observation_measurement']['appearance_identity'][0]['appearance_match'] for r in confirmations),
        same_original_frozen_reference=len({r['data']['observation_measurement']['appearance_identity'][0]['reference_mask_sha256'] for r in confirmations}) == 1 and all(not r['data']['observation_measurement']['appearance_identity'][0]['reference_refreshed'] for r in confirmations),
        distinct_unknown_remote_qualified_acks={r['data']['participant'] for r in dual} == {'unknown', 'remote'} and len(dual) == 2,
        acks_same_request_digest=len(request_digests) == 1,
        ack_digest_matches_exact_request=request_digests == {hashlib.sha256(json.dumps(req['data'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()},
        acks_same_source_stamps=all(r['data']['source_stamps_ns'] == sources for r in dual),
        acks_bound_to_requalifying_request=all(r['data']['request_id'] == req['data']['request_id'] and r['data']['lease_id'] == req['data']['lease_id'] for r in dual),
        all_sources_after_arrival_and_before_deadline=all(arrival < s <= req['data']['capture_deadline_ns'] for s in sources),
        task_qualified_before_original_deadline=q['stamp_ns'] <= req['data']['capture_deadline_ns'],
        ack_bag_receipts_precede_qualified_request=all(r['bag_stamp_ns'] < qualified['bag_stamp_ns'] for r in dual),
        same_epoch_selection_lease=all(q[k] == req['data'][k] for k in ('control_epoch_ns', 'selection', 'lease_id', 'session_id', 'target_epoch', 'target_track_id')),
        phase_bound_to_third_qualified_source=phase['data']['reference_source_stamp']['stamp_ns'] == sources[-1],
        fourth_station_frame_exists=bool(tracked),
        full_task_still_failed=inactive['data']['state'] == 'FAILED' and not inactive['data']['success'],
        no_frozen_contact_reference=not phase['data']['reference_contact_plan']['valid'],
    )
    selected = confirmations + tracked[:4]
    selected_sources = {r['data']['source_stamp_ns'] for r in selected}
    selected_objects = [r for r in objects if r['data']['header']['stamp']['stamp_ns'] in selected_sources]
    exported = {}
    exported['state_events'] = lines('state_events.jsonl', rows)
    exported['detector_timeline'] = lines('detector_timeline.jsonl', small)
    exported['handoff_detector_exact'] = lines('handoff_detector_exact.jsonl', selected)
    exported['handoff_object_geometry'] = lines('handoff_object_geometry.jsonl', selected_objects)
    exported['execution_error_snapshots'] = save('runtime_execution_error_snapshots.json', dict(
        origin='Recorder snapshots of /grasp_6d/runtime_execution_error parameter; bag-only topic, no live parameter read by this audit.',
        timestamp_policy='bag_stamp_ns exact; stamp_sec is original JSON float, not reconstructed exact nanoseconds',
        rows=errors))
    timeline = dict(task_active_start_header_ns=state_stamp(first_active), task_active_start_bag_ns=first_active['bag_stamp_ns'],
                    task_active_end_header_ns=terminal_ns, task_active_end_bag_ns=inactive['bag_stamp_ns'],
                    task_active_duration_sec=(terminal_ns-state_stamp(first_active))/1e9,
                    arrival_ns=arrival, capture_deadline_ns=req['data']['capture_deadline_ns'],
                    qualified_request_header_ns=q['stamp_ns'], qualified_request_bag_ns=qualified['bag_stamp_ns'],
                    arrival_to_qualified_sec=(q['stamp_ns']-arrival)/1e9,
                    qualified_ack_bag_ns={r['data']['participant']:r['bag_stamp_ns'] for r in dual},
                    near_field_phase_start_header_ns=phase['data']['header']['stamp']['stamp_ns'],
                    near_field_phase_start_bag_ns=phase['bag_stamp_ns'],
                    terminal_gate_bag_ns=failed_gate['bag_stamp_ns'])
    exported['timeline'] = save('timeline.json', timeline)
    result = dict(schema=1, scope='real closed recording; state/status-only read; no image or temperature message deserialization; no live ROS or commands',
                  record=str(a.record), source_files=source_files,
                  bag_identity_unchanged=before == identity(bag_path), whitelisted_topics=whitelist,
                  read_topic_counts=dict(counts), metadata_frozen_contact_topics={k:v for k,v in metadata.items() if 'frozen_contact' in k},
                  chronology=timeline, invariants=conditions, handoff_passed=all(v for k,v in conditions.items() if k not in ('full_task_still_failed','no_frozen_contact_reference')),
                  original_appearance_confirmations=[detector_small(r) for r in confirmations],
                  qualified_source_stamps_ns=sources, qualified_request=q,
                  first_station_tracking=detector_small(tracked[0]) if tracked else None,
                  first_station_appearance_diagnostic=detector_small(station_tracked[0]) if station_tracked else None,
                  dual_qualified_acks=dual, near_field_phase=phase,
                  downstream_failure=dict(gate_message=failed_gate, gate_artifact=dict(path=str(gate_path),sha256=sha(gate_path)), terminal_state=inactive,
                      interpretation='Observation handoff succeeded. Current fused near-field snapshot had no locally hard-safe candidate. Later TARGET_LOST retirement is after terminal failure, not the initial mask refusal.'),
                  execution_error_snapshot_count=len(errors), nonempty_execution_error_snapshot_count=sum(bool(r['data']) for r in errors),
                  exported=exported, blocked_audit_events=denied, elapsed_sec=time.monotonic()-started,
                  audit_script=dict(path=str(Path(__file__).resolve()),sha256=sha(Path(__file__))))
    receipt = save('result.json', result)
    print(json.dumps(dict(result=receipt, timeline=exported['timeline'], handoff_passed=result['handoff_passed'], conditions=conditions), ensure_ascii=False))


if __name__ == '__main__':
    main()
