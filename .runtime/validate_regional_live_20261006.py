#!/usr/bin/env python3
"""Observe a NEW 30 s window after a camera-only load, without any writes to ROS.

An earlier failed load receipt is immutable. Its SDK read observer belongs to
an earlier, separate window and is only referenced, never paired with current
publications. No new SDK read rate, target identity, geometry, or grasp success
is inferred. A latched strict evidence error cannot be cleared by later samples.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'
SECONDS = 30.


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def json_safe(value):
    """Preserve invalid numeric evidence as explicit strings in a failed run."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def validate_publications(publications, rows):
    stamps = {kind: {r['source_stamp_ns'] for r in rows[kind]} for kind in ('rgb', 'depth')}
    paired = (len(stamps['rgb'] & stamps['depth']) >= 2
        and len(stamps['rgb']-stamps['depth']) <= 2 and len(stamps['depth']-stamps['rgb']) <= 2)
    return bool(paired and all(
        p.get('fps') is not None and 18 <= p['fps'] <= 22
        and p.get('frames', 0) >= 2 and p.get('header_duplicates') == 0
        and p.get('header_regressions') == 0 and p.get('unique_source_stamps') == p.get('frames')
        for p in (publications['rgb'], publications['depth'])))


def validate_regional_profile(profile, expected_pixels, owner, region_names):
    require(profile.get('enabled') is True and profile.get('regional_enabled') is True
        and profile.get('active') is False and profile.get('whole_frame_stationarity_qualified') is False
        and profile.get('alignment_proven') is True and profile.get('owner_schema') == owner
        and profile.get('alpha') == .4 and profile.get('delta_depth_units') == 100
        and profile.get('persistence') == 0 and profile.get('visual_stationarity_required') is True
        and profile.get('visual_window_sec') == .35
        and profile.get('visual_max_motion_full_resolution_px') == .25,
        'polled regional policy/alignment mismatch')
    regions = profile.get('regions', [])
    if not regions:
        require(profile.get('filtered_pixel_count') == 0 and profile.get('regional_output_active') is False,
                'empty regional evidence claims filtered output')
        return False
    require(len(regions) == 5 and {r.get('name') for r in regions} == region_names
        and all(type(r.get('owned_pixels')) is int and r['owned_pixels'] >= 0 for r in regions)
        and sum(r['owned_pixels'] for r in regions) == expected_pixels, 'regional partition mismatch')
    count = sum(r['owned_pixels'] for r in regions if r.get('qualified') is True)
    require(count == profile.get('filtered_pixel_count')
        and profile.get('regional_output_active') is bool(count)
        and all(r.get('filter_applied') is (r.get('qualified') is True) for r in regions),
        'regional eligibility mismatch')
    return bool(count)


def changed_fields(previous, current):
    if previous is None:
        return []
    fields = []
    for key in ('frame_id', 'names', 'positions', 'velocity', 'effort'):
        if previous.get(key) != current.get(key):
            fields.append(key)
    if current.get('stamp_ns', 0) <= previous.get('stamp_ns', 0):
        fields.append('nonmonotonic_stamp')
    return fields


def joint_summary(rows):
    if not rows:
        return dict(samples=0)
    names = rows[0]['names']
    compatible = [r for r in rows if r['names'] == names and len(r['positions']) == len(names)]
    finite = [r for r in compatible if all(math.isfinite(v) for v in r['positions'])]
    return dict(samples=len(rows), compatible_samples=len(compatible), finite_samples=len(finite),
        names=names, first=rows[0], last=rows[-1],
        per_joint_range=[max(r['positions'][i] for r in finite)-min(r['positions'][i] for r in finite)
                         for i in range(len(names))] if finite else None)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--load-result', type=Path, required=True)
    args = parser.parse_args(argv)
    require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
    original = args.load_result.resolve()
    require(EXT.resolve() in original.parents, 'load receipt must be on SanStand')
    receipt_hash = sha(original)
    receipt = json.loads(original.read_text())
    require(receipt.get('camera_loaded') is True, 'receipt has no loaded camera owner')
    loader = load('regional_live_policy', '.runtime/load_regional_production_20261006.py')
    h = load('regional_live_helper', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    ab = load('regional_live_feedback', '.runtime/camera_preset_ab_observer.py')
    manifest_path = Path(receipt['manifest_path'])
    require(sha(manifest_path) == receipt['manifest_sha256'], 'load manifest changed')
    manifest = json.loads(manifest_path.read_text())
    loader.verify_manifest(manifest)
    for path, expected in receipt['source_sha256'].items():
        require(sha(path) == expected, 'loaded source/baseline changed: '+path)
    observer_paths = [original.parent/name for name in
        ('read_diagnostics.jsonl', 'read_diagnostics.summary.json', 'loaded_observer_modules.json')]
    observer_references = {str(p): sha(p) for p in observer_paths if p.exists()}
    summary_path = original.parent/'read_diagnostics.summary.json'
    summary = json.loads(summary_path.read_text())
    require(summary.get('original_read_restored') is True, 'earlier SDK observer still active')
    loaded = h.validate_loaded_modules(original.parent/'loaded_observer_modules.json', receipt['source_sha256'])
    loader.verify_regional_import(loaded, receipt['source_sha256'])
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    out = EXT/'ros_log'/('post_load_validation_'+stamp)
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header, String
    rospy.init_node('regional_live_readonly_validation', anonymous=True, disable_signals=True)
    lock = threading.RLock()
    records = dict(accepted=[], sdk=[], epoch=[], task=[], mode=[], rgb=[], depth=[], runtime=[], changes=[], errors=[])
    state = dict(frozen=False, mode=None, mode_received=None, callbacks_closed=False)
    report = dict(schema=1, out=str(out), load_result=str(original), load_result_sha256=receipt_hash,
        original_load_passed=receipt.get('static_validation_passed'),
        original_load_error=receipt.get('error'), validation_passed=False,
        requested_window_sec=SECONDS, robot_commands_sent=False, camera_reloaded=False,
        parameters_written=False, selection_changed=False, torque_commands_sent=False,
        temperature_queries_sent=False, validator_sha256=sha(__file__),
        prior_sdk_observer_references=observer_references,
        prior_sdk_observer_scope='Separate earlier window, never paired with these new publication/feedback rows',
        sdk_read_rate_measured_in_this_window=False,
        scope='New strict stationary-feedback and ROS publication window, polled regional runtime snapshots; no target geometry or grasp proof')
    subs, evidence, camera = [], None, {}

    def error(stage, exc):
        with lock:
            text = str(exc)
            if not any(r['stage'] == stage and r['error'] == text for r in records['errors']):
                records['errors'].append(dict(stage=stage, error=text, observed_monotonic=time.monotonic()))

    class RecordingEvidence(ab.RosEvidence):
        def joints(self, kind, message):
            row = dict(received_monotonic=time.monotonic(), stamp_ns=message.header.stamp.to_nsec(),
                frame_id=message.header.frame_id, names=list(message.name), positions=list(message.position),
                velocity=list(message.velocity), effort=list(message.effort))
            with lock:
                if not state['callbacks_closed']:
                    fields = changed_fields(records[kind][-1] if records[kind] else None, row)
                    if fields:
                        records['changes'].append(dict(kind=kind, fields=fields, previous=deepcopy(records[kind][-1]), current=deepcopy(row)))
                    records[kind].append(row)
            super().joints(kind, message)

        def task(self, message):
            with lock:
                if not state['callbacks_closed']:
                    records['task'].append(dict(received_monotonic=time.monotonic(), stamp_ns=message.header.stamp.to_nsec(),
                        **{k: getattr(message, k) for k in ('state', 'stage', 'active', 'success', 'message')}))
            super().task(message)

    def epoch_callback(message):
        with lock:
            if not state['callbacks_closed']:
                records['epoch'].append(dict(received_monotonic=time.monotonic(), stamp_ns=message.stamp.to_nsec(), frame_id=message.frame_id))

    def mode_callback(message):
        try:
            data = json.loads(message.data)
            require(data.get('active') is False and data.get('start_reserved') is False, 'mode active/reserved')
            with lock:
                if not state['callbacks_closed']:
                    state.update(mode=data, mode_received=time.monotonic())
                    records['mode'].append(dict(received_monotonic=state['mode_received'], data=data))
        except Exception as exc:
            error('mode', exc)

    def image_callback(kind, message):
        with lock:
            if not state['callbacks_closed']:
                records[kind].append(dict(received_monotonic=time.monotonic(), received_ros_time_ns=rospy.Time.now().to_nsec(),
                    source_stamp_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id,
                    width=message.width, height=message.height, encoding=message.encoding))

    def owners(check_master):
        for name, owner in dict(receipt['core_preserved'], **{loader.CAMERA: receipt['new_camera']}).items():
            require(h.alive(owner), 'recorded process owner exited: '+name)
            if check_master:
                require(h.node_pid(rospy, name) == owner['pid'], 'ROS process owner changed: '+name)

    def static_source_check():
        require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
        require(sha(original) == receipt_hash, 'original failed receipt changed')
        require(sha(manifest_path) == receipt['manifest_sha256'], 'load manifest changed')
        loader.verify_manifest(manifest)
        for path, expected in receipt['source_sha256'].items():
            require(sha(path) == expected, 'source changed: '+path)

    def poll_runtime(check_master):
        evidence.check_context()
        require(evidence.initial_context['selection'] == receipt['current_selection'], 'selection differs from load receipt')
        owners(check_master)
        camera = rospy.get_param('/camera')
        require(h.stable_camera_configuration(camera) == h.stable_camera_configuration(receipt['desired_camera_configuration']),
                'camera configuration differs from loaded policy')
        require(rospy.get_param('/rgb_acquisition/ledger', {}) == {}, 'observation ledger became active/retained')
        require(loader.baseline_identity(camera) == receipt['baseline_identity'], 'baseline identity changed')
        preset = camera.get('runtime_profile', {}).get('mode_depth_preset', {})
        unknown = camera['mode_depth_preset'].get('unknown_preset', 'high_accuracy')
        require(preset.get('mode') == 'unknown' and preset.get('preset') == unknown
            and preset.get('configured_unknown_preset') == unknown, 'actual SDK preset mismatch')
        profile = camera.get('runtime_profile', {}).get('stationary_temporal', {})
        active = validate_regional_profile(profile, camera['stream_width']*camera['stream_height'], loader.OWNER, loader.REGIONS)
        with lock:
            records['runtime'].append(dict(observed_monotonic=time.monotonic(), regional_active=active,
                policy=camera['mode_depth_preset'], preset=preset, stationary_temporal=profile,
                scope='Parameter snapshot, not bound to any recorded RGB-D source stamp'))

    def strict_guard():
        ready = evidence.guard.check(evidence.now_ns(), freeze=True)
        if ready:
            state['frozen'] = True
            snapshot = evidence.guard.snapshot()
            require(snapshot['epoch'] == receipt['initial_feedback']['epoch'], 'epoch differs from load receipt')
        require(ready, 'fresh complete strict feedback unavailable')
        with lock:
            require(state['mode_received'] is not None and time.monotonic()-state['mode_received'] < 2., 'mode status stale')

    def interrupted(signum, _):
        raise RuntimeError('readonly validator interrupted by signal %d' % signum)

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGALRM):
        signal.signal(signum, interrupted)
    signal.alarm(45)
    try:
        evidence = RecordingEvidence(rospy)
        report['initial_context'] = deepcopy(evidence.initial_context)
        camera = rospy.get_param('/camera')
        owners(True)
        subs.append(rospy.Subscriber('/alicia_d/control_reference_epoch', Header, epoch_callback, queue_size=10))
        subs.append(rospy.Subscriber('/grasp_mode/status', String, mode_callback, queue_size=20))
        for kind, key in (('rgb', 'color_topic'), ('depth', 'depth_topic')):
            subs.append(rospy.Subscriber(camera[key], Image, lambda message, k=kind: image_callback(k, message), queue_size=10, buff_size=2**24))
        ready_deadline = time.monotonic()+3.
        while time.monotonic() < ready_deadline:
            try:
                strict_guard()
                break
            except Exception as exc:
                if evidence.guard.faults:
                    error('initial_strict_guard', exc)
                    break
            time.sleep(.05)
        report['window_started_monotonic'] = time.monotonic()
        report['window_start_feedback'] = evidence.guard.snapshot()
        deadline = report['window_started_monotonic']+SECONDS
        next_runtime, next_owners = 0., 0.
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                strict_guard()
            except Exception as exc:
                error('strict_guard', exc)
            now = time.monotonic()
            if now >= next_runtime:
                try:
                    poll_runtime(now >= next_owners)
                except Exception as exc:
                    error('runtime_source', exc)
                if now >= next_owners:
                    next_owners = now+2.
                next_runtime = now+.5
            time.sleep(.05)
        report['window_ended_monotonic'] = time.monotonic()
        strict_guard()
        poll_runtime(True)
        static_source_check()
    except BaseException as exc:
        error('final_or_setup', exc)
    finally:
        signal.alarm(0)
        with lock:
            state['callbacks_closed'] = True
        for sub in subs:
            sub.unregister()
        if evidence is not None:
            report['final_feedback'] = evidence.guard.snapshot()
            report['strict_guard_faults'] = list(evidence.guard.faults)
            evidence.close()
        start = report.get('window_started_monotonic', math.inf)
        end = report.get('window_ended_monotonic', -math.inf)
        rows = {kind: [r for r in records[kind] if start <= r['received_monotonic'] <= end] for kind in ('rgb', 'depth')}
        publications = {kind: h.summarize_publication(data) for kind, data in rows.items()}
        for kind, encoding in (('rgb', 'bgr8'), ('depth', '16UC1')):
            if any(r['frame_id'] != camera.get('frame_id') or r['width'] != camera.get('width') or r['height'] != camera.get('height')
                   or r['encoding'] != encoding for r in rows[kind]):
                error('publication_geometry', kind+' geometry/encoding changed')
        report.update(publications=publications, joints={kind: joint_summary(records[kind]) for kind in ('accepted', 'sdk')},
            frozen_new_window=state['frozen'], error_count=len(records['errors']),
            errors=records['errors'], changes_recorded=len(records['changes']),
            runtime_snapshots=len(records['runtime']), regional_active_snapshots=sum(r['regional_active'] for r in records['runtime']),
            observed_window_sec=end-start if math.isfinite(start) and math.isfinite(end) else 0.,
            publication_passed=validate_publications(publications, rows))
        report['validation_passed'] = bool(report['publication_passed'] and state['frozen']
            and report['observed_window_sec'] >= SECONDS and not records['errors']
            and not report.get('strict_guard_faults') and report['regional_active_snapshots'] > 0)
        require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
        for kind, data in records.items():
            (out/(kind+'.jsonl')).write_text(''.join(json.dumps(json_safe(r), ensure_ascii=False, allow_nan=False)+'\n' for r in data))
        report['record_files'] = {kind: dict(path=str(out/(kind+'.jsonl')), sha256=sha(out/(kind+'.jsonl')), rows=len(data))
                                  for kind, data in records.items()}
        (out/'result.json').write_text(json.dumps(json_safe(report), ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(out=str(out), validation_passed=report['validation_passed'], errors=report['errors']), ensure_ascii=False), flush=True)
    return 0 if report['validation_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
