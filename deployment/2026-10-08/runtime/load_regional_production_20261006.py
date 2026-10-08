#!/usr/bin/env python3
"""Explicit camera-only regional-depth opt-in and bounded metadata acceptance.

This tool preserves an externally supplied target-alignment or explicitly
camera-only stationary-capture receipt as distinct provenance. It
does not create target identity, select a mode/target, authorize a grasp, or send
robot commands. On any failed condition it records failure and exits. The new
camera, if already launched, remains running for inspection; no action follows.
"""
import argparse
from collections import Counter, deque
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

W = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
EXT = MOUNT/'实验记录/Robot-Controller'
MANIFEST = W/'.runtime/regional_production_validation_20261006.json'
CAMERA = '/supervisor_camera'
CORE = ('/alicia_d_driver_node', '/bessica_d_hw_interface', '/move_group',
        '/robot_state_publisher', '/handeye_transform', '/motion_gateway',
        '/grasp_task_node', '/grasp_mode_router', '/unknown_tabletop_perception',
        '/remote_grasp6d_node')
REGIONS = {'center', 'top_left', 'top_right', 'bottom_left', 'bottom_right'}
OWNER = 'fixed_center_02_08_01_09_then_four_quadrants_v1'
PARAM = '/camera/mode_depth_preset/'
VISUAL_TAP = '.runtime/gen4_root_cause/visual_snapshot_tap.py'
NODE_TIMING_TAP = '.runtime/gen4_root_cause/node_timing_tap.py'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def completed_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return False


def verify_manifest(manifest, root=W, capture_visual_snapshots=False, capture_node_timings=False):
    require(type(capture_visual_snapshots) is bool, 'visual capture flag must be boolean')
    require(type(capture_node_timings) is bool, 'node timing flag must be boolean')
    require(not (capture_visual_snapshots and capture_node_timings), 'diagnostic taps must be isolated')
    require(manifest.get('tests_passed') is True and manifest.get('reviewed') is True,
            'reviewed passing source manifest required')
    files = manifest.get('files', {})
    prefix = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/'
    require(all(prefix+n in files for n in ('realsense_manager.py',
            'regional_stationary_depth.py', 'visual_stationary.py', 'stationary_depth.py',
            'pair_history_continuity.py')),
            'regional source manifest incomplete')
    if capture_visual_snapshots:
        require(all(n in files for n in (VISUAL_TAP,
            '.runtime/gen4_root_cause/camera_existing_read_observer.py',
            '.runtime/load_regional_production_20261006.py')),
            'visual snapshot source manifest incomplete')
    if capture_node_timings:
        require(all(n in files for n in (NODE_TIMING_TAP,
            '.runtime/gen4_root_cause/camera_existing_read_observer.py',
            '.runtime/load_regional_production_20261006.py',
            'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py')),
            'node timing source manifest incomplete')
    for relative, expected in files.items():
        path = (root/relative).resolve()
        require(root.resolve() in path.parents, 'source outside workspace')
        require(sha(path) == expected, 'frozen source changed: '+relative)


def read_receipt(path, external=EXT):
    path = Path(path).resolve()
    require(external.resolve() in path.parents, 'alignment receipt must be archived on SanStand')
    receipt = json.loads(path.read_text())
    event_path = Path(receipt['event_file']).resolve()
    require(external.resolve() in event_path.parents, 'alignment event must be archived on SanStand')
    require(sha(event_path) == receipt.get('event_file_sha256'), 'alignment event hash mismatch')
    event = json.loads(event_path.read_text())
    kind=receipt.get('kind')
    if kind=='explicit_user_stationary_capture':
        require(event.get('kind')==kind,'stationary capture event kind mismatch')
        for field,expected in (('scene_stationary',True),('camera_diagnostic_only',True),
                               ('motion_authorized',False)):
            require(receipt.get(field) is expected and event.get(field) is expected,
                    'stationary capture scope mismatch: '+field)
        require(all('target_aligned' not in item or item['target_aligned'] is False
                    for item in (receipt,event)),
                'stationary capture cannot assert target alignment')
        for field in ('confirmation_id','epoch_ns'):
            require(receipt.get(field)==event.get(field),
                    'stationary capture event/receipt mismatch: '+field)
        for value in (receipt.get('epoch_ns'),receipt.get('confirmed_ns'),
                      receipt.get('bound_feedback',{}).get('stamp_ns')):
            require(type(value) is int or type(value) is str and value.isascii() and value.isdigit(),
                    'stationary capture requires exact integer timestamps')
    else:
        require(kind == 'explicit_user_target_alignment'
                and receipt.get('target_aligned') is True
                and event.get('target_aligned') is True, 'explicit user alignment required')
    source = receipt.get('confirmation_source', {})
    require(source.get('kind') == 'user_message' and isinstance(source.get('text'), str)
            and bool(source['text'].strip()) and bool(source.get('message_id')),
            'actual user message provenance required')
    require(isinstance(receipt.get('confirmation_id'), str) and bool(receipt['confirmation_id']),
            'alignment confirmation id missing')
    if kind=='explicit_user_stationary_capture':
        require(type(source.get('message_id')) is str and bool(source['message_id'].strip())
                and bool(receipt['confirmation_id'].strip()),
                'stationary capture actual user message and confirmation ids required')
    for name in ('confirmation_source', 'confirmed_ns', 'bound_feedback'):
        require(receipt.get(name) == event.get(name), 'alignment event/receipt mismatch: '+name)
    feedback = receipt['bound_feedback']
    require(feedback.get('frame_id') == 'sdk_measured'
            and len(feedback.get('positions', [])) == 6
            and all(type(v) in (int, float) and math.isfinite(v) for v in feedback['positions']),
            'six real measured alignment joints required')
    require(0 < int(receipt['epoch_ns']) <= int(receipt['confirmed_ns'])
            <= int(feedback['stamp_ns']), 'alignment timestamp order invalid')
    return receipt


def receipt_report_fields(receipt,path,digest):
    """Keep camera-only scene consent out of every target-alignment field."""
    kind=receipt.get('kind')
    require(kind in ('explicit_user_target_alignment','explicit_user_stationary_capture'),
            'unsupported confirmation provenance')
    stationary=kind=='explicit_user_stationary_capture'
    prefix='stationary_capture_receipt' if stationary else 'alignment_receipt'
    return dict(confirmation_receipt=receipt,confirmation_receipt_path=str(path),
        confirmation_receipt_sha256=digest,confirmation_receipt_kind=kind,
        confirmation_provenance_scope=('camera_diagnostic_only_no_target_alignment_no_motion'
            if stationary else 'existing_explicit_target_alignment_camera_only_maintenance'),
        target_alignment_confirmation_used=not stationary,
        scene_stationary_confirmation_used=stationary,
        **{prefix:receipt,prefix+'_path':str(path),prefix+'_sha256':digest})


def desired_camera_configuration(before, unknown_preset):
    require(unknown_preset in ('preserve', 'saved_baseline'), 'unsupported unknown preset')
    require(before.get('fps') == 30 and before.get('publish_fps') == 20
            and before.get('simulate') is False and before.get('fallback_to_simulation') is False
            and before.get('align_depth_to_color') is True
            and before.get('pair_rectification', {}).get('enabled') is True,
            'existing real aligned/rectified 30/20 Hz camera policy required')
    policy = before.get('mode_depth_preset', {})
    require(policy.get('enabled') is True and policy.get('stationary_temporal_enabled') is True,
            'existing unknown stationary temporal policy required')
    require(policy.get('unknown_preset', 'high_accuracy') in ('high_accuracy', 'saved_baseline'),
            'unsupported existing unknown preset')
    require(type(policy.get('regional_stationary_enabled', False)) is bool,
            'regional flag must be boolean')
    after = deepcopy(before)
    after['mode_depth_preset']['regional_stationary_enabled'] = True
    if unknown_preset == 'saved_baseline':
        after['mode_depth_preset']['unknown_preset'] = 'saved_baseline'
    return after


def baseline_identity(camera):
    path = Path(camera['mode_depth_preset']['baseline_path']).expanduser().resolve()
    saved = json.loads(path.read_text())
    canonical = hashlib.sha256(json.dumps(saved['advanced_settings']).encode()).hexdigest()
    require(camera.get('runtime_profile', {}).get('mode_depth_preset', {}).get('baseline_sha256') == canonical,
            'current camera does not use the recorded baseline content')
    return dict(path=str(path), file_sha256=sha(path), advanced_settings_sha256=canonical)


def verify_regional_import(loaded, source_hashes):
    for module in ('regional_stationary_depth', 'pair_history_continuity'):
        path = (W/('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/'+module+'.py')).resolve()
        actual = loaded.get('alicia_flexible_grasp.vision.'+module, {})
        require(actual.get('path') == str(path) and actual.get('sha256') == source_hashes[str(path)],
                'actual regional module import does not match frozen source: '+module)


def verify_visual_snapshot_import(loaded, source_hashes):
    path=(W/VISUAL_TAP).resolve()
    actual=loaded.get('diagnostic.visual_snapshot_tap',{})
    require(actual.get('path')==str(path) and actual.get('sha256')==source_hashes.get(str(path)),
            'actual visual snapshot helper import does not match frozen source')


def snapshot_hooks_restored(summary):
    return bool(summary and summary.get('sealed') is True and all(summary.get(k) is True for k in
        ('original_process_restored','original_observe_restored','original_reset_restored')))


def verify_visual_capture(summary, directory):
    """Finite diagnostic acceptance only; no image is used for control."""
    directory=Path(directory)
    require(snapshot_hooks_restored(summary), 'visual snapshot hooks have not been restored')
    require(not summary.get('errors') and summary.get('counts',{}).get('errors',0)==0,
            'visual snapshot capture reported errors')
    require(summary.get('pixels_changed') is False and summary.get('motion_authorized') is False
            and summary.get('diagnostic_only') is True
            and summary.get('coordinate_domain')=='raw_sdk_color_before_pair_rectification',
            'visual snapshot scope mismatch')
    require(summary.get('queued_unwritten')==0 and summary.get('inflight_copies')==0
            and summary.get('queue_bytes')==0 and summary.get('retained_anchor_bytes')==0,
            'visual snapshot evidence not completely sealed')
    require(summary.get('maximum_bytes')==64*1024*1024
            and 0<=summary.get('reserved_bytes',-1)<=64*1024*1024-64*1024
            and 0<=summary.get('anchor_peak_bytes',-1)<=8*1024*1024,
            'visual snapshot budget invalid')
    require(summary.get('requested_window_sec')==30. and summary.get('close_reason')=='window_expired'
            and type(summary.get('first_read_monotonic')) in (int,float)
            and type(summary.get('finished_monotonic')) in (int,float)
            and math.isfinite(summary['first_read_monotonic']) and math.isfinite(summary['finished_monotonic'])
            and summary['finished_monotonic']-summary['first_read_monotonic']>=30.,
            'visual snapshot window incomplete')
    files=summary.get('files',[]);counts=Counter()
    require(isinstance(files,list) and 0<len(files)<=32,'visual snapshot file count invalid')
    names=set();total=0
    for f in files:
        name=f.get('name');category=f.get('category')
        require(type(name)is str and name.startswith('snapshot_') and name.endswith('.npz')
                and Path(name).name==name and name not in names,'invalid visual snapshot path')
        require(category in {r+'/'+s for r in ('center','nested_center') for s in ('visual_motion','stationary')},
                'invalid visual snapshot category')
        p=directory/name
        require(not p.is_symlink() and p.stat().st_size==f.get('bytes') and sha(p)==f.get('sha256'),
                'visual snapshot saved file mismatch')
        names.add(name);counts[category]+=1;total+=p.stat().st_size
    require(all(n<=8 for n in counts.values()) and dict(counts)==summary.get('categories'),
            'visual snapshot category bound mismatch')
    require(summary.get('counts',{}).get('written')==summary.get('counts',{}).get('enqueued')==len(files)
            and summary.get('directory_bytes_before_summary')==total,
            'visual snapshot count/bytes mismatch')
    require({p.name for p in directory.iterdir()}==names|{'summary.json'}
            and total+(directory/'summary.json').stat().st_size<=64*1024*1024,
            'visual snapshot directory budget mismatch')
    return summary


def timing_hooks_restored(summary):
    return (isinstance(summary, dict) and summary.get('sealed') is True
            and summary.get('class_hooks_restored') is True
            and summary.get('observed_bound_callbacks_restored') is True)


def verify_node_timing_capture(summary, directory, loaded, source_hashes, camera_entry=None):
    directory = Path(directory)
    require(timing_hooks_restored(summary), 'node timing hooks are still active')
    require(summary.get('schema') == 1 and summary.get('complete') is True,
            'node timing capture incomplete')
    require(summary.get('source_implementation') == loaded, 'node timing provenance differs from imports')
    source = W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    for key, target in (('diagnostic.node_timing_tap', W/NODE_TIMING_TAP),
                        ('diagnostic.camera_node_timing_entry', camera_entry or source),
                        ('diagnostic.camera_node_timing_source', source)):
        item = loaded.get(key, {})
        path = str(Path(target).resolve())
        require(item.get('path') == path and item.get('sha256') == source_hashes.get(path),
                'node timing import source mismatch')
    require(not summary.get('errors') and summary.get('queued_unwritten') == 0
            and summary.get('unfinished_calls') == [], 'node timing capture failed to drain')
    counts = summary.get('counts', {})
    require(all(counts.get(k) == 0 for k in ('queue_dropped', 'diagnostic_errors', 'budget_rejected')),
            'node timing events were lost')
    require(0 < summary.get('requested_window_sec', 0) <= 30, 'node timing window exceeds limit')
    require(summary.get('diagnostic_only') is True and summary.get('no_pixel_copies') is True
            and summary.get('window_complete') is True and summary.get('close_reason') == 'window_elapsed',
            'node timing window did not complete')
    first, closed = summary.get('first_read_start_ns'), summary.get('closed_monotonic_ns')
    require(type(first) is int and first > 0 and type(closed) is int
            and closed-first >= int(summary['requested_window_sec']*1e9),
            'node timing time boundary incomplete')
    require(type(counts.get('started')) is int and counts['started'] > 0
            and counts['started'] == counts.get('completed')
            and counts.get('written') == counts.get('enqueued') == 2*counts['started'],
            'node timing count mismatch')
    require(set(p.name for p in directory.iterdir()) == {'events.jsonl', 'summary.json'},
            'unexpected node timing artifacts')
    require(all(p.is_file() and not p.is_symlink() for p in directory.iterdir()),
            'node timing artifacts must be regular files')
    size = (directory/'events.jsonl').stat().st_size
    require(size == summary.get('event_bytes') and 0 < size
            and size+(directory/'summary.json').stat().st_size <= 4*1024*1024,
            'node timing byte budget mismatch')
    require(sha(directory/'events.jsonl') == summary.get('events_sha256'), 'node timing event hash mismatch')
    return summary


def camera_command(old_argv, helper):
    """Only known, finished diagnostic wrappers may be unwrapped."""
    argv = list(old_argv)
    known = {str((W/'.runtime'/n).resolve()) for n in
             ('regional_stationary_shadow.py', 'regional_conditioned_shadow.py')}
    if len(argv) > 1 and str(Path(argv[1]).resolve()) in known:
        require(len(argv) >= 8 and argv[2] == '--camera-script'
                and argv[4] == '--output' and argv[6] == '--', 'malformed old shadow argv')
        folder = Path(argv[5]).resolve()
        require(EXT.resolve() in folder.parents, 'old shadow output outside external storage')
        summary = completed_json(folder/'summary.json')
        require(summary and summary.get('original_read_restored') is True,
                'old shadow read hook has not been restored')
        argv = [argv[0], argv[3]]+argv[7:]
    if len(argv)>1 and Path(argv[1]).resolve()==helper.OBSERVER.resolve() and '--capture-visual-snapshots' in argv:
        require(argv.count('--')==1 and argv.count('--capture-visual-snapshots')==1
                and 2<=argv.index('--capture-visual-snapshots')<argv.index('--'),
                'malformed old visual snapshot option')
        diagnostic=Path(argv[argv.index('--diagnostic-path')+1]).resolve()
        require(EXT.resolve() in diagnostic.parents,'old visual snapshot output outside external storage')
        require(snapshot_hooks_restored(completed_json(diagnostic.parent/'visual_snapshots/summary.json')),
                'old visual snapshot hooks are still active')
        argv.remove('--capture-visual-snapshots')
    if len(argv)>1 and Path(argv[1]).resolve()==helper.OBSERVER.resolve() and '--capture-node-timings' in argv:
        require(argv.count('--')==1 and argv.count('--capture-node-timings')==1
                and 2<=argv.index('--capture-node-timings')<argv.index('--'),
                'malformed old node timing option')
        diagnostic=Path(argv[argv.index('--diagnostic-path')+1]).resolve()
        require(EXT.resolve() in diagnostic.parents, 'old node timing output outside external storage')
        require(timing_hooks_restored(completed_json(diagnostic.parent/'node_timings/summary.json')),
                'old node timing hooks are still active')
        argv.remove('--capture-node-timings')
    entry, tail = helper.camera_entry_and_ros_tail(argv)
    if Path(argv[1]).resolve() == helper.OBSERVER.resolve():
        diagnostic = Path(argv[argv.index('--diagnostic-path')+1])
        summary = completed_json(diagnostic.with_suffix('.summary.json'))
        require(summary and summary.get('original_read_restored') is True,
                'old metadata observer is still active')
    return [argv[0], entry]+tail


def regional_read_summary(rows, expected_pixels):
    errors, reasons, active = [], Counter(), 0
    complete = [r for r in rows if r.get('existing_read_diagnostics', {}).get('result') == 'complete']
    for index, row in enumerate(complete):
        diagnostic = row['existing_read_diagnostics']
        evidence = diagnostic.get('regional_stationary_evidence', {})
        profile = row.get('stationary_temporal') or {}
        reasons[str(evidence.get('reason', 'missing'))] += 1
        policy_ok = (profile.get('regional_enabled') is True and profile.get('active') is False
            and profile.get('whole_frame_stationarity_qualified') is False
            and profile.get('alignment_proven') is True
            and profile.get('owner_schema') == OWNER
            and profile.get('alpha') == .4 and profile.get('delta_depth_units') == 100
            and profile.get('persistence') == 0 and profile.get('visual_stationarity_required') is True
            and profile.get('visual_window_sec') == .35
            and profile.get('visual_max_motion_full_resolution_px') == .25
            and diagnostic.get('sdk_color_alignment_applied') is True
            and diagnostic.get('pair_timing', {}).get('accepted') is True)
        if not policy_ok:
            errors.append('read%d regional policy/alignment mismatch' % index)
        if (evidence.get('owner_schema') != OWNER
                or evidence.get('current_zero_pixels_preserved') is not True):
            errors.append('read%d missing ownership/zero-depth invariant' % index)
        regions = evidence.get('regions', [])
        if evidence.get('reason') == 'regional_observation':
            if ({r.get('name') for r in regions} != REGIONS or len(regions) != 5
                    or any(type(r.get('owned_pixels')) is not int or r['owned_pixels'] < 0 for r in regions)
                    or sum(r.get('owned_pixels', 0) for r in regions) != expected_pixels):
                errors.append('read%d region partition mismatch' % index)
            else:
                expected = sum(r['owned_pixels'] for r in regions if r.get('qualified') is True)
                if (expected != evidence.get('filtered_pixel_count')
                        or any(r.get('filter_applied') is not (r.get('qualified') is True) for r in regions)
                        or evidence.get('active') is not bool(expected)):
                    errors.append('read%d output eligibility mismatch' % index)
                active += int(expected > 0)
        elif evidence.get('filtered_pixel_count') != 0 or evidence.get('active') is not False:
            errors.append('read%d unqualified output is not raw' % index)
    return dict(complete_reads=len(complete), active_reads=active, reason_counts=dict(reasons),
                passed=bool(complete and active and not errors), errors=errors[:16], error_count=len(errors),
                scope='Recorded runtime contracts only; no saved pixels or support/geometry/grasp proof')


def validate_static(summary, reads, pubs, regional, rgb, depth):
    counts = summary.get('counts', {})
    accounted = (counts.get('observed', 0) > 0 and counts.get('observed') == counts.get('enqueued')
                 == counts.get('written') == reads.get('rows') and summary.get('queued_unwritten') == 0)
    return bool(summary.get('original_read_restored') is True and not summary.get('errors')
        and accounted and all(counts.get(k, 0) == 0 for k in
            ('queue_dropped', 'copy_errors', 'serialization_errors', 'write_errors'))
        and len(rgb & depth) >= 2 and len(rgb-depth) <= 2 and len(depth-rgb) <= 2
        and reads.get('steady_read_fps') is not None and 27 <= reads['steady_read_fps'] <= 33
        and all(pubs.get(k, {}).get('fps') is not None and 18 <= pubs[k]['fps'] <= 22 for k in ('rgb', 'depth'))
        and all(pubs[k].get('header_duplicates') == 0 and pubs[k].get('header_regressions') == 0
            and pubs[k].get('unique_source_stamps') == pubs[k].get('frames')
            and pubs[k].get('frames', 0) >= 2 for k in ('rgb', 'depth'))
        and reads.get('publication_worker_health', {}).get('passed') is True
        and regional.get('passed') is True)


def transaction(io):
    """No automatic rollback/retry: preserve exact failure and running camera."""
    io.guard(force=True)
    io.verify()
    io.save('prepared')
    io.stop_camera()
    io.configure()
    io.guard(force=True)
    io.start_camera()
    io.validate()
    io.guard(force=True)
    io.verify()
    io.report['static_validation_passed'] = True
    io.save('validated')


def finalize(io):
    """A failed last guard revokes success and retains a nonzero exit."""
    late_error = None
    try:
        if io.report.get('static_validation_passed'):
            io.guard(force=True)
    except BaseException as exc:
        io.report.update(static_validation_passed=False, error=repr(exc))
        late_error = exc
    finally:
        io.close()
        io.save('finished')
    if late_error is not None:
        raise late_error


def runtime_class(rospy, h, ab, alignment):
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    class Runtime:
        def __init__(self, out, receipt_path, receipt, manifest, unknown_preset, manifest_path=MANIFEST,
                     capture_visual_snapshots=False, capture_node_timings=False):
            self.ev, self.subs = None, []
            try:
                self.initialize(out, receipt_path, receipt, manifest, unknown_preset, manifest_path,
                    capture_visual_snapshots, capture_node_timings)
            except BaseException:
                self.close()
                raise

        def initialize(self, out, receipt_path, receipt, manifest, unknown_preset, manifest_path,
                       capture_visual_snapshots=False, capture_node_timings=False):
            require(type(capture_visual_snapshots) is bool,'visual capture flag must be boolean')
            require(type(capture_node_timings) is bool,'node timing flag must be boolean')
            require(not (capture_visual_snapshots and capture_node_timings), 'diagnostic taps must be isolated')
            self.capture_visual_snapshots=capture_visual_snapshots
            self.capture_node_timings=capture_node_timings
            self.out, self.receipt_path, self.receipt, self.manifest = out, receipt_path, receipt, manifest
            self.receipt_sha = sha(receipt_path)
            self.ev = ab.RosEvidence(rospy)
            self.subs, self.faults = [], []
            self.lock = threading.RLock()
            self.images = {'rgb': deque(maxlen=4096), 'depth': deque(maxlen=4096)}
            self.target_status = deque(maxlen=1024)
            self.target_status_count = 0
            self.mode, self.mode_received = None, None
            self.last_slow_guard = -math.inf
            self.monitor = alignment.AlignmentPoseMonitor(receipt)
            self.before = rospy.get_param('/camera')
            self.expected = deepcopy(self.before)
            self.desired = desired_camera_configuration(self.before, unknown_preset)
            self.baseline = baseline_identity(self.before)
            self.old = h.identity(h.node_pid(rospy, CAMERA))
            self.normal = camera_command(self.old['argv'], h)
            self.core = {name: h.identity(h.node_pid(rospy, name)) for name in CORE}
            self.ledger = rospy.get_param('/rgb_acquisition/ledger', {})
            require(not self.ledger, 'active or retained observation ledger requires prior explicit retirement')
            self.extra_files = {str(p): sha(p) for p in set(h.SOURCES + [Path(__file__).resolve(),
                W/'.runtime/camera_preset_ab_observer.py', W/'tools/diagnostics/prepare_rgb_acquisition_retry.py',
                W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py'])}
            pair_history = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/pair_history_continuity.py'
            self.extra_files[str(pair_history)] = sha(pair_history)
            if self.capture_visual_snapshots:self.extra_files[str((W/VISUAL_TAP).resolve())]=sha(W/VISUAL_TAP)
            if self.capture_node_timings:self.extra_files[str((W/NODE_TIMING_TAP).resolve())]=sha(W/NODE_TIMING_TAP)
            self.extra_files[self.baseline['path']] = self.baseline['file_sha256']
            self.unit = 'alicia-regional-production-'+out.name.rsplit('_', 1)[-1].lower()
            old_normal = dict(self.old, argv=self.normal)
            self.command = h.service_argv(self.unit, old_normal, out, h.camera_environment(self.old['pid']), 30.)
            if self.capture_visual_snapshots:
                self.command.insert(self.command.index('--'),'--capture-visual-snapshots')
            if self.capture_node_timings:
                self.command.insert(self.command.index('--'),'--capture-node-timings')
            self.report = dict(out=str(out), camera_loaded=False, static_validation_passed=False,
                robot_commands_sent=False, torque_commands_sent=False, temperature_queries_sent=False,
                selection_changed=False, alignment_confirmation_reissued=False,
                new_motion_authority_created=False, persistent_configuration_changed=False,
                unknown_preset_option=unknown_preset,
                manifest_path=str(manifest_path), manifest_sha256=sha(manifest_path), source_sha256=self.extra_files,
                baseline_identity=self.baseline,
                old_camera=self.old, core_preserved=self.core, camera_parameters_before=self.before,
                desired_camera_configuration=self.desired, command=self.command, events=[],
                upstream_stall_resolution_verified=False)
            self.report.update(receipt_report_fields(receipt,receipt_path,self.receipt_sha))
            self.report['visual_snapshots_requested']=self.capture_visual_snapshots
            self.report['node_timings_requested']=self.capture_node_timings
            self.subs.append(rospy.Subscriber('/grasp_mode/status', String, self.mode_callback, queue_size=20))
            self.subs.append(rospy.Subscriber('/perception/unknown/detector_status', String,
                self.target_callback, queue_size=20))
            for kind, key in (('rgb', 'color_topic'), ('depth', 'depth_topic')):
                self.subs.append(rospy.Subscriber(self.before[key], Image,
                    lambda msg, k=kind: self.image_callback(k, msg), queue_size=10, buff_size=2**24))
            h.wait(lambda: self.ev.guard.check(self.ev.now_ns(), freeze=True) and self.mode is not None,
                   5., 'fresh stationary feedback and inactive mode')
            snap = self.ev.guard.snapshot()
            self.monitor.observe(snap['accepted']['source_stamp_ns'], snap['accepted']['q'][:6])
            require(snap['epoch'] == int(receipt['epoch_ns']), 'alignment epoch no longer current')
            self.report['initial_feedback'] = snap
            self.report['current_selection'] = deepcopy(self.ev.initial_context['selection'])

        def target_callback(self, msg):
            # Diagnostic only: changing availability never creates a new target
            # or makes this camera-only tool submit a motion/selection request.
            try:
                require(len(msg.data) <= 128*1024, 'target status exceeds diagnostic bound')
                status = json.loads(msg.data)
            except Exception as exc:
                status = dict(diagnostic_parse_error=str(exc))
            with self.lock:
                self.target_status_count += 1
                self.target_status.append(dict(received_monotonic=time.monotonic(), status=status))

        def mode_callback(self, msg):
            with self.lock:
                try:
                    data = json.loads(msg.data)
                    if data.get('active') is not False or data.get('start_reserved') is not False:
                        self.faults.append('mode became active or reserved')
                    self.mode, self.mode_received = data, time.monotonic()
                except Exception as exc:
                    self.faults.append('mode invalid: '+str(exc))

        def image_callback(self, kind, msg):
            with self.lock:
                self.images[kind].append(dict(received_monotonic=time.monotonic(),
                    received_ros_time_ns=rospy.Time.now().to_nsec(), source_stamp_ns=msg.header.stamp.to_nsec(),
                    frame_id=msg.header.frame_id, width=msg.width, height=msg.height, encoding=msg.encoding))

        def verify(self):
            verify_manifest(self.manifest,capture_visual_snapshots=self.capture_visual_snapshots,
                            capture_node_timings=self.capture_node_timings)
            require(sha(self.receipt_path) == self.receipt_sha, 'alignment receipt changed')
            read_receipt(self.receipt_path)
            for path, expected in self.extra_files.items():
                require(sha(path) == expected, 'load-time source changed: '+path)

        def guard(self, force=False):
            require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
            require(not rospy.is_shutdown(), 'ROS shutdown during camera load')
            require(self.ev.guard.check(self.ev.now_ns(), freeze=True), 'fresh stationary feedback unavailable')
            snap = self.ev.guard.snapshot()
            self.monitor.observe(snap['accepted']['source_stamp_ns'], snap['accepted']['q'][:6])
            self.monitor.require_stationary()
            require(snap['epoch'] == int(self.receipt['epoch_ns']), 'alignment epoch changed')
            with self.lock:
                require(not self.faults, '; '.join(self.faults))
                require(self.mode is not None and self.mode_received is not None
                        and time.monotonic()-self.mode_received < 2., 'mode status stale')
                require(self.mode.get('active') is False and self.mode.get('start_reserved') is False,
                        'mode active/reserved')
            if force or time.monotonic()-self.last_slow_guard >= .25:
                self.ev.check_context()
                require(self.ev.initial_context['selection'].get('mode') == 'unknown', 'unknown mode required')
                require(rospy.get_param('/rgb_acquisition/ledger', {}) == self.ledger, 'observation ledger changed')
                require(h.stable_camera_configuration(rospy.get_param('/camera')) ==
                        h.stable_camera_configuration(self.expected), 'camera configuration changed')
                for name, owner in self.core.items():
                    require(h.alive(owner) and h.node_pid(rospy, name) == owner['pid'], 'core owner changed: '+name)
                self.last_slow_guard = time.monotonic()

        def save(self, event):
            require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
            self.report['events'].append(dict(event=event, observed_ns=rospy.Time.now().to_nsec()))
            (self.out/'result.json').write_text(json.dumps(self.report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')

        def stop_camera(self):
            self.guard(force=True)
            require(h.alive(self.old) and h.node_pid(rospy, CAMERA) == self.old['pid'], 'camera owner changed before signal')
            os.kill(self.old['pid'], signal.SIGINT)
            self.report['old_camera_sigint_pid'] = self.old['pid']
            h.wait(lambda: not h.alive(self.old), 12., 'owned camera exit', self.guard)
            self.save('old_camera_exited')

        def configure(self):
            require(not h.alive(self.old), 'old camera must exit before changing camera parameters')
            for key in ('unknown_preset', 'regional_stationary_enabled'):
                new = self.desired['mode_depth_preset'].get(key)
                if key in self.desired['mode_depth_preset'] and new != self.expected['mode_depth_preset'].get(key):
                    self.guard(force=True)
                    rospy.set_param(PARAM+key, new)
                    self.expected['mode_depth_preset'][key] = new
                    require(rospy.get_param(PARAM+key) == new, 'camera parameter write not confirmed')
            self.save('allowed_parameters_written')

        def start_camera(self):
            self.guard(force=True)
            result = subprocess.run(self.command, capture_output=True, text=True, timeout=10)
            (self.out/'systemd_start.txt').write_text(result.stdout+result.stderr)
            require(result.returncode == 0, 'camera systemd start failed')
            expected_argv = self.command[self.command.index(self.normal[0]):]
            def registered():
                pid = int(subprocess.check_output(['systemctl', '--user', 'show', self.unit,
                    '--property=MainPID', '--value'], text=True, timeout=3).strip() or 0)
                if pid <= 0:
                    return False
                try:
                    if h.node_pid(rospy, CAMERA) != pid:
                        return False
                except (h.CameraRegistrationPending, ConnectionRefusedError):
                    return False
                owner = h.identity(pid)
                require(owner['argv'] == expected_argv and owner['exe_sha256'] == self.old['exe_sha256'],
                        'new camera executable/argv mismatch')
                return owner
            self.new = h.wait(registered, 20., 'new camera service/master registration', self.guard)
            self.report.update(camera_loaded=True, new_camera=self.new, systemd_user_unit=self.unit)
            self.save('new_camera_registered')

        def validate(self):
            summary = h.wait(lambda: completed_json(self.out/'read_diagnostics.summary.json'),
                55., 'bounded metadata capture completion', self.guard)
            self.guard(force=True)
            require(h.alive(self.new) and h.node_pid(rospy, CAMERA) == self.new['pid'], 'new camera owner changed')
            reads = h.summarize_reads(self.out/'read_diagnostics.jsonl')
            rows = [json.loads(line) for line in (self.out/'read_diagnostics.jsonl').read_text().splitlines()]
            regional = regional_read_summary(rows, self.before.get('stream_width', 640)*self.before.get('stream_height', 480))
            loaded = h.validate_loaded_modules(self.out/'loaded_observer_modules.json', self.extra_files)
            verify_regional_import(loaded, self.extra_files)
            if self.capture_visual_snapshots:
                verify_visual_snapshot_import(loaded,self.extra_files)
                captured=h.wait(lambda: completed_json(self.out/'visual_snapshots/summary.json'),
                    10.,'bounded visual snapshot completion',self.guard)
                self.report['visual_snapshots']=captured
                verify_visual_capture(captured,self.out/'visual_snapshots')
                require(captured.get('source_implementation')==loaded,
                        'visual snapshot source provenance differs from actual imports')
                self.guard(force=True)
            if self.capture_node_timings:
                captured=h.wait(lambda: completed_json(self.out/'node_timings/summary.json'),
                    10., 'bounded node timing completion', self.guard)
                self.report['node_timings']=captured
                verify_node_timing_capture(captured, self.out/'node_timings', loaded, self.extra_files,
                                          self.normal[1])
                self.guard(force=True)
            lo, hi = reads['first_read_monotonic'], reads['last_read_monotonic']
            with self.lock:
                observed = {k: list(v) for k, v in self.images.items()}
                target = dict(total_received=self.target_status_count, rows=list(self.target_status))
            kept = {k: [r for r in v if lo is not None and lo+1 <= r['received_monotonic'] <= hi]
                    for k, v in observed.items()}
            for kind, encoding in (('rgb', 'bgr8'), ('depth', '16UC1')):
                require(all(r['frame_id'] == self.before['frame_id'] and r['width'] == self.before['width']
                    and r['height'] == self.before['height'] and r['encoding'] == encoding for r in kept[kind]),
                    'published image geometry/encoding changed')
            pubs = {k: h.summarize_publication(v) for k, v in kept.items()}
            stamps = {k: {r['source_stamp_ns'] for r in v} for k, v in kept.items()}
            after = rospy.get_param('/camera')
            preset = after.get('runtime_profile', {}).get('mode_depth_preset', {})
            require(preset.get('mode') == 'unknown'
                and preset.get('baseline_sha256') == self.baseline['advanced_settings_sha256']
                and preset.get('configured_unknown_preset') == self.desired['mode_depth_preset'].get('unknown_preset', 'high_accuracy')
                and preset.get('preset') == self.desired['mode_depth_preset'].get('unknown_preset', 'high_accuracy'),
                'actual SDK preset does not match requested unknown policy')
            self.report.update(read_observer=summary, reads=reads, regional_metadata=regional,
                publication=pubs, loaded_modules=loaded, camera_parameters_after=after,
                final_feedback=self.ev.guard.snapshot(), rgb_depth_common_source_stamps=len(stamps['rgb'] & stamps['depth']),
                validation_scope='Camera metadata and cadence only; no target identity, support-fit or grasp acceptance')
            (self.out/'publication_observations.json').write_text(json.dumps(observed, indent=2)+'\n')
            (self.out/'target_status_observations.json').write_text(json.dumps(target, indent=2)+'\n')
            self.report['target_status_observations'] = str(self.out/'target_status_observations.json')
            self.report['target_status_scope'] = 'Recorded lifecycle only; no automatic generation retirement or target re-selection'
            require(validate_static(summary, reads, pubs, regional, stamps['rgb'], stamps['depth']),
                    'camera loaded but regional/cadence metadata acceptance failed')

        def close(self):
            for sub in self.subs:
                sub.unregister()
            self.subs = []
            if self.ev is not None:
                self.ev.close()
                self.ev = None
    return Runtime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--alignment-receipt', type=Path, required=True,
        help='Archived explicit target-alignment OR independent camera-only stationary-capture receipt')
    parser.add_argument('--unknown-preset', choices=('preserve', 'saved_baseline'), default='preserve')
    parser.add_argument('--validation-manifest', type=Path, default=MANIFEST)
    parser.add_argument('--capture-visual-snapshots',action='store_true',
        help='Opt-in bounded raw SDK RGB and actual LK anchor diagnosis; default off')
    parser.add_argument('--capture-node-timings',action='store_true',
        help='Opt-in 30-second original node stage timing; no pixel or policy change')
    args = parser.parse_args(argv)
    if not args.execute:
        parser.error('--execute is required; no commands have been sent')
    require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
    manifest = json.loads(args.validation_manifest.read_text())
    verify_manifest(manifest,capture_visual_snapshots=args.capture_visual_snapshots,
                    capture_node_timings=args.capture_node_timings)
    receipt = read_receipt(args.alignment_receipt)
    h = load('regional_production_owner', '.runtime/gen4_root_cause/reload_camera_visual_gate.py')
    ab = load('regional_production_feedback', '.runtime/camera_preset_ab_observer.py')
    alignment = load('regional_production_alignment', 'tools/diagnostics/prepare_rgb_acquisition_retry.py')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    out = EXT/'ros_log'/('regional_production_'+stamp)
    out.mkdir(); (out/'ros').mkdir()
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    import rospy
    rospy.init_node('load_regional_production_camera_only', anonymous=True, disable_signals=True)
    io = None
    error = None
    def interrupted(sig, _frame):
        raise RuntimeError('regional production loader interrupted by signal %d' % sig)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGALRM):
        signal.signal(sig, interrupted)
    signal.alarm(115)
    try:
        io = runtime_class(rospy, h, ab, alignment)(out, args.alignment_receipt.resolve(), receipt, manifest,
            args.unknown_preset, args.validation_manifest.resolve(),args.capture_visual_snapshots,
            args.capture_node_timings)
        transaction(io)
    except BaseException as exc:
        error = repr(exc)
        if io is not None:
            io.report.update(static_validation_passed=False, error=error)
        raise
    finally:
        signal.alarm(0)
        if io is not None:
            finalize(io)
        else:
            require(MOUNT.is_mount(), 'SanStand unavailable; no fallback')
            failure=dict(camera_loaded=False,static_validation_passed=False,error=error,robot_commands_sent=False)
            failure.update(receipt_report_fields(receipt,args.alignment_receipt,sha(args.alignment_receipt)))
            (out/'result.json').write_text(json.dumps(failure,indent=2)+'\n')
        print(json.dumps(dict(out=str(out), camera_loaded=bool(io and io.report['camera_loaded']),
            static_validation_passed=bool(io and io.report['static_validation_passed']), error=error)), flush=True)


if __name__ == '__main__':
    main()
