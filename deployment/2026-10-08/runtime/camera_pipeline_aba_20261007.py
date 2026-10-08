#!/usr/bin/env python3
"""Opt-in INNER acquisition diagnostic; parent owns camera exclusion/restoration.

One pipeline, A1 original processing / B raw wait + metadata / A2 processing.
No ROS publishers, parameter writes, selection changes, or arm commands. This
does not acquire permission to replace a live camera. Its caller must hold the
existing camera-only maintenance contract and await actual process termination.
The 90 second acquisition deadline is cooperative: an original SDK wait or
pipeline close can exceed it. Boundary reads and incomplete cleanup are explicit.
"""
import argparse
import copy
import hashlib
import importlib.util
import importlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
VISION = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision'
CAMERA = W/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
AB = W/'.runtime/camera_preset_ab_observer.py'
SDK = W/'.runtime/realsense-2.56.5'
SDK_SO = SDK/'pyrealsense2.cpython-38-x86_64-linux-gnu.so'
SDK_LIB = SDK/'lib/librealsense2.so.2.56.5'
PHASES = ('A1', 'B', 'A2')
PHASE_SECONDS = 30.
TOTAL_SECONDS = 90.
MAX_BYTES = 12*1024*1024
METADATA_BYTES = MAX_BYTES-768*1024  # Includes room for bounded ROS files + both sidecars.
EXTRA_METADATA = ('actual_exposure', 'gain_level', 'auto_exposure')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True,
                       separators=(',', ':'), allow_nan=False)+'\n').encode()


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stable_camera(config):
    result = copy.deepcopy(config)
    result.pop('runtime_profile', None)
    return result


def required_sources():
    return [Path(__file__).resolve(), CAMERA, AB, AB.with_name('camera_stage_observer.py'),
            SDK_SO, SDK_LIB] + [VISION/name for name in (
                'realsense_manager.py', 'stationary_depth.py', 'visual_stationary.py',
                'regional_stationary_depth.py', 'pair_history_continuity.py',
                'depth_preset.py', 'rgbd_rectification.py', 'color_projection.py')]


def verify_sources(contract):
    hashes = contract['source_sha256']
    require(isinstance(hashes, dict), 'source_hashes_required')
    required = required_sources()+[Path(contract['camera_config']['mode_depth_preset']['baseline_path']).expanduser().resolve()]
    for path in required:
        require(str(path) in hashes, 'unfrozen_dependency:'+str(path))
    for name, digest in hashes.items():
        require(Path(name).is_absolute() and isinstance(digest, str)
                and len(digest) == 64 and sha(name) == digest, 'source_changed:'+str(name))
    return copy.deepcopy(hashes)


def validate_contract(contract):
    require(contract.get('schema') == 1 and
            contract.get('kind') == 'camera_pipeline_aba_inner', 'wrong_contract_type')
    require(contract.get('camera_diagnostic_only') is True and
            contract.get('motion_authorized') is False, 'diagnostic_only_required')
    c = contract['camera_config']
    require(c.get('simulate') is False and c.get('fallback_to_simulation') is False
            and c.get('align_depth_to_color') is True
            and c.get('pair_rectification', {}).get('enabled') is True
            and c.get('stream_width', c.get('width')) == 640
            and c.get('stream_height', c.get('height')) == 480
            and c.get('fps') == 30, 'native_real_640x480_30_policy_required')
    p = c.get('mode_depth_preset', {})
    require(all(p.get(k) is True for k in ('enabled', 'stationary_temporal_enabled',
                                          'regional_stationary_enabled'))
            and p.get('unknown_preset', 'high_accuracy') == 'high_accuracy',
            'original_regional_high_accuracy_required')
    require(isinstance(contract.get('perception_config'), dict), 'perception_config_required')
    require(contract.get('selection', {}).get('mode') == 'unknown', 'unknown_selection_required')
    epoch = contract.get('epoch_ns')
    require(type(epoch) is int and epoch > 0, 'positive_epoch_required')
    feedback = contract.get('expected_feedback', {})
    require(feedback.get('epoch') == epoch, 'feedback_epoch_mismatch')
    for kind, size in (('sdk', 6), ('accepted', 7)):
        item = feedback.get(kind, {})
        q = item.get('q', ())
        require(len(q) == size and all(type(v) in (float, int) and math.isfinite(v) for v in q),
                'complete_finite_'+kind+'_required')
        require(type(item.get('source_stamp_ns')) is int
                and item['source_stamp_ns'] >= epoch, 'feedback_before_epoch')
    require(isinstance(feedback.get('task'), dict) and feedback['task'].get('active') is False,
            'inactive_task_required')
    return copy.deepcopy(contract)


class MetadataWriter:
    """Single bounded worker; loss/budget errors fail the experiment, never hide."""
    def __init__(self, path, *, maximum=METADATA_BYTES, queue_size=128):
        self.path, self.maximum = Path(path), maximum
        self.queue = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.bytes_reserved = self.rows_reserved = self.rows_written = 0
        self.error = None
        self.closed = False
        self.digest = hashlib.sha256()
        self.file = self.path.open('xb')
        self.thread = threading.Thread(target=self._work, name='aba-metadata', daemon=True)
        self.thread.start()

    def _work(self):
        try:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        break
                    self.file.write(item)
                    self.digest.update(item)
                    self.rows_written += 1
                finally:
                    self.queue.task_done()
            self.file.flush()
        except Exception as exc:
            self.error = type(exc).__name__+': '+str(exc)
        finally:
            try:
                self.file.close()
            except Exception as exc:
                self.error = self.error or str(exc)

    def check(self):
        if self.error:
            raise RuntimeError('metadata_writer_failed:'+self.error)

    def put(self, row):
        data = encoded(row)
        require(len(data) <= 64*1024, 'metadata_row_bound_exceeded')
        with self.lock:
            self.check()
            require(not self.closed, 'metadata_writer_closed')
            require(self.bytes_reserved+len(data) <= self.maximum, 'metadata_budget_exhausted')
            try:
                self.queue.put_nowait(data)
            except queue.Full:
                raise RuntimeError('metadata_queue_full')
            self.bytes_reserved += len(data)
            self.rows_reserved += 1

    def close(self):
        deadline = time.monotonic()+3.
        with self.lock:
            if not self.closed:
                self.closed = True
                try:
                    self.queue.put(None, timeout=max(0., deadline-time.monotonic()))
                except queue.Full:
                    self.error = self.error or 'sentinel_queue_full'
        self.thread.join(max(0., deadline-time.monotonic()))
        complete = (not self.thread.is_alive() and self.error is None
                    and self.rows_reserved == self.rows_written)
        return dict(drained=complete, worker_alive=self.thread.is_alive(), error=self.error,
                    rows_reserved=self.rows_reserved, rows_written=self.rows_written,
                    bytes_reserved=self.bytes_reserved,
                    sha256=self.digest.hexdigest() if complete else None)


def metadata_adapter(original, rs):
    """Inspect only received frames, never options/sensors or image data."""
    def inspect(frames):
        result = original(frames)
        for stream in ('color', 'depth'):
            row = result.get(stream)
            if not isinstance(row, dict) or row.get('present') is not True:
                continue
            available, unavailable = {}, {}
            try:
                frame = getattr(frames, 'get_'+stream+'_frame')()
                for name in EXTRA_METADATA:
                    try:
                        field = getattr(rs.frame_metadata_value, name)
                        if frame.supports_frame_metadata(field):
                            available[name] = int(frame.get_frame_metadata(field))
                        else:
                            unavailable[name] = 'unsupported'
                    except Exception as exc:
                        unavailable[name] = type(exc).__name__
            except Exception as exc:
                unavailable['frame'] = type(exc).__name__
            row['diagnostic_extra_metadata'] = dict(values=available, unavailable=unavailable)
        return result
    return inspect


def raw_read(manager):
    """B has no align/filter/arrays/publishing and does not mutate A histories."""
    started = manager._diagnostic_clock()
    diagnostic = dict(schema=2, result='error', failed_stage=None, stages_sec={}, raw_only=True)
    try:
        host = diagnostic['host_wait'] = {'enter': manager._host_wait_snapshot()}
        try:
            frames = manager.pipeline.wait_for_frames()
        finally:
            host['exit'] = manager._host_wait_snapshot()
            host['elapsed_sec'] = manager._diagnostic_elapsed(host['enter']['monotonic_sec'], host['exit']['monotonic_sec'])
            host['thread_cpu_elapsed_sec'] = manager._diagnostic_elapsed(host['enter']['thread_cpu_sec'], host['exit']['thread_cpu_sec'])
            diagnostic['stages_sec']['wait_for_frames'] = host['elapsed_sec']
        start = manager._diagnostic_clock()
        diagnostic['raw_frames'] = manager._raw_frame_diagnostics(frames)
        diagnostic['stages_sec']['raw_frame_diagnostics'] = manager._diagnostic_clock()-start
        diagnostic['pair_timing'] = manager._pair_timing(diagnostic['raw_frames'])
        diagnostic['result'] = 'raw_complete'  # Includes rejected/missing pairs; see original pair_timing.
        return diagnostic
    except Exception as exc:
        diagnostic['error'] = type(exc).__name__+': '+str(exc)
        raise
    finally:
        diagnostic['total_sec'] = manager._diagnostic_clock()-started
        manager.last_read_diagnostics = diagnostic


def compact_diagnostic(diagnostic, profiles):
    """Keep exact clocks/counters; repeated SDK profile matrices recorded once."""
    keys = ('schema', 'result', 'raw_only', 'failed_stage', 'error', 'total_sec', 'host_wait',
            'stages_sec', 'pair_timing', 'sdk_color_alignment_applied',
            'stationary_pair_filter_host_monotonic')
    result = {k: copy.deepcopy(diagnostic[k]) for k in keys if k in diagnostic}
    raw = copy.deepcopy(diagnostic.get('raw_frames', {}))
    for stream, row in raw.items():
        if not isinstance(row, dict):
            continue
        signature = row.pop('profile_signature', None)
        if signature is not None:
            digest = hashlib.sha256(encoded(signature)).hexdigest()
            if stream in profiles:
                require(profiles[stream]['sha256'] == digest, 'sdk_profile_changed:'+stream)
            else:
                profiles[stream] = dict(sha256=digest, value=signature)
            row['profile_sha256'] = digest
    result['raw_frames'] = raw
    regional = diagnostic.get('regional_stationary_evidence')
    if isinstance(regional, dict):
        result['regional'] = {k: copy.deepcopy(regional[k]) for k in
            ('enabled', 'active', 'fully_active', 'reason', 'global_history_reset_reason', 'filtered_fraction',
             'filtered_pixel_count', 'changed_pixel_count', 'current_zero_pixels_preserved')
            if k in regional}
        result['regional']['regions'] = []
        for region in regional.get('regions', []):
            visual = region.get('visual', {})
            result['regional']['regions'].append({
                **{k: copy.deepcopy(region[k]) for k in ('name', 'qualified', 'warming',
                    'history_reset', 'filter_applied') if k in region},
                # Source is the same original raw color above; immutable gate
                # constants stay in frozen source, not repeated five times.
                'visual': {k: copy.deepcopy(visual[k]) for k in
                    ('qualified', 'reason', 'tracked_points', 'covered_quadrants',
                     'continuous_frames', 'host_quiet_span_sec', 'quiet_span_sec',
                     'motion_median_px', 'motion_p90_px') if k in visual}})
    decision = diagnostic.get('stationary_pair_history')
    if isinstance(decision, dict):
        result['pair_history'] = {k: copy.deepcopy(decision[k]) for k in
            ('state', 'reason', 'qualified', 'failed_stage') if k in decision}
    return result


def acquisition(manager, evidence, writer, *, clock=time.monotonic, cancelled=lambda: False):
    """Injected core: fixed 30/30/30; never starts/stops pipeline or changes preset."""
    start = clock()
    process_cpu_start, thread_cpu_start = time.process_time(), time.thread_time()
    phase = None
    counts = {key: 0 for key in PHASES}
    boundary = 0
    profiles = {}
    owners = tuple(id(getattr(manager, k, None)) for k in
                   ('pipeline', 'align', 'mode_depth_preset', 'stationary_depth_provider'))
    while clock()-start < TOTAL_SECONDS:
        require(not cancelled(), 'diagnostic_cancelled')
        evidence.check()
        writer.check()
        elapsed = clock()-start
        if elapsed >= TOTAL_SECONDS:
            break
        requested = PHASES[int(elapsed//PHASE_SECONDS)]
        if phase != requested:
            manager._reset_stationary_history('diagnostic_stage_transition_'+requested)
            manager._previous_completed_raw_frames = None
            manager._stationary_pair_last_decision = None
            writer.put(dict(kind='stage_transition', phase=requested,
                            monotonic_sec=clock(), elapsed_sec=clock()-start,
                            explicit_history_reset=True, source_counters_unchanged=True))
            phase = requested
        require(owners == tuple(id(getattr(manager, k, None)) for k in
                    ('pipeline', 'align', 'mode_depth_preset', 'stationary_depth_provider')),
                'pipeline_or_filter_owner_changed')
        begin = clock()
        cpu = time.thread_time()
        error = None
        try:
            if requested == 'B':
                raw_read(manager)
            else:
                images = manager.read()
                del images
        except Exception as exc:
            error = type(exc).__name__+': '+str(exc)
        end = clock()
        cpu = time.thread_time()-cpu
        lower = PHASES.index(requested)*PHASE_SECONDS
        inside = lower <= begin-start < end-start <= lower+PHASE_SECONDS
        known_profiles = set(profiles)
        diagnostic = compact_diagnostic(manager.last_read_diagnostics or {}, profiles)
        for stream in profiles.keys()-known_profiles:
            writer.put(dict(kind='sdk_profile', stream=stream, **profiles[stream]))
        row = dict(kind='read', requested_phase=requested,
                   phase=requested if inside else 'boundary',
                   start_monotonic_sec=begin, end_monotonic_sec=end,
                   elapsed_start_sec=begin-start, elapsed_end_sec=end-start,
                   read_thread_cpu_sec=cpu, error=error,
                   diagnostic=diagnostic)
        writer.put(row)
        if inside:
            counts[requested] += 1
        else:
            boundary += 1
        if error:
            raise RuntimeError(error)
        evidence.check()
    require(not cancelled(), 'diagnostic_cancelled')
    require(all(counts.values()), 'phase_has_no_complete_read_window')
    return dict(acquisition_finished=True, acquisition_elapsed_sec=clock()-start,
                process_cpu_sec=time.process_time()-process_cpu_start,
                acquisition_thread_cpu_sec=time.thread_time()-thread_cpu_start,
                nominal_acquisition_sec=TOTAL_SECONDS, phase_counts=counts,
                boundary_reads=boundary, profiles=profiles,
                hard_time_bound_claimed=False)


class FeedbackEvidence:
    """Original callback parsing + guard + camera's actual stationarity provider."""
    def __init__(self, rospy, ab, contract):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        from alicia_flexible_grasp_supervisor.msg import GraspState
        from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
        self.rospy, self.ab, self.contract = rospy, ab, contract
        self.guard, self.window = ab.EvidenceGuard(), StationaryDepthWindow()
        self.subs = []
        self.last_context = float('-inf')
        try:
            for topic, cls, callback, size in (
                ('/alicia_d/accepted_joint_states', JointState, lambda m:self.joints('accepted', m), 10),
                ('/alicia_d/sdk_command', JointState, lambda m:self.joints('sdk', m), 10),
                ('/alicia_d/control_reference_epoch', Header,
                 lambda m:self.guard.observe('epoch', m.stamp.to_nsec(), None), 10),
                ('/grasp/state', GraspState, lambda m:ab.RosEvidence.task(self, m), 20)):
                self.subs.append(rospy.Subscriber(topic, cls, callback, queue_size=size))
        except Exception:
            self.close()
            raise

    def joints(self, kind, message):
        self.ab.RosEvidence.joints(self, kind, message)
        self.window.observe(kind, message)

    def context(self):
        r = self.rospy
        require(stable_camera(r.get_param_cached('/camera')) ==
                stable_camera(self.contract['camera_config']), 'camera_config_changed')
        require(r.get_param_cached('/perception', {}) == self.contract['perception_config'],
                'perception_config_changed')
        require(r.get_param_cached('/grasp_mode/selection') == self.contract['selection'],
                'selection_changed')

    def check(self, *, freeze=False):
        now = self.rospy.Time.now().to_nsec()
        require(self.guard.check(now, freeze=freeze), 'fresh_complete_feedback_required')
        snapshot = self.guard.snapshot()
        expected = self.contract['expected_feedback']
        require(snapshot['epoch'] == self.contract['epoch_ns'], 'epoch_changed')
        for kind in ('accepted', 'sdk'):
            require(snapshot[kind]['q'] == expected[kind]['q'], kind+'_differs_from_outer_contract')
        require(snapshot['task'] == expected['task'], 'task_differs_from_outer_contract')
        if time.monotonic()-self.last_context >= .25 or freeze:
            self.context()
            self.last_context = time.monotonic()

    def ready(self, cancelled):
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline:
            require(not cancelled(), 'diagnostic_cancelled')
            if self.guard.check(self.rospy.Time.now().to_nsec()):
                self.check(freeze=True)
                return
            time.sleep(.01)
        raise RuntimeError('initial_feedback_unavailable')

    def stationary(self):
        return self.window.stationary(self.rospy.get_time())

    def close(self):
        for sub in self.subs:
            sub.unregister()
        self.subs = []


def validate_loaded_sources(contract, rs, camera):
    require(getattr(rs, '__version__', None) == '2.56.5'
            and Path(rs.__file__).resolve() == SDK_SO.resolve(), 'wrong_real_sdk_binding')
    require(str(SDK_LIB.resolve()) in Path('/proc/self/maps').read_text(), 'wrong_loaded_sdk_library')
    modules = [camera] + [importlib.import_module('alicia_flexible_grasp.vision.'+name) for name in (
        'realsense_manager', 'stationary_depth', 'visual_stationary',
        'regional_stationary_depth', 'pair_history_continuity', 'depth_preset',
        'rgbd_rectification', 'color_projection')]
    for module in modules:
        path = str(Path(module.__file__).resolve())
        require(path in contract['source_sha256'] and sha(path) == contract['source_sha256'][path],
                'unexpected_loaded_source:'+path)
    return {module.__name__: str(Path(module.__file__).resolve()) for module in modules}


def validate_runtime_profile(manager, contract):
    actual = manager.runtime_profile
    expected = contract['camera_config'].get('runtime_profile', {})
    for key in ('sdk_version', 'device', 'color_intrinsics', 'native_depth_intrinsics',
                'depth_to_color_extrinsics', 'alignment', 'geometry_intrinsics',
                'depth_scale_m_per_unit', 'depth_filters', 'color_projection_correction',
                'rectification', 'mode_depth_preset'):
        require(key in actual and key in expected and actual[key] == expected[key],
                'active_runtime_profile_mismatch:'+key)
    for key in ('color_intrinsics', 'native_depth_intrinsics'):
        require(actual[key].get('width') == 640 and actual[key].get('height') == 480,
                'active_native_resolution_mismatch')


def close_manager_strict(manager):
    """Original cleanup order, but do not suppress SDK stop failure as manager.stop does."""
    errors = []
    try:
        if manager.mode_depth_preset is not None:
            manager.mode_depth_preset.close()
    except BaseException as exc:
        errors.append('preset_close: '+str(exc))
    try:
        manager._reset_stationary_history('diagnostic_final_close')
        manager._sdk_color_alignment_owner = None
    except BaseException as exc:
        errors.append('history_close: '+str(exc))
    stopped = manager.pipeline is None
    if manager.pipeline is not None:
        try:
            manager.pipeline.stop()
            manager.pipeline = None
            stopped = True
        except BaseException as exc:
            errors.append('pipeline_stop: '+str(exc))
    return dict(pipeline_closed=stopped, errors=errors)


def write_small(path, row):
    data = encoded(row)
    require(len(data) <= 128*1024, 'sidecar_budget_exceeded')
    with Path(path).open('xb') as stream:
        stream.write(data)


def configure_external_logging(out):
    """Same process-local bounded ROS log policy as the existing 15 s recorder."""
    out = Path(out)
    os.environ['ROS_LOG_DIR'] = str(out/'ros')
    config = out/'python_logging.conf'
    config.write_text('''[loggers]
keys=root
[handlers]
keys=fileHandler
[formatters]
keys=defaultFormatter
[logger_root]
level=INFO
handlers=fileHandler
[handler_fileHandler]
class=handlers.RotatingFileHandler
level=INFO
formatter=defaultFormatter
args=(os.environ['ROS_LOG_FILENAME'],'a', 131072, 1)
[formatter_defaultFormatter]
format=[%(name)s][%(levelname)s] %(asctime)s: %(message)s
''')
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = str(config)


def directory_bytes(out):
    return sum(p.stat().st_size for p in Path(out).rglob('*') if p.is_file())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--enable-pipeline-aba', action='store_true')
    args = parser.parse_args()
    if not args.enable_pipeline_aba:
        parser.error('explicit --enable-pipeline-aba required; no camera opened')
    contract = validate_contract(json.loads(args.contract.read_text()))
    verify_sources(contract)
    out = args.output.resolve()
    require(EXT in out.parents and Path('/media/zhuyupei/SanStand').is_mount(), 'SanStand_mount_required')
    out.mkdir(parents=True, exist_ok=False)
    report = dict(schema=1, kind='camera_pipeline_aba_inner', complete=False,
        contract_path=str(args.contract.resolve()), contract_sha256=sha(args.contract),
        source_sha256=contract['source_sha256'], started_wall_ns=time.time_ns(),
        no_image_publications=True, no_robot_commands=True, original_thresholds_unchanged=True,
        limits=dict(acquisition_seconds=90, cooperative_deadline=True, directory_bytes=MAX_BYTES),
        extra_metadata_units=dict(actual_exposure='microseconds', gain_level='SDK_relative_integer',
                                  auto_exposure='SDK_flag'), errors=[])
    manager = evidence = writer = None
    raw_original = None
    cancelled = threading.Event()
    old_signals = {}
    rospy = None
    try:
        configure_external_logging(out)
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_signals[signum] = signal.signal(signum, lambda *_:cancelled.set())
        import rospy
        import cv2
        import pyrealsense2 as rs
        cv2.setNumThreads(1)  # Exactly the current camera process policy.
        report['opencv_threads'] = cv2.getNumThreads()
        rospy.init_node('camera_pipeline_aba_diagnostic', anonymous=True,
                        disable_signals=True, disable_rosout=True)
        ab = load_module('aba_original_evidence', AB)
        camera = load_module('aba_original_camera', CAMERA)
        report['loaded_sources'] = validate_loaded_sources(contract, rs, camera)
        evidence = FeedbackEvidence(rospy, ab, contract)
        evidence.ready(cancelled.is_set)
        verify_sources(contract)
        c = contract['camera_config']
        adapter = SimpleNamespace(cfg=copy.deepcopy(c), stream_width=640, stream_height=480,
                                  fps=30, align_depth_to_color=True)
        manager = camera.CameraNode._make_camera(adapter, False)
        # All subsequent failures, including a partially failed start, close it.
        require(not cancelled.is_set(), 'diagnostic_cancelled')
        report['pipeline_start_begin_monotonic_sec'] = time.monotonic()
        manager.start()
        report['pipeline_started_monotonic_sec'] = time.monotonic()
        manager.stationary_depth_provider = evidence.stationary
        manager.set_depth_preset_mode('unknown')
        report['preset_applied_monotonic_sec'] = time.monotonic()
        validate_runtime_profile(manager, contract)
        require(manager.mode_depth_preset.evidence().get('preset') == 'high_accuracy',
                'actual_high_accuracy_required')
        report['initial_runtime_profile'] = copy.deepcopy(manager.runtime_profile)
        raw_original = manager._raw_frame_diagnostics
        manager._raw_frame_diagnostics = metadata_adapter(raw_original, rs)
        writer = MetadataWriter(out/'metadata.jsonl')
        evidence.check()
        verify_sources(contract)
        require(not cancelled.is_set(), 'diagnostic_cancelled')
        report['initial_feedback'] = evidence.guard.snapshot()
        write_small(out/'ready.json', dict(schema=1, ready=True, pid=os.getpid(),
            monotonic_sec=time.monotonic(), wall_ns=time.time_ns(),
            contract_sha256=report['contract_sha256'], source_sha256=contract['source_sha256'],
            camera_diagnostic_only=True, motion_authorized=False))
        report['acquisition'] = acquisition(manager, evidence, writer, cancelled=cancelled.is_set)
        evidence.check()
        report['final_feedback'] = evidence.guard.snapshot()
    except BaseException as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        if manager is not None:
            if raw_original is not None:
                manager._raw_frame_diagnostics = raw_original
            cleanup = close_manager_strict(manager)
            report['pipeline_closed'] = cleanup['pipeline_closed']
            report['errors'].extend(cleanup['errors'])
        else:
            report['pipeline_closed'] = True
        if evidence is not None:
            try:
                evidence.close()
            except BaseException as exc:
                report['errors'].append('unsubscribe: '+str(exc))
        if writer is not None:
            report['writer'] = writer.close()
            if not report['writer']['drained']:
                report['errors'].append('metadata_not_drained')
        try:
            verify_sources(contract)
            require(sha(args.contract) == report['contract_sha256'], 'contract_changed')
        except BaseException as exc:
            report['errors'].append('final_source_check: '+str(exc))
        report['completed_wall_ns'] = time.time_ns()
        report['complete'] = bool(not report['errors'] and report.get('acquisition', {}).get('acquisition_finished')
                                  and report.get('pipeline_closed') and report.get('writer', {}).get('drained'))
        report['directory_bytes_before_summary'] = directory_bytes(out)
        require(report['directory_bytes_before_summary']+len(encoded(report)) < MAX_BYTES,
                'directory_budget_exceeded')
        write_small(out/'summary.json', report)
        for signum, old in old_signals.items():
            signal.signal(signum, old)
    return 0 if report['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
